"""Transport-independent policy contract. SI units, local NED, FRD body axes.

No simulator, ROS, trainer, or neural-network dependencies. Real transports
implement Backend; policies implement act(observation) -> normalized NED velocity.
"""
from dataclasses import dataclass
import math
import time
from typing import Protocol, Sequence


@dataclass(frozen=True)
class Observation:
    position_ned_m: tuple[float, float, float]
    velocity_ned_mps: tuple[float, float, float]
    received_at_s: float  # host monotonic time, never PX4 boot or wall time

    def vector(self) -> tuple[float, ...]:
        """Stable v1 policy input order: N,E,D,VN,VE,VD (no normalization)."""
        return (*self.position_ned_m, *self.velocity_ned_mps)


class Backend(Protocol):
    def observation(self) -> Observation: ...
    def send_velocity_ned(self, velocity_mps: tuple[float, float, float]) -> None: ...
    def stop(self) -> None: ...


class Policy(Protocol):
    def act(self, observation: Observation) -> Sequence[float]: ...


class PolicyController:
    """Validate telemetry and bound total speed consistently across transports.

    The transport/autopilot owns command expiry and flight-mode supervision.
    This class never arms, takes off, or changes modes implicitly.
    """
    def __init__(self, backend: Backend, max_speed_mps=1.0, max_age_s=0.5):
        if not all(math.isfinite(v) and v > 0 for v in (max_speed_mps, max_age_s)):
            raise ValueError('Speed and telemetry age limits must be finite and positive')
        self.backend = backend
        self.max_speed_mps = max_speed_mps
        self.max_age_s = max_age_s

    def apply(self, action: Sequence[float]) -> None:
        try:
            values = tuple(float(v) for v in action)
            if len(values) != 3 or not all(math.isfinite(v) for v in values):
                raise ValueError('Expected three finite normalized NED velocity values')
            state = self.backend.observation()
            age = time.monotonic() - state.received_at_s
            if (not math.isfinite(age) or not 0 <= age <= self.max_age_s
                    or len(state.vector()) != 6
                    or not all(math.isfinite(v) for v in state.vector())):
                raise RuntimeError('Policy telemetry is stale or invalid')
            bounded = tuple(max(-1., min(1., v)) for v in values)
            scale = self.max_speed_mps / max(1., math.hypot(*bounded))
            self.backend.send_velocity_ned(tuple(v * scale for v in bounded))
        except Exception:
            self.backend.stop()
            raise

    def tick(self, policy: Policy) -> None:
        try:
            action = policy.act(self.backend.observation())
        except Exception:
            self.backend.stop()
            raise
        self.apply(action)
