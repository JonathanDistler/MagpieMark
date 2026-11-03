# Live telemetry and portable policies

Run the normal simulator command with `--dashboard` appended. The launcher builds
the updated ROS package and starts a separate window beside Gazebo:

```bash
python scripts/start_simulation.py --n 1 --dashboard
```

Use the same ROS selection/environment options as your normal simulation command.
Alternatively, in a terminal with the built ROS overlay sourced:

```bash
ros2 run swarm_mavsdk_ros2 telemetry_dashboard --n 3
```

The desktop window requires Tk (`python3-tk` for system Python, `tk` in Conda)
and a graphical display. Select a drone at the top right. The window is read-only;
closing it does not send flight commands. Data older than two seconds is marked
STALE and excluded from current readings. The plot retains the last 400 samples;
each drone has its own local NED origin. The amber goal is the stored position
target; in velocity mode it is not an active position command.

The dashboard subscribes to `/drone_N/state`, `control_state`, `battery`,
`attitude`, `vehicle_status`, and `local_telemetry`. Preflight PASS is PX4's
aggregate `pre_flight_checks_pass`, not a reconstruction of individual PX4 checks.
Estimator position/velocity validity and failure bits are displayed separately.
Attitude is PX4's actual FRD-to-NED quaternion, also copied into odometry;
before attitude arrives, odometry has an unavailable (zero) quaternion.

## Portable policy library

`drone_policy` ships in the ROS Python package but imports only Python's standard
library. It can be copied/packaged independently for a training environment. It
provides an immutable `Observation`, a `Policy` protocol, a transport `Backend`,
and `PolicyController`. Input vector v1 is explicitly ordered
`N,E,D,VN,VE,VD`, in metres and metres/second. Actions are normalized NED
velocity triples; each component is clipped to [-1,1], then the entire vector is
scaled so total speed cannot exceed `max_speed_mps`. Down is positive.

The existing `DroneRlInterface` remains compatible; the new controller is opt-in
because its total-speed limit differs from the older per-axis limit.

Use your existing `DroneCommandClient` with the new adapter:

```python
from drone_policy import PolicyController
from swarm_mavsdk_ros2.policy_backend import RosPolicyBackend

controller = PolicyController(RosPolicyBackend(client), max_speed_mps=0.5,
                              max_age_s=0.5)
# After explicit startup/arm/takeoff, in your ROS control loop:
try:
    controller.tick(policy)  # policy.act(observation) returns [north,east,down]
finally:
    client.stop()
```

Keep servicing the client's ROS executor between ticks. `tick` is nonblocking;
the caller owns inference cadence. Invalid actions, invalid/stale state, inference
errors, and transport errors trigger a backend stop request. A stop request may
itself fail when the transport is unavailable. No method implicitly arms or takes
off. Observation receive times use the host monotonic clock.

For PX4 hardware, use the same DroneNode public topics and set its
`px4_namespace` and `system_id` to match the hardware DDS link. A different
transport implements `observation`, `send_velocity_ned`, and `stop` while keeping
the policy unchanged. Match the trained origin, frame, units, preprocessing,
control cadence and action limits on both systems. An ENU/body-frame backend must
convert to this contract explicitly.

This is a portable deployment interface, not a validated hardware flight stack.
The current DroneNode continuously streams its last command: loss of the policy
process alone does not expire that command. Configure and validate command expiry,
estimator-reset handling, status freshness supervision and flight limits in the
hardware transport/autopilot before flight. The adapter checks armed/OFFBOARD
values but does not yet track their receive ages. No physical hardware test was
performed.
