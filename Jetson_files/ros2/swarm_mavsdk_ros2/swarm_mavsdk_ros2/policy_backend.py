"""Use the same policy contract with PX4 SITL or a hardware DroneNode."""
from drone_policy import Observation


class RosPolicyBackend:
    def __init__(self, client):
        self.client = client

    def observation(self):
        state = self.client.state
        if state is None:
            raise RuntimeError('No vehicle telemetry')
        p, v = state.pose.pose.position, state.twist.twist.linear
        return Observation((p.x, p.y, p.z), (v.x, v.y, v.z),
                           self.client.state_received_at)

    def send_velocity_ned(self, velocity_mps):
        if self.client.armed is not True or self.client.offboard is not True:
            raise RuntimeError('Policy requires confirmed armed OFFBOARD status')
        self.client.set_velocity(*velocity_mps)

    def stop(self):
        self.client.stop()
