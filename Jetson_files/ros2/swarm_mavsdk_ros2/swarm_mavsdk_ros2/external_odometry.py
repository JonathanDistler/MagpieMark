"""PoseStamped tracker -> per-vehicle PX4 visual odometry (pose only)."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TransformStamped
from px4_msgs.msg import VehicleOdometry
from .odometry_conversion import settings, transform, valid_sample
from .vehicle_config import load_vehicles


class ExternalOdometry(Node):
    def __init__(self, **kwargs):
        super().__init__('external_odometry', **kwargs)
        self.declare_parameter('num_drones', 1)
        self.declare_parameter('vehicle_config', '')
        self.vehicles = load_vehicles(self.get_parameter('vehicle_config').value,
                                      self.get_parameter('num_drones').value)
        self.last_sample = {}
        self.last_received = {}
        self.last_warning = {}
        self.publishers_by_id = {}
        self.configs = {}
        self.sample_counts = {}
        self.rate_checked_at = time.monotonic()
        self.declare_parameter('minimum_rate_hz', 20.0)
        self.minimum_rate_hz = float(self.get_parameter('minimum_rate_hz').value)
        if not math.isfinite(self.minimum_rate_hz) or self.minimum_rate_hz <= 0:
            raise ValueError('minimum_rate_hz must be finite and positive')
        for v in self.vehicles:
            i = v['drone_id']
            self.configs[i] = settings(v.get('odometry'))
            topic = v['px4_namespace'] + '/fmu/in/vehicle_visual_odometry'
            self.publishers_by_id[i] = self.create_publisher(VehicleOdometry, topic, 10)
            message_type = PoseStamped if v['tracking_message'] == 'PoseStamped' else TransformStamped
            self.create_subscription(message_type, v['tracking_topic'],
                                     lambda msg, v=v: self.receive_tracking(v, msg), qos_profile_sensor_data)
            self.get_logger().info(f"drone {i}: {v['tracking_topic']} -> {topic}; body={v['mocap_body'] or 'UNASSIGNED'}")
        self.create_timer(1., self.check_tracking)
        self.create_timer(5., self.check_rate)

    def warn(self, i, text):
        now = time.monotonic()
        if now - self.last_warning.get(i, -math.inf) >= 2.:
            self.get_logger().warning(f'drone {i}: {text}')
            self.last_warning[i] = now

    def receive_tracking(self, vehicle, msg):
        if vehicle['tracking_message'] == 'TransformStamped':
            # A dedicated topic is the association for PoseStamped. Transforms
            # additionally carry a body identifier; verify it before conversion.
            if vehicle['mocap_body'] and msg.child_frame_id != vehicle['mocap_body']:
                self.warn(vehicle['drone_id'], 'unexpected rigid body; sample rejected')
                return
            pose = PoseStamped()
            pose.header = msg.header
            t = msg.transform.translation
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = t.x, t.y, t.z
            pose.pose.orientation = msg.transform.rotation
            msg = pose
        self.receive(vehicle, msg)

    def receive(self, vehicle, msg):
        i = vehicle['drone_id']
        cfg = self.configs[i]
        now = self.get_clock().now().nanoseconds
        if msg.header.stamp.sec < 0 or not 0 <= msg.header.stamp.nanosec < 1_000_000_000:
            self.warn(i, 'invalid ROS timestamp fields; sample rejected')
            return
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if msg.header.frame_id != vehicle['tracking_frame']:
            self.warn(i, 'unexpected tracking frame; sample rejected')
            return
        if not valid_sample(stamp, now, self.last_sample.get(i, 0), cfg):
            self.warn(i, 'stale, future, zero, or out-of-order timestamp; sample rejected')
            return
        p, q = msg.pose.position, msg.pose.orientation
        try:
            position, orientation = transform((p.x, p.y, p.z), (q.w, q.x, q.y, q.z), cfg)
            # ROS Python accepts doubles; PX4 arrays are float32. Reject overflow
            # before assignment/serialization so one bad tracker cannot kill routing.
            values = (*position, *orientation, *cfg['position_variance'], *cfg['orientation_variance'])
            if any(not math.isfinite(x) or abs(x) > 3.4028234663852886e38 for x in values):
                raise ValueError('transformed pose or variance exceeds finite float32 range')
        except ValueError as exc:
            self.warn(i, f'invalid pose: {exc}')
            return
        out = VehicleOdometry()
        # XRCE-DDS time synchronization translates ROS epoch times to PX4 time.
        # Hardware tracker timestamps must already share this ROS clock domain.
        out.timestamp = self.get_clock().now().nanoseconds // 1000
        out.timestamp_sample = stamp // 1000
        out.pose_frame = cfg['pose_frame']
        out.position = list(position)
        out.q = list(orientation)
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        out.velocity = [math.nan] * 3
        out.angular_velocity = [math.nan] * 3
        out.velocity_variance = [math.nan] * 3
        out.position_variance = list(cfg['position_variance'])
        out.orientation_variance = list(cfg['orientation_variance'])
        out.reset_counter = 0  # Static calibration; restart disarmed after frame/clock resets.
        out.quality = 0  # This message version documents quality as unused.
        self.publishers_by_id[i].publish(out)
        self.last_sample[i] = stamp
        self.last_received[i] = time.monotonic()
        self.sample_counts[i] = self.sample_counts.get(i, 0) + 1

    def check_tracking(self):
        for v in self.vehicles:
            i = v['drone_id']
            if time.monotonic() - self.last_received.get(i, -math.inf) > self.configs[i]['max_age_s']:
                self.warn(i, 'no fresh valid tracking; odometry publication stopped')

    def check_rate(self):
        now = time.monotonic()
        interval = now - self.rate_checked_at
        for vehicle in self.vehicles:
            i = vehicle['drone_id']
            rate = self.sample_counts.get(i, 0) / interval
            if rate < self.minimum_rate_hz:
                self.warn(i, f'valid tracking rate {rate:.1f} Hz below {self.minimum_rate_hz:.1f} Hz')
            else:
                self.get_logger().debug(f'drone {i}: valid tracking rate {rate:.1f} Hz')
        self.sample_counts.clear()
        self.rate_checked_at = now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ExternalOdometry()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
