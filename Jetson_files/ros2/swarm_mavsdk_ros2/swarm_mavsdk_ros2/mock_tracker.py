"""Synthetic standard-message tracker. No PX4 commands or motor operations."""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from .vehicle_config import load_vehicles


class MockTracker(Node):
    def __init__(self, **kwargs):
        super().__init__('mock_tracker', **kwargs)
        for name, value in [('num_drones', 2), ('vehicle_config', ''),
                            ('rate_hz', 50.), ('axis', 'stationary'),
                            ('amplitude', .5), ('period_s', 8.),
                            ('capture_latency_s', 0.), ('dropout_after_s', 0.)]:
            self.declare_parameter(name, value)
        self.value = lambda name: self.get_parameter(name).value
        self.vehicles = load_vehicles(self.value('vehicle_config'), self.value('num_drones'))
        self.axis = self.value('axis')
        if self.axis not in ('stationary', 'x', 'y', 'z', 'yaw'):
            raise ValueError('axis must be stationary, x, y, z, or yaw')
        for name in ('rate_hz', 'period_s', 'amplitude', 'capture_latency_s', 'dropout_after_s'):
            val = self.value(name)
            if not math.isfinite(val) or val < 0 or (name in ('rate_hz', 'period_s') and val == 0):
                raise ValueError(f'Invalid {name}')
        self.sources = {}
        for v in self.vehicles:
            cls = PoseStamped if v['tracking_message'] == 'PoseStamped' else TransformStamped
            self.sources[v['drone_id']] = self.create_publisher(cls, v['tracking_topic'], 10)
        self.start = time.monotonic()
        self.create_timer(1. / self.value('rate_hz'), self.publish_sample)
        self.get_logger().info('SYNTHETIC tracker running; use a separate ROS domain from physical aircraft')

    def publish_sample(self):
        elapsed = time.monotonic() - self.start
        if self.value('dropout_after_s') > 0 and elapsed >= self.value('dropout_after_s'):
            return
        delta = self.value('amplitude') * math.sin(2 * math.pi * elapsed / self.value('period_s'))
        capture_ns = self.get_clock().now().nanoseconds - int(self.value('capture_latency_s') * 1e9)
        if capture_ns <= 0:
            return
        for v in self.vehicles:
            p = [float(v['drone_id']) * 2., 0., 1.]
            yaw = delta if self.axis == 'yaw' else 0.
            if self.axis in ('x', 'y', 'z'):
                p['xyz'.index(self.axis)] += delta
            q = (0., 0., math.sin(yaw/2), math.cos(yaw/2))
            if v['tracking_message'] == 'PoseStamped':
                msg = PoseStamped()
                position, orientation = msg.pose.position, msg.pose.orientation
            else:
                msg = TransformStamped()
                msg.child_frame_id = v['mocap_body']
                position, orientation = msg.transform.translation, msg.transform.rotation
            msg.header.frame_id = v['tracking_frame']
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(capture_ns, 1_000_000_000)
            position.x, position.y, position.z = p
            orientation.x, orientation.y, orientation.z, orientation.w = q
            self.sources[v['drone_id']].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MockTracker()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
