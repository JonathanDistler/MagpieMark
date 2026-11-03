"""Simulation tracker input; no direct PX4 publishing or velocity synthesis."""
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from .vehicle_config import load_vehicles


class GazeboTracker(Node):
    def __init__(self):
        super().__init__('gazebo_tracker')
        from gz.transport13 import Node as GzNode
        from gz.msgs10.pose_v_pb2 import Pose_V
        self.declare_parameter('num_drones', 1)
        self.declare_parameter('vehicle_config', '')
        self.declare_parameter('world_name', 'blank')
        vehicles = load_vehicles(self.get_parameter('vehicle_config').value,
                                 self.get_parameter('num_drones').value)
        self.vehicles = {v['gazebo_model']: v for v in vehicles}
        self.outputs = {v['drone_id']: self.create_publisher(PoseStamped, v['tracking_topic'], 10)
                        for v in vehicles}
        self.pending = {}
        self.lock = threading.Lock()
        self.topic = '/world/' + self.get_parameter('world_name').value + '/pose/info'
        self.transport = GzNode()
        if not self.transport.subscribe(Pose_V, self.topic, self.receive):
            raise RuntimeError(f'Cannot subscribe to {self.topic}')
        self.create_timer(.02, self.publish_pending)

    def receive(self, poses):
        # GZ callbacks use another thread. Keep ROS publishing in its executor.
        # Timestamp at source receipt: Gazebo simulation time is not ROS epoch time.
        stamp = self.get_clock().now().to_msg()
        with self.lock:
            for pose in poses.pose:
                if pose.name not in self.vehicles:
                    continue
                v = self.vehicles[pose.name]
                msg = PoseStamped()
                msg.header.stamp = stamp
                msg.header.frame_id = v['tracking_frame']
                p, q = pose.position, pose.orientation
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = p.x, p.y, p.z
                msg.pose.orientation.w, msg.pose.orientation.x = q.w, q.x
                msg.pose.orientation.y, msg.pose.orientation.z = q.y, q.z
                self.pending[v['drone_id']] = (time.monotonic(), msg)

    def publish_pending(self):
        with self.lock:
            pending, self.pending = self.pending, {}
        for i, (received, msg) in pending.items():
            if time.monotonic() - received < .2:
                self.outputs[i].publish(msg)

    def destroy_node(self):
        self.transport.unsubscribe(self.topic)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GazeboTracker()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
