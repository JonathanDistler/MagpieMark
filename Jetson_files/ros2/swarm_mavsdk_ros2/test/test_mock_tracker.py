"""Run the real synthetic source and bridge together, without PX4 or Gazebo."""
import json
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from px4_msgs.msg import VehicleOdometry
from swarm_mavsdk_ros2.external_odometry import ExternalOdometry
from swarm_mavsdk_ros2.mock_tracker import MockTracker
from swarm_mavsdk_ros2.vehicle_config import default_vehicles


@pytest.mark.parametrize('message_type,latency', [('PoseStamped', 0.), ('TransformStamped', 0.), ('PoseStamped', 1.)])
def test_mock_source_bridge_and_dropout(tmp_path, message_type, latency):
    vehicles = default_vehicles(2)
    for i, v in enumerate(vehicles):
        v.update(px4_namespace=f'/mock_uav_{i}', tracking_message=message_type, mocap_body=f'body_{i}')
    path = tmp_path / 'vehicles.json'
    path.write_text(json.dumps({'vehicles': vehicles}))
    params = [Parameter('num_drones', value=2), Parameter('vehicle_config', value=str(path))]
    rclpy.init()
    nodes = []
    executor = SingleThreadedExecutor()
    try:
        bridge = ExternalOdometry(parameter_overrides=params)
        source = MockTracker(parameter_overrides=params + [Parameter('capture_latency_s', value=latency)])
        observer = Node('mock_test_observer')
        nodes.extend([bridge, source, observer])
        received = {0: [], 1: []}
        for i, v in enumerate(vehicles):
            observer.create_subscription(VehicleOdometry, v['px4_namespace']+'/fmu/in/vehicle_visual_odometry',
                                         lambda msg, i=i: received[i].append(msg), 10)
        for node in nodes:
            executor.add_node(node)
        end = time.monotonic()+3
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.02)
            if all(received.values()):
                break
        if latency:
            assert not any(received.values())
        else:
            assert all(received.values())
            assert received[1][-1].position[1]-received[0][-1].position[1] == pytest.approx(2.)
            # Trigger the source's dropout and drain samples already in transit.
            source.set_parameters([Parameter('dropout_after_s', value=.001)])
            end = time.monotonic()+.2
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.02)
            counts = [len(received[i]) for i in range(2)]
            end = time.monotonic()+.3
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.02)
            assert [len(received[i]) for i in range(2)] == counts
    finally:
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()
