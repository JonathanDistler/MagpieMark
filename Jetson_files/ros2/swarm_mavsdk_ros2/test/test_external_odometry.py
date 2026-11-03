import json
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry
from swarm_mavsdk_ros2.vehicle_config import default_vehicles, load_vehicles
from swarm_mavsdk_ros2.odometry_conversion import settings, transform, rotate, valid_sample
from swarm_mavsdk_ros2.external_odometry import ExternalOdometry


def test_default_identity_mapping():
    vehicles = load_vehicles('', 3)
    assert [v['system_id'] for v in vehicles] == [1, 2, 3]
    assert [v['dds_key'] for v in vehicles] == [1, 2, 3]
    assert [v['px4_namespace'] for v in vehicles] == ['', '/px4_1', '/px4_2']


@pytest.mark.parametrize('key,value', [('system_id', 0), ('dds_key', 0),
    ('dds_key', 2**32), ('system_id', True), ('px4_namespace', '/bad//name'),
    ('tracking_topic', 'relative'), ('tracking_frame', '')])
def test_reject_invalid_vehicle_identity(tmp_path, key, value):
    vehicles = default_vehicles(1)
    vehicles[0][key] = value
    path = tmp_path / 'vehicles.json'
    path.write_text(json.dumps({'vehicles': vehicles}))
    with pytest.raises(ValueError):
        load_vehicles(path, 1)


@pytest.mark.parametrize('key', ['system_id', 'dds_key', 'px4_namespace', 'tracking_topic', 'mocap_body'])
def test_reject_duplicate_routes(tmp_path, key):
    vehicles = default_vehicles(2)
    vehicles[0][key] = vehicles[1][key] = 'same' if key == 'mocap_body' else vehicles[0][key]
    path = tmp_path / 'vehicles.json'
    path.write_text(json.dumps({'vehicles': vehicles}))
    with pytest.raises(ValueError, match='unique'):
        load_vehicles(path, 2)


def test_enu_flu_to_ned_frd_axes_and_heading():
    p, q = transform((1, 2, 3), (1, 0, 0, 0), settings())
    assert p == pytest.approx((2, 1, -3))
    assert rotate(q, (1, 0, 0)) == pytest.approx((0, 1, 0))
    assert rotate(q, (0, 1, 0)) == pytest.approx((-1, 0, 0))
    assert rotate(q, (0, 0, 1)) == pytest.approx((0, 0, 1))


def test_marker_offset_rotates_with_body_before_subtraction():
    cfg = settings(dict(world_rotation=[1,0,0,0], body_rotation=[1,0,0,0],
                        marker_offset_body=[1,0,0], world_translation=[10,0,0]))
    p, _ = transform((0,1,0), (math.sqrt(.5),0,0,math.sqrt(.5)), cfg)
    assert p == pytest.approx((10,0,0))


@pytest.mark.parametrize('overrides', [dict(body_rotation=[0,0,0,0]),
    dict(world_translation=[math.nan,0,0]), dict(pose_frame=0),
    dict(position_variance=[0,1,1]), dict(max_age_s=-1), dict(future_tolerance_s=-1)])
def test_invalid_transform_configuration(overrides):
    with pytest.raises(ValueError):
        settings(overrides)


@pytest.mark.parametrize('variance', [1e-100, 1e39])
def test_variance_cannot_serialize_as_zero_or_infinity(variance):
    with pytest.raises(ValueError, match='float32'):
        settings({'position_variance': [variance]*3})


@pytest.mark.parametrize('stamp,previous,accepted', [
    (0,0,False), (999_000_000,0,True), (700_000_000,0,False),
    (1_100_000_000,0,False), (999_000_000,999_000_000,False),
    (998_000_000,999_000_000,False)])
def test_sample_timestamp_contract(stamp, previous, accepted):
    assert valid_sample(stamp, 1_000_000_000, previous, settings()) == accepted


def adapter():
    node = object.__new__(ExternalOdometry)
    node.configs = {0: settings(), 1: settings()}
    node.last_sample, node.last_received, node.sample_counts = {}, {}, {}
    node.publishers_by_id = {0: Mock(), 1: Mock()}
    node.get_clock = Mock(return_value=SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)))
    node.warn = Mock()
    return node


def pose():
    msg = PoseStamped()
    msg.header.frame_id = 'lab'
    msg.header.stamp.nanosec = 999_000_000
    msg.pose.orientation.w = 1.
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = 1., 2., 3.
    return msg


def test_odometry_has_nan_unknown_velocity_and_isolated_destination():
    node = adapter()
    node.receive(default_vehicles(2)[1], pose())
    node.publishers_by_id[0].publish.assert_not_called()
    out = node.publishers_by_id[1].publish.call_args.args[0]
    assert out.timestamp == 1_000_000
    assert out.timestamp_sample == 999_000
    assert out.position == pytest.approx([2,1,-3])
    assert out.velocity_frame == VehicleOdometry.VELOCITY_FRAME_UNKNOWN
    assert all(math.isnan(x) for x in [*out.velocity, *out.angular_velocity, *out.velocity_variance])
    assert out.position_variance == pytest.approx([.0025]*3)


@pytest.mark.parametrize('fault', ['frame', 'stale', 'quaternion', 'position', 'duplicate'])
def test_adapter_rejects_bad_tracking_without_republishing(fault):
    node, msg = adapter(), pose()
    if fault == 'frame':
        msg.header.frame_id = 'wrong'
    elif fault == 'stale':
        msg.header.stamp.nanosec = 500_000_000
    elif fault == 'quaternion':
        msg.pose.orientation.w = 0.
    elif fault == 'position':
        msg.pose.position.x = math.nan
    else:
        node.last_sample[0] = 999_000_000
    node.receive(default_vehicles(1)[0], msg)
    node.publishers_by_id[0].publish.assert_not_called()
    node.warn.assert_called_once()


def test_transform_stamped_preserves_acquisition_timestamp():
    from geometry_msgs.msg import TransformStamped
    node = adapter()
    msg = TransformStamped()
    msg.header = pose().header
    msg.transform.rotation.w = 1.
    msg.transform.translation.x = 1.
    vehicle = {**default_vehicles(1)[0], 'tracking_message': 'TransformStamped'}
    node.receive_tracking(vehicle, msg)
    out = node.publishers_by_id[0].publish.call_args.args[0]
    assert out.timestamp_sample == 999_000
    assert out.position == pytest.approx([0,1,0])


def test_live_ros_routes_two_trackers_to_separate_custom_px4_topics(tmp_path):
    import time
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    vehicles = default_vehicles(2)
    for i, v in enumerate(vehicles):
        v['px4_namespace'] = f'/test_bench_{i+1}'
    config = tmp_path / 'vehicles.json'
    config.write_text(json.dumps({'vehicles': vehicles}))
    rclpy.init()
    bridge = observer = executor = None
    try:
        bridge = ExternalOdometry(parameter_overrides=[Parameter('num_drones', value=2),
            Parameter('vehicle_config', value=str(config))])
        observer = Node('tracking_test_observer')
        executor = SingleThreadedExecutor()
        executor.add_node(bridge)
        executor.add_node(observer)
        received = {0: [], 1: []}
        sources = {}
        for i, v in enumerate(vehicles):
            observer.create_subscription(VehicleOdometry, v['px4_namespace']+'/fmu/in/vehicle_visual_odometry',
                                         lambda msg, i=i: received[i].append(msg), 10)
            sources[i] = observer.create_publisher(PoseStamped, v['tracking_topic'], 10)
        deadline = time.monotonic()+5
        while time.monotonic() < deadline and not all(p.get_subscription_count() for p in sources.values()):
            executor.spin_once(timeout_sec=.05)
        assert all(p.get_subscription_count() for p in sources.values())
        # Wait for output discovery as well, so the first volatile sample is observable.
        while time.monotonic() < deadline and not all(p.get_subscription_count() for p in bridge.publishers_by_id.values()):
            executor.spin_once(timeout_sec=.05)
        for i in range(2):
            msg = pose()
            msg.header.stamp = observer.get_clock().now().to_msg()
            msg.pose.position.x = float(10+i)
            sources[i].publish(msg)
        while time.monotonic() < deadline and not all(received.values()):
            executor.spin_once(timeout_sec=.05)
        assert all(received.values())
        assert received[0][-1].position == pytest.approx([2,10,-3])
        assert received[1][-1].position == pytest.approx([2,11,-3])
        counts = [len(received[i]) for i in range(2)]
        end = time.monotonic()+.3
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.05)
        assert [len(received[i]) for i in range(2)] == counts  # no cached-pose replay
    finally:
        if executor:
            executor.shutdown()
        for node in (observer, bridge):
            if node:
                node.destroy_node()
        rclpy.shutdown()


def test_bench_mapping_keeps_simulation_defaults_and_uses_requested_namespaces():
    from pathlib import Path
    path = Path(__file__).resolve().parents[3] / 'config/bench.example.json'
    vehicles = load_vehicles(path, 2)
    assert [v['px4_namespace'] for v in vehicles] == ['/uav_0', '/uav_1']
    assert [v['system_id'] for v in vehicles] == [1, 2]
    assert [v['dds_key'] for v in vehicles] == [101, 102]
    assert default_vehicles(2)[0]['px4_namespace'] == ''


@pytest.mark.parametrize('source,expected', [((1,0,0),(0,1,0)),
    ((0,1,0),(1,0,0)), ((0,0,1),(0,0,-1))])
def test_independent_world_axis_displacements(source, expected):
    origin, _ = transform((0,0,0), (1,0,0,0), settings())
    p, _ = transform(source, (1,0,0,0), settings())
    assert [p[i]-origin[i] for i in range(3)] == pytest.approx(expected)


@pytest.mark.parametrize('angle', [0, math.pi/2, math.pi, -math.pi/2])
def test_ros_yaw_to_px4_yaw(angle):
    _, q = transform((0,0,0), (math.cos(angle/2),0,0,math.sin(angle/2)), settings())
    expected = math.pi/2-angle
    assert rotate(q, (1,0,0)) == pytest.approx((math.cos(expected),math.sin(expected),0))
    assert sum(x*x for x in q) == pytest.approx(1.)


def test_transform_body_identity_mismatch_rejected():
    from geometry_msgs.msg import TransformStamped
    node = adapter()
    msg = TransformStamped()
    msg.header = pose().header
    msg.child_frame_id = 'other_drone'
    msg.transform.rotation.w = 1.
    vehicle = {**default_vehicles(1)[0], 'tracking_message': 'TransformStamped', 'mocap_body': 'bench_drone_1'}
    node.receive_tracking(vehicle, msg)
    node.publishers_by_id[0].publish.assert_not_called()
    msg.child_frame_id = 'bench_drone_1'
    node.receive_tracking(vehicle, msg)
    node.publishers_by_id[0].publish.assert_called_once()


def test_static_pose_is_real_measurement_with_complete_fields():
    node = adapter()
    msg = pose()
    node.receive(default_vehicles(1)[0], msg)
    msg.header.stamp.nanosec += 1000
    node.receive(default_vehicles(1)[0], msg)
    first, second = [call.args[0] for call in node.publishers_by_id[0].publish.call_args_list]
    assert list(first.position) == list(second.position)
    assert list(first.q) == list(second.q)
    assert second.timestamp_sample == first.timestamp_sample+1
    assert second.pose_frame == VehicleOdometry.POSE_FRAME_NED
    assert second.reset_counter == 0 and second.quality == 0
    assert second.orientation_variance == pytest.approx([.0025]*3)
    assert len(second.q) == 4
    for name in ('position','velocity','angular_velocity','position_variance','orientation_variance','velocity_variance'):
        assert len(getattr(second, name)) == 3


def test_tracking_dropout_and_rate_monitor_never_publish():
    import time
    node = adapter()
    node.vehicles = default_vehicles(2)
    node.minimum_rate_hz = 20.
    node.rate_checked_at = time.monotonic()-5
    node.check_tracking()
    node.check_rate()
    assert node.warn.call_count == 4
    for publisher in node.publishers_by_id.values():
        publisher.publish.assert_not_called()


def test_backward_clock_jump_fails_closed():
    node = adapter()
    node.last_sample[0] = 2_000_000_000
    node.receive(default_vehicles(1)[0], pose())
    node.publishers_by_id[0].publish.assert_not_called()


def test_unnormalizable_quaternion_and_malformed_timestamp_rejected():
    node = adapter()
    msg = pose()
    msg.pose.orientation.w = msg.pose.orientation.x = 1.79e308
    node.receive(default_vehicles(1)[0], msg)
    node.publishers_by_id[0].publish.assert_not_called()
    msg = pose()
    msg.header.stamp.nanosec = 1_500_000_000
    node.receive(default_vehicles(1)[0], msg)
    node.publishers_by_id[0].publish.assert_not_called()


def test_vehicle_command_targets_configured_system_without_ros_transport():
    from swarm_mavsdk_ros2.drone_node import DroneNode
    from px4_msgs.msg import VehicleCommand
    for system_id in (21, 42):
        node = SimpleNamespace(system_id=system_id, vehicle_command_pub=Mock(),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)))
        DroneNode.publish_vehicle_command(node, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        out = node.vehicle_command_pub.publish.call_args.args[0]
        assert out.target_system == out.source_system == system_id
        assert out.target_component == out.source_component == 1
        assert out.from_external
        assert out.timestamp == 1_000_000
        assert (out.param1, out.param2) == (1., 6.)


@pytest.mark.parametrize('fault', ['position', 'translation', 'variance'])
def test_float32_overflow_rejected_without_stopping_other_vehicle(fault):
    node, msg = adapter(), pose()
    if fault == 'position':
        msg.pose.position.x = 1e39
    elif fault == 'translation':
        node.configs[0]['world_translation'] = (1e39, 0., 0.)
    else:
        node.configs[0]['position_variance'] = (1e39, 1., 1.)
    node.receive(default_vehicles(2)[0], msg)
    node.publishers_by_id[0].publish.assert_not_called()
    assert 0 not in node.last_sample
    node.receive(default_vehicles(2)[1], pose())
    node.publishers_by_id[1].publish.assert_called_once()
