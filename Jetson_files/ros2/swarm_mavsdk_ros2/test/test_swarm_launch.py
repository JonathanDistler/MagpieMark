# Check swarm launch wiring without starting Gazebo or sending commands
import importlib.util
import json
from pathlib import Path
from unittest.mock import patch

import pytest
from launch import LaunchContext


path = Path(__file__).resolve().parents[1] / 'launch' / 'swarm.launch.py'
spec = importlib.util.spec_from_file_location('swarm_launch', path)
swarm_launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(swarm_launch)


def test_custom_vehicle_mapping_reaches_controller_without_changing_logical_ids(tmp_path):
    config = tmp_path / 'vehicles.json'
    config.write_text(json.dumps({'vehicles': [dict(drone_id=0, px4_namespace='/bench_1',
        system_id=21, dds_key=101), dict(drone_id=1, px4_namespace='/bench_2', system_id=22, dds_key=102)]}))
    context = LaunchContext()
    context.launch_configurations.update(num_drones='2', takeoff_altitude='2.0',
        world_name='', spawn_spacing='2.0', vehicle_config=str(config))
    with patch.object(swarm_launch, 'Node') as node:
        swarm_launch._start_drone_nodes(context)
    for i, invocation in enumerate(node.call_args_list[:2]):
        params = invocation.kwargs['parameters'][0]
        assert params['drone_id'] == i
        assert params['system_id'] == 21+i
        assert params['px4_namespace'] == f'/bench_{i+1}'

@pytest.mark.parametrize('count', [1, 3, 5, 12, 50, 250])
def test_each_member_has_unique_namespace_and_system_id(count):
    context = LaunchContext()
    context.launch_configurations.update(num_drones=str(count), takeoff_altitude='2.0', world_name='drone_obstacle_course', spawn_spacing='2.0')
    with patch.object(swarm_launch, 'Node') as node:
        nodes = swarm_launch._start_drone_nodes(context)
    assert len(nodes) == count + 1
    for i, invocation in enumerate(node.call_args_list[:count]):
        kwargs = invocation.kwargs
        assert kwargs['namespace'] == f'drone_{i}'
        params = kwargs['parameters'][0]
        assert params['drone_id'] == i
        assert params['system_id'] == i + 1
        assert params['swarm_size'] == count
        assert params['position_speed_mps'] == 1.0
        assert params['world_name'] == 'drone_obstacle_course'
        assert params['spawn_world_y'] == i * 2.0
        assert params['px4_namespace'] == ('' if i == 0 else f'/px4_{i}')


def test_position_reference_speed_is_forwarded_to_every_member():
    context = LaunchContext()
    context.launch_configurations.update(num_drones='3', takeoff_altitude='2.0',
        world_name='', spawn_spacing='2.0', position_speed_mps='0.2')
    with patch.object(swarm_launch, 'Node') as node:
        swarm_launch._start_drone_nodes(context)
    assert all(call.kwargs['parameters'][0]['position_speed_mps'] == .2
               for call in node.call_args_list[:3])


@pytest.mark.parametrize('count', [0, -1, 251])
def test_rejects_counts_outside_px4_system_id_range(count):
    context = LaunchContext()
    context.launch_configurations.update(num_drones=str(count), takeoff_altitude='2.0', world_name='drone_obstacle_course', spawn_spacing='2.0')
    with pytest.raises(ValueError, match='between 1 and 250'):
        swarm_launch._start_drone_nodes(context)


@pytest.mark.parametrize('leader', [0, 2])
def test_edge_leader_runs_without_central_controller(leader):
    context = LaunchContext()
    context.launch_configurations.update(
        num_drones='3', takeoff_altitude='2.0', world_name='', spawn_spacing='2.0',
        architecture=f'edge:drone_{leader}',
    )
    with patch.object(swarm_launch, 'Node') as node:
        nodes = swarm_launch._start_drone_nodes(context)
    assert len(nodes) == 3
    assert all(call.kwargs['executable'] == 'drone_node' for call in node.call_args_list)
    assert all(call.kwargs['parameters'][0]['architecture'] == f'edge:drone_{leader}'
               for call in node.call_args_list)
