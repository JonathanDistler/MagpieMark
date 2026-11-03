from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = {'num_drones': ParameterValue(LaunchConfiguration('num_drones'), value_type=int),
              'vehicle_config': LaunchConfiguration('vehicle_config')}
    return LaunchDescription([
        DeclareLaunchArgument('num_drones', default_value='1'),
        DeclareLaunchArgument('vehicle_config', default_value=''),
        DeclareLaunchArgument('gazebo', default_value='false'),
        DeclareLaunchArgument('world_name', default_value='blank'),
        Node(package='swarm_mavsdk_ros2', executable='external_odometry',
             parameters=[params], output='screen', on_exit=[Shutdown(reason='Odometry adapter exited')]),
        Node(package='swarm_mavsdk_ros2', executable='gazebo_tracker',
             parameters=[params, {'world_name': LaunchConfiguration('world_name')}],
             condition=IfCondition(LaunchConfiguration('gazebo')), output='screen',
             on_exit=[Shutdown(reason='Gazebo tracker exited')]),
    ])
