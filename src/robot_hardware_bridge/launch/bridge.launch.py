"""
Hardware Bridge Launch File

AUTHORITY: RELAY ONLY - NO CONTROL AUTHORITY

This launch file starts the Hardware Bridge node which:
- Relays trajectories to the Controller (SIM or REAL)
- Publishes joint states from Controller to ROS
- Performs SCHEMA validation only

Mode is selected via ROBOT_MODE environment variable.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_mode',
            default_value=EnvironmentVariable('ROBOT_MODE', default_value='SIM'),
            description='Robot mode: SIM or REAL'
        ),
        DeclareLaunchArgument(
            'controller_type',
            default_value=EnvironmentVariable('CONTROLLER_TYPE', default_value='FAKE'),
            description='Controller type: FAKE or REAL'
        ),
        DeclareLaunchArgument(
            'controller_ip',
            default_value=EnvironmentVariable('CONTROLLER_IP', default_value='192.168.1.100'),
            description='Controller IP address'
        ),
        DeclareLaunchArgument(
            'controller_port',
            default_value=EnvironmentVariable('CONTROLLER_PORT', default_value='5000'),
            description='Controller port'
        ),
        DeclareLaunchArgument(
            'controller_proto',
            default_value=EnvironmentVariable('CONTROLLER_PROTO', default_value='TCP'),
            description='Controller protocol: TCP or UDP'
        ),
        
        Node(
            package='robot_hardware_bridge',
            executable='bridge_node',
            name='hardware_bridge',
            output='screen',
            parameters=[{
                'robot_mode': LaunchConfiguration('robot_mode'),
                'controller_type': LaunchConfiguration('controller_type'),
                'controller_ip': LaunchConfiguration('controller_ip'),
                'controller_port': LaunchConfiguration('controller_port'),
                'controller_proto': LaunchConfiguration('controller_proto'),
            }]
        ),
    ])
