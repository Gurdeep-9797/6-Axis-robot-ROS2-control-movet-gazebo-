"""
Analysis Launch File

Starts optional analysis tools.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_accuracy_analysis',
            default_value='false',
            description='Enable accuracy logging'
        ),
        
        Node(
            package='robot_analysis',
            executable='accuracy_logger',
            name='accuracy_logger',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_accuracy_analysis')),
            parameters=[{'log_dir': '/ros_ws/logs/analysis'}]
        ),
    ])
