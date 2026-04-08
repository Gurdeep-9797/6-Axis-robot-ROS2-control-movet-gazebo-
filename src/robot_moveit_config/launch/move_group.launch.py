"""
MoveIt Move Group Launch File — Minimal working config
"""
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_description_pkg = FindPackageShare('robot_description')
    moveit_config_pkg = FindPackageShare('robot_moveit_config')
    robot_description_xacro_path = PathJoinSubstitution([
        robot_description_pkg, 'urdf', 'custom_6axis_test.urdf.xacro'
    ])
    # SRDF — read file contents via xacro (or cat if not xacro)
    # MoveIt expects robot_description_semantic to be the XML content, not a path
    srdf_path = PathJoinSubstitution([
        moveit_config_pkg, 'config', 'robot_arm.srdf'
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', robot_description_xacro_path]),
            'use_sim_time': use_sim_time,
        }]
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': Command(['xacro ', robot_description_xacro_path])},
            {'robot_description_semantic': Command(['cat ', srdf_path])},
            {'robot_description_kinematics': {
                'robot_arm': {
                    'kinematics_solver': 'trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin',
                    'kinematics_solver_search_resolution': 0.005,
                    'kinematics_solver_timeout': 1.0,
                    'kinematics_solver_attempts': 5,
                    'solve_type': 'Speed',
                }
            }},
            {'planning_plugin': 'ompl_interface/OMPLPlanner'},
            {'request_adapters': ''},
            {'start_state_max_bounds_error': 0.1},
            {'moveit_manage_controllers': True},
            {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'},
            {'moveit_simple_controller_manager': {
                'robot_arm_controller': {
                    'type': 'FollowJointTrajectory',
                    'action_ns': 'follow_joint_trajectory',
                    'default': True,
                    'joints': ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
                }
            }},
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        robot_state_publisher,
        move_group_node,
    ])
