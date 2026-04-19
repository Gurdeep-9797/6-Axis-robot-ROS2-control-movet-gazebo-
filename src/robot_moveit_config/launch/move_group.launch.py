"""
MoveIt Move Group Launch File — With OMPL + PILZ Industrial Motion Planner
"""
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import yaml

def load_yaml(package_name, file_path):
    """Load a yaml file from a ROS 2 package."""
    package_share = FindPackageShare(package_name)
    # We can't resolve at build time, so we use a workaround
    return None  # Parameters are loaded directly below

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_description_pkg = FindPackageShare('robot_description')
    moveit_config_pkg = FindPackageShare('robot_moveit_config')
    robot_description_xacro_path = PathJoinSubstitution([
        robot_description_pkg, 'urdf', 'custom_6axis_test.urdf.xacro'
    ])
    # SRDF — read file contents via cat
    srdf_path = PathJoinSubstitution([
        moveit_config_pkg, 'config', 'robot_arm.srdf'
    ])

    # Joint limits for PILZ planner
    joint_limits_path = PathJoinSubstitution([
        FindPackageShare('robot_description'), 'config', 'joint_limits.yaml'
    ])

    # PILZ cartesian limits
    pilz_cartesian_limits_path = PathJoinSubstitution([
        moveit_config_pkg, 'config', 'pilz_cartesian_limits.yaml'
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
            # Kinematics — TRAC-IK
            {'robot_description_kinematics': {
                'robot_arm': {
                    'kinematics_solver': 'trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin',
                    'kinematics_solver_search_resolution': 0.005,
                    'kinematics_solver_timeout': 1.0,
                    'kinematics_solver_attempts': 5,
                    'solve_type': 'Speed',
                }
            }},
            # Planning pipelines — OMPL + PILZ
            {'planning_pipelines': ['ompl', 'pilz']},
            {'default_planning_pipeline': 'ompl'},
            # OMPL config
            {'planning_plugin': 'ompl_interface/OMPLPlanner'},
            {'request_adapters': ''},
            {'start_state_max_bounds_error': 0.1},
            # PILZ config — loaded separately
            {'robot_description_planning': {
                'cartesian_limits': {
                    'max_trans_vel': 1.0,
                    'max_trans_acc': 2.5,
                    'max_trans_dec': 2.5,
                    'max_rot_vel': 1.57,
                },
                'joint_limits': {
                    'joint_1': {'has_velocity_limits': True, 'max_velocity': 4.363, 'has_acceleration_limits': True, 'max_acceleration': 10.0},
                    'joint_2': {'has_velocity_limits': True, 'max_velocity': 4.363, 'has_acceleration_limits': True, 'max_acceleration': 10.0},
                    'joint_3': {'has_velocity_limits': True, 'max_velocity': 4.363, 'has_acceleration_limits': True, 'max_acceleration': 10.0},
                    'joint_4': {'has_velocity_limits': True, 'max_velocity': 5.585, 'has_acceleration_limits': True, 'max_acceleration': 20.0},
                    'joint_5': {'has_velocity_limits': True, 'max_velocity': 5.585, 'has_acceleration_limits': True, 'max_acceleration': 20.0},
                    'joint_6': {'has_velocity_limits': True, 'max_velocity': 8.028, 'has_acceleration_limits': True, 'max_acceleration': 20.0},
                }
            }},
            # Controller manager
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
            # Trajectory execution
            {'trajectory_execution': {
                'allowed_execution_duration_scaling': 1.2,
                'allowed_goal_duration_margin': 0.5,
                'allowed_start_tolerance': 0.01,
                'execution_duration_monitoring': True,
            }},
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        robot_state_publisher,
        move_group_node,
    ])
