"""
MoveIt Move Group Launch File
Launches the MoveIt planning pipeline (NON-REAL-TIME, REQUEST ONLY)

Authority: REQUEST ONLY - generates trajectories, does not control motors
The Hardware Bridge relays trajectories to the RT Controller for execution
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Package paths
    robot_description_pkg = FindPackageShare('robot_description')
    moveit_config_pkg = FindPackageShare('robot_moveit_config')
    
    # Robot description
    robot_description_path = PathJoinSubstitution([
        robot_description_pkg, 'urdf', 'robot.urdf.xacro'
    ])
    
    # MoveIt configs
    kinematics_yaml = PathJoinSubstitution([
        moveit_config_pkg, 'config', 'kinematics.yaml'
    ])
    ompl_yaml = PathJoinSubstitution([
        moveit_config_pkg, 'config', 'ompl_planning.yaml'
    ])
    controllers_yaml = PathJoinSubstitution([
        moveit_config_pkg, 'config', 'controllers.yaml'
    ])
    trajectory_execution_yaml = PathJoinSubstitution([
        moveit_config_pkg, 'config', 'trajectory_execution.yaml'
    ])
    joint_limits_yaml = PathJoinSubstitution([
        robot_description_pkg, 'config', 'joint_limits.yaml'
    ])
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_path,
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Move Group Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description_path},
            kinematics_yaml,
            ompl_yaml,
            controllers_yaml,
            trajectory_execution_yaml,
            joint_limits_yaml,
            {'moveit_manage_controllers': True},
            {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'},
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        robot_state_publisher,
        move_group_node,
    ])
