"""
Gazebo Launch File

AUTHORITY: EMULATED (No Real Authority)

This launch file starts Gazebo simulation which:
- Provides physics emulation (NON-RT, NON-CERTIFIED)
- Spawns the robot model
- Runs simulated controllers

WARNING: SIM mode does not constitute safety validation.
All safety testing must be with real hardware in controlled conditions.
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    headless = LaunchConfiguration('headless', default='false')
    world = LaunchConfiguration('world', default='default.world')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Package paths
    robot_description_pkg = FindPackageShare('robot_description')
    robot_gazebo_pkg = FindPackageShare('robot_gazebo')
    gazebo_ros_pkg = FindPackageShare('gazebo_ros')
    
    # Robot description
    robot_description_path = PathJoinSubstitution([
        robot_description_pkg, 'urdf', 'robot.urdf.xacro'
    ])
    
    robot_description = Command(['xacro ', robot_description_path])
    
    # World file
    world_path = PathJoinSubstitution([
        robot_gazebo_pkg, 'worlds', world
    ])
    
    # Controllers config
    controllers_config = PathJoinSubstitution([
        robot_gazebo_pkg, 'config', 'gazebo_controllers.yaml'
    ])
    
    # Gazebo server (with GUI)
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([gazebo_ros_pkg, 'launch', 'gzserver.launch.py'])
        ]),
        launch_arguments={
            'world': world_path,
            'pause': 'false',
        }.items(),
    )
    
    # Gazebo client (GUI) - only if not headless
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([gazebo_ros_pkg, 'launch', 'gzclient.launch.py'])
        ]),
        condition=UnlessCondition(headless),
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot_arm',
            '-x', '0', '-y', '0', '-z', '0',
        ],
        output='screen',
    )
    
    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )
    
    # Robot arm controller
    robot_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robot_arm_controller', '-c', '/controller_manager'],
        output='screen',
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Gazebo headless (no GUI)'),
        DeclareLaunchArgument('world', default_value='default.world',
                              description='World file to load'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation time'),
        
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster_spawner,
        robot_arm_controller_spawner,
    ])
