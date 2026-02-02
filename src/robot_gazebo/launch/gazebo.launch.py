"""
Gazebo Launch File - FIXED

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
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    # Declare arguments
    headless = LaunchConfiguration('headless', default='false')
    world = LaunchConfiguration('world', default='default.world')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Package paths
    robot_description_pkg = FindPackageShare('robot_description')
    robot_gazebo_pkg = FindPackageShare('robot_gazebo')
    gazebo_ros_pkg = FindPackageShare('gazebo_ros')
    
    # Robot description - FIXED: Use correct URDF file name
    robot_description_path = PathJoinSubstitution([
        robot_description_pkg, 'urdf', 'custom_6axis_test.urdf.xacro'
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
    
    # Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([gazebo_ros_pkg, 'launch', 'gzserver.launch.py'])
        ]),
        launch_arguments={
            'world': world_path,
            'pause': 'false',
            'verbose': 'true',
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
            'robot_description': ParameterValue(robot_description, value_type=str),
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
    
    # FIXED: Add delays for controller spawners to ensure robot and controller_manager are ready
    # Joint state broadcaster - delay 10 seconds after launch to ensure Gazebo is ready
    joint_state_broadcaster_spawner = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '-c', '/controller_manager',
                    '--controller-manager-timeout', '60',
                ],
                output='screen',
            )
        ]
    )
    
    # Robot arm controller - delay 15 seconds
    robot_arm_controller_spawner = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'robot_arm_controller',
                    '-c', '/controller_manager',
                    '--controller-manager-timeout', '60',
                ],
                output='screen',
            )
        ]
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
