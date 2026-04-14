"""
Gazebo Harmonic Launch File - FIXED

AUTHORITY: EMULATED (No Real Authority)

This launch file starts Gazebo Harmonic simulation which:
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

def generate_launch_description():
    # Declare arguments
    headless = LaunchConfiguration('headless', default='false')
    world = LaunchConfiguration('world', default='empty.sdf')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Package paths
    robot_description_pkg = FindPackageShare('robot_description')
    robot_gazebo_pkg = FindPackageShare('robot_gazebo')
    ros_gz_sim_pkg = FindPackageShare('ros_gz_sim')
    
    # Robot description
    robot_description_path = PathJoinSubstitution([
        robot_description_pkg, 'urdf', 'custom_6axis_test.urdf.xacro'
    ])
    
    robot_description = Command(['xacro ', robot_description_path], on_stderr='warn')
    
    # World file
    world_path = world
    
    # Gazebo Harmonic (gz_sim) - Headless
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': ['-r -s -v4 ', world_path]}.items(),
        condition=IfCondition(headless)
    )

    # Gazebo Harmonic (gz_sim) - GUI
    gazebo_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': ['-r -v4 ', world_path]}.items(),
        condition=UnlessCondition(headless)
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
    
    # Generate SDF and spawn robot in Gazebo Harmonic
    generate_sdf = ExecuteProcess(
        cmd=['bash', '-c', ['xacro ', robot_description_path, ' > /tmp/robot.urdf && gz sdf -p /tmp/robot.urdf > /tmp/robot.sdf']],
        output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot_arm',
            '-file', '/tmp/robot.sdf',
            '-x', '0', '-y', '0', '-z', '0',
        ],
        output='screen',
    )
    
    # Bridge for ROS 2 <-> Gazebo Harmonic
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/gazebo/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/gazebo/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory',
        ],
        remappings=[
            ('/gazebo/joint_states', '/joint_states'),
            ('/gazebo/joint_trajectory', '/robot_arm_controller/joint_trajectory'),
        ],
        output='screen'
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '-c', '/controller_manager', '--controller-manager-timeout', '60'],
                output='screen',
            )
        ]
    )
    
    # Robot arm controller
    robot_arm_controller_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['robot_arm_controller', '-c', '/controller_manager', '--controller-manager-timeout', '60'],
                output='screen',
            )
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo headless'),
        DeclareLaunchArgument('world', default_value='empty.sdf', description='World file to load'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        
        gazebo_sim,
        gazebo_sim_gui,
        bridge,
        robot_state_publisher,
        generate_sdf,
        TimerAction(period=2.0, actions=[spawn_robot]),
        joint_state_broadcaster_spawner,
        robot_arm_controller_spawner,
    ])
