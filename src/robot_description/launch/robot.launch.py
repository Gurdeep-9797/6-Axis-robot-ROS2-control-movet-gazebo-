from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # Arguments
    mode = LaunchConfiguration('mode').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Package Paths
    robot_desc_pkg = FindPackageShare('robot_description')
    robot_gazebo_pkg = FindPackageShare('robot_gazebo')
    moveit_config_pkg = FindPackageShare('robot_moveit_config')
    hardware_pkg = FindPackageShare('robot_hardware_bridge')
    analysis_pkg = FindPackageShare('robot_analysis')
    
    nodes_to_start = []
    
    # ---------------------------------------------------------
    # 1. Common: Robot State Publisher (URDF)
    # ---------------------------------------------------------
    # Always needed for TF, Rviz, and MoveIt
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([robot_desc_pkg, 'launch', 'display.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    nodes_to_start.append(rsp_launch)

    # ---------------------------------------------------------
    # 2. Backend: SIMULATION (Gazebo)
    # ---------------------------------------------------------
    if mode == 'sim':
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([robot_gazebo_pkg, 'launch', 'gazebo.launch.py'])
            )
        )
        nodes_to_start.append(gazebo_launch)
        
    # ---------------------------------------------------------
    # 3. Backend: REAL HARDWARE (ESP32 Bridge)
    # ---------------------------------------------------------
    elif mode == 'real':
        # Bridge Node: Gateway to ESP32
        bridge_node = Node(
            package='robot_hardware_bridge',
            executable='bridge_node',
            name='hardware_bridge',
            output='screen',
            parameters=[{'controller_ip': '192.168.1.100', 'mode': 'REAL'}]
        )
        nodes_to_start.append(bridge_node)
        
        # MoveIt Adapter: Action Server -> Trajectory Topic
        adapter_node = Node(
            package='robot_hardware_bridge',
            executable='moveit_adapter',
            name='moveit_adapter',
            output='screen'
        )
        nodes_to_start.append(adapter_node)

    # ---------------------------------------------------------
    # 4. Common: MoveIt (Motion Planning)
    # ---------------------------------------------------------
    # Wait for backend to be ready
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([moveit_config_pkg, 'launch', 'move_group.launch.py'])
        ),
        launch_arguments={
            'allow_trajectory_execution': 'true', 
            'fake_execution': 'false', 
            'info': 'true',
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Delay MoveIt to ensure Gazebo/Bridge is ready
    delay_sec = 10.0 if mode == 'sim' else 5.0
    nodes_to_start.append(TimerAction(period=delay_sec, actions=[move_group_launch]))

    # ---------------------------------------------------------
    # 5. Common: Analysis & Observability
    # ---------------------------------------------------------
    accuracy_node = Node(
        package='robot_analysis',
        executable='accuracy_node',
        name='accuracy_analysis',
        output='screen'
    )
    nodes_to_start.append(accuracy_node)

    return nodes_to_start

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', 
            default_value='sim', 
            description='Operation mode: "sim" (Gazebo) or "real" (ESP32)'
        ),
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='false', 
            description='Use simulation time if true'
        ),
        OpaqueFunction(function=launch_setup)
    ])
