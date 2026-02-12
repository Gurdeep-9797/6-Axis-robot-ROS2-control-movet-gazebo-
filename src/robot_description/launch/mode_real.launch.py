from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Paths
    robot_description_pkg = FindPackageShare('robot_description')
    moveit_config_pkg = FindPackageShare('robot_moveit_config')
    hardware_bridge_pkg = FindPackageShare('robot_hardware_bridge')
    robot_analysis_pkg = FindPackageShare('robot_analysis')
    
    # 1. State Publisher (URDF) - Source of Truth for TF
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([robot_description_pkg, 'launch', 'display.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # 2. Hardware Bridge (The Adapter to ESP32)
    # This node is the Gateway. It subscribes to /planned_trajectory and publishes to ESP32.
    # It also receives /joint_states from ESP32.
    bridge_node = Node(
        package='robot_hardware_bridge',
        executable='bridge_node',
        name='hardware_bridge',
        output='screen',
        parameters=[{'controller_ip': '192.168.1.100', 'mode': 'REAL'}]
    )

    # 3. MoveIt Adapter (The Action Server)
    # Converts MoveIt's FollowJointTrajectory Action -> /planned_trajectory Topic
    moveit_adapter = Node(
        package='robot_hardware_bridge',
        executable='moveit_adapter',
        name='moveit_adapter',
        output='screen'
    )
    
    # 4. MoveIt (Motion Planning)
    # Launches move_group without Sim Controllers
    # We must ensure it listens to /joint_states from the Bridge
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
    
    # 5. Accuracy Analysis (Observability)
    accuracy_node = Node(
        package='robot_analysis',
        executable='accuracy_node',
        name='accuracy_analysis',
        output='screen'
    )
    
    # 6. Rviz (Visualization)
    # MoveIt launch usually includes Rviz, but if not we can add it here.
    # For now, relying on move_group.launch.py to include Rviz if configured, 
    # OR we can add a specific Rviz node if needed for "Mirror". 
    # The 'display.launch.py' also usually launches Rviz.
    
    return LaunchDescription([
        display_launch,
        bridge_node,
        moveit_adapter,
        # Delayed launch for MoveIt to ensure URDF is ready
        TimerAction(
            period=5.0,
            actions=[move_group_launch]
        ),
        accuracy_node
    ])
