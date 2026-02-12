from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Paths
    robot_gazebo_pkg = FindPackageShare('robot_gazebo')
    moveit_config_pkg = FindPackageShare('robot_moveit_config')
    robot_analysis_pkg = FindPackageShare('robot_analysis')
    
    # 1. Gazebo Simulation (Physics + Controllers)
    # This launches Gazebo, Spawns Robot, and Starts ros2_control Controller Manager
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([robot_gazebo_pkg, 'launch', 'gazebo.launch.py'])
        )
    )
    
    # 2. MoveIt (Motion Planning)
    # Configured to talk to Gazebo's /follow_joint_trajectory Action Server
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
    
    # 3. Accuracy Analysis (Observability)
    accuracy_node = Node(
        package='robot_analysis',
        executable='accuracy_node',
        name='accuracy_analysis',
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        # Delayed MoveIt to wait for Gazebo
        TimerAction(
            period=10.0,
            actions=[move_group_launch]
        ),
        accuracy_node
    ])
