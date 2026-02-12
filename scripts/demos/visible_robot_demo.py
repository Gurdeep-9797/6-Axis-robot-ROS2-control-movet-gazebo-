#!/usr/bin/env python3
"""
VISIBLE ROBOT SIMULATION - Antigravity Active Control Demo
===========================================================
This script creates a VISIBLE robot simulation where you can SEE
the robot moving in real-time, controlled by this script.

Run in WSL2 Ubuntu with ROS 2 Humble installed:
    python3 visible_robot_demo.py

What you will see:
- RViz window opens with a 6-axis robot arm
- Robot joints move continuously in a wave pattern
- Motion is smooth and changes over time
- This script is the source of all motion commands
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import time
import subprocess
import os
import signal
import sys

# Simple 6-axis robot URDF (embedded)
SIMPLE_URDF = """<?xml version="1.0"?>
<robot name="simple_arm">
  
  <material name="orange">
    <color rgba="1.0 0.5 0.0 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>

  <!-- Base -->
  <link name="base_link">
    <visual>
      <geometry><cylinder radius="0.15" length="0.1"/></geometry>
      <origin xyz="0 0 0.05"/>
      <material name="grey"/>
    </visual>
  </link>

  <!-- Link 1 -->
  <link name="link_1">
    <visual>
      <geometry><cylinder radius="0.08" length="0.3"/></geometry>
      <origin xyz="0 0 0.15"/>
      <material name="orange"/>
    </visual>
  </link>
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>
  </joint>

  <!-- Link 2 -->
  <link name="link_2">
    <visual>
      <geometry><box size="0.08 0.08 0.25"/></geometry>
      <origin xyz="0 0 0.125"/>
      <material name="grey"/>
    </visual>
  </link>
  <joint name="joint_2" type="revolute">
    <parent link="link_1"/>
    <child link="link_2"/>
    <origin xyz="0 0 0.3"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Link 3 -->
  <link name="link_3">
    <visual>
      <geometry><cylinder radius="0.06" length="0.2"/></geometry>
      <origin xyz="0 0 0.1"/>
      <material name="orange"/>
    </visual>
  </link>
  <joint name="joint_3" type="revolute">
    <parent link="link_2"/>
    <child link="link_3"/>
    <origin xyz="0 0 0.25"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="100" velocity="1"/>
  </joint>

  <!-- Link 4 -->
  <link name="link_4">
    <visual>
      <geometry><cylinder radius="0.05" length="0.15"/></geometry>
      <origin xyz="0 0 0.075"/>
      <material name="grey"/>
    </visual>
  </link>
  <joint name="joint_4" type="revolute">
    <parent link="link_3"/>
    <child link="link_4"/>
    <origin xyz="0 0 0.2"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="50" velocity="2"/>
  </joint>

  <!-- Link 5 -->
  <link name="link_5">
    <visual>
      <geometry><cylinder radius="0.04" length="0.1"/></geometry>
      <origin xyz="0 0 0.05"/>
      <material name="orange"/>
    </visual>
  </link>
  <joint name="joint_5" type="revolute">
    <parent link="link_4"/>
    <child link="link_5"/>
    <origin xyz="0 0 0.15"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="50" velocity="2"/>
  </joint>

  <!-- Link 6 (End Effector) -->
  <link name="link_6">
    <visual>
      <geometry><cylinder radius="0.03" length="0.05"/></geometry>
      <origin xyz="0 0 0.025"/>
      <material name="grey"/>
    </visual>
  </link>
  <joint name="joint_6" type="revolute">
    <parent link="link_5"/>
    <child link="link_6"/>
    <origin xyz="0 0 0.1"/>
    <axis xyz="1 0 0"/>
    <limit lower="-6.28" upper="6.28" effort="20" velocity="3"/>
  </joint>

</robot>
"""

# RViz config for robot visualization
RVIZ_CONFIG = """
Panels:
  - Class: rviz_common/Displays
Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Enabled: true
      Description Topic:
        Value: /robot_description
      Visual Enabled: true
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
  Global Options:
    Fixed Frame: base_link
  Tools:
    - Class: rviz_default_plugins/MoveCamera
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 2.0
      Pitch: 0.5
      Yaw: 0.8
"""


class VisibleRobotController(Node):
    """
    Antigravity Active Control Node
    
    This node publishes joint states to make the robot move visibly.
    The motion pattern is a continuous wave that demonstrates active control.
    """
    
    def __init__(self):
        super().__init__('antigravity_controller')
        
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Timer for continuous motion (50 Hz)
        self.timer = self.create_timer(0.02, self.publish_motion)
        
        self.start_time = time.time()
        self.get_logger().info('=== ANTIGRAVITY CONTROLLER ACTIVE ===')
        self.get_logger().info('Publishing continuous motion to /joint_states')
        self.get_logger().info('Watch the robot move in RViz!')
        
    def publish_motion(self):
        """Generate and publish animated joint positions."""
        t = time.time() - self.start_time
        
        # Create wave-like motion for each joint
        positions = [
            0.8 * math.sin(t * 0.5),          # Joint 1: slow rotation
            0.5 * math.sin(t * 0.8 + 0.5),    # Joint 2: shoulder wave
            0.6 * math.sin(t * 0.6 + 1.0),    # Joint 3: elbow wave
            1.0 * math.sin(t * 1.2),          # Joint 4: wrist rotation
            0.5 * math.sin(t * 0.9 + 1.5),    # Joint 5: wrist bend
            2.0 * math.sin(t * 1.5),          # Joint 6: tool rotation
        ]
        
        # Create JointState message
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = positions
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 6
        
        self.joint_pub.publish(msg)


def write_temp_files():
    """Write URDF and RViz config to temp files."""
    urdf_path = '/tmp/simple_arm.urdf'
    rviz_path = '/tmp/robot_view.rviz'
    
    with open(urdf_path, 'w') as f:
        f.write(SIMPLE_URDF)
    
    with open(rviz_path, 'w') as f:
        f.write(RVIZ_CONFIG)
    
    return urdf_path, rviz_path


def main():
    print("=" * 60)
    print("ANTIGRAVITY VISIBLE ROBOT SIMULATION")
    print("=" * 60)
    print()
    print("This will open an RViz window showing a 6-axis robot arm")
    print("moving continuously under Antigravity control.")
    print()
    print("Press Ctrl+C to stop.")
    print()
    
    # Write temp files
    urdf_path, rviz_path = write_temp_files()
    print(f"[OK] URDF written to {urdf_path}")
    print(f"[OK] RViz config written to {rviz_path}")
    
    # Read URDF content
    with open(urdf_path, 'r') as f:
        urdf_content = f.read()
    
    # Initialize ROS
    rclpy.init()
    
    processes = []
    
    try:
        # Start robot_state_publisher
        print("[LAUNCH] Starting robot_state_publisher...")
        rsp_proc = subprocess.Popen([
            'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
            '--ros-args', '-p', f'robot_description:={urdf_content}'
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        processes.append(rsp_proc)
        time.sleep(2)
        
        # Start RViz
        print("[LAUNCH] Starting RViz (GUI window should appear)...")
        rviz_proc = subprocess.Popen([
            'ros2', 'run', 'rviz2', 'rviz2', '-d', rviz_path
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        processes.append(rviz_proc)
        time.sleep(3)
        
        # Create and run controller node
        print("[LAUNCH] Starting Antigravity controller...")
        print()
        print("=" * 60)
        print("ROBOT SHOULD NOW BE VISIBLE AND MOVING")
        print("=" * 60)
        
        controller = VisibleRobotController()
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print("\n[STOP] Shutting down...")
    finally:
        # Cleanup
        for proc in processes:
            proc.terminate()
            proc.wait()
        rclpy.shutdown()
        print("[DONE] Simulation stopped.")


if __name__ == '__main__':
    main()
