#!/usr/bin/env python3
"""
INDUSTRIAL ROBOT GAZEBO SIMULATION
===================================
Professional factory-style simulation with:
- Industrial 6-axis robot (ABB IRB 120-like geometry)
- Robot work cage with safety barriers
- Pick station and place station
- Gripper tool holder
- Pickable object (box)
- Complete pick-and-place cycle

Run in WSL2 Ubuntu:
    python3 industrial_gazebo_demo.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
import subprocess
import time
import math
import os
import signal
import sys

# ============================================================
# INDUSTRIAL ROBOT URDF (ABB IRB 120-like proportions)
# ============================================================
INDUSTRIAL_ROBOT_URDF = '''<?xml version="1.0"?>
<robot name="industrial_arm" xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- Colors -->
  <material name="industrial_orange">
    <color rgba="1.0 0.45 0.0 1.0"/>
  </material>
  <material name="dark_grey">
    <color rgba="0.2 0.2 0.2 1.0"/>
  </material>
  <material name="light_grey">
    <color rgba="0.7 0.7 0.7 1.0"/>
  </material>
  <material name="safety_yellow">
    <color rgba="1.0 0.9 0.0 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.1 0.1 0.1 1.0"/>
  </material>

  <!-- ========== WORLD LINK ========== -->
  <link name="world"/>
  
  <!-- ========== ROBOT BASE PLATE ========== -->
  <link name="base_plate">
    <visual>
      <geometry><box size="0.4 0.4 0.02"/></geometry>
      <origin xyz="0 0 0.01"/>
      <material name="dark_grey"/>
    </visual>
    <collision>
      <geometry><box size="0.4 0.4 0.02"/></geometry>
      <origin xyz="0 0 0.01"/>
    </collision>
    <inertial>
      <mass value="20.0"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.5"/>
    </inertial>
  </link>
  <joint name="world_to_base" type="fixed">
    <parent link="world"/>
    <child link="base_plate"/>
    <origin xyz="0 0 0"/>
  </joint>

  <!-- ========== ROBOT BASE (Pedestal) ========== -->
  <link name="base_link">
    <visual>
      <geometry><cylinder radius="0.12" length="0.15"/></geometry>
      <origin xyz="0 0 0.075"/>
      <material name="industrial_orange"/>
    </visual>
    <collision>
      <geometry><cylinder radius="0.12" length="0.15"/></geometry>
      <origin xyz="0 0 0.075"/>
    </collision>
    <inertial>
      <mass value="8.0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.03"/>
    </inertial>
  </link>
  <joint name="base_plate_to_base" type="fixed">
    <parent link="base_plate"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.02"/>
  </joint>

  <!-- ========== LINK 1 (Rotating Base) ========== -->
  <link name="link_1">
    <visual>
      <geometry><cylinder radius="0.1" length="0.12"/></geometry>
      <origin xyz="0 0 0.06"/>
      <material name="industrial_orange"/>
    </visual>
    <visual>
      <geometry><box size="0.12 0.08 0.18"/></geometry>
      <origin xyz="0 0 0.21"/>
      <material name="industrial_orange"/>
    </visual>
    <collision>
      <geometry><cylinder radius="0.1" length="0.3"/></geometry>
      <origin xyz="0 0 0.15"/>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.15"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.97" upper="2.97" effort="150" velocity="4.36"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <!-- ========== LINK 2 (Shoulder) ========== -->
  <link name="link_2">
    <visual>
      <geometry><box size="0.08 0.12 0.27"/></geometry>
      <origin xyz="0 0 0.135"/>
      <material name="industrial_orange"/>
    </visual>
    <collision>
      <geometry><box size="0.08 0.12 0.27"/></geometry>
      <origin xyz="0 0 0.135"/>
    </collision>
    <inertial>
      <mass value="4.0"/>
      <inertia ixx="0.03" ixy="0" ixz="0" iyy="0.03" iyz="0" izz="0.005"/>
    </inertial>
  </link>
  <joint name="joint_2" type="revolute">
    <parent link="link_1"/>
    <child link="link_2"/>
    <origin xyz="0 0 0.3"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.92" upper="1.92" effort="150" velocity="4.36"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <!-- ========== LINK 3 (Elbow) ========== -->
  <link name="link_3">
    <visual>
      <geometry><box size="0.07 0.1 0.07"/></geometry>
      <origin xyz="0 0 0.035"/>
      <material name="industrial_orange"/>
    </visual>
    <visual>
      <geometry><cylinder radius="0.04" length="0.22"/></geometry>
      <origin xyz="0 0 0.18"/>
      <material name="industrial_orange"/>
    </visual>
    <collision>
      <geometry><box size="0.08 0.1 0.3"/></geometry>
      <origin xyz="0 0 0.15"/>
    </collision>
    <inertial>
      <mass value="2.5"/>
      <inertia ixx="0.015" ixy="0" ixz="0" iyy="0.015" iyz="0" izz="0.003"/>
    </inertial>
  </link>
  <joint name="joint_3" type="revolute">
    <parent link="link_2"/>
    <child link="link_3"/>
    <origin xyz="0 0 0.27"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.22" upper="1.92" effort="100" velocity="4.36"/>
    <dynamics damping="0.3" friction="0.1"/>
  </joint>

  <!-- ========== LINK 4 (Wrist 1) ========== -->
  <link name="link_4">
    <visual>
      <geometry><cylinder radius="0.035" length="0.12"/></geometry>
      <origin xyz="0 0 0.06" rpy="0 0 0"/>
      <material name="dark_grey"/>
    </visual>
    <collision>
      <geometry><cylinder radius="0.035" length="0.12"/></geometry>
      <origin xyz="0 0 0.06"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.003" ixy="0" ixz="0" iyy="0.003" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  <joint name="joint_4" type="revolute">
    <parent link="link_3"/>
    <child link="link_4"/>
    <origin xyz="0 0 0.29"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2.79" upper="2.79" effort="50" velocity="5.58"/>
    <dynamics damping="0.1" friction="0.05"/>
  </joint>

  <!-- ========== LINK 5 (Wrist 2) ========== -->
  <link name="link_5">
    <visual>
      <geometry><cylinder radius="0.03" length="0.08"/></geometry>
      <origin xyz="0 0.04 0" rpy="1.5708 0 0"/>
      <material name="dark_grey"/>
    </visual>
    <collision>
      <geometry><cylinder radius="0.03" length="0.08"/></geometry>
      <origin xyz="0 0.04 0" rpy="1.5708 0 0"/>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  <joint name="joint_5" type="revolute">
    <parent link="link_4"/>
    <child link="link_5"/>
    <origin xyz="0 0 0.12"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.09" upper="2.09" effort="50" velocity="5.58"/>
    <dynamics damping="0.1" friction="0.05"/>
  </joint>

  <!-- ========== LINK 6 (Tool Flange) ========== -->
  <link name="link_6">
    <visual>
      <geometry><cylinder radius="0.025" length="0.02"/></geometry>
      <origin xyz="0 0 0.01"/>
      <material name="black"/>
    </visual>
    <collision>
      <geometry><cylinder radius="0.025" length="0.02"/></geometry>
      <origin xyz="0 0 0.01"/>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.0005" iyz="0" izz="0.0003"/>
    </inertial>
  </link>
  <joint name="joint_6" type="revolute">
    <parent link="link_5"/>
    <child link="link_6"/>
    <origin xyz="0 0.08 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-6.98" upper="6.98" effort="20" velocity="7.33"/>
    <dynamics damping="0.05" friction="0.02"/>
  </joint>

  <!-- ========== GRIPPER BASE ========== -->
  <link name="gripper_base">
    <visual>
      <geometry><box size="0.06 0.04 0.03"/></geometry>
      <origin xyz="0 0 0.015"/>
      <material name="dark_grey"/>
    </visual>
    <collision>
      <geometry><box size="0.06 0.04 0.03"/></geometry>
      <origin xyz="0 0 0.015"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>
  <joint name="tool_joint" type="fixed">
    <parent link="link_6"/>
    <child link="gripper_base"/>
    <origin xyz="0 0 0.02"/>
  </joint>

  <!-- ========== GRIPPER FINGER LEFT ========== -->
  <link name="finger_left">
    <visual>
      <geometry><box size="0.01 0.015 0.05"/></geometry>
      <origin xyz="0 0 0.025"/>
      <material name="light_grey"/>
    </visual>
    <collision>
      <geometry><box size="0.01 0.015 0.05"/></geometry>
      <origin xyz="0 0 0.025"/>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
    </inertial>
  </link>
  <joint name="finger_left_joint" type="prismatic">
    <parent link="gripper_base"/>
    <child link="finger_left"/>
    <origin xyz="-0.02 0 0.03"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="0.02" effort="10" velocity="0.1"/>
  </joint>

  <!-- ========== GRIPPER FINGER RIGHT ========== -->
  <link name="finger_right">
    <visual>
      <geometry><box size="0.01 0.015 0.05"/></geometry>
      <origin xyz="0 0 0.025"/>
      <material name="light_grey"/>
    </visual>
    <collision>
      <geometry><box size="0.01 0.015 0.05"/></geometry>
      <origin xyz="0 0 0.025"/>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
    </inertial>
  </link>
  <joint name="finger_right_joint" type="prismatic">
    <parent link="gripper_base"/>
    <child link="finger_right"/>
    <origin xyz="0.02 0 0.03"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.02" upper="0" effort="10" velocity="0.1"/>
  </joint>

</robot>
'''

# ============================================================
# GAZEBO WORLD WITH WORK CAGE
# ============================================================
FACTORY_WORLD = '''<?xml version="1.0"?>
<sdf version="1.6">
  <world name="factory_cell">
    
    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Ground Plane (Factory Floor) -->
    <model name="factory_floor">
      <static>true</static>
      <link name="floor_link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.4 0.4 0.4 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Safety Cage Posts -->
    <model name="cage_post_1">
      <static>true</static>
      <pose>-1 -1 0.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>0.03</radius><length>1.0</length></cylinder></geometry>
          <material><ambient>1 0.9 0 1</ambient><diffuse>1 0.9 0 1</diffuse></material>
        </visual>
      </link>
    </model>
    <model name="cage_post_2">
      <static>true</static>
      <pose>1 -1 0.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>0.03</radius><length>1.0</length></cylinder></geometry>
          <material><ambient>1 0.9 0 1</ambient><diffuse>1 0.9 0 1</diffuse></material>
        </visual>
      </link>
    </model>
    <model name="cage_post_3">
      <static>true</static>
      <pose>1 1 0.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>0.03</radius><length>1.0</length></cylinder></geometry>
          <material><ambient>1 0.9 0 1</ambient><diffuse>1 0.9 0 1</diffuse></material>
        </visual>
      </link>
    </model>
    <model name="cage_post_4">
      <static>true</static>
      <pose>-1 1 0.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>0.03</radius><length>1.0</length></cylinder></geometry>
          <material><ambient>1 0.9 0 1</ambient><diffuse>1 0.9 0 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Pick Station (Left Table) -->
    <model name="pick_station">
      <static>true</static>
      <pose>0.5 0 0.25 0 0 0</pose>
      <link name="link">
        <visual name="table_top">
          <geometry><box><size>0.4 0.3 0.02</size></box></geometry>
          <pose>0 0 0.24 0 0 0</pose>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>
        </visual>
        <visual name="leg1">
          <geometry><cylinder><radius>0.02</radius><length>0.48</length></cylinder></geometry>
          <pose>0.15 0.1 0 0 0 0</pose>
          <material><ambient>0.3 0.3 0.3 1</ambient></material>
        </visual>
        <visual name="leg2">
          <geometry><cylinder><radius>0.02</radius><length>0.48</length></cylinder></geometry>
          <pose>-0.15 0.1 0 0 0 0</pose>
          <material><ambient>0.3 0.3 0.3 1</ambient></material>
        </visual>
        <visual name="leg3">
          <geometry><cylinder><radius>0.02</radius><length>0.48</length></cylinder></geometry>
          <pose>0.15 -0.1 0 0 0 0</pose>
          <material><ambient>0.3 0.3 0.3 1</ambient></material>
        </visual>
        <visual name="leg4">
          <geometry><cylinder><radius>0.02</radius><length>0.48</length></cylinder></geometry>
          <pose>-0.15 -0.1 0 0 0 0</pose>
          <material><ambient>0.3 0.3 0.3 1</ambient></material>
        </visual>
        <collision name="collision">
          <geometry><box><size>0.4 0.3 0.5</size></box></geometry>
        </collision>
      </link>
    </model>

    <!-- Place Station (Right Table) -->
    <model name="place_station">
      <static>true</static>
      <pose>-0.5 0 0.25 0 0 0</pose>
      <link name="link">
        <visual name="table_top">
          <geometry><box><size>0.4 0.3 0.02</size></box></geometry>
          <pose>0 0 0.24 0 0 0</pose>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>
        </visual>
        <visual name="leg1">
          <geometry><cylinder><radius>0.02</radius><length>0.48</length></cylinder></geometry>
          <pose>0.15 0.1 0 0 0 0</pose>
          <material><ambient>0.3 0.3 0.3 1</ambient></material>
        </visual>
        <visual name="leg2">
          <geometry><cylinder><radius>0.02</radius><length>0.48</length></cylinder></geometry>
          <pose>-0.15 0.1 0 0 0 0</pose>
          <material><ambient>0.3 0.3 0.3 1</ambient></material>
        </visual>
        <visual name="leg3">
          <geometry><cylinder><radius>0.02</radius><length>0.48</length></cylinder></geometry>
          <pose>0.15 -0.1 0 0 0 0</pose>
          <material><ambient>0.3 0.3 0.3 1</ambient></material>
        </visual>
        <visual name="leg4">
          <geometry><cylinder><radius>0.02</radius><length>0.48</length></cylinder></geometry>
          <pose>-0.15 -0.1 0 0 0 0</pose>
          <material><ambient>0.3 0.3 0.3 1</ambient></material>
        </visual>
        <collision name="collision">
          <geometry><box><size>0.4 0.3 0.5</size></box></geometry>
        </collision>
      </link>
    </model>

    <!-- Pickable Object (Red Box) -->
    <model name="pick_object">
      <pose>0.5 0 0.52 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>0.1</mass>
          <inertia>
            <ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry><box><size>0.04 0.04 0.04</size></box></geometry>
          <material><ambient>0.8 0.1 0.1 1</ambient><diffuse>1 0.2 0.2 1</diffuse></material>
        </visual>
        <collision name="collision">
          <geometry><box><size>0.04 0.04 0.04</size></box></geometry>
        </collision>
      </link>
    </model>

    <!-- Control Panel -->
    <model name="control_panel">
      <static>true</static>
      <pose>0 -0.8 0.5 0 0 0</pose>
      <link name="link">
        <visual name="cabinet">
          <geometry><box><size>0.3 0.15 1.0</size></box></geometry>
          <material><ambient>0.3 0.3 0.35 1</ambient><diffuse>0.4 0.4 0.45 1</diffuse></material>
        </visual>
        <visual name="screen">
          <geometry><box><size>0.2 0.01 0.15</size></box></geometry>
          <pose>0 -0.08 0.3 0 0 0</pose>
          <material><ambient>0.1 0.2 0.3 1</ambient><diffuse>0.2 0.3 0.5 1</diffuse></material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
'''


class IndustrialRobotController(Node):
    """
    Industrial Pick-and-Place Controller
    
    Executes smooth pick-and-place cycles:
    1. Home position
    2. Move to pick station
    3. Lower to pick
    4. Close gripper
    5. Lift
    6. Move to place station
    7. Lower to place
    8. Open gripper
    9. Lift and return home
    """
    
    def __init__(self):
        super().__init__('industrial_controller')
        
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3', 
            'joint_4', 'joint_5', 'joint_6',
            'finger_left_joint', 'finger_right_joint'
        ]
        
        # Key poses (joint angles in radians)
        self.poses = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, -0.01],
            'pick_approach': [1.2, 0.3, -0.2, 0.0, -0.1, 0.0, 0.01, -0.01],
            'pick_down': [1.2, 0.5, -0.3, 0.0, -0.2, 0.0, 0.01, -0.01],
            'pick_grab': [1.2, 0.5, -0.3, 0.0, -0.2, 0.0, 0.0, 0.0],
            'pick_lift': [1.2, 0.2, -0.1, 0.0, -0.1, 0.0, 0.0, 0.0],
            'place_approach': [-1.2, 0.3, -0.2, 0.0, -0.1, 0.0, 0.0, 0.0],
            'place_down': [-1.2, 0.5, -0.3, 0.0, -0.2, 0.0, 0.0, 0.0],
            'place_release': [-1.2, 0.5, -0.3, 0.0, -0.2, 0.0, 0.01, -0.01],
            'place_lift': [-1.2, 0.2, -0.1, 0.0, -0.1, 0.0, 0.01, -0.01],
        }
        
        self.sequence = [
            ('home', 2.0),
            ('pick_approach', 1.5),
            ('pick_down', 1.0),
            ('pick_grab', 0.5),
            ('pick_lift', 1.0),
            ('place_approach', 2.0),
            ('place_down', 1.0),
            ('place_release', 0.5),
            ('place_lift', 1.0),
            ('home', 1.5),
        ]
        
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.02, self.update)  # 50 Hz
        
        self.current_positions = list(self.poses['home'])
        self.target_positions = list(self.poses['home'])
        self.current_step = 0
        self.step_time = 0.0
        self.step_duration = 2.0
        self.cycle_count = 0
        
        self.get_logger().info('=== INDUSTRIAL ROBOT CONTROLLER ACTIVE ===')
        self.get_logger().info('Executing pick-and-place cycles...')
        
    def update(self):
        """Update robot motion - smooth interpolation between poses."""
        dt = 0.02
        self.step_time += dt
        
        # Check if current step is complete
        if self.step_time >= self.step_duration:
            self.step_time = 0.0
            self.current_positions = list(self.target_positions)
            self.current_step = (self.current_step + 1) % len(self.sequence)
            
            if self.current_step == 0:
                self.cycle_count += 1
                self.get_logger().info(f'Completed cycle {self.cycle_count}')
            
            pose_name, duration = self.sequence[self.current_step]
            self.target_positions = list(self.poses[pose_name])
            self.step_duration = duration
            self.get_logger().info(f'Moving to: {pose_name}')
        
        # Smooth interpolation
        t = self.step_time / self.step_duration
        t = t * t * (3 - 2 * t)  # Smoothstep
        
        positions = []
        for i in range(len(self.current_positions)):
            pos = self.current_positions[i] + t * (self.target_positions[i] - self.current_positions[i])
            positions.append(pos)
        
        # Publish joint state
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = positions
        msg.velocity = [0.0] * len(positions)
        msg.effort = [0.0] * len(positions)
        self.joint_pub.publish(msg)


def main():
    print("=" * 60)
    print("INDUSTRIAL ROBOT GAZEBO SIMULATION")
    print("=" * 60)
    print()
    print("Features:")
    print("  - Industrial 6-axis robot (ABB-style)")
    print("  - Robot work cage with safety posts")
    print("  - Pick and place stations with tables")
    print("  - Gripper with movable fingers")
    print("  - Continuous pick-and-place cycle")
    print()
    print("Press Ctrl+C to stop.")
    print()
    
    # Write temp files
    urdf_path = '/tmp/industrial_robot.urdf'
    world_path = '/tmp/factory_cell.world'
    
    with open(urdf_path, 'w') as f:
        f.write(INDUSTRIAL_ROBOT_URDF)
    print(f"[OK] Robot URDF: {urdf_path}")
    
    with open(world_path, 'w') as f:
        f.write(FACTORY_WORLD)
    print(f"[OK] Factory world: {world_path}")
    
    # Read URDF for robot_state_publisher
    with open(urdf_path, 'r') as f:
        urdf_content = f.read()
    
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
        
        # Start Gazebo with factory world
        print("[LAUNCH] Starting Gazebo (window should appear)...")
        gz_proc = subprocess.Popen([
            'gazebo', '--verbose', world_path
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        processes.append(gz_proc)
        time.sleep(5)
        
        # Spawn robot in Gazebo
        print("[LAUNCH] Spawning robot in Gazebo...")
        spawn_proc = subprocess.Popen([
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'industrial_arm',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0'
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        spawn_proc.wait()
        time.sleep(2)
        
        # Start RViz for additional visualization
        print("[LAUNCH] Starting RViz...")
        rviz_proc = subprocess.Popen([
            'ros2', 'run', 'rviz2', 'rviz2'
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        processes.append(rviz_proc)
        time.sleep(2)
        
        print()
        print("=" * 60)
        print("INDUSTRIAL SIMULATION RUNNING")
        print("=" * 60)
        print()
        print("You should see:")
        print("  1. Gazebo window with factory cell")
        print("  2. Robot arm in center")
        print("  3. Yellow safety posts")
        print("  4. Pick/Place stations with tables")
        print("  5. Red box on pick table")
        print()
        
        # Start controller
        controller = IndustrialRobotController()
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print("\n[STOP] Shutting down...")
    finally:
        for proc in processes:
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except:
                proc.kill()
        rclpy.shutdown()
        print("[DONE] Simulation stopped.")


if __name__ == '__main__':
    main()
