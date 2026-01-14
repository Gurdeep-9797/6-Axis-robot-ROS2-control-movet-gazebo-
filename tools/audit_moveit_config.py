#!/usr/bin/env python3
"""
AGENT 3: MoveIt Config Audit Tool
Validates all MoveIt configuration files.
Exits non-zero on ANY misconfiguration.
"""
import sys
import os
import yaml
import xml.etree.ElementTree as ET

ROBOT_SYSTEM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MOVEIT_CONFIG = os.path.join(ROBOT_SYSTEM, "src", "robot_moveit_config", "config")
ROBOT_DESC = os.path.join(ROBOT_SYSTEM, "src", "robot_description")

def fail(msg):
    print(f"[FAIL] {msg}")
    sys.exit(1)

def ok(msg):
    print(f"[PASS] {msg}")

def audit_kinematics():
    path = os.path.join(MOVEIT_CONFIG, "kinematics.yaml")
    if not os.path.exists(path):
        fail("kinematics.yaml not found")
    
    with open(path) as f:
        data = yaml.safe_load(f)
    
    if "robot_arm" not in data:
        fail("kinematics.yaml: 'robot_arm' group not defined")
    
    cfg = data["robot_arm"]
    if "kinematics_solver" not in cfg:
        fail("kinematics.yaml: No IK solver defined")
    
    if cfg.get("kinematics_solver_timeout", 0) <= 0:
        fail("kinematics.yaml: Timeout must be > 0")
    
    ok(f"kinematics.yaml: Solver={cfg['kinematics_solver']}")

def audit_ompl():
    path = os.path.join(MOVEIT_CONFIG, "ompl_planning.yaml")
    if not os.path.exists(path):
        fail("ompl_planning.yaml not found")
    
    with open(path) as f:
        data = yaml.safe_load(f)
    
    if "planner_configs" not in data:
        fail("ompl_planning.yaml: No planner_configs")
    
    if "RRTConnect" not in data["planner_configs"]:
        fail("ompl_planning.yaml: RRTConnect not defined")
    
    if "robot_arm" not in data:
        fail("ompl_planning.yaml: 'robot_arm' group not configured")
    
    ok("ompl_planning.yaml: OMPL planners configured")

def audit_trajectory_execution():
    path = os.path.join(MOVEIT_CONFIG, "trajectory_execution.yaml")
    if not os.path.exists(path):
        fail("trajectory_execution.yaml not found")
    
    with open(path) as f:
        data = yaml.safe_load(f)
    
    if "trajectory_execution" not in data:
        fail("trajectory_execution.yaml: No trajectory_execution section")
    
    if "planning_pipelines" not in data:
        fail("trajectory_execution.yaml: No planning_pipelines")
    
    ok("trajectory_execution.yaml: Time parameterization configured")

def audit_controllers():
    path = os.path.join(MOVEIT_CONFIG, "controllers.yaml")
    if not os.path.exists(path):
        fail("controllers.yaml not found")
    
    with open(path) as f:
        data = yaml.safe_load(f)
    
    if "controller_names" not in data:
        fail("controllers.yaml: No controller_names")
    
    if "robot_arm_controller" not in data:
        fail("controllers.yaml: robot_arm_controller not defined")
    
    ok("controllers.yaml: Controller configured")

def audit_srdf():
    path = os.path.join(MOVEIT_CONFIG, "robot_arm.srdf")
    if not os.path.exists(path):
        fail("robot_arm.srdf not found")
    
    tree = ET.parse(path)
    root = tree.getroot()
    
    groups = root.findall(".//group")
    if not groups:
        fail("SRDF: No planning groups defined")
    
    chains = root.findall(".//chain")
    if not chains:
        fail("SRDF: No kinematic chain defined")
    
    ok("robot_arm.srdf: Planning group and chain defined")

def audit_urdf():
    path = os.path.join(ROBOT_DESC, "urdf", "robot.urdf.xacro")
    if not os.path.exists(path):
        fail("robot.urdf.xacro not found")
    
    with open(path) as f:
        content = f.read()
    
    joint_count = content.count('type="revolute"')
    if joint_count < 6:
        fail(f"URDF: Only {joint_count} revolute joints (need 6)")
    
    if "<limit" not in content:
        fail("URDF: No joint limits defined")
    
    ok(f"URDF: {joint_count} revolute joints with limits")

def audit_launch():
    path = os.path.join(ROBOT_SYSTEM, "src", "robot_moveit_config", "launch", "move_group.launch.py")
    if not os.path.exists(path):
        fail("move_group.launch.py not found")
    
    with open(path) as f:
        content = f.read()
    
    if "move_group" not in content:
        fail("Launch: move_group node not defined")
    
    if "kinematics_yaml" not in content:
        fail("Launch: kinematics.yaml not loaded")
    
    if "ompl_yaml" not in content:
        fail("Launch: ompl_planning.yaml not loaded")
    
    ok("move_group.launch.py: Correctly configured")

def main():
    print("=== MOVEIT CONFIG AUDIT ===")
    audit_kinematics()
    audit_ompl()
    audit_trajectory_execution()
    audit_controllers()
    audit_srdf()
    audit_urdf()
    audit_launch()
    print("\n=== ALL AUDITS PASSED ===")
    return 0

if __name__ == '__main__':
    sys.exit(main())
