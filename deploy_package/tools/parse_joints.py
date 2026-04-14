#!/usr/bin/env python3
"""Parse joint states from ROS2 topic and output as CSV"""
import subprocess
import sys

try:
    result = subprocess.run(
        ['ros2', 'topic', 'echo', '/joint_states', '--once'],
        capture_output=True, text=True, timeout=5
    )
    lines = result.stdout.split('\n')
    in_position = False
    positions = []
    
    for line in lines:
        if line.strip() == 'position:':
            in_position = True
            continue
        if in_position:
            if line.strip().startswith('velocity:') or line.strip().startswith('effort:'):
                break
            if line.strip().startswith('- '):
                try:
                    positions.append(float(line.strip()[2:]))
                except:
                    pass
    
    if len(positions) == 6:
        print(','.join(map(str, positions)))
    else:
        print("ERROR: Expected 6 joints, got {}".format(len(positions)), file=sys.stderr)
        sys.exit(1)
except Exception as e:
    print("ERROR: {}".format(str(e)), file=sys.stderr)
    sys.exit(1)
