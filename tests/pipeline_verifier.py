import asyncio
import websockets
import json
import time

async def verify_pipeline():
    uri = "ws://127.0.0.1:9090"
    print(f"Connecting to ROS Bridge at {uri}...")
    try:
        async with websockets.connect(uri, subprotocols=["json", "ros2"]) as ws:
            print("[OK] WebSocket connected.")
            
            # Subscribe to joint states
            sub_msg = {
                "op": "subscribe",
                "topic": "/joint_states",
                "type": "sensor_msgs/JointState"
            }
            await ws.send(json.dumps(sub_msg))
            
            # Wait for first joint state
            start = time.time()
            initial_pos = None
            while time.time() - start < 5.0:
                raw = await ws.recv()
                msg = json.loads(raw)
                if msg.get('type') == 'joint_states' or msg.get('topic') == '/joint_states':
                    data = msg.get('data', {}) if 'data' in msg else msg.get('msg', {})
                    if 'joint_pos_rad' in data:
                        initial_pos = data['joint_pos_rad']
                    elif 'position' in data:
                        initial_pos = data['position']
                    if initial_pos:
                        print(f"[OK] Received initial joint states: {initial_pos}")
                        break
            
            if not initial_pos:
                print("[FAIL] Did not receive /joint_states within 5 seconds.")
                return
                
            # Send an IK request
            ik_req = {
                "op": "call_service", "id": "ik_test_1", "service": "/compute_ik",
                "args": {
                    "ik_request": {
                        "group_name": "manipulator",
                        "pose_stamped": {
                            "header": {"frame_id": "base_link"},
                            "pose": {
                                "position": {"x": 0.3, "y": 0.0, "z": 0.4},
                                "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
                            }
                        }
                    }
                }
            }
            print("Sending Inverse Kinematics (MoveIt2) request...")
            await ws.send(json.dumps(ik_req))
            
            # Wait for IK response
            ik_sol = None
            start = time.time()
            while time.time() - start < 5.0:
                raw = await ws.recv()
                msg = json.loads(raw)
                if msg.get('id') == 'ik_test_1' and msg.get('op') == 'service_response':
                    if msg.get('result'):
                        vals = msg.get('values', {})
                        if 'solution' in vals:
                            ik_sol = vals['solution']['joint_state']['position'][:6]
                            print(f"[OK] IK Solution verified: {ik_sol}")
                            break
                    else:
                        print(f"[FAIL] IK failed: {msg}")
                        return
                        
            if not ik_sol:
                print("[FAIL] No IK response.")
                return

            # Execute trajectory dynamically
            prog = {
                "op": "roboforge/execute_program",
                "program": [{
                    "duration_s": 2.0,
                    "points": [{
                        "positions": ik_sol,
                        "velocities": [0]*6,
                        "time_from_start": 2.0
                    }]
                }]
            }
            print("Sending trajectory execution command...")
            await ws.send(json.dumps(prog))
            
            # Monitor states until completion
            print("Monitoring /joint_states interpolation...")
            success = False
            for _ in range(30):
                raw = await ws.recv()
                msg = json.loads(raw)
                if msg.get('type') == 'joint_states':
                    data = msg.get('data', {})
                    pos = data.get('joint_pos_rad', [])
                    if pos:
                        diff = sum(abs(a - b) for a,b in zip(pos, ik_sol))
                        if diff < 0.1:
                            print(f"[OK] Pipeline execution successful! Error: {diff:.3f}")
                            success = True
                            break
            
            if not success:
                print("[FAIL] Joints did not reach target.")
                
    except Exception as e:
        print(f"[FAIL] Pipeline error: {e}")

if __name__ == "__main__":
    asyncio.run(verify_pipeline())
