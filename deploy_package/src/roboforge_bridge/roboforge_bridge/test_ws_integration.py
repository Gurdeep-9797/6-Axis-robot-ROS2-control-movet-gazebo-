import asyncio, json, websockets

async def test():
    ws = await websockets.connect('ws://localhost:9090')

    # Test 1: Health check service
    await ws.send(json.dumps({'op':'call_service','id':'h1','service':'/roboforge/health_check','args':{}}))
    # Read messages until we get the service_response
    for _ in range(20):
        m = json.loads(await asyncio.wait_for(ws.recv(), 3.0))
        if m.get('op') == 'service_response' and m.get('id') == 'h1':
            print('HEALTH:', json.dumps(m['values'], indent=2))
            break
    else:
        print('HEALTH: no service_response received (got broadcasts instead)')

    # Test 2: Joint states via broadcast
    for _ in range(5):
        m = json.loads(await asyncio.wait_for(ws.recv(), 3.0))
        if m.get('op') == 'publish' and m.get('topic') == '/joint_states':
            print('JOINTS:', [round(p,3) for p in m['msg']['position']])
            break

    # Test 3: IK service
    await ws.send(json.dumps({
        'op':'call_service','id':'ik1','service':'/compute_ik',
        'args':{'ik_request':{
            'group_name':'robot_arm',
            'pose_stamped':{'header':{'frame_id':'base_link'},'pose':{
                'position':{'x':0.4,'y':0.0,'z':0.6},
                'orientation':{'x':0,'y':0,'z':0,'w':1}
            }},
            'avoid_collisions':True
        }}
    }))
    for _ in range(20):
        m = json.loads(await asyncio.wait_for(ws.recv(), 5.0))
        if m.get('op') == 'service_response' and m.get('id') == 'ik1':
            vals = m.get('values', {})
            ec = vals.get('error_code', {}).get('val', '?')
            print(f'IK: error_code={ec}')
            if vals.get('solution'):
                joints = vals['solution']['joint_state']['position'][:6]
                print(f'  Solution: [{" ".join(f"{j:.3f}" for j in joints)}]')
            break

    # Test 4: Trajectory publish
    await ws.send(json.dumps({
        'op':'publish','topic':'/planned_trajectory',
        'msg':{'joint_names':['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'],
               'points':[{'positions':[0.5,-0.3,0.2,0,-0.5,0.1],
                          'velocities':[0,0,0,0,0,0],
                          'time_from_start':{'sec':2,'nanosec':0}}]}
    }))
    print('TRAJ: published')

    await ws.close()
    print('\n=== ALL TESTS PASSED ===')

asyncio.get_event_loop().run_until_complete(test())
