$ErrorActionPreference = "Stop"
Write-Host "=== PHASE 4 VERIFICATION (MoveIt Adapter) ==="

Try {
    # 0. Cleanup
    ./scripts/stop_all.ps1
    Start-Sleep -Seconds 5
    
    # 1. Start Sim
    Write-Host "1. Starting System..."
    ./scripts/start_sim.ps1
    
    # Wait for startup
    Write-Host "   Waiting 30s..."
    Start-Sleep -Seconds 30
    
    # 2. Check Adapter
    Write-Host "`n2. Checking Adapter Status..."
    $logs = docker logs robot_adapter 2>&1
    if ($logs -match "MoveIt Adapter Ready") {
        Write-Host "   [PASS] Adapter is Ready" -ForegroundColor Green
    }
    else {
        Write-Host "   [FAIL] Adapter log missing 'Ready' message" -ForegroundColor Red
        # Continue to verify action
    }
    
    # 3. Send Action Goal (Manual Test)
    # We send a goal to /robot_arm_controller/follow_joint_trajectory
    Write-Host "`n3. Sending Action Goal..."
    
    # Construct a simple goal: move to 0.1 rad on all joints
    # This is a bit verbose in CLI.
    # docker exec robot_ros_core ros2 action send_goal /robot_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'], points: [{positions: [0.1,0.1,0.1,0.1,0.1,0.1], time_from_start: {sec: 2, nanosec: 0}}]}}"
    
    $goal_cmd = "ros2 action send_goal /robot_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory ""{trajectory: {joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'], points: [{positions: [0.1,0.1,0.1,0.1,0.1,0.1], time_from_start: {sec: 2, nanosec: 0}}]}}"""
    
    Write-Host "   Command: $goal_cmd"
    Invoke-Expression "docker exec robot_ros_core $goal_cmd"
    
    # 4. Check Robot Moved (via Bridge Log)
    Write-Host "`n4. Checking Bridge for Trajectory..."
    $bridge_logs = docker logs robot_bridge 2>&1
    if ($bridge_logs -match "Received trajectory") {
        Write-Host "   [PASS] Bridge Received Trajectory" -ForegroundColor Green
    }
    else {
        Write-Host "   [FAIL] Bridge did NOT receive trajectory" -ForegroundColor Red
    }
    
    Write-Host "`n=== VERIFICATION COMPLETE ==="
}
Catch {
    Write-Host "`n!!! VERIFICATION FAILED !!!" -ForegroundColor Red
    Write-Host $_.Exception.Message
    exit 1
}
