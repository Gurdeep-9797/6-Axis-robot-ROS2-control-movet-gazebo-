$ErrorActionPreference = "Stop"
Write-Host "=== PHASE 2 VERIFICATION (GAZEBO INTEGRATION) ==="

Try {
    # 1. Start Sim Mode
    Write-Host "1. Starting System (SIM Mode)..."
    ./scripts/start_sim.ps1
    
    # Wait for startup
    Write-Host "   Waiting 20s for Gazebo warmup..."
    Start-Sleep -Seconds 20
    
    # 2. Check Bridge Mode
    Write-Host "`n2. Checking Bridge Log for GAZEBO Mode..."
    $logs = docker logs robot_bridge 2>&1
    if ($logs -match "Using Simulation Backend \(GAZEBO Mode\)") {
        Write-Host "   [PASS] Bridge is in GAZEBO Mode" -ForegroundColor Green
    }
    else {
        Write-Host "   [FAIL] Bridge NOT in GAZEBO Mode" -ForegroundColor Red
        # Continue anyway to check other things?
    }
    
    # 3. Check Message Flow (Joint States)
    # We expect Bridge to publish /joint_states. We can check via topic hz (requires exec)
    # Or just rely on visual check if user was watching.
    # Automated check: exec into bridge and echo topic?
    
    # 4. Run Task Node
    Write-Host "`n4. Executing ROS Task Node..."
    docker exec robot_bridge python3 /ros_ws/scripts/ros_task_node.py
    
    Write-Host "`n=== VERIFICATION COMPLETE ==="
}
Catch {
    Write-Host "`n!!! VERIFICATION FAILED !!!" -ForegroundColor Red
    Write-Host $_.Exception.Message
    exit 1
}
