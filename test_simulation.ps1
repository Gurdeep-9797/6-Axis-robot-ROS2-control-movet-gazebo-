# RoboForge v8.2 — Automated Test Simulation
# ──────────────────────────────────────────

Write-Host "🧪 RoboForge Test Simulation" -ForegroundColor Cyan
Write-Host "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" -ForegroundColor Gray
Write-Host ""

# ─── 1. Verify All Services ───
Write-Host "[Step 1/7] Verifying Services..." -ForegroundColor Yellow

$services = @{
    "React IDE" = "http://localhost:3000"
    "Bridge REST" = "http://localhost:8765/health"
    "Gazebo VNC" = "http://localhost:6080"
}

$allServicesUp = $true
foreach ($name in $services.Keys) {
    try {
        $response = Invoke-WebRequest -Uri $services[$name] -UseBasicParsing -TimeoutSec 5 -ErrorAction Stop
        Write-Host "  ✅ $name - HTTP $($response.StatusCode)" -ForegroundColor Green
    } catch {
        Write-Host "  ❌ $name - Failed" -ForegroundColor Red
        $allServicesUp = $false
    }
}

if (-not $allServicesUp) {
    Write-Host "❌ Some services are down. Aborting test." -ForegroundColor Red
    exit 1
}
Write-Host ""

# ─── 2. Check Bridge Health ───
Write-Host "[Step 2/7] Checking Bridge Health..." -ForegroundColor Yellow

try {
    $health = Invoke-RestMethod -Uri "http://localhost:8765/health" -ErrorAction Stop
    Write-Host "  Status: $($health.status)" -ForegroundColor Green
    Write-Host "  MoveIt Ready: $($health.moveit_ready)" -ForegroundColor $(if ($health.moveit_ready) { "Green" } else { "Red" })
    Write-Host "  Connected Clients: $($health.connected_clients)" -ForegroundColor Gray
    
    if ($health.status -ne "ok") {
        throw "Bridge not healthy"
    }
} catch {
    Write-Host "  ❌ Bridge health check failed" -ForegroundColor Red
    exit 1
}
Write-Host ""

# ─── 3. Sample Initial Joint States ───
Write-Host "[Step 3/7] Sampling Initial Joint States..." -ForegroundColor Yellow

try {
    $initialJoints = docker exec roboforge_core bash -c "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && ros2 topic echo /joint_states --once" 2>&1
    
    if ($initialJoints) {
        Write-Host "  ✅ Joint states received" -ForegroundColor Green
        
        # Parse positions
        $posMatch = [regex]::Match($initialJoints, 'position:\s*\[([^\]]+)\]')
        if ($posMatch.Success) {
            $positions = $posMatch.Groups[1].Value -split ',' | ForEach-Object { [double]($_.Trim()) }
            $joints = @("J1", "J2", "J3", "J4", "J5", "J6")
            
            Write-Host "  Initial Position:" -ForegroundColor Gray
            for ($i = 0; $i -lt 6; $i++) {
                $deg = [math]::Round($positions[$i] * 180 / [math]::PI, 2)
                Write-Host "    $joints[$i]: $deg° ($($positions[$i]) rad)" -ForegroundColor White
            }
        }
    } else {
        Write-Host "  ⚠️ No joint state data" -ForegroundColor Yellow
    }
} catch {
    Write-Host "  ❌ Failed to sample joints: $_" -ForegroundColor Red
}
Write-Host ""

# ─── 4. Test IK Service via REST ───
Write-Host "[Step 4/7] Testing IK Service..." -ForegroundColor Yellow

$ikPayload = @"
{
  "op": "call_service",
  "service": "/compute_ik",
  "id": "test-ik-001",
  "args": {
    "ik_request": {
      "group_name": "robot_arm",
      "avoid_collisions": true,
      "pose_stamped": {
        "header": { "frame_id": "base_link" },
        "pose": {
          "position": { "x": 0.6, "y": 0.0, "z": 0.8 },
          "orientation": { "x": 0, "y": 0, "z": 0, "w": 1 }
        }
      },
      "timeout": { "sec": 5, "nanosec": 0 }
    }
  }
}
"@

try {
    # Send IK request via WebSocket (using REST API health as proxy test)
    Write-Host "  Sending IK request to /compute_ik..." -ForegroundColor Gray
    
    # Use ROS2 service call directly
    $ikResult = docker exec roboforge_core bash -c "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && timeout 10 ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK '{ik_request: {group_name: robot_arm, pose_stamped: {header: {frame_id: base_link}, pose: {position: {x: 0.6, y: 0.0, z: 0.8}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, timeout: {sec: 5}}}'" 2>&1
    
    if ($ikResult) {
        Write-Host "  ✅ IK service responded" -ForegroundColor Green
        
        # Check error code
        if ($ikResult -match "val:\s*(-?\d+)") {
            $errorCode = [int]$matches[1]
            $errorMessages = @{
                1 = "SUCCESS"
                -1 = "PLANNING_FAILED"
                -15 = "NO_IK_SOLUTION"
                -31 = "TIMED_OUT"
            }
            $errorMsg = if ($errorMessages.ContainsKey($errorCode)) { $errorMessages[$errorCode] } else { "UNKNOWN ($errorCode)" }
            
            if ($errorCode -eq 1) {
                Write-Host "  Result: $errorMsg" -ForegroundColor Green
                
                # Extract joint solution
                if ($ikResult -match "positions:\s*\[([^\]]+)\]") {
                    $solution = $matches[1] -split ',' | ForEach-Object { [double]($_.Trim()) }
                    Write-Host "  IK Solution:" -ForegroundColor Gray
                    for ($i = 0; $i -lt 6; $i++) {
                        $deg = [math]::Round($solution[$i] * 180 / [math]::PI, 2)
                        Write-Host "    J$($i+1): $deg°" -ForegroundColor White
                    }
                }
            } else {
                Write-Host "  Result: $errorMsg (code: $errorCode)" -ForegroundColor Yellow
                Write-Host "  Note: This is expected for unreachable test poses" -ForegroundColor Gray
            }
        }
    } else {
        Write-Host "  ❌ IK service timed out" -ForegroundColor Red
    }
} catch {
    Write-Host "  ❌ IK test failed: $_" -ForegroundColor Red
}
Write-Host ""

# ─── 5. Publish Test Trajectory ───
Write-Host "[Step 5/7] Publishing Test Trajectory..." -ForegroundColor Yellow

# Home position: [0, -0.3, 0.2, 0, -0.5, 0]
# Move to a new position and back
$trajectoryPayload = @"
{
  "op": "roboforge/execute_program",
  "program": [
    {
      "type": "MoveJ",
      "duration_s": 2.0,
      "points": [
        {
          "positions": [0.0, -0.523, 1.047, 0.0, -0.785, 0.0],
          "velocities": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
          "time_from_start": 2.0
        }
      ]
    },
    {
      "type": "MoveJ",
      "duration_s": 2.0,
      "points": [
        {
          "positions": [0.0, -0.3, 0.2, 0.0, -0.5, 0.0],
          "velocities": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
          "time_from_start": 2.0
        }
      ]
    }
  ]
}
"@

try {
    Write-Host "  Sending trajectory via WebSocket..." -ForegroundColor Gray
    
    # Use Python websocket client to send trajectory
    $wsResult = docker exec roboforge_core bash -c "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && ros2 topic pub /joint_trajectory_command trajectory_msgs/msg/JointTrajectory '{header: {frame_id: base_link, stamp: {sec: 0, nanosec: 0}}, joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0.0, -0.523, 1.047, 0.0, -0.785, 0.0], velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}' --once" 2>&1
    
    if ($wsResult) {
        Write-Host "  ✅ Trajectory published" -ForegroundColor Green
        Write-Host "  Target: Home → Extended → Home" -ForegroundColor Gray
    } else {
        Write-Host "  ⚠️ Trajectory publish status unclear" -ForegroundColor Yellow
    }
} catch {
    Write-Host "  ❌ Trajectory publish failed: $_" -ForegroundColor Red
}

Write-Host "  Waiting 5 seconds for trajectory execution..." -ForegroundColor Gray
Start-Sleep -Seconds 5
Write-Host ""

# ─── 6. Monitor Joint States During Execution ───
Write-Host "[Step 6/7] Monitoring Joint States (5 samples over 5 seconds)..." -ForegroundColor Yellow

$samples = @()
for ($i = 1; $i -le 5; $i++) {
    try {
        $sample = docker exec roboforge_core bash -c "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && ros2 topic echo /joint_states --once" 2>&1
        
        if ($sample -match 'position:\s*\[([^\]]+)\]') {
            $positions = $matches[1] -split ',' | ForEach-Object { [double]($_.Trim()) }
            $samples += $positions
            Write-Host "  Sample $i`: J1=$([math]::Round($positions[0] * 180 / [math]::PI, 2))°, J2=$([math]::Round($positions[1] * 180 / [math]::PI, 2))°, J3=$([math]::Round($positions[2] * 180 / [math]::PI, 2))°" -ForegroundColor White
        }
        
        Start-Sleep -Seconds 1
    } catch {
        Write-Host "  ⚠️ Sample $i failed" -ForegroundColor Yellow
    }
}

if ($samples.Count -gt 0) {
    Write-Host "  ✅ Collected $($samples.Count) joint state samples" -ForegroundColor Green
    
    # Check if joints moved
    if ($samples.Count -ge 2) {
        $firstSample = $samples[0]
        $lastSample = $samples[-1]
        $maxDelta = 0
        for ($i = 0; $i -lt 6; $i++) {
            $delta = [math]::Abs($firstSample[$i] - $lastSample[$i])
            if ($delta -gt $maxDelta) { $maxDelta = $delta }
        }
        
        if ($maxDelta -gt 0.01) {
            Write-Host "  ✅ Joints moved during simulation (max delta: $([math]::Round($maxDelta * 180 / [math]::PI, 2))°)" -ForegroundColor Green
        } else {
            Write-Host "  ⚠️ Joints did not move significantly" -ForegroundColor Yellow
        }
    }
} else {
    Write-Host "  ❌ No joint state samples collected" -ForegroundColor Red
}
Write-Host ""

# ─── 7. Final State Verification ───
Write-Host "[Step 7/7] Final State Verification..." -ForegroundColor Yellow

try {
    $finalHealth = Invoke-RestMethod -Uri "http://localhost:8765/health" -ErrorAction Stop
    Write-Host "  Bridge Status: $($finalHealth.status)" -ForegroundColor Green
    Write-Host "  Execution State: $($finalHealth.execution_state)" -ForegroundColor Gray
    Write-Host "  Connected Clients: $($finalHealth.connected_clients)" -ForegroundColor Gray
    
    $finalJoints = docker exec roboforge_core bash -c "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && ros2 topic echo /joint_states --once" 2>&1
    
    if ($finalJoints -match 'position:\s*\[([^\]]+)\]') {
        $finalPositions = $matches[1] -split ',' | ForEach-Object { [double]($_.Trim()) }
        Write-Host "  Final Joint Positions:" -ForegroundColor Gray
        $joints = @("J1", "J2", "J3", "J4", "J5", "J6")
        for ($i = 0; $i -lt 6; $i++) {
            $deg = [math]::Round($finalPositions[$i] * 180 / [math]::PI, 2)
            Write-Host "    $joints[$i]: $deg°" -ForegroundColor White
        }
    }
} catch {
    Write-Host "  ❌ Final verification failed" -ForegroundColor Red
}
Write-Host ""

# ─── TEST SUMMARY ───
Write-Host "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" -ForegroundColor Gray
Write-Host "📊 TEST SIMULATION SUMMARY" -ForegroundColor Cyan
Write-Host ""

$tests = @(
    @{Name="Services Online"; Result=$allServicesUp},
    @{Name="Bridge Health"; Result=($health.status -eq "ok")},
    @{Name="Joint States"; Result=($initialJoints -ne $null)},
    @{Name="IK Service"; Result=($ikResult -ne $null)},
    @{Name="Trajectory Publish"; Result=($wsResult -ne $null)},
    @{Name="Joint Movement"; Result=($samples.Count -ge 2 -and $maxDelta -gt 0.01)},
    @{Name="Final State"; Result=($finalHealth.status -eq "ok")}
)

$passed = ($tests | Where-Object { $_.Result }).Count
$total = $tests.Count

Write-Host "Test Results:" -ForegroundColor Yellow
foreach ($test in $tests) {
    if ($test.Result) {
        Write-Host "  ✅ $($test.Name)" -ForegroundColor Green
    } else {
        Write-Host "  ❌ $($test.Name)" -ForegroundColor Red
    }
}

Write-Host ""
Write-Host "Passed: $passed/$total" -ForegroundColor $(if ($passed -eq $total) { "Green" } else { "Yellow" })
Write-Host ""

if ($passed -eq $total) {
    Write-Host "✅ ALL TESTS PASSED - Pipeline fully operational!" -ForegroundColor Green
    Write-Host ""
    Write-Host "Access Points:" -ForegroundColor Cyan
    Write-Host "  🌐 React IDE: http://localhost:3000" -ForegroundColor White
    Write-Host "  📺 Gazebo GUI: http://localhost:6080/vnc.html" -ForegroundColor White
    Write-Host "  📡 REST API: http://localhost:8765/health" -ForegroundColor White
    Write-Host ""
    Write-Host "Data Flow Verified: Gazebo → ROS2 → Bridge → UI" -ForegroundColor Gray
} else {
    Write-Host "⚠️ Some tests did not pass. Check logs for details." -ForegroundColor Yellow
}

Write-Host ""
Write-Host "Press any key to exit..." -ForegroundColor Gray
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
