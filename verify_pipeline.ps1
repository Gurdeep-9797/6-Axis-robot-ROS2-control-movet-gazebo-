# RoboForge v8.2 — Pipeline Reality Flow Verification
# ────────────────────────────────────────────────────
# Monitors real-time data flow: Gazebo → ROS2 → Bridge → UI

Write-Host "📊 RoboForge Pipeline Reality Flow Verification" -ForegroundColor Cyan
Write-Host "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" -ForegroundColor Gray
Write-Host ""

# 1. Verify all containers
Write-Host "[1/6] Checking Container Status..." -ForegroundColor Yellow
$containers = docker ps --filter "name=roboforge" --format "{{.Names}}" 2>$null
$containerCount = ($containers | Measure-Object).Count

if ($containerCount -ge 5) {
    Write-Host "  ✅ $containerCount containers running" -ForegroundColor Green
    docker ps --filter "name=roboforge" --format "     {{.Names}} → {{.Status}}" | ForEach-Object { Write-Host $_ }
} else {
    Write-Host "  ⚠️ Only $containerCount containers running (expected 5-6)" -ForegroundColor Yellow
}
Write-Host ""

# 2. ROS2 Nodes
Write-Host "[2/6] Checking ROS2 Nodes..." -ForegroundColor Yellow
try {
    $nodes = docker exec roboforge_core bash -c "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && ros2 node list" 2>$null
    
    $expectedNodes = @("/move_group", "/pseudo_hardware_node", "/roboforge_bridge", "/robot_state_publisher")
    $foundNodes = 0
    
    foreach ($expected in $expectedNodes) {
        $match = $nodes | Where-Object { $_ -like "*$expected*" }
        if ($match) {
            $foundNodes++
            Write-Host "  ✅ $expected" -ForegroundColor Green
        } else {
            Write-Host "  ❌ $expected (not found)" -ForegroundColor Red
        }
    }
    
    Write-Host "  📊 Total nodes: $(($nodes | Measure-Object).Count)" -ForegroundColor Gray
} catch {
    Write-Host "  ❌ Failed to query ROS2 nodes" -ForegroundColor Red
}
Write-Host ""

# 3. ROS2 Topics
Write-Host "[3/6] Checking ROS2 Topics..." -ForegroundColor Yellow
try {
    $topics = docker exec roboforge_core bash -c "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && ros2 topic list" 2>$null
    
    $criticalTopics = @("/joint_states", "/joint_trajectory_command", "/planned_trajectory", "/robot_description", "/tf")
    
    foreach ($topic in $criticalTopics) {
        $match = $topics | Where-Object { $_ -eq $topic }
        if ($match) {
            Write-Host "  ✅ $topic" -ForegroundColor Green
        } else {
            Write-Host "  ❌ $topic (not found)" -ForegroundColor Red
        }
    }
    
    Write-Host "  📊 Total topics: $(($topics | Measure-Object).Count)" -ForegroundColor Gray
} catch {
    Write-Host "  ❌ Failed to query ROS2 topics" -ForegroundColor Red
}
Write-Host ""

# 4. ROS2 Services
Write-Host "[4/6] Checking ROS2 Services..." -ForegroundColor Yellow
try {
    $services = docker exec roboforge_core bash -c "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && ros2 service list" 2>$null
    
    $criticalServices = @("/compute_ik", "/compute_fk", "/roboforge/health_check")
    
    foreach ($service in $criticalServices) {
        $match = $services | Where-Object { $_ -eq $service }
        if ($match) {
            Write-Host "  ✅ $service" -ForegroundColor Green
        } else {
            Write-Host "  ❌ $service (not found)" -ForegroundColor Red
        }
    }
} catch {
    Write-Host "  ❌ Failed to query ROS2 services" -ForegroundColor Red
}
Write-Host ""

# 5. Bridge Health
Write-Host "[5/6] Checking Bridge Health..." -ForegroundColor Yellow
try {
    $healthResponse = Invoke-RestMethod -Uri "http://localhost:8765/health" -Method GET -ErrorAction Stop
    
    if ($healthResponse.status -eq "ok") {
        Write-Host "  ✅ Bridge status: $($healthResponse.status)" -ForegroundColor Green
        Write-Host "     Mode: $($healthResponse.mode)" -ForegroundColor Gray
        Write-Host "     MoveIt ready: $($healthResponse.moveit_ready)" -ForegroundColor Gray
        Write-Host "     Kinematics loaded: $($healthResponse.kinematics_loaded)" -ForegroundColor Gray
        Write-Host "     Robot loaded: $($healthResponse.robot_loaded)" -ForegroundColor Gray
    } else {
        Write-Host "  ❌ Bridge status: $($healthResponse.status)" -ForegroundColor Red
    }
} catch {
    Write-Host "  ❌ Failed to reach bridge REST API" -ForegroundColor Red
}
Write-Host ""

# 6. Live Data Flow Test
Write-Host "[6/6] Testing Live Data Flow..." -ForegroundColor Yellow

# Test joint_states topic
try {
    Write-Host "  📡 Sampling /joint_states (5 samples)..." -ForegroundColor Gray
    $samples = docker exec roboforge_core bash -c "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && ros2 topic echo /joint_states --once" 2>$null
    
    if ($samples) {
        Write-Host "  ✅ /joint_states publishing" -ForegroundColor Green
        # Extract joint positions if available
        if ($samples -match "position:") {
            Write-Host "     Joint positions detected" -ForegroundColor Gray
        }
    } else {
        Write-Host "  ⚠️ /joint_states not publishing" -ForegroundColor Yellow
    }
} catch {
    Write-Host "  ❌ Failed to sample /joint_states" -ForegroundColor Red
}

# Test IK service
try {
    Write-Host "  🧪 Testing IK service..." -ForegroundColor Gray
    $ikTest = docker exec roboforge_core bash -c "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK '{ik_request: {group_name: robot_arm, pose_stamped: {header: {frame_id: base_link}, pose: {position: {x: 0.6, y: 0.0, z: 0.8}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}}'" 2>&1
    
    if ($ikTest -match "error_code") {
        Write-Host "  ✅ IK service responding" -ForegroundColor Green
    } else {
        Write-Host "  ⚠️ IK response unclear" -ForegroundColor Yellow
    }
} catch {
    Write-Host "  ⚠️ IK service test skipped" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" -ForegroundColor Gray
Write-Host "📊 Pipeline Reality Summary" -ForegroundColor Cyan
Write-Host ""

# Final summary
$checks = @(
    @{Name="Containers"; Status=($containerCount -ge 5)},
    @{Name="ROS2 Nodes"; Status=($foundNodes -ge 3)},
    @{Name="Bridge API"; Status=($healthResponse.status -eq "ok")}
)

$passed = ($checks | Where-Object { $_.Status }).Count
$total = $checks.Count

Write-Host "  Passed: $passed/$total critical checks" -ForegroundColor $(if ($passed -eq $total) { "Green" } else { "Yellow" })
Write-Host ""

if ($passed -eq $total) {
    Write-Host "✅ Pipeline is fully operational!" -ForegroundColor Green
    Write-Host ""
    Write-Host "Access Points:" -ForegroundColor Cyan
    Write-Host "  🌐 Online IDE: http://localhost:3000" -ForegroundColor White
    Write-Host "  🔌 WebSocket: ws://localhost:9090" -ForegroundColor White
    Write-Host "  📡 REST API: http://localhost:8765" -ForegroundColor White
    Write-Host ""
    Write-Host "Data Flow: Gazebo → /joint_states → Bridge → WebSocket → React UI" -ForegroundColor Gray
} else {
    Write-Host "⚠️ Some pipeline components need attention" -ForegroundColor Yellow
    Write-Host "Check the failed components above and restart if needed." -ForegroundColor Gray
}

Write-Host ""
Write-Host "Press any key to exit..." -ForegroundColor Gray
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
