# RoboForge v8.2 — Real-Time Pipeline Monitoring Dashboard
# ────────────────────────────────────────────────────────
# Monitors Gazebo → ROS2 → Bridge → UI data flow in real-time

param(
    [int]$Interval = 5  # Update interval in seconds
)

Write-Host "📊 RoboForge Real-Time Pipeline Monitor" -ForegroundColor Cyan
Write-Host "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" -ForegroundColor Gray
Write-Host "Press Ctrl+C to stop monitoring" -ForegroundColor Yellow
Write-Host ""

# Function to clear screen and move cursor to top
function Clear-Display {
    Clear-Host
}

# Main monitoring loop
while ($true) {
    Clear-Display
    
    Write-Host "📊 RoboForge Pipeline Reality Dashboard" -ForegroundColor Cyan
    Write-Host "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" -ForegroundColor Gray
    Write-Host "Updated: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')" -ForegroundColor Gray
    Write-Host ""

    # ─── 1. Container Status ───
    Write-Host "📦 CONTAINER STATUS" -ForegroundColor Yellow
    Write-Host "─────────────────────────────────────────────────" -ForegroundColor Gray
    
    $containers = docker ps --filter "name=roboforge" --format "{{.Names}}|{{.Status}}|{{.Ports}}" 2>$null
    
    foreach ($container in $containers) {
        $parts = $container -split '\|'
        $name = $parts[0]
        $status = $parts[1]
        $ports = $parts[2]
        
        # Color code based on status
        if ($status -match "healthy") {
            Write-Host "  ✅ $name" -ForegroundColor Green
        } elseif ($status -match "Up") {
            Write-Host "  ✅ $name" -ForegroundColor Green
        } else {
            Write-Host "  ❌ $name - $status" -ForegroundColor Red
        }
        
        if ($ports) {
            Write-Host "     Ports: $ports" -ForegroundColor DarkGray
        }
    }
    Write-Host ""

    # ─── 2. ROS2 Topic Activity ───
    Write-Host "📡 ROS2 TOPIC ACTIVITY" -ForegroundColor Yellow
    Write-Host "─────────────────────────────────────────────────" -ForegroundColor Gray
    
    $criticalTopics = @(
        @{Name="/joint_states"; Desc="Joint positions (250Hz)"},
        @{Name="/joint_trajectory_command"; Desc="Trajectory commands"},
        @{Name="/planned_trajectory"; Desc="MoveIt planned paths"},
        @{Name="/robot_description"; Desc="URDF model"},
        @{Name="/tf"; Desc="Transform frames"}
    )
    
    try {
        $topics = docker exec roboforge_core bash -c "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && ros2 topic list" 2>$null
        
        foreach ($topic in $criticalTopics) {
            $match = $topics | Where-Object { $_ -eq $topic.Name }
            if ($match) {
                Write-Host "  ✅ $($topic.Name)" -ForegroundColor Green
                Write-Host "     $($topic.Desc)" -ForegroundColor DarkGray
            } else {
                Write-Host "  ❌ $($topic.Name)" -ForegroundColor Red
            }
        }
    } catch {
        Write-Host "  ⚠️ Unable to query topics" -ForegroundColor Yellow
    }
    Write-Host ""

    # ─── 3. Bridge Health ───
    Write-Host "🌉 BRIDGE HEALTH" -ForegroundColor Yellow
    Write-Host "─────────────────────────────────────────────────" -ForegroundColor Gray
    
    try {
        $healthResponse = Invoke-RestMethod -Uri "http://localhost:8765/health" -Method GET -ErrorAction Stop
        
        Write-Host "  Status: $($healthResponse.status)" -ForegroundColor $(if ($healthResponse.status -eq "ok") { "Green" } else { "Red" })
        Write-Host "  Mode: $($healthResponse.mode)" -ForegroundColor Gray
        Write-Host "  MoveIt Ready: $($healthResponse.moveit_ready)" -ForegroundColor $(if ($healthResponse.moveit_ready) { "Green" } else { "Red" })
        Write-Host "  Kinematics: $($healthResponse.kinematics_loaded)" -ForegroundColor $(if ($healthResponse.kinematics_loaded) { "Green" } else { "Red" })
        Write-Host "  Connected Clients: $($healthResponse.connected_clients)" -ForegroundColor Gray
    } catch {
        Write-Host "  ❌ Bridge unreachable" -ForegroundColor Red
    }
    Write-Host ""

    # ─── 4. Live Joint States ───
    Write-Host "🦾 LIVE JOINT STATES (Latest)" -ForegroundColor Yellow
    Write-Host "─────────────────────────────────────────────────" -ForegroundColor Gray
    
    try {
        # Sample joint states (one shot)
        $jointData = docker exec roboforge_core bash -c "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && ros2 topic echo /joint_states --once" 2>$null
        
        if ($jointData) {
            # Parse joint positions
            $positions = $jointData | Select-String "position:" -Context 0, 6
            if ($positions) {
                $joints = @("J1", "J2", "J3", "J4", "J5", "J6")
                $positionValues = ($positions -split ',' | Where-Object { $_ -match '[-\d.]+' } | ForEach-Object { [double]($_ -replace '[^-\d.]', '') })
                
                for ($i = 0; $i -lt 6; $i++) {
                    if ($i -lt $positionValues.Count) {
                        $jointName = $joints[$i]
                        $pos = $positionValues[$i]
                        $posDeg = [math]::Round($pos * 180 / [math]::PI, 2)
                        Write-Host "  $jointName`: $posDeg° ($pos rad)" -ForegroundColor White
                    }
                }
            }
        } else {
            Write-Host "  ⚠️ No joint state data" -ForegroundColor Yellow
        }
    } catch {
        Write-Host "  ⚠️ Unable to sample joint states" -ForegroundColor Yellow
    }
    Write-Host ""

    # ─── 5. Access URLs ───
    Write-Host "🌐 ACCESS POINTS" -ForegroundColor Yellow
    Write-Host "─────────────────────────────────────────────────" -ForegroundColor Gray
    Write-Host "  React Online IDE: http://localhost:3000" -ForegroundColor Cyan
    Write-Host "  Gazebo GUI (VNC): http://localhost:6080/vnc.html" -ForegroundColor Cyan
    Write-Host "  Bridge REST API: http://localhost:8765/health" -ForegroundColor Cyan
    Write-Host "  WebSocket: ws://localhost:9090" -ForegroundColor Cyan
    Write-Host ""

    # ─── 6. Data Flow Diagram ───
    Write-Host "📊 DATA FLOW PIPELINE" -ForegroundColor Yellow
    Write-Host "─────────────────────────────────────────────────" -ForegroundColor Gray
    Write-Host "  [GAZEBO GUI] ←→ [ROS2 Topics] ←→ [Bridge] ←→ [React UI]" -ForegroundColor White
    Write-Host "       ↓                ↓              ↓            ↓" -ForegroundColor DarkGray
    Write-Host "    3D View       /joint_states    WebSocket    3D Viewport" -ForegroundColor DarkGray
    Write-Host "    Physics       /tf              REST API     Joint States" -ForegroundColor DarkGray
    Write-Host "    Simulation    /planned_traj    /compute_ik  IK Results" -ForegroundColor DarkGray
    Write-Host ""

    # ─── Footer ───
    Write-Host "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" -ForegroundColor Gray
    Write-Host "Next update in $Interval seconds..." -ForegroundColor DarkGray
    
    # Countdown
    for ($i = $Interval; $i -gt 0; $i--) {
        Write-Host "`rNext update in $i seconds...   " -NoNewline -ForegroundColor DarkGray
        Start-Sleep -Seconds 1
    }
}
