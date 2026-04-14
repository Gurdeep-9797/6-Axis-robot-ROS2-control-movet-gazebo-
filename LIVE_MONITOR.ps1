# RoboForge Live Data Flow Monitor
# ─────────────────────────────
# Real-time monitoring of program execution, Gazebo data, and data flow

param(
    [int]$UpdateIntervalMs = 500  # Update every 500ms
)

# Clear screen
$Host.UI.RawUI.WindowTitle = "RoboForge — Live Data Flow Monitor"
Clear-Host

Write-Host "╔══════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║        RoboForge Live Data Flow Monitor                 ║" -ForegroundColor Cyan
Write-Host "║        Real-time program execution & Gazebo data       ║" -ForegroundColor Cyan
Write-Host "╚══════════════════════════════════════════════════════════╝" -ForegroundColor Cyan
Write-Host ""

# Check if backend is running
$bridgeOnline = $false
try {
    $response = Invoke-RestMethod -Uri "http://localhost:8765/health" -ErrorAction Stop
    $bridgeOnline = $true
    Write-Host "  ✅ ROS2 Bridge: Connected (ws://localhost:9090)" -ForegroundColor Green
    Write-Host "     MoveIt Ready: $($response.moveit_ready)" -ForegroundColor Gray
    Write-Host "     Mode: $($response.mode)" -ForegroundColor Gray
    Write-Host "     Execution State: $($response.execution_state)" -ForegroundColor Gray
} catch {
    Write-Host "  ❌ ROS2 Bridge: Offline" -ForegroundColor Red
}

# Check Docker containers
$containers = docker ps --filter "name=roboforge" --format "{{.Names}}" 2>$null
$containerCount = ($containers | Measure-Object).Count
Write-Host "  ✅ Docker Containers: $containerCount running" -ForegroundColor Green

Write-Host ""
Write-Host "Press Ctrl+C to exit" -ForegroundColor DarkGray
Write-Host "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" -ForegroundColor Gray
Write-Host ""

# Main monitoring loop
$iteration = 0
while ($true) {
    $iteration++
    Clear-Host
    
    Write-Host "╔══════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
    Write-Host "║        RoboForge Live Data Flow Monitor                 ║" -ForegroundColor Cyan
    Write-Host "║        Updated: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')          ║" -ForegroundColor Cyan
    Write-Host "╚══════════════════════════════════════════════════════════╝" -ForegroundColor Cyan
    Write-Host ""

    # ─── 1. BACKEND STATUS ─────────────────────────────────────────────
    Write-Host "📡 BACKEND STATUS" -ForegroundColor Yellow
    Write-Host "──────────────────────────────────────────────────────────" -ForegroundColor Gray
    
    try {
        $health = Invoke-RestMethod -Uri "http://localhost:8765/health" -ErrorAction Stop
        Write-Host "  Bridge Status:  " -NoNewline
        Write-Host "✅ $($health.status.ToUpper())" -ForegroundColor Green
        Write-Host "  Mode:           " -NoNewline
        Write-Host "$($health.mode)" -ForegroundColor Cyan
        Write-Host "  MoveIt Ready:   " -NoNewline
        Write-Host "$($health.moveit_ready)" -ForegroundColor $(if ($health.moveit_ready) { "Green" } else { "Red" })
        Write-Host "  Kinematics:     " -NoNewline
        Write-Host "$($health.kinematics_loaded)" -ForegroundColor $(if ($health.kinematics_loaded) { "Green" } else { "Red" })
        Write-Host "  Connected:      " -NoNewline
        Write-Host "$($health.connected_clients) client(s)" -ForegroundColor Gray
    } catch {
        Write-Host "  ❌ Bridge unreachable" -ForegroundColor Red
    }
    Write-Host ""

    # ─── 2. DOCKER CONTAINERS ─────────────────────────────────────────
    Write-Host "🐳 DOCKER CONTAINERS" -ForegroundColor Yellow
    Write-Host "──────────────────────────────────────────────────────────" -ForegroundColor Gray
    
    docker ps --filter "name=roboforge" --format "  {{.Names}} → {{.Status}}" | ForEach-Object {
        if ($_ -match "healthy") {
            Write-Host $_ -ForegroundColor Green
        } else {
            Write-Host $_ -ForegroundColor White
        }
    }
    Write-Host ""

    # ─── 3. LIVE JOINT STATES FROM GAZEBO ─────────────────────────────
    Write-Host "🦾 LIVE JOINT STATES (Gazebo → ROS2 → Bridge)" -ForegroundColor Yellow
    Write-Host "──────────────────────────────────────────────────────────" -ForegroundColor Gray
    
    try {
        $jointData = docker exec roboforge_core bash -c "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && ros2 topic echo /joint_states --once" 2>&1
        
        if ($jointData -match 'position:\s*\n-\s*([-\d.]+)\s*\n-\s*([-\d.]+)\s*\n-\s*([-\d.]+)\s*\n-\s*([-\d.]+)\s*\n-\s*([-\d.]+)\s*\n-\s*([-\d.]+)') {
            $positions = @([double]$matches[1], [double]$matches[2], [double]$matches[3], 
                          [double]$matches[4], [double]$matches[5], [double]$matches[6])
            $joints = @("J1", "J2", "J3", "J4", "J5", "J6")
            
            for ($i = 0; $i -lt 6; $i++) {
                $rad = $positions[$i]
                $deg = [math]::Round($rad * 180 / [math]::PI, 2)
                $bar = "#" * ([math]::Abs([int]($deg / 5)))
                Write-Host "  $($joints[$i]): " -NoNewline
                Write-Host "$deg° " -NoNewline -ForegroundColor Cyan
                Write-Host "($rad rad)" -NoNewline -ForegroundColor DarkGray
                Write-Host " $bar" -ForegroundColor Yellow
            }
        }
    } catch {
        Write-Host "  ⚠️ Unable to read joint states" -ForegroundColor Yellow
    }
    Write-Host ""

    # ─── 4. ROS2 TOPICS ACTIVITY ──────────────────────────────────────
    Write-Host "📊 ROS2 TOPIC ACTIVITY" -ForegroundColor Yellow
    Write-Host "──────────────────────────────────────────────────────────" -ForegroundColor Gray
    
    try {
        $topics = docker exec roboforge_core bash -c "source /opt/ros/humble/setup.bash && ros2 topic list" 2>&1
        $criticalTopics = @("/joint_states", "/joint_trajectory_command", "/planned_trajectory", "/robot_description", "/tf")
        
        foreach ($topic in $criticalTopics) {
            $found = $topics -match $topic
            if ($found) {
                Write-Host "  ✅ $topic" -ForegroundColor Green
            } else {
                Write-Host "  ❌ $topic" -ForegroundColor Red
            }
        }
    } catch {
        Write-Host "  ⚠️ Unable to query topics" -ForegroundColor Yellow
    }
    Write-Host ""

    # ─── 5. DATA FLOW DIAGRAM ─────────────────────────────────────────
    Write-Host "🔄 LIVE DATA FLOW" -ForegroundColor Yellow
    Write-Host "──────────────────────────────────────────────────────────" -ForegroundColor Gray
    
    Write-Host ""
    Write-Host "  ┌─────────────────┐" -ForegroundColor DarkGray
    Write-Host "  │  GAZEBO SIM     │  ← 3D Physics Engine" -ForegroundColor DarkGray
    Write-Host "  │  (Docker)       │  ← Publishing /joint_states @ 250Hz" -ForegroundColor DarkGray
    Write-Host "  └────────┬────────┘" -ForegroundColor DarkGray
    Write-Host "           │" -ForegroundColor DarkGray
    Write-Host "           ▼" -ForegroundColor DarkGray
    Write-Host "  ┌─────────────────┐" -ForegroundColor DarkGray
    Write-Host "  │  ROS2 TOPICS    │  ← /joint_states, /tf, /planned_trajectory" -ForegroundColor DarkGray
    Write-Host "  │  (Middleware)   │  ← 20 active topics" -ForegroundColor DarkGray
    Write-Host "  └────────┬────────┘" -ForegroundColor DarkGray
    Write-Host "           │" -ForegroundColor DarkGray
    Write-Host "           ▼" -ForegroundColor DarkGray
    Write-Host "  ┌─────────────────┐" -ForegroundColor DarkGray
    Write-Host "  │  ROBOFORGE      │  ← WebSocket 9090 + REST 8765" -ForegroundColor DarkGray
    Write-Host "  │  BRIDGE         │  ← Parsing joint states, calling IK/FK" -ForegroundColor DarkGray
    Write-Host "  └────────┬────────┘" -ForegroundColor DarkGray
    Write-Host "           │" -ForegroundColor DarkGray
    Write-Host "           ▼" -ForegroundColor DarkGray
    Write-Host "  ┌─────────────────────────────┐" -ForegroundColor Cyan
    Write-Host "  │  REACT ONLINE IDE           │  ← http://localhost:3000" -ForegroundColor Cyan
    Write-Host "  │  (Browser)                  │  ← Receiving joint states, updating 3D" -ForegroundColor Cyan
    Write-Host "  └─────────────────────────────┘" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "  ┌─────────────────────────────┐" -ForegroundColor Magenta
    Write-Host "  │  WPF OFFLINE CLIENT         │  ← Ros2BridgeClient" -ForegroundColor Magenta
    Write-Host "  │  (Desktop .NET 8)           │  ← Subscribing to Gazebo data" -ForegroundColor Magenta
    Write-Host "  └─────────────────────────────┘" -ForegroundColor Magenta
    Write-Host ""

    # ─── 6. QUICK COMMANDS ────────────────────────────────────────────
    Write-Host "⌨️  QUICK COMMANDS" -ForegroundColor Yellow
    Write-Host "──────────────────────────────────────────────────────────" -ForegroundColor Gray
    Write-Host "  View Gazebo GUI:    " -NoNewline
    Write-Host "http://localhost:6080/vnc.html" -ForegroundColor Cyan
    Write-Host "  View React IDE:     " -NoNewline
    Write-Host "http://localhost:3000" -ForegroundColor Cyan
    Write-Host "  Run WPF Client:     " -NoNewline
    Write-Host "dotnet run --project src/RoboForge.Wpf/RoboForge.Wpf.csproj" -ForegroundColor Magenta
    Write-Host "  View Bridge Logs:   " -NoNewline
    Write-Host "docker logs 9f5db7628868_roboforge_bridge -f" -ForegroundColor DarkGray
    Write-Host ""

    Write-Host "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" -ForegroundColor Gray
    Write-Host "Next update in $($UpdateIntervalMs)ms... (Ctrl+C to exit)" -ForegroundColor DarkGray
    
    Start-Sleep -Milliseconds $UpdateIntervalMs
}
