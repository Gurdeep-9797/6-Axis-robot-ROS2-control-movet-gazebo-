$ErrorActionPreference = "Stop"
Write-Host "=== PHASE 1 VERIFICATION START ==="

Try {
    Write-Host "1. Checking Docker Connectivity..."
    docker version --format 'Docker Server Version: {{.Server.Version}}'
    
    Write-Host "`n2. Checking Running Containers..."
    docker ps --format "table {{.Names}}\t{{.Status}}"
    
    Write-Host "`n3. Checking Bridge Log for Initialization..."
    docker logs --tail 20 robot_bridge
    
    Write-Host "`n4. OPTIONAL: Ensuring Scripts availability..."
    # We can't easily check internal FS without exec, but exec is the next step.
    
    Write-Host "`n5. EXECUTING ROS TASK NODE (Verification)..."
    # We use 'docker exec' to run the script inside the container
    docker exec robot_bridge python3 /ros_ws/scripts/ros_task_node.py
    
    Write-Host "`n=== VERIFICATION SUCCEEDED ==="
}
Catch {
    Write-Host "`n!!! VERIFICATION FAILED !!!" -ForegroundColor Red
    Write-Host $_.Exception.Message
    Write-Host $_.ScriptStackTrace
    exit 1
}
