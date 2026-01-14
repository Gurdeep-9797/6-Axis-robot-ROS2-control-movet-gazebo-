$ErrorActionPreference = "Stop"
Write-Host "=== AUTOMATED LOGIC TEST RUNNER ==="

Try {
    # 1. Start Sim
    Write-Host "1. Starting System (SIM Mode)..."
    ./scripts/start_sim.ps1
    
    # Wait for warmup
    Write-Host "   Waiting 30s for system warmup..."
    Start-Sleep -Seconds 30
    
    # 2. Run Automated Logic Test
    Write-Host "`n2. Running Automated Logic Tests..."
    docker exec robot_bridge python3 /ros_ws/scripts/automated_logic_test.py
    
    # 3. Cleanup
    Write-Host "`n3. Cleaning up..."
    ./scripts/stop_all.ps1
    
    Write-Host "`n=== TEST RUN COMPLETE ==="
}
Catch {
    Write-Host "`n!!! TEST RUN FAILED !!!" -ForegroundColor Red
    Write-Host $_.Exception.Message
    ./scripts/stop_all.ps1
    exit 1
}
