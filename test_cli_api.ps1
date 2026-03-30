$ErrorActionPreference = "Stop"
Write-Host "Testing RoboForge WPF CLI API..." -ForegroundColor Cyan

# 1. Test GET /api/state
Write-Host "Fetching Robot State..."
$state = Invoke-RestMethod -Uri "http://127.0.0.1:5050/api/state" -Method Get
Write-Host "State:" $state

# 2. Test POST /api/jog
Write-Host "Sending Jog Command..."
$jogBody = @{
    Joints = @(10.0, 20.0, -30.0, 45.0, 0.0, 90.0)
} | ConvertTo-Json
$jogResponse = Invoke-RestMethod -Uri "http://127.0.0.1:5050/api/jog" -Method Post -Body $jogBody -ContentType "application/json"
Write-Host "Jog Response:" $jogResponse

# 3. Test POST /api/execute
Write-Host "Injecting Script Block List..."
$scriptBody = @(
    @{ Type = "MoveJ"; Target = "WP_Pick"; Speed = 100 },
    @{ Type = "SetDO"; Port = 1; Value = 1 },
    @{ Type = "Wait"; Delay = 1500 },
    @{ Type = "MoveL"; Target = "WP_Place"; Speed = 50 }
) | ConvertTo-Json
$scriptResponse = Invoke-RestMethod -Uri "http://127.0.0.1:5050/api/execute" -Method Post -Body $scriptBody -ContentType "application/json"
Write-Host "Script Execution Response:" $scriptResponse

Write-Host "All API Tests Passed!" -ForegroundColor Green
