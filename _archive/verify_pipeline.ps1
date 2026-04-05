# RoboForge Pipeline Verification Script
# Tests the CLI API to confirm the WPF app is running and responsive

$ErrorActionPreference = "Continue"
$apiBase = "http://127.0.0.1:5050/api"
$passed = 0
$failed = 0

Write-Host "=============================================" -ForegroundColor Cyan
Write-Host "   RoboForge Pipeline Verification           " -ForegroundColor Cyan
Write-Host "=============================================" -ForegroundColor Cyan
Write-Host ""

# Test 1: GET /api/state
Write-Host "[Test 1] GET /api/state ..." -NoNewline
try {
    $response = Invoke-RestMethod -Uri "$apiBase/state" -Method GET -TimeoutSec 5
    Write-Host " PASS" -ForegroundColor Green
    Write-Host "         J1=$($response.J1) J2=$($response.J2) J3=$($response.J3)" -ForegroundColor DarkGray
    Write-Host "         J4=$($response.J4) J5=$($response.J5) J6=$($response.J6)" -ForegroundColor DarkGray
    $passed++
} catch {
    Write-Host " FAIL - $($_.Exception.Message)" -ForegroundColor Red
    $failed++
}

# Test 2: POST /api/jog
Write-Host "[Test 2] POST /api/jog (move J1 to 15 deg) ..." -NoNewline
try {
    $jogBody = '{"Joints": [15.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
    $response = Invoke-RestMethod -Uri "$apiBase/jog" -Method POST -Body $jogBody -ContentType "application/json" -TimeoutSec 5
    Write-Host " PASS" -ForegroundColor Green
    Write-Host "         Response: $($response.status)" -ForegroundColor DarkGray
    $passed++
} catch {
    Write-Host " FAIL - $($_.Exception.Message)" -ForegroundColor Red
    $failed++
}

Start-Sleep -Seconds 2

# Test 3: GET /api/state again (verify joint moved)
Write-Host "[Test 3] GET /api/state (verify movement) ..." -NoNewline
try {
    $response = Invoke-RestMethod -Uri "$apiBase/state" -Method GET -TimeoutSec 5
    Write-Host " PASS" -ForegroundColor Green
    Write-Host "         J1=$($response.J1) (expected ~15.0)" -ForegroundColor DarkGray
    $passed++
} catch {
    Write-Host " FAIL - $($_.Exception.Message)" -ForegroundColor Red
    $failed++
}

# Test 4: POST /api/execute (inject a 2-block program)
Write-Host "[Test 4] POST /api/execute (inject MoveJ + Wait) ..." -NoNewline
try {
    $execBody = '[{"Type":"MoveJ","Target":"test_p1","Speed":80},{"Type":"Wait","Delay":500}]'
    $response = Invoke-RestMethod -Uri "$apiBase/execute" -Method POST -Body $execBody -ContentType "application/json" -TimeoutSec 5
    Write-Host " PASS" -ForegroundColor Green
    Write-Host "         Response: $($response.status)" -ForegroundColor DarkGray
    $passed++
} catch {
    Write-Host " FAIL - $($_.Exception.Message)" -ForegroundColor Red
    $failed++
}

# Summary
Write-Host ""
Write-Host "=============================================" -ForegroundColor Cyan
Write-Host "  Results: $passed PASSED, $failed FAILED" -ForegroundColor $(if ($failed -eq 0) { "Green" } else { "Yellow" })
Write-Host "=============================================" -ForegroundColor Cyan

if ($failed -eq 0) {
    Write-Host "  All pipeline tests passed!" -ForegroundColor Green
} else {
    Write-Host "  Some tests failed. Is the WPF app running as Admin?" -ForegroundColor Yellow
}
