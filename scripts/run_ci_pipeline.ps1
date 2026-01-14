$ErrorActionPreference = "Stop"
Write-Host "=== FULL CI PIPELINE ==="

$failures = 0

# Stage 1: Audit MoveIt Config
Write-Host "`n[STAGE 1] MoveIt Config Audit"
python tools/audit_moveit_config.py
if ($LASTEXITCODE -ne 0) { $failures++ }

# Stage 2: IK Solver Selection
Write-Host "`n[STAGE 2] IK Solver Selection"
python tools/select_ik_solver.py
if ($LASTEXITCODE -ne 0) { $failures++ }

# Summary
Write-Host "`n=== PIPELINE SUMMARY ==="
if ($failures -eq 0) {
    Write-Host "ALL STAGES PASSED" -ForegroundColor Green
    exit 0
}
else {
    Write-Host "$failures STAGE(S) FAILED" -ForegroundColor Red
    exit 1
}
