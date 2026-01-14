# Stop Scripts (PowerShell)

Write-Host "Stopping all robot containers..." -ForegroundColor Yellow

docker compose -f docker-compose.yml -f docker-compose.sim.yml -f docker-compose.real.yml down

Write-Host "System stopped." -ForegroundColor Green
