# RoboForge v8.0 — Master Launch Script (Windows)
# ───────────────────────────────────────────────

Write-Host "🤖 Starting RoboForge v8.0 Unified Stack..." -ForegroundColor Cyan

# 1. Check for Docker
if (!(Get-Command docker -ErrorAction SilentlyContinue)) {
    Write-Error "Docker not found. Please install Docker Desktop to proceed."
    exit
}

# 2. Cleanup old containers
Write-Host "🧹 Cleaning up previous instances..." -ForegroundColor Gray
docker compose down --remove-orphans

# 3. Build & Launch
Write-Host "🏗 Building and starting services (this may take a minute)..." -ForegroundColor Yellow
docker compose up --build -d

# 4. Wait for Frontend
Write-Host "⏳ Waiting for UI to become alive..." -ForegroundColor Gray
$maxRetries = 30
$retryCount = 0
$url = "http://localhost:3000"

while ($retryCount -lt $maxRetries) {
    try {
        $response = Invoke-WebRequest -Uri $url -UseBasicParsing -ErrorAction SilentlyContinue
        if ($response.StatusCode -eq 200) {
            Write-Host "✅ RoboForge UI is LIVE at $url" -ForegroundColor Green
            break
        }
    } catch {
        # Continue waiting
    }
    Start-Sleep -Seconds 2
    $retryCount++
    Write-Host "  . " -NoNewline
}

if ($retryCount -eq $maxRetries) {
    Write-Host "⚠️ UI startup is taking longer than expected. Check 'docker logs roboforge_frontend'." -ForegroundColor Red
} else {
    # 5. Open Browser
    Write-Host "🌐 Opening browser..." -ForegroundColor Cyan
    Start-Process $url
}

Write-Host "🚀 RoboForge is ready for Operation!" -ForegroundColor Green
