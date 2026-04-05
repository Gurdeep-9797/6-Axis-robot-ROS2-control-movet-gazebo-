# RoboForge v8.0 — WPF Build Script
# ───────────────────────────────

Write-Host "🔨 Building RoboForge WPF Offline Client..." -ForegroundColor Cyan

# 1. Check for .NET SDK
if (!(Get-Command dotnet -ErrorAction SilentlyContinue)) {
    Write-Error ".NET SDK not found. Please run 'tools/install_vs_tools.ps1' first."
    exit
}

# 2. Restore & Build
Write-Host "📦 Restoring NuGet packages..." -ForegroundColor Gray
dotnet restore src/RoboForge.Wpf/RoboForge.Wpf.csproj

Write-Host "🏗 Compiling Project (Debug)..." -ForegroundColor Yellow
dotnet build src/RoboForge.Wpf/RoboForge.Wpf.csproj --configuration Debug --no-restore

if ($LASTEXITCODE -eq 0) {
    Write-Host "✅ Build Succeeded!" -ForegroundColor Green
    $exePath = Resolve-Path "src/RoboForge.Wpf/bin/Debug/net8.0-windows/RoboForge.Wpf.exe"
    Write-Host "🚀 Executable: $exePath" -ForegroundColor Cyan
} else {
    Write-Error "❌ Build Failed. Check the errors above."
}
