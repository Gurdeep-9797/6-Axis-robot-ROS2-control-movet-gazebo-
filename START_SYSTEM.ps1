<#
.SYNOPSIS
    One-click setup and launcher for the C++ Robot Simulator.
    Handles vcpkg dependencies, CMake configuration, and Docker integration.
#>

$ErrorActionPreference = "Stop"
$ProjectRoot = (Get-Item -Path ".\").FullName
$VcpkgPath = "$ProjectRoot\tools\vcpkg"
$BuildDir = "$ProjectRoot\RobotSimulator_CPP\build"
$ExePath = "$BuildDir\Release\RobotSimulator_CPP.exe"

Write-Host "=== 6-AXIS C++ SIMULATOR LAUNCHER ===" -ForegroundColor Cyan

# 1. Check vcpkg
if (-not (Test-Path "$VcpkgPath\vcpkg.exe")) {
    Write-Host "[1/4] Installing vcpkg..." -ForegroundColor Yellow
    if (-not (Test-Path "$VcpkgPath")) {
        git clone https://github.com/microsoft/vcpkg.git "$VcpkgPath"
    }
    Start-Process -FilePath "$VcpkgPath\bootstrap-vcpkg.bat" -Wait -NoNewWindow
}
else {
    Write-Host "[1/4] vcpkg found." -ForegroundColor Green
}

# 1.5 Find CMake
$cmakePath = "cmake"
if (-not (Get-Command "cmake" -ErrorAction SilentlyContinue)) {
    $vsCmake = "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
    if (Test-Path $vsCmake) {
        $cmakePath = $vsCmake
        Write-Host "CMake found in Visual Studio." -ForegroundColor Cyan
    }
    else {
        Write-Host "Error: CMake not found in PATH or standard VS location." -ForegroundColor Red
        Write-Host "Please install CMake or add it to your PATH."
        exit 1
    }
}

# 2. Build C++ Project
Write-Host "`n[2/4] Building Simulator..." -ForegroundColor Yellow
if (-not (Test-Path $BuildDir)) { New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null }

Push-Location "$ProjectRoot\RobotSimulator_CPP"
try {
    # Configure
    Write-Host "Configuring..." -ForegroundColor Gray
    $configArgs = "-S . -B build -DCMAKE_TOOLCHAIN_FILE=`"$VcpkgPath\scripts\buildsystems\vcpkg.cmake`" -DVCPKG_TARGET_TRIPLET=x64-windows"
    $procConfig = Start-Process -FilePath $cmakePath -ArgumentList $configArgs -PassThru -Wait -NoNewWindow
    if ($procConfig.ExitCode -ne 0) { throw "CMake Configuration failed with exit code $($procConfig.ExitCode)" }
    
    # Build
    Write-Host "Building..." -ForegroundColor Gray
    $buildArgs = "--build build --config Release"
    $procBuild = Start-Process -FilePath $cmakePath -ArgumentList $buildArgs -PassThru -Wait -NoNewWindow
    if ($procBuild.ExitCode -ne 0) { throw "CMake Build failed with exit code $($procBuild.ExitCode)" }
}
catch {
    Write-Host "Build Failed: $_" -ForegroundColor Red
    exit 1
}
Pop-Location

# 3. Start ROS 2 Backend (Docker)
Write-Host "`n[3/4] Starting ROS 2 Docker Backend..." -ForegroundColor Cyan
# PROJECT_ROOT is now loaded from .env by Docker Compose automatically
docker compose -f docker/docker-compose.sim.yml up -d

# Wait for ROS Bridge
Write-Host "Waiting for ROS Bridge..." -ForegroundColor Yellow
$retries = 0
while ($retries -lt 30) {
    if (Test-NetConnection -ComputerName localhost -Port 9090 -InformationLevel Quiet) {
        Write-Host "ROS Bridge Ready!" -ForegroundColor Green
        break
    }
    Start-Sleep -Seconds 1
    $retries++
}

# 4. Launch Simulator
Write-Host "`n[4/4] Launching C++ Simulator..." -ForegroundColor Cyan
if (Test-Path $ExePath) {
    Start-Process $ExePath
}
else {
    Write-Host "Error: Executable not found at $ExePath" -ForegroundColor Red
}
