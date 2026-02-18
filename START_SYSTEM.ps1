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
        $cmakePath = "& `"$vsCmake`""
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
    Invoke-Expression "$cmakePath -B build -S . -DCMAKE_TOOLCHAIN_FILE=`"$VcpkgPath\scripts\buildsystems\vcpkg.cmake`" -DVCPKG_TARGET_TRIPLET=x64-windows"
    
    # Build
    Invoke-Expression "$cmakePath --build build --config Release"
}
catch {
    Write-Host "Build Failed!" -ForegroundColor Red
    exit 1
}
Pop-Location

# 3. Start ROS 2 Backend (Docker)
Write-Host "`n[3/4] Starting ROS 2 Docker Backend..." -ForegroundColor Cyan
$Env:PROJECT_ROOT = $ProjectRoot
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
