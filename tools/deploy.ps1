<#
.SYNOPSIS
    Packages the built application for distribution.
    Creates a 'Release_Package' folder with EXE, DLLs, and Assets.
#>
$ErrorActionPreference = "Stop"

$RootDir = "$PSScriptRoot\.."
$BuildDir = "$RootDir\RobotSimulator_CPP\build\Release"
$DistDir = "$RootDir\Release_Package"
$AssetsDir = "$RootDir\RobotSimulator_CPP\assets"

Write-Host "Packaging Application..." -ForegroundColor Cyan

if (-not (Test-Path "$BuildDir\RobotSimulator_CPP.exe")) {
    Write-Error "Build not found at $BuildDir. Please run RUN.bat first."
}

# Clean Dist
if (Test-Path $DistDir) { Remove-Item $DistDir -Recurse -Force }
New-Item -ItemType Directory -Path $DistDir | Out-Null

# Copy Executable
Copy-Item "$BuildDir\RobotSimulator_CPP.exe" -Destination $DistDir
Write-Host "Copied Executable."

# Copy DLLs (vcpkg usually puts them in Release root or bin)
# We check build directory for any DLLs
Get-ChildItem -Path "$BuildDir" -Filter "*.dll" | Copy-Item -Destination $DistDir
Write-Host "Copied Dependencies."

# Copy Assets
Copy-Item -Path $AssetsDir -Destination "$DistDir\assets" -Recurse
Write-Host "Copied Assets."

Write-Host "Package created at: $DistDir" -ForegroundColor Green
Invoke-Item $DistDir
