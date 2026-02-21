<#
.SYNOPSIS
    Verify that the build environment matches version-lock.yaml.
    Exit code 0 = ALL checks passed. Non-zero = failures detected.
.EXAMPLE
    .\verification.ps1
#>

$ErrorActionPreference = "Continue"
$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$allPassed = $true

Write-Host ""
Write-Host "======================================================" -ForegroundColor Cyan
Write-Host "  Robot Simulator - Environment Verification           " -ForegroundColor Cyan
Write-Host "======================================================" -ForegroundColor Cyan
Write-Host ""

function Check {
    param([string]$Name, [bool]$Condition, [string]$Detail)
    if ($Condition) {
        Write-Host "  [PASS] $Name - $Detail" -ForegroundColor Green
    }
    else {
        Write-Host "  [FAIL] $Name - $Detail" -ForegroundColor Red
        $script:allPassed = $false
    }
}

# ===========================================================
#  1. VISUAL STUDIO 2026
# ===========================================================
Write-Host "=== Visual Studio 2026 ===" -ForegroundColor Yellow

$vsRoot = "D:\VS 26"
$vcvars64 = "$vsRoot\VC\Auxiliary\Build\vcvars64.bat"
Check -Name "VS 2026 Install" -Condition (Test-Path $vsRoot) -Detail $vsRoot
Check -Name "vcvars64.bat" -Condition (Test-Path $vcvars64) -Detail $vcvars64

# ===========================================================
#  2. MSVC COMPILER
# ===========================================================
Write-Host ""
Write-Host "=== MSVC Compiler ===" -ForegroundColor Yellow

# Source vcvars64 to get cl.exe in environment
$vcvarsCmd = 'call "' + $vcvars64 + '" >nul 2>&1 && set'
cmd /s /c $vcvarsCmd | ForEach-Object {
    if ($_ -match '^([^=]+)=(.*)$') {
        [System.Environment]::SetEnvironmentVariable($matches[1], $matches[2], "Process")
    }
}

$clOutput = (cmd /c "cl.exe 2>&1") | Out-String
$msvcVersion = ""
if ($clOutput -match "(\d+\.\d+\.\d+)") {
    $msvcVersion = $matches[1]
}
Check -Name "MSVC Version (19.50.x)" -Condition ($msvcVersion -like "19.50*") -Detail "Got: $msvcVersion"

# Toolset folder
$toolsetPath = "$vsRoot\VC\Tools\MSVC\14.50.35717"
Check -Name "Toolset 14.50.35717" -Condition (Test-Path $toolsetPath) -Detail $toolsetPath

# ===========================================================
#  3. WINDOWS SDK
# ===========================================================
Write-Host ""
Write-Host "=== Windows SDK ===" -ForegroundColor Yellow

$sdkRoot = "C:\Program Files (x86)\Windows Kits\10\Include\10.0.26100.0"
Check -Name "SDK 10.0.26100.0" -Condition (Test-Path $sdkRoot) -Detail $sdkRoot

# ===========================================================
#  4. CMAKE (VS 2026 Bundled)
# ===========================================================
Write-Host ""
Write-Host "=== CMake ===" -ForegroundColor Yellow

$cmakeExe = "$vsRoot\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
$cmakeExists = Test-Path $cmakeExe
Check -Name "CMake bundled" -Condition $cmakeExists -Detail $cmakeExe

if ($cmakeExists) {
    $cmakeVerRaw = & $cmakeExe --version | Select-Object -First 1
    $cmakeVer = $cmakeVerRaw -replace "cmake version ", ""
    $cmakeOk = [version]($cmakeVer -replace "-.*$", "") -ge [version]"3.28"
    Check -Name "CMake >= 3.28" -Condition $cmakeOk -Detail "Got: $cmakeVer"
}

# ===========================================================
#  5. NINJA (VS 2026 Bundled)
# ===========================================================
Write-Host ""
Write-Host "=== Ninja ===" -ForegroundColor Yellow

$ninjaExe = "$vsRoot\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"
Check -Name "Ninja bundled" -Condition (Test-Path $ninjaExe) -Detail $ninjaExe

# ===========================================================
#  6. VCPKG
# ===========================================================
Write-Host ""
Write-Host "=== vcpkg ===" -ForegroundColor Yellow

$vcpkgExe = "C:\vcpkg\vcpkg.exe"
Check -Name "vcpkg.exe" -Condition (Test-Path $vcpkgExe) -Detail $vcpkgExe

$vcpkgJson = Join-Path $ProjectRoot "RobotSimulator_CPP\vcpkg.json"
Check -Name "vcpkg.json" -Condition (Test-Path $vcpkgJson) -Detail $vcpkgJson

$toolchainFile = Join-Path $ProjectRoot "RobotSimulator_CPP\toolchain-msvc1950.cmake"
Check -Name "Toolchain file" -Condition (Test-Path $toolchainFile) -Detail $toolchainFile

# ===========================================================
#  7. DOCKER
# ===========================================================
Write-Host ""
Write-Host "=== Docker ===" -ForegroundColor Yellow

$dockerCmd = Get-Command docker -ErrorAction SilentlyContinue
$dockerInstalled = $null -ne $dockerCmd
$dockerPath = if ($dockerCmd) { $dockerCmd.Path } else { "Not found" }
Check -Name "Docker installed" -Condition $dockerInstalled -Detail $dockerPath

if ($dockerInstalled) {
    docker info 2>$null | Out-Null
    $dockerRunning = $LASTEXITCODE -eq 0
    Check -Name "Docker running" -Condition $dockerRunning -Detail "daemon status"

    $composeFile = Join-Path $ProjectRoot "docker\docker-compose.sim.yml"
    Check -Name "Compose file" -Condition (Test-Path $composeFile) -Detail $composeFile
}

# ===========================================================
#  8. PROJECT FILES
# ===========================================================
Write-Host ""
Write-Host "=== Project Files ===" -ForegroundColor Yellow

Check -Name "CMakeLists.txt" -Condition (Test-Path (Join-Path $ProjectRoot "RobotSimulator_CPP\CMakeLists.txt")) -Detail "exists"
Check -Name "version-lock.yaml" -Condition (Test-Path (Join-Path $ProjectRoot "version-lock.yaml")) -Detail "exists"
Check -Name "build.ps1" -Condition (Test-Path (Join-Path $ProjectRoot "build.ps1")) -Detail "exists"

# ===========================================================
#  SUMMARY
# ===========================================================
Write-Host ""
if ($allPassed) {
    Write-Host "======================================================" -ForegroundColor Green
    Write-Host "  ALL CHECKS PASSED                                    " -ForegroundColor Green
    Write-Host "======================================================" -ForegroundColor Green
    exit 0
}
else {
    Write-Host "======================================================" -ForegroundColor Red
    Write-Host "  SOME CHECKS FAILED - See above                       " -ForegroundColor Red
    Write-Host "======================================================" -ForegroundColor Red
    exit 1
}
