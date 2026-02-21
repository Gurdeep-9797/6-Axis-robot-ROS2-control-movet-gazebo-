<#
.SYNOPSIS
    Production build script for Robot Simulator C++.
    Uses Ninja generator with VS 2026 toolchain. No global PATH pollution.

.PARAMETER Config
    Build configuration: Release (default) or Debug.

.PARAMETER Clean
    Remove build directory and reconfigure from scratch.

.PARAMETER Wipe
    Full wipe: remove build dir + vcpkg installed packages.

.PARAMETER SkipTests
    Skip running CTest after build.

.EXAMPLE
    .\build.ps1
    .\build.ps1 -Config Debug
    .\build.ps1 -Clean
    .\build.ps1 -Wipe
#>
param(
    [ValidateSet("Release", "Debug")]
    [string]$Config = "Release",
    [switch]$Clean,
    [switch]$Wipe,
    [switch]$SkipTests
)

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path

# ===========================================================
#  PINNED PATHS - No system PATH dependency
# ===========================================================
$VS_ROOT = "D:\VS 26"
$VCVARS = "$VS_ROOT\VC\Auxiliary\Build\vcvars64.bat"
$CMAKE_EXE = "$VS_ROOT\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
$CTEST_EXE = "$VS_ROOT\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\ctest.exe"
$NINJA_EXE = "$VS_ROOT\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"

$SRC_DIR = Join-Path $ProjectRoot "RobotSimulator_CPP"
$BUILD_DIR = Join-Path $ProjectRoot "build"
$TOOLCHAIN = Join-Path $SRC_DIR "toolchain-msvc1950.cmake"
$VCPKG_ROOT = "C:\vcpkg"
$VCPKG_TC = "$VCPKG_ROOT\scripts\buildsystems\vcpkg.cmake"

# ===========================================================
#  VALIDATION
# ===========================================================
Write-Host ""
Write-Host "======================================================" -ForegroundColor Cyan
Write-Host "  Robot Simulator - Production Build (VS 2026 + Ninja) " -ForegroundColor Cyan
Write-Host "======================================================" -ForegroundColor Cyan
Write-Host ""

$requiredPaths = @(
    @{ Name = "vcvars64.bat"; Path = $VCVARS },
    @{ Name = "cmake.exe"; Path = $CMAKE_EXE },
    @{ Name = "ninja.exe"; Path = $NINJA_EXE },
    @{ Name = "vcpkg"; Path = "$VCPKG_ROOT\vcpkg.exe" },
    @{ Name = "Toolchain"; Path = $TOOLCHAIN }
)

foreach ($item in $requiredPaths) {
    if (-not (Test-Path $item.Path)) {
        Write-Host "  [FATAL] $($item.Name) not found: $($item.Path)" -ForegroundColor Red
        exit 1
    }
    Write-Host "  [OK] $($item.Name)" -ForegroundColor Green
}
Write-Host ""

# ===========================================================
#  CLEAN / WIPE
# ===========================================================
if ($Wipe) {
    Write-Host "=== WIPE: Removing build + vcpkg installed ===" -ForegroundColor Red
    if (Test-Path $BUILD_DIR) { Remove-Item -Recurse -Force $BUILD_DIR }
    $vcpkgInstalled = Join-Path $SRC_DIR "vcpkg_installed"
    if (Test-Path $vcpkgInstalled) { Remove-Item -Recurse -Force $vcpkgInstalled }
    Write-Host "  [OK] Wiped" -ForegroundColor Green
    Write-Host ""
}
elseif ($Clean) {
    Write-Host "=== CLEAN: Removing build directory ===" -ForegroundColor Yellow
    if (Test-Path $BUILD_DIR) { Remove-Item -Recurse -Force $BUILD_DIR }
    Write-Host "  [OK] Cleaned" -ForegroundColor Green
    Write-Host ""
}

# ===========================================================
#  BUILD VIA CMD SUBPROCESS
#  vcvars64.bat must be sourced in cmd.exe, then cmake/ninja
#  run in that same environment. We chain everything with &&.
# ===========================================================
Write-Host "=== CMake Configure + Build ($Config) ===" -ForegroundColor Yellow
Write-Host "  Generator:  Ninja" -ForegroundColor Gray
Write-Host "  Config:     $Config" -ForegroundColor Gray
Write-Host "  Source:     $SRC_DIR" -ForegroundColor Gray
Write-Host "  Build:      $BUILD_DIR" -ForegroundColor Gray
Write-Host ""

# Build the cmd command string - all in one subprocess so vcvars env is preserved
$cmdArgs = @(
    "`"$VCVARS`"",
    "&&",
    "`"$CMAKE_EXE`"",
    "-S `"$SRC_DIR`"",
    "-B `"$BUILD_DIR`"",
    "-G Ninja",
    "-DCMAKE_BUILD_TYPE=$Config",
    "-DCMAKE_MAKE_PROGRAM=`"$NINJA_EXE`"",
    "-DCMAKE_TOOLCHAIN_FILE=`"$VCPKG_TC`"",
    "-DVCPKG_TARGET_TRIPLET=x64-windows",
    "-DVCPKG_CHAINLOAD_TOOLCHAIN_FILE=`"$TOOLCHAIN`"",
    "&&",
    "`"$CMAKE_EXE`"",
    "--build `"$BUILD_DIR`"",
    "--parallel"
)

$cmdLine = $cmdArgs -join " "
cmd /s /c $cmdLine

if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "  [ERROR] Build failed (exit $LASTEXITCODE)" -ForegroundColor Red
    exit 1
}
Write-Host ""
Write-Host "  [OK] Build succeeded" -ForegroundColor Green
Write-Host ""

# ===========================================================
#  TESTS
# ===========================================================
if (-not $SkipTests) {
    Write-Host "=== Running Tests ===" -ForegroundColor Yellow
    $testCmd = "`"$VCVARS`" && `"$CTEST_EXE`" --test-dir `"$BUILD_DIR`" -C $Config --output-on-failure"
    cmd /s /c $testCmd

    if ($LASTEXITCODE -ne 0) {
        Write-Host "  [WARN] Some tests failed" -ForegroundColor Yellow
    }
    else {
        Write-Host "  [OK] All tests passed" -ForegroundColor Green
    }
    Write-Host ""
}

# ===========================================================
#  SUMMARY
# ===========================================================
$exePath = Join-Path $BUILD_DIR "RobotSimulator_CPP.exe"
if (Test-Path $exePath) {
    Write-Host "======================================================" -ForegroundColor Green
    Write-Host "  BUILD COMPLETE                                       " -ForegroundColor Green
    Write-Host "======================================================" -ForegroundColor Green
    Write-Host "  Executable: $exePath" -ForegroundColor White
}
else {
    Write-Host "  [WARN] Executable not found at expected path" -ForegroundColor Yellow
    Write-Host "  Build dir contents:" -ForegroundColor Gray
    Get-ChildItem $BUILD_DIR -Filter "*.exe" -Recurse -ErrorAction SilentlyContinue | ForEach-Object { Write-Host "    $($_.FullName)" }
}
Write-Host ""
