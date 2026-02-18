@echo off
REM ═══════════════════════════════════════════════════════════
REM  Robot Simulator — One-Click Local Build
REM  This script auto-detects MSVC, sets up vcpkg, and builds.
REM ═══════════════════════════════════════════════════════════

setlocal enabledelayedexpansion

echo.
echo ╔════════════════════════════════════════════╗
echo ║  Robot Simulator — Local Build Script      ║
echo ╚════════════════════════════════════════════╝
echo.

REM ─────────────────────────────────────────────
REM Step 1: Find Visual Studio Developer Environment
REM ─────────────────────────────────────────────
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"

if not exist "%VSWHERE%" (
    echo [ERROR] Visual Studio not found. Install with:
    echo   winget install Microsoft.VisualStudio.2022.BuildTools --override "--quiet --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"
    pause
    exit /b 1
)

REM Find latest VS installation with C++ tools
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do (
    set "VS_PATH=%%i"
)

if not defined VS_PATH (
    echo [ERROR] Visual Studio C++ tools not found.
    echo Run: winget install Microsoft.VisualStudio.2022.BuildTools --override "--quiet --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"
    pause
    exit /b 1
)

echo [OK] Found Visual Studio: %VS_PATH%

REM Activate the Developer Command Prompt environment
call "%VS_PATH%\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1

REM Verify cl.exe is now available
where cl.exe >nul 2>&1
if errorlevel 1 (
    echo [ERROR] cl.exe not found after activating VS environment.
    pause
    exit /b 1
)
echo [OK] MSVC Compiler: Ready

REM ─────────────────────────────────────────────
REM Step 2: Setup vcpkg
REM ─────────────────────────────────────────────
set "VCPKG_ROOT=C:\vcpkg"

if not exist "%VCPKG_ROOT%\vcpkg.exe" (
    echo [INFO] Installing vcpkg...
    git clone https://github.com/microsoft/vcpkg.git "%VCPKG_ROOT%"
    call "%VCPKG_ROOT%\bootstrap-vcpkg.bat" -disableMetrics
)
echo [OK] vcpkg: %VCPKG_ROOT%

REM ─────────────────────────────────────────────
REM Step 3: Configure CMake
REM ─────────────────────────────────────────────
set "SRC_DIR=%~dp0RobotSimulator_CPP"
set "BUILD_DIR=%~dp0build_local"
set "TOOLCHAIN=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake"

echo [INFO] Configuring CMake...
cmake -S "%SRC_DIR%" -B "%BUILD_DIR%" ^
    -DCMAKE_TOOLCHAIN_FILE="%TOOLCHAIN%" ^
    -DVCPKG_TARGET_TRIPLET=x64-windows ^
    -G "Visual Studio 17 2022" -A x64

if errorlevel 1 (
    echo [ERROR] CMake configuration failed.
    pause
    exit /b 1
)
echo [OK] CMake configured.

REM ─────────────────────────────────────────────
REM Step 4: Build Release
REM ─────────────────────────────────────────────
echo [INFO] Building Release...
cmake --build "%BUILD_DIR%" --config Release -- /m

if errorlevel 1 (
    echo [ERROR] Build failed.
    pause
    exit /b 1
)
echo [OK] Build succeeded!

REM ─────────────────────────────────────────────
REM Step 5: Run Tests
REM ─────────────────────────────────────────────
echo [INFO] Running Component Tests...
pushd "%BUILD_DIR%"
ctest -C Release --output-on-failure
popd

if errorlevel 1 (
    echo [WARNING] Some tests failed. Check output above.
)

REM ─────────────────────────────────────────────
REM Step 6: Launch!
REM ─────────────────────────────────────────────
echo.
echo ╔════════════════════════════════════════════╗
echo ║  BUILD COMPLETE                            ║
echo ╚════════════════════════════════════════════╝
echo.
echo Executable: %BUILD_DIR%\Release\RobotSimulator_CPP.exe
echo.

set /p LAUNCH="Launch simulator now? [Y/n]: "
if /i "%LAUNCH%" neq "n" (
    start "" "%BUILD_DIR%\Release\RobotSimulator_CPP.exe"
)

endlocal
