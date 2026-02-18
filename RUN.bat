@echo off
setlocal

echo ====================================
echo    Auto-Initializing VS2022 Envir
echo ====================================

REM 1. Locate vswhere.exe
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" goto :ErrorVsWhere

REM 2. Run vswhere to temp file
set "VS_PATH_FILE=%TEMP%\vs_path_%RANDOM%.txt"
"%VSWHERE%" -latest -products * -property installationPath > "%VS_PATH_FILE%"

REM 3. Read path from file
set /p VS_PATH=<"%VS_PATH_FILE%"
del "%VS_PATH_FILE%"

if "%VS_PATH%"=="" (
    echo [ERROR] No Visual Studio installation found.
    pause
    exit /b 1
)

echo [INFO] VS Path: "%VS_PATH%"

REM *** FIX FOR VCPKG ***
set "VCPKG_VISUAL_STUDIO_PATH=%VS_PATH%"

REM 4. Initialize Environment
set "DEVCMD=%VS_PATH%\Common7\Tools\VsDevCmd.bat"
if not exist "%DEVCMD%" goto :ErrorNoDevCmd

echo [INFO] Calling VsDevCmd...
call "%DEVCMD%" -arch=amd64 -host_arch=amd64 -no_logo

REM 5. Verify Compiler
where cl.exe > nul 2>&1
if %errorlevel% neq 0 goto :ErrorNoCompiler

echo [INFO] cl.exe found.

REM 6. Run Start System
echo [INFO] Launching System...
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0START_SYSTEM.ps1"

if %errorlevel% neq 0 goto :ErrorScriptFailed

endlocal
exit /b 0

:ErrorVsWhere
    echo [ERROR] Visual Studio is not installed.
    echo.
    echo Running automated setup...
    powershell -NoProfile -ExecutionPolicy Bypass -Command "Start-Process powershell -Verb RunAs -ArgumentList '-ExecutionPolicy Bypass -File \"%~dp0SETUP_AND_BUILD.ps1\"'"
    echo.
    echo Setup launched in an Administrator window.
    echo After it completes, re-run RUN.bat.
    pause
    exit /b 1

:ErrorNoVS
    echo [ERROR] No Visual Studio installation found.
    pause
    exit /b 1

:ErrorNoDevCmd
    echo [ERROR] VsDevCmd.bat not found at:
    echo "%DEVCMD%"
    echo Please ensure 'Desktop development with C++' is installed.
    pause
    exit /b 1

:ErrorNoCompiler
    echo [ERROR] cl.exe not found in PATH!
    echo.
    echo The C++ Build Tools are missing.
    echo.
    echo We can install them automatically for you.
    echo Press any key to start the installation ^(Requires Admin^)...
    pause
    
    powershell -NoProfile -ExecutionPolicy Bypass -Command "Start-Process powershell -Verb RunAs -ArgumentList '-ExecutionPolicy Bypass -File \"%~dp0SETUP_AND_BUILD.ps1\"'"
    
    echo.
    echo Installation finished. Please re-run RUN.bat.
    pause
    exit /b 1

:ErrorScriptFailed
    echo [ERROR] Script execution failed.
    pause
    exit /b 1
