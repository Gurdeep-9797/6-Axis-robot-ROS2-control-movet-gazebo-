@echo off
setlocal

echo ==========================================
echo    Auto-Launching with VS Developer Env
echo ==========================================

REM 1. Find vswhere.exe
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
    echo Error: vswhere.exe not found at "%VSWHERE%"
    pause
    exit /b 1
)

REM 2. Find Visual Studio Path using vswhere
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -property installationPath`) do (
    set "VS_PATH=%%i"
)

if not defined VS_PATH (
    echo Error: Could not find Visual Studio 2022 Installation.
    pause
    exit /b 1
)
echo Found Visual Studio: "%VS_PATH%"

REM 3. Find VsDevCmd.bat
set "DEVCMD=%VS_PATH%\Common7\Tools\VsDevCmd.bat"
if not exist "%DEVCMD%" (
    echo Error: VsDevCmd.bat not found at "%DEVCMD%"
    echo You may need to install 'Desktop development with C++' workload.
    pause
    exit /b 1
)

echo Loading Developer Environment...
REM Use cmd /k to keep the window open after execution if desired, or call to run in this process
call "%DEVCMD%" -arch=amd64 -no_logo

if %errorlevel% neq 0 (
    echo Error initializing environment.
    pause
    exit /b 1
)

echo Environment Loaded. Launching Build System...
call .\RUN.bat

if %errorlevel% neq 0 (
    echo Build Script Exited with Error.
    pause
) else (
    echo success.
    timeout /t 5
)
endlocal
