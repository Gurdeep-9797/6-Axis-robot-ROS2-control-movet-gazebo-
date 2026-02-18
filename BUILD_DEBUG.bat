@echo off
setlocal

echo ==========================================
echo      Starting Debug Build Process (VsDevCmd)
echo ==========================================

set "VS_PATH=C:\Program Files\Microsoft Visual Studio\2022\Community"
set "DEV_CMD=%VS_PATH%\Common7\Tools\VsDevCmd.bat"

if not exist "%DEV_CMD%" (
    echo Error: VsDevCmd.bat not found at "%DEV_CMD%"
    exit /b 1
)

echo Calling VsDevCmd: "%DEV_CMD%"
call "%DEV_CMD%" -arch=amd64
if %errorlevel% neq 0 (
    echo Error calling VsDevCmd.bat
    exit /b 1
)

echo Environment Initialized. Checking cl.exe...
where cl.exe
if %errorlevel% neq 0 (
    echo Error: cl.exe not in PATH. Usage failed.
    exit /b 1
)

echo Checking CMake...
where cmake
if %errorlevel% neq 0 (
    echo CMake not in PATH. Searching VS...
    set "CMAKE_PATH=%VS_PATH%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin"
    set "PATH=%PC_PATH%;%CMAKE_PATH%"
)

where cmake
if %errorlevel% neq 0 (
    echo Error: CMake still not found.
    exit /b 1
)

echo Build Directory Cleaning...
if exist build rmdir /s /q build

echo Configuring...
cmake -S RobotSimulator_CPP -B build -G "Visual Studio 17 2022" -A x64 "-DCMAKE_TOOLCHAIN_FILE=tools/vcpkg/scripts/buildsystems/vcpkg.cmake"
if %errorlevel% neq 0 (
    echo Configuration Failed.
    tail build\vcpkg-manifest-install.log
    exit /b 1
)

echo Building...
cmake --build build --config Release
if %errorlevel% neq 0 (
    echo Build Failed.
    exit /b 1
)

echo Build Complete.
if exist "RobotSimulator_CPP\build\Release\RobotSimulator_CPP.exe" (
    echo Success! Executable found.
)
endlocal
