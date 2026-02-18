@echo off
echo Cleaning build directories...
if exist build rmdir /s /q build
if exist cmake-build-debug rmdir /s /q cmake-build-debug
if exist RobotSimulator_CPP\build rmdir /s /q RobotSimulator_CPP\build
if exist RobotSimulator_CPP\vcpkg_installed rmdir /s /q RobotSimulator_CPP\vcpkg_installed
echo Done.
