cmake_minimum_required(VERSION 3.20)

project(RobotSimulator_CPP LANGUAGES CXX)

# C++20 Standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Dependencies (vcpkg integration assumed)
find_package(directx-headers CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(assimp CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(tinyxml2 CONFIG REQUIRED)
find_package(ixwebsocket CONFIG REQUIRED)

# Source Files
file(GLOB_RECURSE LIB_SOURCES 
    "engine/*.cpp" "engine/*.h"
    "robot/*.cpp" "robot/*.h"
    "ui/*.cpp" "ui/*.h"
    "simulation/*.cpp" "simulation/*.h"
)

# Static Library for Logic
add_library(RobotLib STATIC ${LIB_SOURCES})

# Component Component Libraries
target_link_libraries(RobotLib PUBLIC
    Microsoft::DirectX-Headers
    imgui::imgui
    assimp::assimp
    nlohmann_json::nlohmann_json
    spdlog::spdlog
    glm::glm
    tinyxml2::tinyxml2
    d3d12.lib
    dxgi.lib
    d3dcompiler.lib
    ixwebsocket::ixwebsocket
)

# Main Executable
add_executable(RobotSimulator_CPP main.cpp)
target_link_libraries(RobotSimulator_CPP PRIVATE RobotLib)

# Tests
enable_testing()
add_executable(RobotSimulator_Tests tests/test_main.cpp)
target_link_libraries(RobotSimulator_Tests PRIVATE RobotLib)
add_test(NAME CoreComponents COMMAND RobotSimulator_Tests)

# Copy Assets to Output
add_custom_command(TARGET RobotSimulator_CPP POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/assets
    $<TARGET_FILE_DIR:RobotSimulator_CPP>/assets
)
