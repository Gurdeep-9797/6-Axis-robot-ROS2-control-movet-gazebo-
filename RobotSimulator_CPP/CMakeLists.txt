cmake_minimum_required(VERSION 3.20)

project(RobotSimulator_CPP LANGUAGES CXX)

# C++20 Standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Pin Windows SDK — vcpkg directx-headers requires >= 10.0.22621
set(CMAKE_SYSTEM_VERSION 10.0.22621.0 CACHE STRING "Windows SDK version")

# Dependencies (vcpkg integration assumed)
find_package(directx-headers CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(assimp CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(tinyxml2 CONFIG REQUIRED)
find_package(ixwebsocket CONFIG REQUIRED)
find_package(cxxopts CONFIG REQUIRED)

# ─────────────────────────────────────────────────────────
# Core Logic Library (No DX12, No UI — safe for headless testing)
# ─────────────────────────────────────────────────────────
file(GLOB CORE_SOURCES
    "robot/*.cpp" "robot/*.h"
    "simulation/*.cpp" "simulation/*.h"
    "engine/ros_client.cpp" "engine/ros_client.h"
)

add_library(RobotCore STATIC ${CORE_SOURCES})

target_link_libraries(RobotCore PUBLIC
    assimp::assimp
    nlohmann_json::nlohmann_json
    spdlog::spdlog
    glm::glm
    tinyxml2::tinyxml2
    ixwebsocket::ixwebsocket
)

# ─────────────────────────────────────────────────────────
# Full Application Library (DX12 + UI + Core)
# ─────────────────────────────────────────────────────────
file(GLOB UI_SOURCES
    "engine/dx12_core.cpp" "engine/dx12_core.h"
    "ui/*.cpp" "ui/*.h"
)

add_library(RobotApp STATIC ${UI_SOURCES})

target_link_libraries(RobotApp PUBLIC
    RobotCore
    Microsoft::DirectX-Headers
    Microsoft::DirectX-Guids
    imgui::imgui
    d3d12.lib
    dxgi.lib
    d3dcompiler.lib
)

# ─────────────────────────────────────────────────────────
# Main Executable
# ─────────────────────────────────────────────────────────
add_executable(RobotSimulator_CPP WIN32 main.cpp)
target_link_libraries(RobotSimulator_CPP PRIVATE RobotApp cxxopts::cxxopts)

# ─────────────────────────────────────────────────────────
# Tests (links only RobotCore — no DX12 needed)
# ─────────────────────────────────────────────────────────
enable_testing()
add_executable(RobotSimulator_Tests tests/test_main.cpp)
target_link_libraries(RobotSimulator_Tests PRIVATE RobotCore)
add_test(NAME CoreComponents COMMAND RobotSimulator_Tests)

# ─────────────────────────────────────────────────────────
# Post-Build: Copy Assets
# ─────────────────────────────────────────────────────────
add_custom_command(TARGET RobotSimulator_CPP POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/assets
    $<TARGET_FILE_DIR:RobotSimulator_CPP>/assets
)
