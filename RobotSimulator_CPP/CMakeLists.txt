cmake_minimum_required(VERSION 3.20)

project(RobotSimulator_CPP LANGUAGES CXX)

# C++20 Standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Force Windows 11 SDK (match local dev)
set(CMAKE_SYSTEM_VERSION 10.0.26100.0 CACHE STRING "Force Windows SDK version" FORCE)

# vcpkg directx-headers has its own d3d12.h. Using 10.0.26100.0 helps compatibility,
# but our include order fix is the real key.

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
target_include_directories(RobotCore PUBLIC ${CMAKE_SOURCE_DIR})

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
target_include_directories(RobotApp PUBLIC ${CMAKE_SOURCE_DIR})

# CRITICAL: vcpkg directx-headers include paths MUST come BEFORE system SDK
# to avoid include guard conflicts (system d3d12.h guards block vcpkg definitions).
# Using BEFORE ensures the vcpkg includes are searched first.
target_link_libraries(RobotApp PUBLIC
    RobotCore
    Microsoft::DirectX-Headers
    Microsoft::DirectX-Guids
    imgui::imgui
    d3d12.lib
    dxgi.lib
    d3dcompiler.lib
)

# Force DirectX-Headers includes to be searched BEFORE system SDK includes.
# This prevents the system d3d12.h include guards from blocking vcpkg's d3d12.h.
get_target_property(DX_DIRS Microsoft::DirectX-Headers INTERFACE_INCLUDE_DIRECTORIES)
if(DX_DIRS)
    target_include_directories(RobotApp BEFORE PUBLIC ${DX_DIRS})
endif()

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
