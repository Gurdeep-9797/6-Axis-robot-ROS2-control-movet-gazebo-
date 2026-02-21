cmake_minimum_required(VERSION 3.28...4.2)

project(RobotSimulator_CPP LANGUAGES CXX)

# NOTE: C++ standard, SDK version, and toolset are set in
# toolchain-msvc1950.cmake (chainloaded by vcpkg).
# Do NOT duplicate them here.

# ── Dependencies (vcpkg manifest mode) ──────────────────
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
    "engine/dx12_mesh.cpp" "engine/dx12_mesh.h"
    "engine/scene_renderer.cpp" "engine/scene_renderer.h"
    "engine/camera.cpp" "engine/camera.h"
    "ui/*.cpp" "ui/*.h"
)

add_library(RobotApp STATIC ${UI_SOURCES})
target_include_directories(RobotApp PUBLIC ${CMAKE_SOURCE_DIR})

target_link_libraries(RobotApp PUBLIC
    RobotCore
    Microsoft::DirectX-Headers
    Microsoft::DirectX-Guids
    imgui::imgui
    d3d12.lib
    dxgi.lib
    d3dcompiler.lib
)

# Force DirectX-Headers includes BEFORE system SDK includes.
# Prevents system d3d12.h include guards from blocking vcpkg d3d12.h.
get_target_property(DX_DIRS Microsoft::DirectX-Headers INTERFACE_INCLUDE_DIRECTORIES)
if(DX_DIRS)
    target_include_directories(RobotApp BEFORE PUBLIC ${DX_DIRS})
endif()

# ─────────────────────────────────────────────────────────
# Main Executable
# ─────────────────────────────────────────────────────────
add_executable(RobotSimulator_CPP main.cpp)
target_link_libraries(RobotSimulator_CPP PRIVATE RobotApp cxxopts::cxxopts bcrypt)
target_compile_definitions(RobotSimulator_CPP PRIVATE UNICODE _UNICODE)

add_executable(test_dummy test_dummy.cpp)
target_link_libraries(test_dummy PRIVATE RobotApp cxxopts::cxxopts bcrypt)
target_compile_definitions(test_dummy PRIVATE UNICODE _UNICODE)

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
if(EXISTS "${CMAKE_SOURCE_DIR}/assets")
    add_custom_command(TARGET RobotSimulator_CPP POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/assets
        $<TARGET_FILE_DIR:RobotSimulator_CPP>/assets
    )
endif()
