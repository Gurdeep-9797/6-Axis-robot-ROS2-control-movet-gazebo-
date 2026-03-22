# ===========================================================
#  Toolchain: MSVC 19.50 / VS 2026 / SDK 10.0.26100.0
#  Chainloaded by vcpkg via VCPKG_CHAINLOAD_TOOLCHAIN_FILE.
#
#  Uses Ninja generator via build.ps1. vcvars64.bat is
#  sourced first to set up the MSVC x64 environment.
# ===========================================================

# -- Windows SDK --
set(CMAKE_SYSTEM_VERSION "10.0.26100.0" CACHE STRING "Windows SDK version" FORCE)

# -- C++ Standard --
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# -- MSVC Runtime --
# MultiThreadedDLL (/MD) for Release, MultiThreadedDebugDLL (/MDd) for Debug
cmake_policy(SET CMP0091 NEW)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL" CACHE STRING "" FORCE)

# -- Compiler Flags --
# /W4             = High warning level
# /MP             = Multi-process compilation
# /Zc:__cplusplus = Report correct __cplusplus macro
# /permissive-    = Strict conformance
# /utf-8          = Source and execution charset
set(CMAKE_CXX_FLAGS_INIT "/W4 /MP /Zc:__cplusplus /permissive- /utf-8")
set(CMAKE_CXX_FLAGS_RELEASE_INIT "/O2 /DNDEBUG /GL")
set(CMAKE_CXX_FLAGS_DEBUG_INIT "/Od /Zi /RTC1")

# -- Linker Flags --
set(CMAKE_EXE_LINKER_FLAGS_RELEASE_INIT "/LTCG /OPT:REF /OPT:ICF")
