# Robot Simulator C++ Setup Guide

## Prerequisites
1.  **Visual Studio 2022** (with "Desktop development with C++" workload).
2.  **vcpkg** package manager.
3.  **CMake** (included in VS2022).

## Install Dependencies
Please install `vcpkg` if you haven't already. Then run:

```powershell
# Clone vcpkg (if needed)
git clone https://github.com/microsoft/vcpkg.git
.\vcpkg\bootstrap-vcpkg.bat

# Install packages
vcpkg install directx-headers imgui[core,dx12-binding,win32-binding,docking-experimental] assimp nlohmann-json spdlog glm --triplet x64-windows
```

## Build
1.  Open this folder (`RobotSimulator_CPP`) in Visual Studio.
2.  Or use command line:
    ```powershell
    cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=[path/to/vcpkg]/scripts/buildsystems/vcpkg.cmake
    cmake --build build --config Release
    ```
