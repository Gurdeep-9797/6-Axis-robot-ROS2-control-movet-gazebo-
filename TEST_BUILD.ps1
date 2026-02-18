$ErrorActionPreference = "Continue"

if (Test-Path "build") {
    Write-Host "Cleaning build directory..."
    Remove-Item -Path "build" -Recurse -Force
}

$vsRoot = & "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -property installationPath
if (-not $vsRoot) { Write-Error "Visual Studio not found!"; exit 1 }
$vcvars = "$vsRoot\VC\Auxiliary\Build\vcvars64.bat"

$vcpkgToolchain = Resolve-Path "tools/vcpkg/scripts/buildsystems/vcpkg.cmake" -ErrorAction SilentlyContinue
if (-not $vcpkgToolchain) {
    if (Test-Path "tools/vcpkg/vcpkg.exe") {
        # Bootstrap assumed
    }
    $vcpkgToolchain = Resolve-Path "tools/vcpkg/scripts/buildsystems/vcpkg.cmake"
}
Write-Host "Toolchain: $vcpkgToolchain"

# Using format operator for safer string construction
$configureCmd = 'cmake -S RobotSimulator_CPP -B build -G "Visual Studio 17 2022" -A x64 "-DCMAKE_TOOLCHAIN_FILE={0}"' -f $vcpkgToolchain
$buildCmd = "cmake --build build --config Release"

Write-Host "Configuring..."
# Construct the cmd string carefully.
# cmd /c "call "path\to\vcvars.bat" && cmake ..."
$cmdArgs = "/c call `"{0}`" && {1}" -f $vcvars, $configureCmd

Start-Process -FilePath "cmd.exe" -ArgumentList $cmdArgs -Wait -NoNewWindow -RedirectStandardOutput "build_log.txt" -RedirectStandardError "build_log.txt"

if ((Select-String -Path "build_log.txt" -Pattern "Configuring incomplete, errors occurred").Count -gt 0) {
    Write-Host "Configuration Failed. See build_log.txt"
    type build_log.txt
    exit 1
}

Write-Host "Building..."
$cmdArgsBuild = "/c call `"{0}`" && {1}" -f $vcvars, $buildCmd
Start-Process -FilePath "cmd.exe" -ArgumentList $cmdArgsBuild -Wait -NoNewWindow >> build_log.txt 2>&1

# Check for success by looking for executable?
if (Test-Path "RobotSimulator_CPP/build/Release/RobotSimulator_CPP.exe") {
    Write-Host "Build Complete!"
} else {
    Write-Host "Build seems to have failed or exe not found. Checking log tail..."
    Get-Content build_log.txt -Tail 20
}
