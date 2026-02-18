# ═══════════════════════════════════════════════════════════
#  Robot Simulator — SETUP & BUILD (Run as Administrator)
#  This script installs all build dependencies and builds.
# ═══════════════════════════════════════════════════════════

$ErrorActionPreference = "Continue"
$ProjectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$VcpkgRoot = "C:\vcpkg"

Write-Host ""
Write-Host "╔════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║  Robot Simulator — Environment Setup       ║" -ForegroundColor Cyan
Write-Host "╚════════════════════════════════════════════╝" -ForegroundColor Cyan
Write-Host ""

# ─────────────────────────────────────────────
# Step 1: Inventory existing tools
# ─────────────────────────────────────────────
Write-Host "═══ STEP 1: Checking installed tools ═══" -ForegroundColor Yellow

# Check Git
$gitPath = (Get-Command git -ErrorAction SilentlyContinue).Path
if ($gitPath) {
    Write-Host "  [OK] Git: $gitPath" -ForegroundColor Green
} else {
    Write-Host "  [MISSING] Git — will install" -ForegroundColor Red
}

# Check CMake
$cmakePath = (Get-Command cmake -ErrorAction SilentlyContinue).Path
if ($cmakePath) {
    Write-Host "  [OK] CMake: $cmakePath" -ForegroundColor Green
} else {
    Write-Host "  [MISSING] CMake — will be installed with VS Build Tools" -ForegroundColor Red
}

# Check vswhere (indicates VS installer is present)
$vswherePath = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$vsInstalled = $false
$vsPath = $null
if (Test-Path $vswherePath) {
    $vsPath = & $vswherePath -latest -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2>$null
    if ($vsPath) {
        Write-Host "  [OK] Visual Studio C++ Tools: $vsPath" -ForegroundColor Green
        $vsInstalled = $true
    } else {
        $vsAny = & $vswherePath -latest -property installationPath 2>$null
        if ($vsAny) {
            Write-Host "  [PARTIAL] VS found at $vsAny but missing C++ workload" -ForegroundColor Yellow
        } else {
            Write-Host "  [MISSING] Visual Studio Build Tools" -ForegroundColor Red
        }
    }
} else {
    Write-Host "  [MISSING] Visual Studio — not installed at all" -ForegroundColor Red
}

# Check vcpkg
if (Test-Path "$VcpkgRoot\vcpkg.exe") {
    Write-Host "  [OK] vcpkg: $VcpkgRoot" -ForegroundColor Green
} else {
    Write-Host "  [MISSING] vcpkg — will install to $VcpkgRoot" -ForegroundColor Red
}

# Check Windows SDK
$sdkPath = Get-ItemProperty "HKLM:\SOFTWARE\Microsoft\Windows Kits\Installed Roots" -ErrorAction SilentlyContinue
if ($sdkPath.KitsRoot10) {
    Write-Host "  [OK] Windows SDK: $($sdkPath.KitsRoot10)" -ForegroundColor Green
} else {
    Write-Host "  [MISSING] Windows SDK — will be installed with VS Build Tools" -ForegroundColor Red
}

Write-Host ""

# ─────────────────────────────────────────────
# Step 2: Install Visual Studio Build Tools (if missing)
# ─────────────────────────────────────────────
if (-not $vsInstalled) {
    Write-Host "═══ STEP 2: Adding C++ Desktop workload to Visual Studio ═══" -ForegroundColor Yellow
    Write-Host "  This will install: MSVC compiler, Windows SDK, CMake, MSBuild" -ForegroundColor Gray
    Write-Host "  Estimated time: 5-15 minutes depending on internet speed" -ForegroundColor Gray
    Write-Host ""
    
    # Check if VS is installed but just missing C++ workload
    $vsAnyPath = $null
    if (Test-Path $vswherePath) {
        $vsAnyPath = & $vswherePath -latest -property installationPath 2>$null
    }
    
    if ($vsAnyPath) {
        # Modify existing VS installation to add C++ workload
        Write-Host "  Found VS at: $vsAnyPath — adding C++ workload..." -ForegroundColor Gray
        $vsInstallerPath = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vs_installer.exe"
        $installArgs = @(
            "modify",
            "--installPath", "`"$vsAnyPath`"",
            "--add", "Microsoft.VisualStudio.Workload.VCTools",
            "--add", "Microsoft.VisualStudio.Component.VC.CMake.Project",
            "--add", "Microsoft.VisualStudio.Component.Windows11SDK.22621",
            "--includeRecommended",
            "--quiet", "--wait", "--norestart"
        )
        $process = Start-Process -FilePath $vsInstallerPath -ArgumentList $installArgs -Wait -PassThru
    } else {
        # No VS at all — install Build Tools from scratch
        Write-Host "  No VS found — installing Build Tools..." -ForegroundColor Gray
        $installerUrl = "https://aka.ms/vs/17/release/vs_BuildTools.exe"
        $installerPath = "$env:TEMP\vs_BuildTools.exe"
        
        Write-Host "  Downloading installer..." -ForegroundColor Gray
        Invoke-WebRequest -Uri $installerUrl -OutFile $installerPath -UseBasicParsing
        
        $installArgs = @(
            "--quiet", "--wait", "--norestart", "--nocache",
            "--add", "Microsoft.VisualStudio.Workload.VCTools",
            "--add", "Microsoft.VisualStudio.Component.VC.CMake.Project",
            "--add", "Microsoft.VisualStudio.Component.Windows11SDK.22621",
            "--includeRecommended"
        )
        $process = Start-Process -FilePath $installerPath -ArgumentList $installArgs -Wait -PassThru
    }
    
    if ($process.ExitCode -eq 0 -or $process.ExitCode -eq 3010) {
        Write-Host "  [OK] C++ tools installed successfully!" -ForegroundColor Green
        if ($process.ExitCode -eq 3010) {
            Write-Host "  [NOTE] A restart may be required for full functionality" -ForegroundColor Yellow
        }
    } else {
        Write-Host "  [ERROR] Installation failed with code: $($process.ExitCode)" -ForegroundColor Red
        Write-Host "  Try running this script as Administrator" -ForegroundColor Red
        Read-Host "Press Enter to exit"
        exit 1
    }
    
    # Re-detect VS path
    $vsPath = & $vswherePath -latest -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2>$null
    Write-Host ""
} else {
    Write-Host "═══ STEP 2: VS C++ Tools already installed — skipping ═══" -ForegroundColor Green
    Write-Host ""
}

# ─────────────────────────────────────────────
# Step 3: Install vcpkg (if missing)
# ─────────────────────────────────────────────
if (-not (Test-Path "$VcpkgRoot\vcpkg.exe")) {
    Write-Host "═══ STEP 3: Installing vcpkg ═══" -ForegroundColor Yellow
    
    if (Test-Path $VcpkgRoot) {
        Write-Host "  Cleaning stale vcpkg directory..." -ForegroundColor Gray
        Remove-Item -Recurse -Force $VcpkgRoot
    }
    
    & git clone https://github.com/microsoft/vcpkg.git $VcpkgRoot
    & "$VcpkgRoot\bootstrap-vcpkg.bat" -disableMetrics
    
    if (Test-Path "$VcpkgRoot\vcpkg.exe") {
        Write-Host "  [OK] vcpkg installed" -ForegroundColor Green
    } else {
        Write-Host "  [ERROR] vcpkg bootstrap failed" -ForegroundColor Red
        exit 1
    }
    
    # Set global environment variable so it persists
    [System.Environment]::SetEnvironmentVariable("VCPKG_ROOT", $VcpkgRoot, "User")
    $env:VCPKG_ROOT = $VcpkgRoot
    Write-Host "  [OK] VCPKG_ROOT set to $VcpkgRoot" -ForegroundColor Green
    Write-Host ""
} else {
    Write-Host "═══ STEP 3: vcpkg already installed — skipping ═══" -ForegroundColor Green
    Write-Host ""
}

# ─────────────────────────────────────────────
# Step 4: Activate MSVC environment
# ─────────────────────────────────────────────
Write-Host "═══ STEP 4: Activating MSVC Developer Environment ═══" -ForegroundColor Yellow

$vcvarsall = Join-Path $vsPath "VC\Auxiliary\Build\vcvars64.bat"
if (-not (Test-Path $vcvarsall)) {
    Write-Host "  [ERROR] vcvars64.bat not found at: $vcvarsall" -ForegroundColor Red
    exit 1
}

# Import VS environment variables into PowerShell
cmd /c "`"$vcvarsall`" >nul 2>&1 && set" | ForEach-Object {
    if ($_ -match '^([^=]+)=(.*)$') {
        [System.Environment]::SetEnvironmentVariable($matches[1], $matches[2], "Process")
    }
}
Write-Host "  [OK] MSVC environment activated" -ForegroundColor Green
Write-Host ""

# ─────────────────────────────────────────────
# Step 5: Configure & Build
# ─────────────────────────────────────────────
$srcDir = Join-Path $ProjectRoot "RobotSimulator_CPP"
$buildDir = Join-Path $ProjectRoot "build_local"
$toolchain = Join-Path $VcpkgRoot "scripts\buildsystems\vcpkg.cmake"

Write-Host "═══ STEP 5: CMake Configure ═══" -ForegroundColor Yellow
Write-Host "  Source:    $srcDir" -ForegroundColor Gray
Write-Host "  Build:     $buildDir" -ForegroundColor Gray
Write-Host "  Toolchain: $toolchain" -ForegroundColor Gray
Write-Host ""

cmake -S $srcDir -B $buildDir `
    -DCMAKE_TOOLCHAIN_FILE="$toolchain" `
    -DVCPKG_TARGET_TRIPLET=x64-windows `
    -G "Visual Studio 17 2022" -A x64

if ($LASTEXITCODE -ne 0) {
    Write-Host "  [ERROR] CMake configuration failed" -ForegroundColor Red
    Read-Host "Press Enter to exit"
    exit 1
}
Write-Host "  [OK] CMake configured" -ForegroundColor Green
Write-Host ""

Write-Host "═══ STEP 6: Building Release ═══" -ForegroundColor Yellow
cmake --build $buildDir --config Release -- /m

if ($LASTEXITCODE -ne 0) {
    Write-Host "  [ERROR] Build failed" -ForegroundColor Red
    Read-Host "Press Enter to exit"
    exit 1
}
Write-Host "  [OK] Build succeeded!" -ForegroundColor Green
Write-Host ""

# ─────────────────────────────────────────────
# Step 6: Run Tests
# ─────────────────────────────────────────────
Write-Host "═══ STEP 7: Running Tests ═══" -ForegroundColor Yellow
Push-Location $buildDir
ctest -C Release --output-on-failure
Pop-Location
Write-Host ""

# ─────────────────────────────────────────────
# Done!
# ─────────────────────────────────────────────
$exePath = Join-Path $buildDir "Release\RobotSimulator_CPP.exe"

Write-Host "╔════════════════════════════════════════════╗" -ForegroundColor Green
Write-Host "║  SETUP COMPLETE — READY TO GO!             ║" -ForegroundColor Green
Write-Host "╚════════════════════════════════════════════╝" -ForegroundColor Green
Write-Host ""
Write-Host "  Executable: $exePath" -ForegroundColor White
Write-Host ""
Write-Host "  Next time, just run: .\BUILD.bat" -ForegroundColor Gray
Write-Host "  (It will skip installation and go straight to build)" -ForegroundColor Gray
Write-Host ""

$launch = Read-Host "Launch simulator now? [Y/n]"
if ($launch -ne "n") {
    Start-Process $exePath
}
