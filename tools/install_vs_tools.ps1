<#
.SYNOPSIS
    Automated Installer for Visual Studio 2022 Build Tools.
    Installs ONLY the required C++ components (MSVC, CMake, SDK).
    Requires Administrator Privileges.
#>

$ErrorActionPreference = "Stop"

if (-not ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole] "Administrator")) {
    # Relaunch as Admin, Hidden
    Start-Process powershell -WindowStyle Hidden -Verb RunAs -ArgumentList "-File", "`"$PSCommandPath`""
    exit
}

# Download silently
$InstallerUrl = "https://aka.ms/vs/17/release/vs_buildtools.exe"
$InstallerPath = "$env:TEMP\vs_buildtools.exe"
Invoke-WebRequest -Uri $InstallerUrl -OutFile $InstallerPath

# Run Installer Silently (but errors might still pop up from OS)
# --quiet: No UI at all
# --norestart: Don't reboot automatically
$Args = "--quiet --wait --norestart --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended --nocache"

Start-Process -FilePath $InstallerPath -ArgumentList $Args -Wait -WindowStyle Hidden

Write-Host "Installation Complete!" -ForegroundColor Green
Write-Host "You can now run RUN.bat" -ForegroundColor Cyan
Pause
