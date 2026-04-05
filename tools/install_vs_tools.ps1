<#
.SYNOPSIS
    Automated Installer for Visual Studio 2022 Build Tools & .NET 8.0 SDK.
    Installs required C++ and WPF Managed Desktop components.
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

# --quiet: No UI at all
# --norestart: Don't reboot automatically
$Args = "--quiet --wait --norestart --add Microsoft.VisualStudio.Workload.VCTools --add Microsoft.VisualStudio.Workload.ManagedDesktop --includeRecommended --nocache"

Start-Process -FilePath $InstallerPath -ArgumentList $Args -Wait -WindowStyle Hidden

Write-Host "Installation Complete!" -ForegroundColor Green
Write-Host "You can now run 'build_wpf.ps1' followed by 'run_roboforge.ps1'" -ForegroundColor Cyan
Pause
