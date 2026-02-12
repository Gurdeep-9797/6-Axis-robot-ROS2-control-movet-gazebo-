<#
.SYNOPSIS
    Imports a SolidWorks URDF export into the ROS 2 project.
.DESCRIPTION
    1. Copies URDF and Meshes from the Export folder to a staging area.
    2. Runs validation inside the Docker container.
    3. If valid, installs files to src/robot_description/
#>
param(
    [Parameter(Mandatory=$true)]
    [string]$ExportDir,
    
    [string]$PackageName = "robot_description"
)

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path $MyInvocation.MyCommand.Path
$ProjectRoot = Resolve-Path "$ScriptDir\..\.."
$StagingDir = "$ProjectRoot\docker\staging"
$TargetDir = "$ProjectRoot\src\$PackageName"

Write-Host "Starting URDF Import..."
Write-Host "Source: $ExportDir"
Write-Host "Target: $TargetDir"

# 1. Prepare Staging
if (Test-Path $StagingDir) { Remove-Item -Recurse -Force $StagingDir }
New-Item -ItemType Directory -Path $StagingDir | Out-Null
Copy-Item -Recurse "$ExportDir\*" "$StagingDir"

# Find .urdf file
$UrdfFile = Get-ChildItem "$StagingDir" -Filter "*.urdf" | Select-Object -First 1
if (-not $UrdfFile) {
    Write-Error "No URDF file found in export directory."
}

# 2. Run Validation in Docker
# We assume the container 'robot-dev' is running or we use a temporary one
# Mapping /ros_ws/staging to local StagingDir
Write-Host "Validating URDF inside Docker..."

# For now, we'll try to run via the base image if the main container isn't up
# Adjust image name 'robotics_base:latest' as needed
docker run --rm -v "${ProjectRoot}:/ros_ws" -w /ros_ws robotics_base:latest python3 tools/urdf_post_processing/validate_and_process.py "docker/staging/$($UrdfFile.Name)" --package $PackageName

if ($LASTEXITCODE -ne 0) {
    Write-Error "URDF Validation Failed. See output above."
}

# 3. Install to Source
Write-Host "Validation Passed. Installing files..."

# Ensure target directories exist
New-Item -ItemType Directory -Force "$TargetDir\urdf" | Out-Null
New-Item -ItemType Directory -Force "$TargetDir\meshes" | Out-Null

# Copy validated URDF
Copy-Item "$StagingDir\$($UrdfFile.Name)" "$TargetDir\urdf\robot.urdf" -Force

# Copy Meshes (STL)
$MeshFiles = Get-ChildItem "$StagingDir\meshes" -Filter "*.stl" 
if ($MeshFiles) {
    Copy-Item "$StagingDir\meshes\*.stl" "$TargetDir\meshes\" -Force
} else {
    # Sometimes SW export puts meshes in subfolders or root
    $AllStls = Get-ChildItem "$StagingDir" -Recurse -Filter "*.stl"
    foreach ($stl in $AllStls) {
        Copy-Item $stl.FullName "$TargetDir\meshes\" -Force
    }
}

Write-Host "Import Complete. Please strict-rebuild the workspace."
