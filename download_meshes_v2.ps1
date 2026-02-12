$baseUrls = @(
    "https://raw.githubusercontent.com/ros-industrial/abb/master/abb_irb120_support/meshes/irb120/visual",
    "https://raw.githubusercontent.com/ros-industrial/abb/kinetic-devel/abb_irb120_support/meshes/irb120/visual"
)
$extensions = @(".stl", ".dae")
$destDir = "d:\Projects\ROBOT\6-Axis-robot-ROS2-control-movet-gazebo-\RobotSimulator\Assets\Meshes"
New-Item -ItemType Directory -Force -Path $destDir | Out-Null

$links = @("base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6")

foreach ($link in $links) {
    if (Test-Path "$destDir\$link.stl") { continue }
    
    $found = $false
    foreach ($baseUrl in $baseUrls) {
        foreach ($ext in $extensions) {
            $url = "$baseUrl/$link$ext"
            $output = "$destDir\$link$ext"
            Write-Host "Trying $url..."
            try {
                Invoke-WebRequest -Uri $url -OutFile $output -ErrorAction Stop
                Write-Host "Success: $link$ext"
                $found = $true
                break
            }
            catch {
                # Continue
            }
        }
        if ($found) { break }
    }
    if (-not $found) { Write-Host "Could not find $link" }
}
