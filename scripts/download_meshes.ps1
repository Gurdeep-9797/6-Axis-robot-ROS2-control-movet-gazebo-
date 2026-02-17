$baseUrl = "https://raw.githubusercontent.com/ros-industrial/abb/kinetic-devel/abb_irb120_support/meshes/irb120/visual"
$destDir = "d:\Projects\ROBOT\6-Axis-robot-ROS2-control-movet-gazebo-\RobotSimulator\Assets\Meshes"
New-Item -ItemType Directory -Force -Path $destDir | Out-Null

$links = @("base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6")

foreach ($link in $links) {
    $url = "$baseUrl/$link.stl"
    $output = "$destDir\$link.stl"
    Write-Host "Downloading $link.stl..."
    try {
        Invoke-WebRequest -Uri $url -OutFile $output
        Write-Host "Success."
    }
    catch {
        Write-Host "Failed to download $link.stl: $_"
    }
}
