Add-Type @"
using System;
using System.Runtime.InteropServices;
public class WinUser {
    [DllImport("user32.dll")]
    public static extern bool SetForegroundWindow(IntPtr hWnd);
    [DllImport("user32.dll", SetLastError = true)]
    public static extern bool BringWindowToTop(IntPtr hWnd);
    [DllImport("user32.dll")]
    public static extern bool ShowWindow(IntPtr hWnd, int nCmdShow);
}
"@

$process = Get-Process -Name "RoboForge_WPF" -ErrorAction SilentlyContinue
if (-not $process) {
    # Try finding the dotnet process running it
    $process = Get-Process | Where-Object { $_.MainWindowTitle -match "RoboForge" }
}

if ($process) {
    # Try multiple ways to bring it to front
    [WinUser]::ShowWindow($process.MainWindowHandle, 9)
    [WinUser]::SetForegroundWindow($process.MainWindowHandle)
    [WinUser]::BringWindowToTop($process.MainWindowHandle)
    Start-Sleep -Seconds 1
}

Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
$Screen = [System.Windows.Forms.Screen]::PrimaryScreen.Bounds
$Bitmap = New-Object System.Drawing.Bitmap $Screen.Width, $Screen.Height
$Graphics = [System.Drawing.Graphics]::FromImage($Bitmap)
$Graphics.CopyFromScreen($Screen.X, $Screen.Y, 0, 0, $Bitmap.Size)
$Bitmap.Save("C:\Users\Admin\.gemini\antigravity\brain\219f70af-d147-4ca1-aaed-8fb5415355bc\wpf_live_ui.png", [System.Drawing.Imaging.ImageFormat]::Png)
$Graphics.Dispose()
$Bitmap.Dispose()
Write-Host "Screenshot of Desktop saved."
