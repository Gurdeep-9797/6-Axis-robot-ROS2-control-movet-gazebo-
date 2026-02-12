# SolidWorks URDF Exporter Add-In

## Prerequisites
- SolidWorks 2018 or newer
- Visual Studio 2017/2019/2022 (with .NET Desktop Development workload)
- .NET Framework 4.8

## Building the Add-In
1. Open `SolidWorksURDFExporter.csproj` in Visual Studio.
2. Check References:
   - Ensure `SolidWorks.Interop.*` references point to your SolidWorks installation (usually `C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS\api\redist\`).
3. Build the Solution (Release mode recommended).

## Registering the Add-In
1. Open a Command Prompt as Administrator.
2. Navigate to `bin\Release\net48`.
3. Run:
   ```cmd
   %SystemRoot%\Microsoft.NET\Framework64\v4.0.30319\RegAsm.exe /codebase SolidWorksURDFExporter.dll
   ```
4. Open SolidWorks.
5. Go to **Tools > Add-Ins**.
6. Check **SW2URDF Exporter** (Active and Start Up).

## Usage
1. Open an Assembly (.SLDASM).
2. Click **Tools > Export to URDF**.
3. Select an output directory.
4. The exporter will generate:
   - `.urdf` file
   - `meshes/` folder (STLs)

## Importing to ROS
1. Open PowerShell in the project root.
2. Run:
   ```powershell
   .\tools\import_urdf.ps1 -ExportDir "C:\Path\To\Export"
   ```
3. Rebuild your workspace:
   ```powershell
   docker compose -f docker-compose.sim.yml run --rm ros_core colcon build
   ```
