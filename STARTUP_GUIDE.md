# 6-Axis Robot Digital Twin: Startup Guide

This guide details how to launch the system in **Simulation Mode** (Automated) and **Real Mode** (Hardware).

## Prerequisites
- **Docker Desktop** (Running with WSL2 backend).
- **Windows 10/11**.
- **.NET 8 Runtime** (Included in Docker for backend, required on host for Simulator).

---

## 1. Simulation Mode (One-Click Launch)
This mode launches the full ROS 2 backend (MoveIt, Gazebo, Bridge) and the Windows Client, automatically connecting them.

### Steps
1.  Open PowerShell in the project root.
2.  Run the automated launcher:
    ```powershell
    .\START_SYSTEM.ps1
    ```
3.  **That's it!** The script will:
    -   Start Docker Compose (`ros_core`, `moveit`, `gazebo`, `rosbridge`).
    -   Wait for the ROS Bridge to be ready (Port 9090).
    -   Launch `RobotSimulator.exe` which **auto-connects** to the stack.

### What to Expect
-   **Green Status**: The Simulator's status bar will show "Connected to ROS".
-   **Planning**: You can now use the "Teach" or "Run" functions. The requests are sent to MoveIt (Docker) and visualized in the Simulator.

---

## 2. Real Mode (Physical Robot)
To control the physical 6-axis arm.

### Hardware Setup
1.  Connect the robot controller via **USB** to your Windows PC.
2.  Ensure the robot power supply is ON.

### Software Setup
1.  Run the system as above (`.\START_SYSTEM.ps1`) to get the MoveIt backend running.
2.  In the **Robot Simulator**, go to the **Settings** tab (Right Panel).
3.  **Connection**:
    -   Select your COM port (e.g., `COM3`) from the dropdown.
    -   Click **Connect Serial**.
4.  **Control Loop**:
    -   The Simulator now acts as the bridge.
    -   **MoveIt Plans** -> **Simulator** -> **Serial** -> **Robot**.
    -   **Robot Encoders** -> **Serial** -> **Simulator** -> **ROS Visualization**.

---

## 3. Deployment (GitHub)
The project is configured for "Clone & Run".
1.  Clone repo.
2.  `docker compose -f docker\docker-compose.sim.yml pull` (Optional, to pre-download).
3.  `.\START_SYSTEM.ps1`.

## Troubleshooting
-   **"ROS Bridge did not become ready"**: Check Docker logs (`docker compose logs -f`). Ensure port 9090 is not blocked by firewall.
-   **Simulator fails to connect**: Click the "Connect ROS" button in the Toolbar manually.
-   **Docker Volume Issues**: If you changed files in `src/`, run `docker compose build` to update the containers.
