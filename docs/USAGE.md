# Setup & Usage Guide

## 1. Hardware Requirements

| Component | Purpose | Notes |
|-----------|---------|-------|
| ESP32 Dev Module | Main controller | 30-pin or 38-pin |
| PCA9685 | 16-channel PWM servo driver | I2C address `0x40` |
| 6× Servos | Joint actuators | MG996R or similar |
| 5V 6A PSU | Servo power | **Do NOT power from ESP32** |

## 2. Wiring

| Component | Signal | ESP32 Pin | PCA9685 Ch |
|-----------|--------|-----------|------------|
| **PCA9685** | SDA | GPIO 21 | — |
| | SCL | GPIO 22 | — |
| | V+ | **External PSU** | — |
| **Joint 1** | Servo | — | Ch 0 |
| | Enc A/B | GPIO 34/35 | — |
| **Joint 2** | Servo | — | Ch 1 |
| | Enc A/B | GPIO 32/33 | — |
| **Joint 3** | Servo | — | Ch 2 |
| | Enc A/B | GPIO 25/26 | — |
| **Joint 4** | Servo | — | Ch 3 |
| | Enc A/B | GPIO 27/14 | — |
| **Joint 5** | Servo | — | Ch 4 |
| | Enc A/B | GPIO 12/13 | — |
| **Joint 6** | Servo | — | Ch 5 |
| | Enc A/B | GPIO 4/16 | — |

> [!WARNING]
> **NEVER** power servos from the ESP32. Use an external 5V supply on the PCA9685 V+ terminal. Share ground between ESP32 and PSU.

## 3. Firmware

Two firmware versions for different use cases:

### Serial Mode (`firmware/serial_control/`)
For **direct USB control** from the simulator. No WiFi needed.

```bash
# Install PlatformIO, then:
cd firmware/serial_control
pio run --target upload
```

**Protocol:** `<J0:90.50,J1:45.00,...,J5:0.00>` over USB serial at 115200 baud.

### WiFi Mode (`firmware/wifi_controller/`)
For **ROS 2 Hardware Bridge** via TCP. Operates as a real-time controller.

1. Edit `config.h` → set `WIFI_SSID` and `WIFI_PASS`
2. Open in Arduino IDE or PlatformIO
3. Upload to ESP32

**Protocol:** Binary TCP on port 5000 (matches `real_backend.py`).

> [!TIP]
> Choose **Serial** if you just want slider → robot. Choose **WiFi** if you want MoveIt path planning and collision avoidance.

## 4. Running

### A. Simulator Only (Windows)
No Docker, no ROS, no WiFi required.

1. Run `release/RobotSimulator.exe` (or build from `RobotSimulator/`)
2. Use sliders to jog joints
3. **Serial connection** (optional): Select COM port → Click **Serial**

### B. Simulator + MoveIt (Recommended)
Full collision-aware path planning.

1. Start Docker: `docker compose -f docker/docker-compose.sim.yml up`
2. Run `RobotSimulator.exe`
3. Enter `ws://localhost:9090` → Click **ROS**
4. "Go To Point" now plans through MoveIt (TRAC-IK + OMPL RRTConnect)

### C. Real Robot (Docker + WiFi Firmware)

1. Flash WiFi firmware (see above)
2. Edit `.env` → set `CONTROLLER_IP` to ESP32's IP
3. Run: `docker compose -f docker/docker-compose.real.yml up`

## 5. Troubleshooting

| Problem | Fix |
|---------|-----|
| Jittery servos | Power supply too weak or no shared ground |
| Inverted motion | Toggle `servo_inverted` in `config.h` or `hardware_map.yaml` |
| COM port unavailable | Close other serial monitors; only one app can hold the port |
| MoveIt planning fails | Check Docker logs; ensure `move_group` node is running |
| WiFi won't connect | Verify SSID/password in `config.h`; check ESP32 serial monitor |
