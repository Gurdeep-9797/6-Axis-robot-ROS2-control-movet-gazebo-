# RoboForge v8.2 — Physical Hardware Integration Guide

> **Complete reference for connecting, configuring, and operating the physical 6-axis robot arm.**
> Verified: 2026-04-08

---

## 1. System Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                        6-DOF ROBOT ARM                                │
│                                                                      │
│  J1 (Shoulder Pan)     J2 (Shoulder Lift)    J3 (Elbow)             │
│  ├─ PCA9685 Ch 0       ├─ PCA9685 Ch 1       ├─ PCA9685 Ch 2       │
│  ├─ Enc: Pins 34/35    ├─ Enc: Pins 32/33    ├─ Enc: Pins 25/26    │
│  ├─ PPR: 4096          ├─ PPR: 4096          ├─ PPR: 4096          │
│  ├─ Range: ±180°       ├─ Range: ±135°       ├─ Range: ±135°       │
│  └─ Gear: 100:1        └─ Gear: 100:1        └─ Gear: 80:1         │
│                                                                      │
│  J4 (Forearm Roll)     J5 (Wrist Pitch)      J6 (Wrist Roll)        │
│  ├─ PCA9685 Ch 3       ├─ PCA9685 Ch 4       ├─ PCA9685 Ch 5       │
│  ├─ Enc: Pins 27/14    ├─ Enc: Pins 12/13    ├─ Enc: Pins 4/16     │
│  ├─ PPR: 2048          ├─ PPR: 2048          ├─ PPR: 2048          │
│  ├─ Range: ±180°       ├─ Range: ±120°       ├─ Range: ±360°       │
│  └─ Gear: 50:1         └─ Gear: 30:1         └─ Gear: 30:1         │
│                                                                      │
│  I2C: SDA=21, SCL=22, 400kHz, PCA9685 @ 0x40                       │
│  Power: 5V 6A PSU (DO NOT power servos from ESP32)                  │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 2. Hardware Components

### ESP32 Microcontroller
| Spec | Value |
|------|-------|
| Model | ESP32-WROOM-32D |
| CPU | Xtensa dual-core LX6 @ 240 MHz |
| WiFi | 802.11 b/g/n @ 2.4 GHz |
| GPIO | 34 pins (34-39 input-only) |

### PCA9685 PWM Servo Driver
| Spec | Value |
|------|-------|
| Channels | 16 (6 used: 0-5) |
| Resolution | 12-bit (4096 steps) |
| Frequency | 50 Hz |
| I2C | 0x40, 400 kHz Fast Mode |
| Pulse Width | 500-2500 μs (configurable per joint) |
| Neutral | 1500 μs |

### Wiring
| ESP32 GPIO | Function | Notes |
|------------|----------|-------|
| 21 | I2C SDA | Also J5 Dir Pin - (DC mode conflict) |
| 22 | I2C SCL | Also J6 Dir Pin + (shared, OK) |
| 34, 35 | J1 Encoder A/B | Input-only pins |
| 32, 33 | J2 Encoder A/B | |
| 25, 26 | J3 Encoder A/B | |
| 27, 14 | J4 Encoder A/B | |
| 12, 13 | J5 Encoder A/B | |
| 4, 16 | J6 Encoder A/B | Pin 16 also J3 Dir + (DC mode conflict) |
| 15, 2 | J1 Direction + / - | |
| 0, 4 | J2 Direction + / - | Pin 0 = boot button |
| 16, 17 | J3 Direction + / - | Pin 16 = J6 Encoder B (conflict) |
| 5, 18 | J4 Direction + / - | |
| 19, 21 | J5 Direction + / - | Pin 21 = I2C SDA (conflict) |
| 22, 23 | J6 Direction + / - | Pin 22 = I2C SCL (shared) |

> **⚠️ DC Motor Mode Conflicts**:
> - GPIO 16: J3 Dir+ AND J6 Encoder B
> - GPIO 21: J5 Dir- AND I2C SDA
> - Remap required for simultaneous DC motor + encoder operation

---

## 3. Communication Protocols

### 3.1 WiFi Controller (TCP Binary)

**Firmware**: `firmware/wifi_controller/esp32_robot_controller.ino`
**Connection**: TCP `192.168.1.100:5000`

**Message Frame**:
```
┌─────────────────┬──────────────────┬──────────────────┐
│  type (1 byte)  │  length (2 B)    │  length bytes    │
└─────────────────┴──────────────────┴──────────────────┘
```

**Message Types**:
| Type | Hex | Direction | Description |
|------|-----|-----------|-------------|
| TRAJECTORY | 0x01 | PC → ESP32 | Up to 50 waypoints (104 bytes each) |
| STOP | 0x02 | PC → ESP32 | Emergency stop |
| JOINT_STATE | 0x10 | ESP32 → PC | 152 bytes (timestamp + 6 pos + 6 vel + 6 eff) |
| TRAJECTORY_ACK | 0x11 | ESP32 → PC | 73 bytes (ID + status + reason) |
| EXECUTION_STATE | 0x12 | ESP32 → PC | 71 bytes (state + progress + fault) |

**Control Loop**: 50 Hz
1. Read 6 encoders → `current_pos`
2. Write 6 servos via PCA9685 from `target_pos`
3. Publish joint state at 25 Hz (every 2nd iteration)

### 3.2 Serial Controller (Text Protocol)

**Firmware**: `firmware/serial_control/src/main.cpp`
**Connection**: USB-Serial 115200 baud

**Command**: `<J0:90.50,J1:45.00,J2:30.00,J3:0.00,J4:-15.00,J5:0.00>`
**Response**: `OK:J0=90.50,J1=45.00,...` or `ERR:Joint N out of range`

**Fixed this session**: Added missing `#ifdef MOTOR_TYPE_DC_ENCODER` preprocessor guard

### 3.3 Real Backend (ROS2 → ESP32)

**Implementation**: `src/robot_hardware_bridge/real_backend.py`
- TCP connection with auto-reconnect
- Binary protocol matching wifi_controller firmware
- Trajectory sending with acknowledgment
- Joint state reception at 25 Hz

---

## 4. Data Flow — Physical Mode

```
UI: Run Program
  → Compile blocks → TrajectorySegment[]
  → WS: roboforge/execute_program
  ▼
roboforge_bridge
  → For each segment: publish /joint_trajectory_command
  ▼
RealControllerBackend (REAL mode)
  → TCP 192.168.1.100:5000
  → MSG_TRAJECTORY (0x01)
  ▼
ESP32 wifi_controller
  → 50Hz loop: read encoders, write servos
  → MSG_JOINT_STATE (0x10) every 40ms
  ▼
PCA9685 PWM → Servos
Encoders → ESP32 → TCP → RealControllerBackend → /joint_states → Bridge → UI
```

---

## 5. Configuration Files

### Hardware Map (`controller/hardware_map.yaml`)
```yaml
communication:
  serial_baud: 115200
  command_timeout_ms: 500  # E-STOP
  feedback_rate_hz: 50

i2c:
  sda_pin: 21
  scl_pin: 22
  frequency_hz: 400000
  pca9685_address: 0x40
```

### Encoder Calibration (`config/encoder_config.yaml`)
```yaml
encoder:
  counts_per_rev: 4096
  gear_ratio: 100
  zero_offset_counts: 2048  # Requires actual calibration
  direction: 1
```

### System Parameters (`config/robot_params.yaml`)
```yaml
robot_name: "robot_arm_01"
controller:
  watchdog_timeout_ms: 10
  control_loop_hz: 1000
  controller_ip: "192.168.1.100"
  controller_port: 5000
safety:
  velocity_scale: 0.1
  collision_stop_threshold: 0.05
```

---

## 6. Deployment Modes

### Simulation (Docker)
```bash
docker compose up -d
```
| Service | Purpose |
|---------|---------|
| roboforge_core | robot_state_publisher |
| roboforge_moveit | Motion planning (MoveIt 2) |
| roboforge_pseudo_hw | 250Hz simulated joint states |
| roboforge_bridge | WS:9090, REST:8765 |
| roboforge_frontend | React IDE :3000 |

### Real Hardware
```bash
# Flash firmware
pio run -e wifi_controller -t upload

# Set controller IP
export CONTROLLER_IP=192.168.1.100

# Start stack
docker compose up -d
```

---

## 7. Safe Start Protocol

1. **Health Check**: Call `/roboforge/health_check` service → verify all components OK
2. **Encoder Verification**: Subscribe to `/joint_states`, wait for 3 consecutive valid messages
3. **Synchronization**: UI reads physical joint angles → 3D gizmos snap to match → transition to Live mode
4. **Motion Authorization**: Every trajectory requires ESP32 ACK within 100ms; 3 missed ACKs → COMM_LOSS state

---

## 8. Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Servo jitter | Insufficient power | Use dedicated 5V 6A PSU |
| Encoder failures | Invalid GPIO pins | Verify pins 34-39 are input-only |
| I2C bus hangs | Long cables, no pull-ups | Add 4.7kΩ pull-ups, keep cables < 30cm |
| WiFi drops | AP mode range limited | Use STA mode on dedicated network |
| Trajectory rejected | Joint/velocity violation | Check limits in `hardware_map.yaml` |
| High tracking error | Servo latency | Calibrate min/max pulse width per servo |
| DC motor won't compile | Preprocessor bug | Fixed: added `#ifdef MOTOR_TYPE_DC_ENCODER` |

---

## 9. Firmware Build & Flash

```bash
# WiFi Controller
pio run -e wifi_controller
pio run -e wifi_controller -t upload --upload-port 192.168.1.100

# Serial Controller
pio run -e serial_control
pio run -e serial_control -t upload --upload-port COM3

# Serial Controller with Encoder PID
pio run -e serial_control_encoder
pio run -e serial_control_encoder -t upload --upload-port COM3
```
