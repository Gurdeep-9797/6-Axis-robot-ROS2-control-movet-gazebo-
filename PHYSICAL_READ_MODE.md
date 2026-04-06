# RoboForge v8.1

## Physical Encoder Read Mode

The "Physical Read Mode" establishes a direct telemetry bridge between the hardware robotic joints and the Software Frontend (React Web-IDE & WPF Offline Client). This document deeply explores the signal processing and data flow of the physical hardware loop.

### 1. Hardware Overview (ESP32 FOC Node)

Each of the 6 robot joint interfaces uses an ESP32 microcontroller paired with a SimpleFOC driver. 
- **Component**: AMT103-V Capacitive Encoders
- **Resolution**: 2048 PPR (Pulses Per Revolution), interpolated to 8192 ticks/rev.
- **Sampling Frequency**: $1000\text{Hz}$ internal FOC loop, $250\text{Hz}$ UDP telemetry broadcast.

### 2. High-Level Telemetry Flow

```text
┌─────────────────┐             ┌────────────────────┐             ┌───────────────────┐
│ ESP32 FOC Board │ ──►UDP──►   │ ros2_bridge (Port) │ ──►WS──►    │ React / WPF (UI)  │
└─────────────────┘             └────────────────────┘             └───────────────────┘
   (Hardware Lvl)                  (Middleware Lvl)                  (Application Lvl)
   Raw tick counts                 Float32 Radians                   Visual Mesh IK
```

### 3. Inputs from Physical Hardware (ESP32 ➔ Backend)

The ESP32 broadcasts a highly condensed C-struct representing the raw positional data of the joint over an isolated Wi-Fi subnet `192.168.4.X`.

#### UDP Packet Structure (Input to System)
- `byte 0`: Start frame (`0xAA`)
- `byte 1`: Joint ID (`0x01` to `0x06`)
- `byte 2-5`: Position (`int32_t` cumulative tick count)
- `byte 6`: Status Flag (`0x00` OK, `0x01` Error, `0x02` Calibrating)
- `byte 7`: CRC-8 Checksum

The `ros2_bridge` listens on `0.0.0.0:8888`. When a packet arrives, it:
1. Validates the CRC.
2. Performs a Gear Ratio offset calculation: $\theta_{rad} = \frac{Ticks}{8192 \times GearRatio} \times 2\pi$.
3. Injects this $\theta_{rad}$ into the ROS 2 `/joint_states` topic asynchronously.

### 4. Outputs to Physical Hardware (Backend ➔ ESP32)

When the trajectory planner yields a valid collision-free spline, points are sampled at $10\text{ms}$ intervals and pushed to the controllers.

#### UDP Command Structure (Output from System)
- `byte 0`: Command frame (`0xBB`)
- `byte 1`: Control Mode (`0x01` Position, `0x02` Velocity)
- `byte 2-5`: Target Position (`float32` radians)
- `byte 6-9`: Feedforward Torque (`float32` Nm)

### 5. Safe Start Protocol (Live Mode Handshake)

Before bridging physical control to the GUI:
1. **Calibration Check**: The bridge pulses `/request_home_status`. All 6 ESP32s must reply with `0x00 OK`.
2. **Ping**: 100 dummy packets are sent; a drop rate $\ge 1\%$ aborts connection.
3. **Synchronization**: The frontend receives the physical angle array, forcing the 3D Gizmos to snap to their physical equivalents. Once snapped, the interface transitions from `Simulate` to `Live`. This ensures immediate alignment between physical robot posture and visual representations inside the Web/Desktop Client.
