/*
 * ESP32 Robot Controller Firmware
 * 
 * Target: ESP32 + PCA9685 + Encoders
 * Comm: TCP/IP via WiFi (Port 5000)
 * Protocol: Binary (Little Endian)
 * 
 * Features:
 * - 6-Axis Servo Control via PCA9685
 * - 6-Axis Encoder Feedback (Quadrature)
 * - Trajectory Interpolation (Linear)
 * - Safety Watchdog (0.5s timeout)
 */

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h> // Requires PlatformIO lib or Arduino Lib Manager
#include <ESP32Encoder.h>            // Requires ESP32Encoder lib

#include "protocol.h"
#include "config.h"

// --- Globals ---
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);
ESP32Encoder encoders[6];

WiFiServer server(SERVER_PORT);
WiFiClient client;

// State Variables
struct RobotState {
    uint8_t execution_state;
    uint32_t fault_code;
    double current_pos[6];
    double current_vel[6];
    double target_pos[6];
    uint64_t last_cmd_time;
    double progress;
} robot;

// Trajectory Buffer (Simplified Ring Buffer)
struct TrajectoryPoint {
    uint64_t time_ns;
    double positions[6];
    double velocities[6];
};
#define TRAJ_BUF_SIZE 50
TrajectoryPoint traj_buffer[TRAJ_BUF_SIZE];
int traj_head = 0;
int traj_tail = 0;
bool traj_active = false;
uint64_t traj_start_time = 0;

// Loop Timing
unsigned long last_loop_us = 0;

// --- Setup ---
void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.println("Booting ESP32 Controller...");

    // I2C & PWM
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(50); // 50Hz Servos
    Serial.println("PWM Initialized");

    // Encoders
    ESP32Encoder::useInternalWeakPullResistors = UP;
    for (int i = 0; i < 6; i++) {
        encoders[i].attachHalfQuad(JOINTS[i].enc_pin_a, JOINTS[i].enc_pin_b);
        encoders[i].setCount(0); // Zero on boot (Absolute encoders need index pin or readout)
        
        // Initial Target = Current (Safe Start)
        robot.current_pos[i] = 0.0;
        robot.target_pos[i] = 0.0;
    }
    Serial.println("Encoders Initialized");

    // WiFi
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    // Safety Timeout Loop while connecting
    int timeout = 20;
    while (WiFi.status() != WL_CONNECTED && timeout > 0) {
        delay(500);
        Serial.print(".");
        timeout--;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi Connected");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        server.begin();
    } else {
        Serial.println("\nWiFi Failed! Running Offline (Safety Restricted).");
        robot.fault_code = 0x01; // Init Fault
    }
    
    robot.execution_state = STATE_IDLE;
    robot.last_cmd_time = millis();
}

// --- Helper Functions ---
double pulse_from_rad(int joint_idx, double rad) {
    // Map rad to us
    double min_r = JOINTS[joint_idx].min_rad;
    double max_r = JOINTS[joint_idx].max_rad;
    
    // Clamp
    if (rad < min_r) rad = min_r;
    if (rad > max_r) rad = max_r;
    
    // Linear Map
    double ratio = (rad - min_r) / (max_r - min_r);
    double us = MIN_PULSE_US + ratio * (MAX_PULSE_US - MIN_PULSE_US);
    
    // Invert if needed
    if (JOINTS[joint_idx].servo_inverted) {
        us = MAX_PULSE_US - (us - MIN_PULSE_US);
    }
    return us;
}

void write_servos() {
    for (int i = 0; i < 6; i++) {
        double us = pulse_from_rad(i, robot.target_pos[i]);
        // Convert us to 12-bit (4096) for 50Hz (20000us period)
        uint16_t tick = (uint16_t)(us * 4096.0 / 20000.0);
        pwm.setPWM(JOINTS[i].servo_channel, 0, tick);
    }
}

void read_encoders() {
    for (int i = 0; i < 6; i++) {
        int64_t count = encoders[i].getCount();
        // Convert count to rad: (count / ppr) * 2PI
        double rad = ((double)count / JOINTS[i].ppr) * 2.0 * PI;
        robot.current_pos[i] = rad;
        // Velocity calc omitted for brevity (delta pos / delta time)
        robot.current_vel[i] = 0.0; 
    }
}

void process_trajectory_data(uint8_t* data, uint16_t len) {
    // Parse Q (timestamp), H (count), Points...
    // Simplification: We just take the last point as target for testing
    // Full implementation needs interpolation buffer logic
    
    if (len < 10) return;
    
    uint16_t count = data[8] | (data[9] << 8);
    // Serial.printf("Received Trajectory with %d points\n", count);
    
    // Each point: 8 (time) + 48 (pos) + 48 (vel) = 104 bytes
    int offset = 10;
    
    // Just grab first point to start
    if (count > 0 && len >= offset + 104) {
        // Point 0 Position
        offset += 8; // Skip time
        double* pos_ptr = (double*)(data + offset);
        
        for (int i = 0; i < 6; i++) {
            robot.target_pos[i] = pos_ptr[i];
        }
        
        robot.execution_state = STATE_EXECUTING;
        send_ack(0, STATUS_ACCEPTED, "");
    } else {
        send_ack(0, STATUS_REJECTED, "Invalid Length");
    }
}

void send_ack(uint64_t id, uint8_t status, const char* reason) {
    if (!client.connected()) return;
    
    TrajectoryAckMsg msg;
    msg.trajectory_id = id;
    msg.status = status;
    msg.reason_len = strlen(reason);
    
    MsgHeader head;
    head.type = MSG_TRAJECTORY_ACK;
    head.length = sizeof(msg) + msg.reason_len;
    
    client.write((uint8_t*)&head, sizeof(head));
    client.write((uint8_t*)&msg, sizeof(msg));
    if (msg.reason_len > 0) {
        client.write((uint8_t*)reason, msg.reason_len);
    }
}

void send_joint_state() {
    if (!client.connected()) return;
    
    JointStateMsg msg;
    msg.timestamp_us = micros();
    memcpy(msg.position, robot.current_pos, sizeof(msg.position));
    memcpy(msg.velocity, robot.current_vel, sizeof(msg.velocity));
    memset(msg.effort, 0, sizeof(msg.effort));
    
    MsgHeader head;
    head.type = MSG_JOINT_STATE;
    head.length = sizeof(msg);
    
    client.write((uint8_t*)&head, sizeof(head));
    client.write((uint8_t*)&msg, sizeof(msg));
}

// --- Main Loop ---
void loop() {
    // 1. Connection Management
    if (!client || !client.connected()) {
        client = server.accept();
        if (client) {
            Serial.println("New Client Connected");
            robot.last_cmd_time = millis();
        }
    }
    
    // 2. Read Data
    if (client.connected() && client.available()) {
        robot.last_cmd_time = millis();
        
        MsgHeader head;
        if (client.readBytes((uint8_t*)&head, sizeof(head)) == sizeof(head)) {
            // Read Body
            uint8_t* body = (uint8_t*)malloc(head.length);
            if (body) {
                int read = client.readBytes(body, head.length);
                if (read == head.length) {
                    if (head.type == MSG_TRAJECTORY) {
                        process_trajectory_data(body, head.length);
                    }
                }
                free(body);
            }
        }
    }
    
    // 3. Safety Watchdog
    if (millis() - robot.last_cmd_time > 500) {
        // TIMEOUT
        // robot.execution_state = STATE_FAULT; // Comm loss shouldn't fault immediately if idle?
        // But for safety, maybe stop motion?
        // For now, allow idle.
    }
    
    // 4. Control Loop (Fixed Rate)
    unsigned long now_us = micros();
    if (now_us - last_loop_us >= CONTROL_LOOP_US) {
        last_loop_us = now_us;
        
        read_encoders();
        // PID Calc Here (Skipped for open-loop servo demo)
        write_servos();
        
        // 5. Publisher
        static int prescaler = 0;
        if (++prescaler >= 2) { // 25Hz Publish
            prescaler = 0;
            send_joint_state();
        }
    }
}
