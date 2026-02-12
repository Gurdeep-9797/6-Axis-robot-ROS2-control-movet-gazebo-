#ifndef CONFIG_H
#define CONFIG_H

// --- WiFi Configuration ---
#define WIFI_SSID "ROBOT_WIFI"
#define WIFI_PASS "robot123"
#define SERVER_PORT 5000

// --- Hardware Settings ---
#define SERIAL_BAUD 115200
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQ 400000
#define PCA9685_ADDR 0x40

// --- Control Loop ---
#define CONTROL_LOOP_HZ 50
#define CONTROL_LOOP_US (1000000 / CONTROL_LOOP_HZ)

// --- Limits & Conversion ---
// Assumed standard 180 deg servos (500-2500us)
#define MIN_PULSE_US 500
#define MAX_PULSE_US 2500
#define NEUTRAL_PULSE_US 1500

// --- Joint Mapping (Matches hardware_map.yaml) ---
struct JointConfig {
    uint8_t servo_channel;
    bool servo_inverted;
    int8_t enc_pin_a;
    int8_t enc_pin_b;
    uint16_t ppr; // Pulses per revolution
    bool enc_inverted;
    double min_rad;
    double max_rad;
};

const JointConfig JOINTS[6] = {
    {0, false, 34, 35, 4096, false, -3.14159, 3.14159}, // Joint 1
    {1, false, 32, 33, 4096, false, -1.5708, 1.5708},   // Joint 2
    {2, false, 25, 26, 4096, false, -2.3562, 2.3562},   // Joint 3
    {3, false, 27, 14, 2048, false, -1.5708, 1.5708},   // Joint 4
    {4, false, 12, 13, 2048, false, -3.14159, 3.14159}, // Joint 5
    {5, false, 4,  16, 2048, false, -1.5708, 1.5708}    // Joint 6
};

#endif
