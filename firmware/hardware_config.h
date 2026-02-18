#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

// --- ENCODER PINS (ESP32) ---
// Based on wifi_controller config (matches typical 6-axis layout)
#define J1_ENC_A 34
#define J1_ENC_B 35
#define J1_PPR 4096

#define J2_ENC_A 32
#define J2_ENC_B 33
#define J2_PPR 4096

#define J3_ENC_A 25
#define J3_ENC_B 26
#define J3_PPR 4096

#define J4_ENC_A 27
#define J4_ENC_B 14
#define J4_PPR 2048

#define J5_ENC_A 12
#define J5_ENC_B 13
#define J5_PPR 2048

#define J6_ENC_A 4
#define J6_ENC_B 16
#define J6_PPR 2048

// --- MOTOR PINS (DC Driver Dir Pins) ---
// Assumed mappings, verify with hardware!
#define J1_DIR_1 15
#define J1_DIR_2 2

#define J2_DIR_1 0
#define J2_DIR_2 4

#define J3_DIR_1 16
#define J3_DIR_2 17

#define J4_DIR_1 5
#define J4_DIR_2 18

#define J5_DIR_1 19
#define J5_DIR_2 21

#define J6_DIR_1 22
#define J6_DIR_2 23

// --- I2C / PWM ---
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define PCA9685_ADDR_HEX 0x40

#endif
