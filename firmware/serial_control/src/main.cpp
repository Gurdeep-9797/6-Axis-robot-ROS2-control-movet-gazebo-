#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>

// Configuration
#define NUM_JOINTS 6
#define BAUD_RATE 115200
#define PCA9685_ADDR 0x40

void parseCommand(String cmd);

// Motor Types (Define via build flags or uncomment here)
// #define MOTOR_TYPE_SERVO
// #define MOTOR_TYPE_DC_ENCODER

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

// Joint state
double target_angles[NUM_JOINTS];
double current_angles[NUM_JOINTS];
String inputBuffer = "";
bool receiving = false;

// --- SERVO CONFIGURATION ---
#ifdef MOTOR_TYPE_SERVO
// Map angles to pulse lengths (Customize these calibration values)
const int SERVO_MIN[NUM_JOINTS] = {150, 150, 150, 150, 150, 150}; // ~0 degrees
const int SERVO_MAX[NUM_JOINTS] = {600, 600, 600, 600, 600, 600}; // ~180 degrees

void setServoAngle(int joint, double angle) {
    // Map -90..90 to SERVO_MIN..SERVO_MAX
    int pulse = map(angle, -90, 90, SERVO_MIN[joint], SERVO_MAX[joint]);
    pwm.setPWM(joint, 0, pulse);
}
#endif

// --- DC MOTOR + ENCODER CONFIGURATION ---
#include "hardware_config.h"

// Encoder Pins (A, B) for 6 joints
const int ENC_PINS[NUM_JOINTS][2] = {
    {J1_ENC_A, J1_ENC_B}, 
    {J2_ENC_A, J2_ENC_B}, 
    {J3_ENC_A, J3_ENC_B}, 
    {J4_ENC_A, J4_ENC_B}, 
    {J5_ENC_A, J5_ENC_B}, 
    {J6_ENC_A, J6_ENC_B}
};

// Motor Pins (PWM channel is handled by PCA9685, Direction pins on ESP32)
// Dir pins: {IN1, IN2}
const int DIR_PINS[NUM_JOINTS][2] = {
    {J1_DIR_1, J1_DIR_2}, 
    {J2_DIR_1, J2_DIR_2}, 
    {J3_DIR_1, J3_DIR_2}, 
    {J4_DIR_1, J4_DIR_2}, 
    {J5_DIR_1, J5_DIR_2}, 
    {J6_DIR_1, J6_DIR_2}
};

ESP32Encoder encoders[NUM_JOINTS];
double Setpoint[NUM_JOINTS], Input[NUM_JOINTS], Output[NUM_JOINTS];
// PID Tunings
double Kp=2, Ki=5, Kd=1;
PID* pids[NUM_JOINTS];

void setupDCMotors() {
    ESP32Encoder::useInternalWeakPullResistors = UP;
    for(int i=0; i<NUM_JOINTS; i++) {
        encoders[i].attachHalfQuad(ENC_PINS[i][0], ENC_PINS[i][1]);
        pinMode(DIR_PINS[i][0], OUTPUT);
        pinMode(DIR_PINS[i][1], OUTPUT);
        
        // Initialize PID
        pids[i] = new PID(&Input[i], &Output[i], &Setpoint[i], Kp, Ki, Kd, DIRECT);
        pids[i]->SetMode(AUTOMATIC);
        pids[i]->SetOutputLimits(-4095, 4095); // PCA9685 12-bit resolution
    }
}

void updateDCMotors() {
    for(int i=0; i<NUM_JOINTS; i++) {
        Input[i] = encoders[i].getCount(); // Needs conversion to degrees
        Setpoint[i] = target_angles[i];    // Needs conversion to counts
        
        pids[i]->Compute();
        
        // Drive Motor
        double val = Output[i];
        if(val > 0) {
            digitalWrite(DIR_PINS[i][0], HIGH);
            digitalWrite(DIR_PINS[i][1], LOW);
        } else {
            digitalWrite(DIR_PINS[i][0], LOW);
            digitalWrite(DIR_PINS[i][1], HIGH);
            val = -val;
        }
        pwm.setPWM(i, 0, (int)val);
    }
}
#endif

// --- MAIN SETUP ---
void setup() {
    Serial.begin(BAUD_RATE);
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(50); // 50Hz for Servos
    
    #ifdef MOTOR_TYPE_DC_ENCODER
    setupDCMotors();
    #endif
    
    Serial.println("ROBOT_CONTROLLER_READY");
}

// --- COMMS LOOP ---
void loop() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '<') {
            inputBuffer = "";
            receiving = true;
        } else if (c == '>') {
            receiving = false;
            parseCommand(inputBuffer);
        } else if (receiving) {
            inputBuffer += c;
        }
    }
    
    #ifdef MOTOR_TYPE_DC_ENCODER
    updateDCMotors();
    #endif
}

void parseCommand(String cmd) {
    // Format: J0:90.5,J1:45.0,...
    int idx = 0;
    char *ptr = strtok((char*)cmd.c_str(), ",");
    while(ptr != NULL && idx < NUM_JOINTS) {
        String part = String(ptr);
        int colon = part.indexOf(':');
        if(colon > 0) {
            double ang = part.substring(colon+1).toDouble();
            target_angles[idx] = ang;
            
            #ifdef MOTOR_TYPE_SERVO
            setServoAngle(idx, ang);
            #endif
        }
        ptr = strtok(NULL, ",");
        idx++;
    }
}

