/*
 * PID_Control_V3.ino
 * เฟิร์มแวร์สำหรับหุ่นยนต์ Mecanum 4 ล้อ (ใช้ PID Controller Library)
 * V3: ปรับโครงสร้างใช้ไลบรารี, เพิ่ม Deadzone, และปรับปรุงการ Reset
 *
 * หลักการทำงาน:
 * 1.  อ่านค่าจากรีโมทเพื่อกำหนด "ความเร็วเป้าหมาย" (Setpoint)
 * 2.  อ่านค่าจาก Encoder เพื่อหา "ความเร็วจริง" (Current Speed)
 * 3.  ใช้ไลบรารี PIDController เพื่อคำนวณค่า PWM ที่เหมาะสม
 * 4.  จัดการ Failsafe, Deadzone, และการ Reset สถานะของ PID อย่างถูกต้อง
 */

// --- 1. รวมไลบรารีที่จำเป็น ---
#include "Encoder.h"
#include "IBT2_MotorDriver.h"
#include "FlySkyi6Remote.h"
#include "PIDController.h" // V3 Added

// --- 2. การกำหนดค่าขา (Pin Definitions) ---
// (เหมือนเดิม)
#define FL_ENC_A PA2
#define FL_ENC_B PA3
#define FL_MOTOR_RPWM PA8
#define FL_MOTOR_LPWM PB6
#define FL_MOTOR_IS PA0
#define FR_ENC_A PA4
#define FR_ENC_B PA5
#define FR_MOTOR_RPWM PB7
#define FR_MOTOR_LPWM PB8
#define FR_MOTOR_IS PA1
#define BL_ENC_A PB12
#define BL_ENC_B PB13
#define BL_MOTOR_RPWM PB9
#define BL_MOTOR_LPWM PA15
#define BL_MOTOR_IS PA6
#define BR_ENC_A PB14
#define BR_ENC_B PB15
#define BR_MOTOR_RPWM PB0
#define BR_MOTOR_LPWM PB1
#define BR_MOTOR_IS PA7
#define PUMP_PIN PC13

// --- 3. ค่าคงที่และตัวแปรทั่วไป ---
const int SWITCH_MID_THRESHOLD = 1500;
const int SWITCH_LOW_THRESHOLD = 1200;
const int LOW_GEAR_SPEED = 2400;  // ticks/sec
const int HIGH_GEAR_SPEED = 4800; // ticks/sec

// --- CHANGED: PI Controller Gains ---
// ปรับค่า Kp เพื่อให้หุ่นตอบสนองเร็วขึ้น (เริ่มจากค่าน้อยๆ เช่น 0.05)
// ปรับค่า Ki เพื่อลดความคลาดเคลื่อนเมื่อวิ่งคงที่ (เริ่มจากค่าน้อยๆ เช่น 0.005)
const float Kp = 0.05; // Proportional Gain
const float Ki = 0.008; // Integral Gain
// --- END CHANGED ---

const unsigned long LOOP_INTERVAL_US = 10000; // 100 Hz control loop

// --- ADDED: Fail-safe Timeout ---
const unsigned long REMOTE_TIMEOUT_MS = 200; // หยุดมอเตอร์ถ้าไม่ได้รับสัญญาณใหม่ใน 200 ms
const int JOYSTICK_DEADZONE = 5; // V3 Added
unsigned long lastRemoteReadTime = 0;
// --- END ADDED ---

// --- 4. การสร้างอ็อบเจกต์ ---
Encoder encoder_FL(FL_ENC_A, FL_ENC_B);
Encoder encoder_FR(FR_ENC_A, FR_ENC_B);
Encoder encoder_BL(BL_ENC_A, BL_ENC_B);
Encoder encoder_BR(BR_ENC_A, BR_ENC_B);

IBT2_MotorDriver motor_FL(FL_MOTOR_RPWM, FL_MOTOR_LPWM, FL_MOTOR_IS);
IBT2_MotorDriver motor_FR(FR_MOTOR_RPWM, FR_MOTOR_LPWM, FR_MOTOR_IS);
IBT2_MotorDriver motor_BL(BL_MOTOR_RPWM, BL_MOTOR_LPWM, BL_MOTOR_IS);
IBT2_MotorDriver motor_BR(BR_MOTOR_RPWM, BR_MOTOR_LPWM, BR_MOTOR_IS);

FlySkyi6Remote remote(Serial1);
RemoteState remote_state;

// --- V3 MODIFIED: Create PID Controller objects ---
// สร้าง PID controller สำหรับแต่ละล้อ โดยใช้ค่า Kp, Ki ที่กำหนดไว้ และ Kd=0
// Output Min/Max ถูกตั้งเป็น -255/255 ซึ่งเป็นค่า PWM
PIDController pid_FL(Kp, Ki, 0, -255, 255);
PIDController pid_FR(Kp, Ki, 0, -255, 255);
PIDController pid_BL(Kp, Ki, 0, -255, 255);
PIDController pid_BR(Kp, Ki, 0, -255, 255);
// --- END V3 MODIFIED ---

// --- 5. ตัวแปรสำหรับลูปควบคุม ---
unsigned long lastRunTime = 0;
long prevPos_FL = 0, prevPos_FR = 0, prevPos_BL = 0, prevPos_BR = 0;
unsigned long prevTime = 0;


// --- V3 ADDED: Helper function for deadzone ---
int apply_deadzone(int value, int deadzone) {
    if (abs(value) <= deadzone) {
        return 0;
    }
    return value;
}
// --- END V3 ADDED ---

// --- 6. ฟังก์ชันสำหรับ Interrupt (ISR) ---
void isr_FL() { encoder_FL.update(); }
void isr_FR() { encoder_FR.update(); }
void isr_BL() { encoder_BL.update(); }
void isr_BR() { encoder_BR.update(); }

// ==================== SETUP ====================
void setup() {
    encoder_FL.begin(); motor_FL.begin();
    encoder_FR.begin(); motor_FR.begin();
    encoder_BL.begin(); motor_BL.begin();
    encoder_BR.begin(); motor_BR.begin();

    remote.begin();
    pinMode(PUMP_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW);

    attachInterrupt(digitalPinToInterrupt(FL_ENC_A), isr_FL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(FR_ENC_A), isr_FR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BL_ENC_A), isr_BL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BR_ENC_A), isr_BR, CHANGE);
    
    prevTime = micros();
    // --- ADDED: Initialize remote read time ---
    lastRemoteReadTime = millis();
    // --- END ADDED ---
}

// ==================== MAIN LOOP ====================
void loop() {
    // --- V3 MODIFIED: Remote Reading & Failsafe ---
    if (remote.read(remote_state)) {
        lastRemoteReadTime = millis();
    }

    bool is_armed = (remote_state.switch_B < SWITCH_LOW_THRESHOLD);

    // ตรวจสอบ Failsafe หรือ Disarm
    if ((millis() - lastRemoteReadTime > REMOTE_TIMEOUT_MS) || !is_armed) {
        pid_FL.reset(); pid_FR.reset();
        pid_BL.reset(); pid_BR.reset();
        motor_FL.stop(); motor_FR.stop();
        motor_BL.stop(); motor_BR.stop();
        return;
    }
    // --- END V3 MODIFIED ---

    // --- V3 MODIFIED: Apply deadzone and calculate setpoint ---
    int stick_y_raw = apply_deadzone(remote_state.right_stick_y, JOYSTICK_DEADZONE);
    int stick_x_raw = apply_deadzone(remote_state.right_stick_x, JOYSTICK_DEADZONE);
    int stick_yaw_raw = apply_deadzone(remote_state.left_stick_x, JOYSTICK_DEADZONE);

    // ถ้า Joystick อยู่ตรงกลาง, reset PID และหยุด
    if (stick_y_raw == 0 && stick_x_raw == 0 && stick_yaw_raw == 0) {
        pid_FL.reset(); pid_FR.reset();
        pid_BL.reset(); pid_BR.reset();
    }

    int max_speed = (remote_state.switch_B < SWITCH_MID_THRESHOLD) ? LOW_GEAR_SPEED : HIGH_GEAR_SPEED;
    int stick_y = map(stick_y_raw, -100, 100, -max_speed, max_speed);
    int stick_x = map(stick_x_raw, -100, 100, -max_speed, max_speed);
    int stick_yaw = map(stick_yaw_raw, -100, 100, -max_speed, max_speed);

    float setpoint_FL = stick_y + stick_x + stick_yaw;
    float setpoint_FR = stick_y - stick_x - stick_yaw;
    float setpoint_BL = stick_y - stick_x + stick_yaw;
    float setpoint_BR = stick_y + stick_x - stick_yaw;
    // --- END V3 MODIFIED ---

    // (Pump logic can be placed here)

    // --- ลูปควบคุม (ทำงานที่ความถี่คงที่) ---
    unsigned long currentTime = micros();
    if (currentTime - lastRunTime >= LOOP_INTERVAL_US) {
        lastRunTime = currentTime;
        
        long currentPos_FL = encoder_FL.getPosition();
        long currentPos_FR = encoder_FR.getPosition();
        long currentPos_BL = encoder_BL.getPosition();
        long currentPos_BR = encoder_BR.getPosition();

        unsigned long deltaTime = currentTime - prevTime;
        float current_speed_FL = 0, current_speed_FR = 0, current_speed_BL = 0, current_speed_BR = 0;
        if (deltaTime > 0) {
            current_speed_FL = (float)(currentPos_FL - prevPos_FL) * 1000000.0 / deltaTime;
            current_speed_FR = (float)(currentPos_FR - prevPos_FR) * 1000000.0 / deltaTime;
            current_speed_BL = (float)(currentPos_BL - prevPos_BL) * 1000000.0 / deltaTime;
            current_speed_BR = (float)(currentPos_BR - prevPos_BR) * 1000000.0 / deltaTime;
        }
        prevPos_FL = currentPos_FL; prevPos_FR = currentPos_FR;
        prevPos_BL = currentPos_BL; prevPos_BR = currentPos_BR;
        prevTime = currentTime;

        // --- V3 MODIFIED: Use PID Library to compute output ---
        float pwm_FL = pid_FL.compute(setpoint_FL, current_speed_FL);
        float pwm_FR = pid_FR.compute(setpoint_FR, current_speed_FR);
        float pwm_BL = pid_BL.compute(setpoint_BL, current_speed_BL);
        float pwm_BR = pid_BR.compute(setpoint_BR, current_speed_BR);
        
        motor_FL.drive(pwm_FL);
        motor_FR.drive(pwm_FR);
        motor_BL.drive(pwm_BL);
        motor_BR.drive(pwm_BR);
        // --- END V3 MODIFIED ---
    }
}