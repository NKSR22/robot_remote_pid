/*
 * SimpleFeedbackControlV3.ino
 * เฟิร์มแวร์สำหรับหุ่นยนต์ Mecanum 4 ล้อ (Simple Feedback)
 * V3: แก้ไขบั๊ก, เพิ่ม Failsafe, Joystick Deadzone, และ Integral Reset
 *
 * หลักการทำงาน:
 * 1.  อ่านค่าจากรีโมทเพื่อกำหนด "ความเร็วเป้าหมาย" (Setpoint) ในหน่วย ticks/sec
 * 2.  อ่านค่าจาก Encoder เพื่อหา "ความเร็วจริง" (Current Speed)
 * 3.  คำนวณหาค่าความผิดพลาด (Error = Setpoint - Current Speed)
 * 4.  นำค่า Error มาปรับแก้ค่า PWM ที่ส่งให้มอเตอร์อย่างต่อเนื่อง
 * - ถ้าหมุนช้าไป PWM จะเพิ่มขึ้น
 * - ถ้าหมุนเร็วไป PWM จะลดลง
 * 5.  ผลลัพธ์คือระบบควบคุมแบบ Closed-loop ที่เรียบง่าย (คล้าย Integral Controller)
 * ซึ่งพยายามรักษารอบหมุนให้ได้ตามเป้าหมาย
 */

// --- 1. รวมไลบรารีที่จำเป็น ---
#include "Encoder.h"
#include "IBT2_MotorDriver.h"
#include "FlySkyi6Remote.h"

// --- 2. การกำหนดค่าขา (Pin Definitions) ---
// ล้อหน้าซ้าย (Front Left)
#define FL_ENC_A PA2
#define FL_ENC_B PA3
#define FL_MOTOR_RPWM PA8
#define FL_MOTOR_LPWM PB6
#define FL_MOTOR_IS PA0
// ล้อหน้าขวา (Front Right)
#define FR_ENC_A PA4
#define FR_ENC_B PA5
#define FR_MOTOR_RPWM PB7
#define FR_MOTOR_LPWM PB8
#define FR_MOTOR_IS PA1
// ล้อหลังซ้าย (Back Left)
#define BL_ENC_A PB12
#define BL_ENC_B PB13
#define BL_MOTOR_RPWM PB9
#define BL_MOTOR_LPWM PA15
#define BL_MOTOR_IS PA6
// ล้อหลังขวา (Back Right)
#define BR_ENC_A PB14
#define BR_ENC_B PB15
#define BR_MOTOR_RPWM PB0
#define BR_MOTOR_LPWM PB1
#define BR_MOTOR_IS PA7

// อุปกรณ์เสริม
#define PUMP_PIN PC13

// --- 3. ค่าคงที่และตัวแปรทั่วไป ---
const int SWITCH_MID_THRESHOLD = 1500;
const int SWITCH_LOW_THRESHOLD = 1200;

// --- V3 ADDED: Failsafe and Deadzone ---
const unsigned long REMOTE_TIMEOUT_MS = 200; // หยุดมอเตอร์ถ้าไม่ได้รับสัญญาณใหม่ใน 200 ms
const int JOYSTICK_DEADZONE = 5;
// --- END V3 ADDED ---

// ค่าความเร็วสูงสุดในหน่วย (ticks/sec) สำหรับแต่ละเกียร์
const int LOW_GEAR_SPEED = 2400;  // เกียร์ช้า
const int HIGH_GEAR_SPEED = 4800; // เกียร์เร็ว

// Gain สำหรับการปรับแก้ PWM (คล้าย Ki ใน PID)
// - ค่ามากไป: ระบบจะสั่น (Overshoot)
// - ค่าน้อยไป: ระบบจะตอบสนองช้า
const float K_ADJUST = 0.005;

const unsigned long LOOP_INTERVAL_US = 10000; // 100 Hz control loop

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

// --- 5. ตัวแปรสำหรับลูปควบคุม ---
unsigned long lastRemoteReadTime = 0; // V3 Added
unsigned long lastRunTime = 0;
long prevPos_FL = 0, prevPos_FR = 0, prevPos_BL = 0, prevPos_BR = 0;
unsigned long prevTime = 0;

// ตัวแปรสำหรับเก็บค่า PWM ของแต่ละล้อ
float output_pwm_FL = 0, output_pwm_FR = 0, output_pwm_BL = 0, output_pwm_BR = 0;

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
    lastRemoteReadTime = millis(); // V3 Added: Initialize failsafe timer
}

// ==================== MAIN LOOP ====================
void loop() {
    // --- V3 MODIFIED: Remote Reading & Failsafe ---
    if (remote.read(remote_state)) {
        lastRemoteReadTime = millis(); // อัปเดตเวลาเมื่ออ่านสำเร็จ
    }

    bool is_armed = (remote_state.switch_B < SWITCH_LOW_THRESHOLD);

    // ตรวจสอบ Failsafe หรือ Disarm: ถ้าอย่างใดอย่างหนึ่งเป็นจริง ให้หยุดทุกอย่าง
    if ((millis() - lastRemoteReadTime > REMOTE_TIMEOUT_MS) || !is_armed) {
        // Reset PWM values (Integral term)
        output_pwm_FL = 0; output_pwm_FR = 0;
        output_pwm_BL = 0; output_pwm_BR = 0;
        // Stop motors
        motor_FL.stop(); motor_FR.stop();
        motor_BL.stop(); motor_BR.stop();
        return; // ออกจาก loop เพื่อรอสถานะปกติ
    }
    // --- END V3 MODIFIED ---

    // --- V3 MODIFIED: Apply deadzone and calculate setpoint ---
    int stick_y_raw = apply_deadzone(remote_state.right_stick_y, JOYSTICK_DEADZONE);
    int stick_x_raw = apply_deadzone(remote_state.right_stick_x, JOYSTICK_DEADZONE);
    int stick_yaw_raw = apply_deadzone(remote_state.left_stick_x, JOYSTICK_DEADZONE);

    // คำนวณความเร็วเป้าหมาย (Setpoint) จากรีโมท
    int max_speed = (remote_state.switch_B < SWITCH_MID_THRESHOLD) ? LOW_GEAR_SPEED : HIGH_GEAR_SPEED;
    int stick_y = map(stick_y_raw, -100, 100, -max_speed, max_speed);
    int stick_x = map(stick_x_raw, -100, 100, -max_speed, max_speed);
    int stick_yaw = map(stick_yaw_raw, -100, 100, -max_speed, max_speed);

    float setpoint_FL = stick_y + stick_x + stick_yaw;
    float setpoint_FR = stick_y - stick_x - stick_yaw;
    float setpoint_BL = stick_y - stick_x + stick_yaw;
    float setpoint_BR = stick_y + stick_x - stick_yaw;
    // --- END V3 MODIFIED ---

    // --- V3 ADDED: Reset Integral when sticks are centered ---
    // ถ้าไม่มีคำสั่งเคลื่อนที่ ให้รีเซ็ตค่า PWM ที่สะสมไว้เพื่อป้องกันการ "ไหล"
    if (stick_y_raw == 0 && stick_x_raw == 0 && stick_yaw_raw == 0) {
        output_pwm_FL = 0; output_pwm_FR = 0;
        output_pwm_BL = 0; output_pwm_BR = 0;
    }
    // --- END V3 ADDED ---

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

        float error_FL = setpoint_FL - current_speed_FL;
        float error_FR = setpoint_FR - current_speed_FR;
        float error_BL = setpoint_BL - current_speed_BL;
        float error_BR = setpoint_BR - current_speed_BR;

        output_pwm_FL += K_ADJUST * error_FL;
        output_pwm_FR += K_ADJUST * error_FR;
        output_pwm_BL += K_ADJUST * error_BL;
        output_pwm_BR += K_ADJUST * error_BR;
        
        output_pwm_FL = constrain(output_pwm_FL, -255, 255);
        output_pwm_FR = constrain(output_pwm_FR, -255, 255);
        output_pwm_BL = constrain(output_pwm_BL, -255, 255);
        output_pwm_BR = constrain(output_pwm_BR, -255, 255);

        motor_FL.drive(output_pwm_FL);
        motor_FR.drive(output_pwm_FR);
        motor_BL.drive(output_pwm_BL);
        motor_BR.drive(output_pwm_BR);
    }
}
