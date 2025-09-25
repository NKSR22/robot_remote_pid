/*
 * main_feedback_control_v2_corrected.ino
 * เฟิร์มแวร์สำหรับหุ่นยนต์ Mecanum 4 ล้อ (ปรับปรุงเป็น PI Controller)
 *
 * หลักการทำงาน:
 * 1.  อ่านค่าจากรีโมทเพื่อกำหนด "ความเร็วเป้าหมาย" (Setpoint)
 * 2.  อ่านค่าจาก Encoder เพื่อหา "ความเร็วจริง" (Current Speed)
 * 3.  คำนวณหาค่าความผิดพลาด (Error = Setpoint - Current Speed)
 * 4.  คำนวณค่า PWM จากระบบควบคุมแบบ PI Controller:
 * - P (Proportional): ตอบสนองทันทีตามขนาดของ Error (Kp * error)
 * - I (Integral): ค่อยๆ ปรับแก้เพื่อลด Error ที่ค้างอยู่ (Ki * integral_of_error)
 * - Output PWM = P + I
 * 5.  เพิ่ม Failsafe กรณีสัญญาณรีโมทขาดหาย
 */

// --- 1. รวมไลบรารีที่จำเป็น ---
#include "Encoder.h"
#include "IBT2_MotorDriver.h"
#include "FlySkyi6Remote.h"

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

// --- 5. ตัวแปรสำหรับลูปควบคุม ---
unsigned long lastRunTime = 0;
long prevPos_FL = 0, prevPos_FR = 0, prevPos_BL = 0, prevPos_BR = 0;
unsigned long prevTime = 0;

// --- CHANGED: Variables for PI controller ---
// เปลี่ยนชื่อเพื่อความชัดเจน ทำหน้าที่เก็บค่าผลรวมของ Error (Integral Term)
float integral_term_FL = 0, integral_term_FR = 0, integral_term_BL = 0, integral_term_BR = 0;
// --- END CHANGED ---

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
    // --- UPDATED LOGIC FOR REMOTE READING & FAILSAFE ---
    if (remote.read(remote_state)) {
        lastRemoteReadTime = millis(); // อัปเดตเวลาเมื่ออ่านสำเร็จ
    }

    if (millis() - lastRemoteReadTime > REMOTE_TIMEOUT_MS) {
        // หยุดมอเตอร์ทั้งหมดถ้าสัญญาณขาด
        motor_FL.stop(); motor_FR.stop();
        motor_BL.stop(); motor_BR.stop();
        return; 
    }
    // --- END UPDATED LOGIC ---

    bool is_armed = (remote_state.switch_B < SWITCH_LOW_THRESHOLD);
    if (!is_armed) {
        // Reset integral terms when disarmed to prevent sudden movements
        integral_term_FL = 0; integral_term_FR = 0;
        integral_term_BL = 0; integral_term_BR = 0;
        
        motor_FL.stop(); motor_FR.stop();
        motor_BL.stop(); motor_BR.stop();
        return;
    }

    // คำนวณความเร็วเป้าหมาย (Setpoint) จากรีโมท
    int max_speed = (remote_state.switch_B < SWITCH_MID_THRESHOLD) ? LOW_GEAR_SPEED : HIGH_GEAR_SPEED;
    int stick_y = map(remote_state.right_stick_y, -100, 100, -max_speed, max_speed);
    int stick_x = map(remote_state.right_stick_x, -100, 100, -max_speed, max_speed);
    int stick_yaw = map(remote_state.left_stick_x, -100, 100, -max_speed, max_speed);

    float setpoint_FL = stick_y + stick_x + stick_yaw;
    float setpoint_FR = stick_y - stick_x - stick_yaw;
    float setpoint_BL = stick_y - stick_x + stick_yaw;
    float setpoint_BR = stick_y + stick_x - stick_yaw;

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

        // --- HEART OF THE PI CONTROLLER ---
        // 1. คำนวณ Error
        float error_FL = setpoint_FL - current_speed_FL;
        float error_FR = setpoint_FR - current_speed_FR;
        float error_BL = setpoint_BL - current_speed_BL;
        float error_BR = setpoint_BR - current_speed_BR;

        // 2. คำนวณ Integral Term (พร้อม Anti-windup)
        integral_term_FL += Ki * error_FL;
        integral_term_FR += Ki * error_FR;
        integral_term_BL += Ki * error_BL;
        integral_term_BR += Ki * error_BR;
        
        // จำกัดค่า Integral Term เพื่อป้องกัน "Integral Windup"
        integral_term_FL = constrain(integral_term_FL, -200, 200);
        integral_term_FR = constrain(integral_term_FR, -200, 200);
        integral_term_BL = constrain(integral_term_BL, -200, 200);
        integral_term_BR = constrain(integral_term_BR, -200, 200);

        // 3. คำนวณ Final Output PWM จาก P-Term และ I-Term
        float output_pwm_FL = (Kp * error_FL) + integral_term_FL;
        float output_pwm_FR = (Kp * error_FR) + integral_term_FR;
        float output_pwm_BL = (Kp * error_BL) + integral_term_BL;
        float output_pwm_BR = (Kp * error_BR) + integral_term_BR;

        // 4. จำกัดค่า PWM สุดท้ายให้อยู่ในช่วง -255 ถึง 255
        output_pwm_FL = constrain(output_pwm_FL, -255, 255);
        output_pwm_FR = constrain(output_pwm_FR, -255, 255);
        output_pwm_BL = constrain(output_pwm_BL, -255, 255);
        output_pwm_BR = constrain(output_pwm_BR, -255, 255);
        
        // 5. สั่งขับเคลื่อนมอเตอร์
        motor_FL.drive(output_pwm_FL);
        motor_FR.drive(output_pwm_FR);
        motor_BL.drive(output_pwm_BL);
        motor_BR.drive(output_pwm_BR);
    }
}