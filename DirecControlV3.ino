/*
 * DirecControlV3.ino
 * เฟิร์มแวร์เวอร์ชันตัดทอนสำหรับหุ่นยนต์ Mecanum 4 ล้อ (ควบคุมตรง)
 * V3: เพิ่ม Joystick Deadzone เพื่อการหยุดที่แม่นยำ
 *
 * การทำงานหลัก:
 * 1.  อ่านค่าจากรีโมทคอนโทรล FlySky i6 ผ่านโปรโตคอล i-Bus
 * 2.  ตรวจสอบสวิตช์ความปลอดภัย (Arm/Disarm)
 * 3.  แปลงค่าจาก Joystick เป็นค่า PWM โดยตรง (Open-loop control)
 * 4.  คำนวณ PWM สำหรับล้อแต่ละข้าง (Mecanum Mixing)
 * 5.  สั่งขับเคลื่อนมอเตอร์โดยตรง
 *
 * **หมายเหตุ:** โค้ดเวอร์ชันนี้ไม่ใช้ PID Controller, Encoder, หรือจอ LCD
 * เหมาะสำหรับทดสอบการทำงานพื้นฐานของฮาร์ดแวร์ (มอเตอร์, ไดรเวอร์, รีโมท)
 */

// --- 1. รวมไลบรารีที่จำเป็น ---
#include "IBT2_MotorDriver.h"
#include "FlySkyi6Remote.h"

// --- 2. การกำหนดค่าขา (Pin Definitions) ---
// ล้อหน้าซ้าย (Front Left)
#define FL_MOTOR_RPWM PA8
#define FL_MOTOR_LPWM PB6
#define FL_MOTOR_IS PA0
// ล้อหน้าขวา (Front Right)
#define FR_MOTOR_RPWM PB7
#define FR_MOTOR_LPWM PB8
#define FR_MOTOR_IS PA1
// ล้อหลังซ้าย (Back Left)
#define BL_MOTOR_RPWM PB9
#define BL_MOTOR_LPWM PA15
#define BL_MOTOR_IS PA6
// ล้อหลังขวา (Back Right)
#define BR_MOTOR_RPWM PB0
#define BR_MOTOR_LPWM PB1
#define BR_MOTOR_IS PA7

// อุปกรณ์เสริม
#define PUMP_PIN PC13 // สามารถใช้ขาใดก็ได้ที่ว่าง

// --- 3. ค่าคงที่และตัวแปรทั่วไป ---
const int SWITCH_MID_THRESHOLD = 1500;
const int SWITCH_LOW_THRESHOLD = 1200;

// --- V3 ADDED: Joystick Deadzone ---
// ค่า +/- ที่จะถือว่าเป็น 0 เพื่อป้องกัน Joystick drift
const int JOYSTICK_DEADZONE = 5;
// --- END V3 ADDED ---

// ความเร็วสูงสุดในหน่วย PWM สำหรับแต่ละเกียร์
const int LOW_GEAR_PWM = 150;  // เกียร์ช้า (PWM สูงสุดที่ 150)
const int HIGH_GEAR_PWM = 255; // เกียร์เร็ว (PWM สูงสุดที่ 255)

// --- NEW: Fail-safe Timeout ---
const unsigned long REMOTE_TIMEOUT_MS = 200; // หยุดมอเตอร์ถ้าไม่ได้รับสัญญาณใหม่ใน 200 ms
unsigned long lastRemoteReadTime = 0;        // เวลาล่าสุดที่อ่านค่ารีโมทสำเร็จ
// --- END NEW ---


// --- 4. การสร้างอ็อบเจกต์ ---
// สร้างอ็อบเจกต์ Motor Driver สำหรับทุกล้อ
IBT2_MotorDriver motor_FL(FL_MOTOR_RPWM, FL_MOTOR_LPWM, FL_MOTOR_IS);
IBT2_MotorDriver motor_FR(FR_MOTOR_RPWM, FR_MOTOR_LPWM, FR_MOTOR_IS);
IBT2_MotorDriver motor_BL(BL_MOTOR_RPWM, BL_MOTOR_LPWM, BL_MOTOR_IS);
IBT2_MotorDriver motor_BR(BR_MOTOR_RPWM, BR_MOTOR_LPWM, BR_MOTOR_IS);

// สร้างอ็อบเจกต์สำหรับรีโมท (ใช้ Serial1) และ struct สำหรับเก็บสถานะ
FlySkyi6Remote remote(Serial1);
RemoteState remote_state;

// --- V3 ADDED: Helper function for deadzone ---
int apply_deadzone(int value, int deadzone) {
    if (abs(value) <= deadzone) {
        return 0;
    }
    return value;
}
// --- END V3 ADDED ---


// ==================== SETUP ====================
void setup() {
    // เริ่มการทำงานของอ็อบเจกต์ทั้งหมด
    motor_FL.begin();
    motor_FR.begin();
    motor_BL.begin();
    motor_BR.begin();

    // เริ่มการทำงานของรีโมทและตั้งค่าขาปั๊มน้ำ
    remote.begin(); // Baud rate 115200 เป็นค่า default
    pinMode(PUMP_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW); // เริ่มต้นโดยปิดปั๊ม

    // --- NEW: Initialize remote read time ---
    lastRemoteReadTime = millis();
    // --- END NEW ---
}

// ==================== MAIN LOOP ====================
void loop() {
    // --- UPDATED LOGIC FOR REMOTE READING ---
    // 1. พยายามอ่านค่าล่าสุดจากรีโมท
    if (remote.read(remote_state)) {
        // ถ้าอ่านสำเร็จ, อัปเดตเวลาล่าสุดที่ได้รับสัญญาณ
        lastRemoteReadTime = millis();
    }

    // 1.1. ตรวจสอบ Failsafe (สัญญาณขาดหาย)
    // ถ้าเวลาปัจจุบันห่างจากเวลาที่ได้รับสัญญาณครั้งล่าสุดเกินกว่าค่า Timeout
    if (millis() - lastRemoteReadTime > REMOTE_TIMEOUT_MS) {
        // ให้หยุดมอเตอร์ทั้งหมดเพื่อความปลอดภัย
        motor_FL.stop();
        motor_FR.stop();
        motor_BL.stop();
        motor_BR.stop();
        return; // ออกจาก loop เพื่อรอสัญญาณกลับมา
    }
    // --- END UPDATED LOGIC ---


    // 2. ตรวจสอบสวิตช์ความปลอดภัย (Arm/Disarm) - Switch B
    bool is_armed = (remote_state.switch_B < SWITCH_LOW_THRESHOLD);
    if (!is_armed) {
        motor_FL.stop();
        motor_FR.stop();
        motor_BL.stop();
        motor_BR.stop();
        return; // ออกจาก loop เพื่อรอคำสั่ง Arm
    }

    // 3. แปลงค่า Joystick เป็นค่า PWM โดยตรง
    int max_pwm = (remote_state.switch_B < SWITCH_MID_THRESHOLD) ? LOW_GEAR_PWM : HIGH_GEAR_PWM;

    // --- V3 MODIFIED: Apply deadzone before mapping ---
    int stick_y = apply_deadzone(remote_state.right_stick_y, JOYSTICK_DEADZONE);
    int stick_x = apply_deadzone(remote_state.right_stick_x, JOYSTICK_DEADZONE);
    int stick_yaw = apply_deadzone(remote_state.left_stick_x, JOYSTICK_DEADZONE);

    int stick_y_pwm = map(stick_y, -100, 100, -max_pwm, max_pwm); // เดินหน้า/ถอยหลัง
    int stick_x_pwm = map(stick_x, -100, 100, -max_pwm, max_pwm); // สไลด์ข้าง
    int stick_yaw_pwm = map(stick_yaw, -100, 100, -max_pwm, max_pwm); // หมุนตัว
    // --- END V3 MODIFIED ---

    // 4. คำนวณ PWM สำหรับแต่ละล้อตามหลักการ Mecanum
    int pwm_FL = stick_y_pwm + stick_x_pwm + stick_yaw_pwm;
    int pwm_FR = stick_y_pwm - stick_x_pwm - stick_yaw_pwm;
    int pwm_BL = stick_y_pwm - stick_x_pwm + stick_yaw_pwm;
    int pwm_BR = stick_y_pwm + stick_x_pwm - stick_yaw_pwm;

    // 5. จำกัดค่า PWM ให้อยู่ในช่วงที่ถูกต้อง (-255 ถึง 255)
    //    (สำคัญมาก เพราะผลรวมจากการ mixing อาจเกินค่าสูงสุดได้)
    pwm_FL = constrain(pwm_FL, -255, 255);
    pwm_FR = constrain(pwm_FR, -255, 255);
    pwm_BL = constrain(pwm_BL, -255, 255);
    pwm_BR = constrain(pwm_BR, -255, 255);

    // 6. สั่งขับเคลื่อนมอเตอร์
    motor_FL.drive(pwm_FL);
    motor_FR.drive(pwm_FR);
    motor_BL.drive(pwm_BL);
    motor_BR.drive(pwm_BR);

    // 7. จัดการฟังก์ชันปั๊มน้ำ (Toggle Switch A)
    static bool pump_on = false;
    static int last_pump_switch_state = remote_state.switch_A;
    if (remote_state.switch_A > SWITCH_MID_THRESHOLD && last_pump_switch_state <= SWITCH_MID_THRESHOLD) {
      pump_on = !pump_on;
    } else if (remote_state.switch_A < SWITCH_MID_THRESHOLD && last_pump_switch_state >= SWITCH_MID_THRESHOLD) {
      pump_on = !pump_on;
    }
    digitalWrite(PUMP_PIN, pump_on);
    last_pump_switch_state = remote_state.switch_A;
}

