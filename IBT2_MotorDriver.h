/*
 * IBT2_MotorDriver.h
 * คลาสสำหรับควบคุมมอเตอร์ไดรเวอร์รุ่น IBT-2 (BTS7960 H-Bridge)
 *
 * คลาสนี้จะจัดการการทำงานพื้นฐานของมอเตอร์ไดรเวอร์ ได้แก่:
 * - การขับเคลื่อนมอเตอร์ไปข้างหน้าและถอยหลังด้วยสัญญาณ PWM
 * - การหยุดมอเตอร์ (Brake)
 * - การอ่านค่ากระแสไฟฟ้าที่มอเตอร์ใช้งานผ่านขา Current Sense (IS)
 *
 * การใช้งาน:
 * 1. สร้างอ็อบเจกต์ IBT2_MotorDriver โดยระบุขา RPWM, LPWM, และ IS
 * 2. เรียกใช้ begin() ในฟังก์ชัน setup()
 * 3. เรียกใช้ drive(speed) เพื่อควบคุมความเร็วและทิศทาง (-255 ถึง 255)
 * 4. เรียกใช้ stop() เพื่อหยุดมอเตอร์
 * 5. เรียกใช้ getCurrent() เพื่ออ่านค่ากระแส (หน่วยเป็นแอมป์)
 */

#ifndef IBT2_MOTOR_DRIVER_H
#define IBT2_MOTOR_DRIVER_H

#include <Arduino.h>

class IBT2_MotorDriver {
public:
    /**
     * @brief Constructor ของคลาส
     * @param pinRPWM หมายเลขขาสำหรับ Right PWM (ควบคุมการหมุนไปข้างหน้า)
     * @param pinLPWM หมายเลขขาสำหรับ Left PWM (ควบคุมการหมุนถอยหลัง)
     * @param pinIS หมายเลขขาสำหรับ Current Sense (ต้องเป็นขา Analog)
     */
    IBT2_MotorDriver(int pinRPWM, int pinLPWM, int pinIS);

    /**
     * @brief เริ่มต้นการทำงานของมอเตอร์ไดรเวอร์
     *        ตั้งค่าโหมดของขาและหยุดมอเตอร์เพื่อความปลอดภัย
     */
    void begin();

    /**
     * @brief ขับเคลื่อนมอเตอร์ตามความเร็วและทิศทางที่กำหนด
     * @param speed ค่าความเร็ว, -255 (ถอยหลังเต็มที่) ถึง 255 (เดินหน้าเต็มที่)
     *              ค่า 0 จะเป็นการหยุดแบบ Brake
     */
    void drive(int speed);

    /**
     * @brief หยุดมอเตอร์ (Brake)
     *        โดยการจ่าย PWM 0 ให้กับขาทั้งสองข้าง
     */
    void stop();

    /**
     * @brief อ่านและคำนวณค่ากระแสไฟฟ้าที่มอเตอร์ใช้งาน
     * @return ค่ากระแสไฟฟ้าในหน่วยแอมแปร์ (Ampere)
     */
    float getCurrent();

private:
    // หมายเลขขาสำหรับควบคุม
    int _pinRPWM; // Right PWM (Forward)
    int _pinLPWM; // Left PWM (Reverse)
    int _pinIS;   // Current Sense (Analog)

    // ค่าคงที่สำหรับคำนวณกระแสจากขา IS (อ้างอิงจาก STM32 และ BTS7960)
    // ความละเอียดของ ADC บน STM32 คือ 12-bit (0-4095)
    const float _ADC_RESOLUTION = 4095.0;
    // แรงดันอ้างอิงของ ADC บนบอร์ด STM32
    const float _V_REF = 3.3;
    // ค่าความต้านทานของ Sense Resistor บนบอร์ด IBT-2 (ปกติคือ 1k Ohm)
    const float _R_IS = 1000.0;
    // อัตราส่วนกระแสต่อแรงดัน (Current Sense Ratio) จาก Datasheet ของ BTS7960
    // โดยทั่วไปมีค่าประมาณ 8.5 A / V หรือ 8500 A / (V*1000)
    const float _K_ILIS = 8500.0;
};

#endif // IBT2_MOTOR_DRIVER_H