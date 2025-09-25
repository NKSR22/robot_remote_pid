/*
 * Encoder.h
 * คลาสสำหรับจัดการ Quadrature Encoder
 *
 * คลาสนี้มีหน้าที่อ่านสัญญาณจาก Quadrature Encoder สองช่อง (A และ B)
 * เพื่อคำนวณตำแหน่งหรือการหมุนของมอเตอร์อย่างต่อเนื่อง
 * การอัปเดตตำแหน่งถูกออกแบบมาให้ทำงานภายใน Interrupt Service Routine (ISR)
 * เพื่อความแม่นยำและไม่ให้พลาดการนับสัญญาณ
 *
 * การใช้งาน:
 * 1. สร้างอ็อบเจกต์ Encoder โดยระบุขา A และ B
 * 2. เรียกใช้ begin() ในฟังก์ชัน setup()
 * 3. ผูกฟังก์ชัน update() กับ external interrupt ของขาใดขาหนึ่ง
 * 4. เรียกใช้ getPosition() เพื่ออ่านค่าตำแหน่งล่าสุด
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
public:
    /**
     * @brief Constructor ของคลาส Encoder
     * @param pinA หมายเลขขาที่เชื่อมต่อกับช่องสัญญาณ A ของ Encoder
     * @param pinB หมายเลขขาที่เชื่อมต่อกับช่องสัญญาณ B ของ Encoder
     */
    Encoder(int pinA, int pinB);

    /**
     * @brief เริ่มต้นการทำงานของ Encoder
     *        ตั้งค่าโหมดของขาเป็น INPUT_PULLUP และอ่านสถานะเริ่มต้น
     *        หมายเหตุ: สำหรับ Encoder แบบ Open-Collector ควรใช้ตัวต้านทาน Pull-up ภายนอก
     *        เพื่อความเสถียรของสัญญาณ
     */
    void begin();

    /**
     * @brief ดึงค่าตำแหน่ง (จำนวน step) ปัจจุบันของ Encoder
     *        ฟังก์ชันนี้จะปิด Interrupt ชั่วคราวเพื่อป้องกัน Race Condition
     * @return ค่าตำแหน่งปัจจุบัน (long)
     */
    long getPosition();

    /**
     * @brief รีเซ็ตค่าตำแหน่งของ Encoder ให้เป็น 0
     */
    void reset();

    /**
     * @brief กำหนดค่าตำแหน่งของ Encoder เป็นค่าที่ต้องการ
     * @param newPosition ค่าตำแหน่งใหม่ที่ต้องการตั้ง
     */
    void setPosition(long newPosition);

    /**
     * @brief อัปเดตตำแหน่งของ Encoder ตามการเปลี่ยนแปลงของสัญญาณ
     *        ฟังก์ชันนี้ควรถูกเรียกจาก Interrupt Service Routine (ISR) เท่านั้น
     *        เพื่อการตอบสนองที่รวดเร็วและแม่นยำ
     */
    void update();

private:
    // หมายเลขขาสำหรับสัญญาณ Encoder Phase A และ B
    int _pinA;
    int _pinB;

    // ตัวแปรเก็บตำแหน่งปัจจุบัน (จำนวน step)
    // ต้องเป็น volatile เพราะค่านี้ถูกแก้ไขใน ISR และถูกอ่านจาก main loop
    volatile long _position;

    // ตัวแปรเก็บสถานะล่าสุดของขา A และ B เพื่อใช้เปรียบเทียบใน state machine
    // ต้องเป็น volatile เพราะถูกใช้งานทั้งใน ISR และนอก ISR (ใน begin)
    volatile byte _lastState;
};

#endif // ENCODER_H