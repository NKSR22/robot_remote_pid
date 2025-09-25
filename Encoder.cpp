/*
 * Encoder.cpp
 * ไฟล์การ υλολο ของคลาส Encoder
 */

#include "Encoder.h"

// Constructor: กำหนดค่าเริ่มต้นให้กับหมายเลขขาและตัวแปรภายใน
Encoder::Encoder(int pinA, int pinB) {
    _pinA = pinA;
    _pinB = pinB;
    _position = 0;
    _lastState = 0;
}

// begin: ตั้งค่าโหมดของขาและอ่านสถานะเริ่มต้น
void Encoder::begin() {
    // ตั้งค่าขา A และ B เป็น Input พร้อมเปิดใช้งาน Pull-up resistor ภายใน
    // หมายเหตุ: สำหรับ Encoder บางประเภท (เช่น Open-Collector) อาจต้องการ
    // Pull-up resistor ภายนอกเพื่อความเสถียรของสัญญาณ
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);

    // อ่านสถานะดิจิทัลเริ่มต้นของขา A และ B
    byte a = digitalRead(_pinA);
    byte b = digitalRead(_pinB);
    // รวมสถานะของ a และ b เป็น byte เดียว (2 บิต) เพื่อเก็บไว้เปรียบเทียบ
    _lastState = (a << 1) | b;
}

// getPosition: คืนค่าตำแหน่งปัจจุบัน (atomic operation)
long Encoder::getPosition() {
    long pos;
    // ปิดการทำงานของ Interrupt ชั่วคราวเพื่อป้องกันไม่ให้ค่า _position
    // ถูกแก้ไขโดย ISR ในขณะที่กำลังอ่านค่า
    noInterrupts();
    pos = _position;
    interrupts(); // เปิดการทำงานของ Interrupt กลับคืน
    return pos;
}

// reset: รีเซ็ตตำแหน่งเป็น 0 (atomic operation)
void Encoder::reset() {
    noInterrupts();
    _position = 0;
    interrupts();
}

// setPosition: ตั้งค่าตำแหน่งใหม่ (atomic operation)
void Encoder::setPosition(long newPosition) {
    noInterrupts();
    _position = newPosition;
    interrupts();
}

// update: ฟังก์ชันหลักที่ใช้ใน ISR เพื่อถอดรหัสสัญญาณ Encoder
void Encoder::update() {
    // อ่านสถานะปัจจุบันของขา A และ B
    byte a = digitalRead(_pinA);
    byte b = digitalRead(_pinB);
    byte currentState = (a << 1) | b;

    // สร้าง "state" ขนาด 4 บิต โดยการรวมสถานะเก่า (2 บิต) กับสถานะปัจจุบัน (2 บิต)
    // ทำให้เราสามารถตรวจสอบลำดับการเปลี่ยนแปลงของสัญญาณได้
    // ตัวอย่าง: ถ้าสถานะเก่าคือ 01 และสถานะใหม่คือ 00, state จะเป็น 0b0100
    byte state = (_lastState << 2) | currentState;

    // ใช้ State Machine เพื่อตรวจสอบทิศทางการหมุน
    // ตารางนี้อ้างอิงจากหลักการทำงานของ Quadrature Encoder
    switch (state) {
        // กรณีที่หมุนตามเข็มนาฬิกา (หรือทิศทางไปข้างหน้า)
        case 0b0100: // 1 -> 0
        case 0b1101: // 3 -> 1
        case 0b1011: // 2 -> 3
        case 0b0010: // 0 -> 2
            _position++;
            break;
        // กรณีที่หมุนทวนเข็มนาฬิกา (หรือทิศทางถอยหลัง)
        case 0b1000: // 2 -> 0
        case 0b0001: // 0 -> 1
        case 0b0111: // 1 -> 3
        case 0b1110: // 3 -> 2
            _position--;
            break;
    }

    // บันทึกสถานะปัจจุบันไว้สำหรับการเปรียบเทียบในครั้งต่อไป
    _lastState = currentState;
}