/*
 * IBT2_MotorDriver.cpp
 * ไฟล์การ υλολο ของคลาส IBT2_MotorDriver
 */

#include "IBT2_MotorDriver.h"

// Constructor: กำหนดค่าเริ่มต้นให้กับหมายเลขขา
IBT2_MotorDriver::IBT2_MotorDriver(int pinRPWM, int pinLPWM, int pinIS) {
    _pinRPWM = pinRPWM;
    _pinLPWM = pinLPWM;
    _pinIS = pinIS;
}

// begin: ตั้งค่าโหมดของขา
void IBT2_MotorDriver::begin() {
    // ตั้งค่าขา PWM เป็น OUTPUT
    pinMode(_pinRPWM, OUTPUT);
    pinMode(_pinLPWM, OUTPUT);
    // ตั้งค่าขา Current Sense เป็น INPUT_ANALOG
    pinMode(_pinIS, INPUT_ANALOG);
    // เริ่มต้นโดยการหยุดมอเตอร์เพื่อความปลอดภัย
    stop();
}

// drive: ขับเคลื่อนมอเตอร์
void IBT2_MotorDriver::drive(int speed) {
    // จำกัดค่า speed ให้อยู่ในช่วง -255 ถึง 255
    speed = constrain(speed, -255, 255);

    if (speed > 0) {
        // หมุนไปข้างหน้า: ให้ LPWM เป็น 0 และส่งค่า speed ให้ RPWM
        analogWrite(_pinLPWM, 0);
        analogWrite(_pinRPWM, speed);
    } else {
        // หมุนถอยหลัง: ให้ RPWM เป็น 0 และส่งค่า speed (ที่เป็นบวก) ให้ LPWM
        // ถ้า speed เป็น 0, -speed ก็คือ 0 ซึ่งจะทำให้มอเตอร์หยุด (Brake)
        analogWrite(_pinRPWM, 0);
        analogWrite(_pinLPWM, -speed);
    }
}

// stop: หยุดมอเตอร์ (Brake mode)
void IBT2_MotorDriver::stop() {
    // การตั้งค่า PWM ทั้งสองเป็น 0 จะทำให้เกิดการเบรกไฟฟ้า
    analogWrite(_pinRPWM, 0);
    analogWrite(_pinLPWM, 0);
}

// getCurrent: อ่านและคำนวณกระแสไฟฟ้า
float IBT2_MotorDriver::getCurrent() {
    // 1. อ่านค่าดิบจาก Analog-to-Digital Converter (ADC)
    int adc_raw = analogRead(_pinIS);

    // 2. แปลงค่า ADC ที่อ่านได้ (0-4095) ให้เป็นแรงดันไฟฟ้า (0-3.3V)
    float v_sense = (adc_raw / _ADC_RESOLUTION) * _V_REF;

    // 3. คำนวณกระแสที่ไหลผ่าน Sense Resistor (I = V/R)
    float i_sense = v_sense / _R_IS;

    // 4. แปลงกระแส i_sense เป็นกระแสของมอเตอร์จริงโดยใช้อัตราส่วนจาก datasheet (K_ILIS)
    float motor_current = i_sense * _K_ILIS;

    return motor_current;
}