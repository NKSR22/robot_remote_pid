/*
 * PIDController.cpp
 * ไฟล์การ υλολο ของคลาส PIDController
 */

#include "PIDController.h"
#include <Arduino.h>

// Constructor: กำหนดค่าเริ่มต้นที่ปลอดภัย
PIDController::PIDController() {
  _kp = 0.0;
  _ki = 0.0;
  _kd = 0.0;
  _setpoint = 0.0;
  _min_output = -255.0; // ค่าเริ่มต้นสำหรับ PWM
  _max_output = 255.0;  // ค่าเริ่มต้นสำหรับ PWM
  reset();
}

// setGains: ตั้งค่าสัมประสิทธิ์ PID
void PIDController::setGains(float p, float i, float d) {
  _kp = p;
  _ki = i;
  _kd = d;
}

// setSetpoint: ตั้งค่าเป้าหมาย
void PIDController::setSetpoint(float setpoint) {
  _setpoint = setpoint;
}

// setOutputLimits: กำหนดขอบเขตของ output
void PIDController::setOutputLimits(float min, float max) {
  _min_output = min;
  _max_output = max;
}

// reset: รีเซ็ตสถานะภายในของ PID
void PIDController::reset() {
  _last_error = 0.0f;
  _integral = 0.0f;
  _last_time = micros(); // ใช้ micros() เพื่อความละเอียดสูง
}

// compute: คำนวณ PID output
float PIDController::compute(float actual_value) {
  // --- 1. คำนวณเวลาที่ผ่านไป (Delta Time) ---
  unsigned long current_time = micros();
  unsigned long dt_us = current_time - _last_time;

  // ป้องกันการหารด้วยศูนย์ และการกระชากของค่า dt หากเวลาผ่านไปน้อยมาก
  if (dt_us == 0) {
    // หากไม่มีเวลาผ่านไปเลย อาจคืนค่า output เดิม หรือไม่คำนวณใหม่
    // ในที่นี้เลือกที่จะไม่คำนวณใหม่เพื่อรอรอบถัดไป (โดยคืนค่าที่ถูก clamp แล้ว)
    return constrain(_kp * (_setpoint - actual_value) + _ki * _integral, _min_output, _max_output);
  }

  // แปลง dt จาก microsecond เป็น second
  float dt_s = dt_us / 1000000.0f;

  // --- 2. คำนวณค่า Error ---
  float error = _setpoint - actual_value;

  // --- 3. คำนวณ Proportional (P) Term ---
  // P term ตอบสนองต่อ error ปัจจุบัน
  float p_out = _kp * error;

  // --- 4. คำนวณ Integral (I) Term ---
  // I term สะสม error ในอดีต เพื่อกำจัด steady-state error
  _integral += error * dt_s;
  // Anti-windup: จำกัดค่า integral ไม่ให้โตเกินขอบเขตของ output
  // เพื่อป้องกันการ "สะสม" มากเกินไปเมื่อระบบไม่สามารถตอบสนองได้
  _integral = constrain(_integral, _min_output, _max_output);
  float i_out = _ki * _integral;

  // --- 5. คำนวณ Derivative (D) Term ---
  // D term ทำนาย error ในอนาคต โดยดูจากอัตราการเปลี่ยนแปลงของ error
  float derivative = (error - _last_error) / dt_s;
  float d_out = _kd * derivative;

  // --- 6. รวมผลลัพธ์และจำกัดขอบเขต ---
  float output = p_out + i_out + d_out;
  output = constrain(output, _min_output, _max_output);

  // --- 7. บันทึกสถานะสำหรับรอบถัดไป ---
  _last_error = error;
  _last_time = current_time;

  return output;
}