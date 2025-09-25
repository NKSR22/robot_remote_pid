/*
 * PIDController.h
 * คลาสสำหรับระบบควบคุมแบบ PID (Proportional-Integral-Derivative)
 *
 * คลาสนี้ใช้ในการคำนวณค่า output ที่เหมาะสมเพื่อควบคุมระบบให้เข้าสู่
 * ค่าเป้าหมาย (Setpoint) ที่ต้องการ เช่น การควบคุมความเร็วมอเตอร์
 *
 * คุณสมบัติ:
 * - คำนวณ PID โดยใช้เวลาที่ผ่านไปจริง (Delta Time) ทำให้แม่นยำ
 * - มีระบบ Anti-windup เพื่อป้องกัน Integral term โตเกินขอบเขต
 * - สามารถกำหนดขอบเขตของ Output ได้
 * - สามารถรีเซ็ตสถานะภายในได้
 *
 * การใช้งาน:
 * 1. สร้างอ็อบเจกต์ PIDController
 * 2. เรียกใช้ setGains() เพื่อตั้งค่า Kp, Ki, Kd
 * 3. เรียกใช้ setOutputLimits() เพื่อกำหนดช่วงของค่า output
 * 4. ใน loop ควบคุม, เรียกใช้ setSetpoint() เพื่อตั้งค่าเป้าหมาย
 * 5. เรียกใช้ compute(current_value) อย่างสม่ำเสมอเพื่อคำนวณ output
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
public:
  /**
   * @brief Constructor ของคลาส PIDController
   */
  PIDController();

  /**
   * @brief คำนวณค่า PID output
   * @param actual_value ค่าที่วัดได้จริงจากระบบ (เช่น ความเร็วปัจจุบัน)
   * @return ค่า output ที่คำนวณได้
   */
  float compute(float actual_value);

  /**
   * @brief ตั้งค่าสัมประสิทธิ์ของ PID (Gains)
   * @param p ค่า Proportional gain (Kp)
   * @param i ค่า Integral gain (Ki)
   * @param d ค่า Derivative gain (Kd)
   */
  void setGains(float p, float i, float d);

  /**
   * @brief ตั้งค่าเป้าหมายที่ต้องการให้ระบบไปถึง
   * @param setpoint ค่าเป้าหมาย (Setpoint)
   */
  void setSetpoint(float setpoint);

  /**
   * @brief กำหนดขอบเขตของค่า output
   * @param min ค่า output ต่ำสุด
   * @param max ค่า output สูงสุด
   */
  void setOutputLimits(float min, float max);

  /**
   * @brief รีเซ็ตสถานะภายในของ PID controller
   *        (เช่น integral term, last error)
   */
  void reset();

private:
  // สัมประสิทธิ์ PID
  float _kp; // Proportional
  float _ki; // Integral
  float _kd; // Derivative

  // ค่าเป้าหมาย
  float _setpoint;

  // ขอบเขตของ Output
  float _min_output;
  float _max_output;

  // ตัวแปรสถานะภายใน
  float _last_error;      // ค่า error ของรอบที่แล้ว
  float _integral;        // ค่าที่สะสมของ error
  unsigned long _last_time; // เวลาที่คำนวณครั้งล่าสุด (หน่วยเป็น microsecond)
};

#endif // PID_CONTROLLER_H