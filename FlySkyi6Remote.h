/*
 * FlySkyi6Remote.h
 * คลาสสำหรับอ่านและถอดรหัสข้อมูลจากภาครับ (Receiver) ของ FlySky
 * ที่ใช้โปรโตคอล i-Bus
 *
 * คลาสนี้จะจัดการการสื่อสารผ่าน Serial, ตรวจสอบความถูกต้องของข้อมูล (Checksum),
 * และแปลงข้อมูลดิบจากรีโมทให้อยู่ในรูปแบบที่ใช้งานง่าย
 *
 * การใช้งาน:
 * 1. สร้างอ็อบเจกต์ FlySkyi6Remote โดยระบุ HardwareSerial ที่ใช้ (เช่น Serial1)
 * 2. เรียกใช้ begin() ในฟังก์ชัน setup()
 * 3. ใน loop(), เรียกใช้ read() เพื่อพยายามอ่านข้อมูลล่าสุด
 * 4. ถ้า read() คืนค่า true, แสดงว่าได้รับข้อมูลใหม่แล้วและจะถูกเก็บใน struct RemoteState
 */

#ifndef FLYSKY_I6_REMOTE_H
#define FLYSKY_I6_REMOTE_H

#include <Arduino.h>

// ค่าคงที่สำหรับโปรโตคอล i-Bus
#define IBUS_MAX_CHANNELS 14  // i-Bus รองรับสูงสุด 14 ช่องสัญญาณ
#define IBUS_PACKET_SIZE 32   // ขนาดของ i-Bus packet คือ 32 bytes

/**
 * @struct RemoteState
 * @brief โครงสร้างสำหรับเก็บสถานะของรีโมทที่ผ่านการประมวลผลแล้ว
 *        ทำให้ง่ายต่อการนำไปใช้งานในโค้ดหลัก
 */
struct RemoteState {
  // ค่าของ Stick จะถูก map ให้อยู่ในช่วง -100 ถึง 100
  // โดยมี deadband อยู่ตรงกลาง (ค่า 0)
  int right_stick_x; // Channel 1
  int right_stick_y; // Channel 2
  int left_stick_y;  // Channel 3
  int left_stick_x;  // Channel 4

  // ค่าของสวิตช์ จะเป็นค่าดิบจากรีโมท (ประมาณ 1000, 1500, 2000)
  int switch_A; // Channel 5
  int switch_B; // Channel 6
  int switch_C; // Channel 7
  int switch_D; // Channel 8
  int switch_E; // Channel 9
  int switch_F; // Channel 10
};

class FlySkyi6Remote {
public:
  /**
   * @brief Constructor ของคลาส
   * @param serial_port Reference ไปยัง HardwareSerial port ที่เชื่อมต่อกับ i-Bus receiver
   */
  FlySkyi6Remote(HardwareSerial& serial_port);

  /**
   * @brief เริ่มต้นการสื่อสารผ่าน Serial
   * @param baud_rate อัตราเร็วในการสื่อสาร (Baud rate), โดยปกติสำหรับ i-Bus คือ 115200
   */
  void begin(long baud_rate = 115200);

  /**
   * @brief อ่านและประมวลผลข้อมูลจาก i-Bus receiver
   *        หากได้รับ packet ที่สมบูรณ์และถูกต้อง, จะอัปเดตค่าใน 'state'
   * @param state Reference ไปยัง RemoteState struct ที่จะใช้เก็บข้อมูล
   * @return true หากได้รับข้อมูลใหม่, false หากยังไม่มีข้อมูลหรือข้อมูลผิดพลาด
   */
  bool read(RemoteState& state);

  /**
   * @brief คืนค่าจำนวน packet ที่ได้รับสำเร็จ
   * @return จำนวน packet
   */
  uint32_t getPacketCount();

  /**
   * @brief คืนค่าจำนวน packet ที่มีข้อผิดพลาด (Checksum ไม่ตรง)
   * @return จำนวน packet ที่ผิดพลาด
   */
  uint32_t getErrorCount();

private:
  // Reference ไปยัง HardwareSerial port
  HardwareSerial& _serial;
  // Buffer สำหรับเก็บข้อมูลดิบที่อ่านจาก Serial
  uint8_t _buffer[IBUS_PACKET_SIZE];
  // Array สำหรับเก็บค่าของแต่ละช่องสัญญาณ (16-bit)
  uint16_t _channels[IBUS_MAX_CHANNELS];
  // ตัวนับจำนวนข้อผิดพลาดในการรับข้อมูล
  uint32_t _rxErrors;
  // ตัวนับจำนวน packet ที่ได้รับสำเร็จ
  uint32_t _packets;

  /**
   * @brief พยายามถอดรหัส i-Bus packet จาก serial buffer
   * @return true หากถอดรหัสสำเร็จ, false หากล้มเหลว
   */
  bool _parseIbus();
  
  /**
   * @brief แปลงค่าดิบของ stick (1000-2000) เป็นค่ามาตรฐาน (-100 ถึง 100)
   *        พร้อมกับใช้ deadband ตรงกลาง
   * @param raw_value ค่าดิบจากช่องสัญญาณของ stick
   * @return ค่าที่แปลงแล้วในช่วง -100 ถึง 100
   */
  int _mapStickValue(int raw_value);
};

#endif // FLYSKY_I6_REMOTE_H
