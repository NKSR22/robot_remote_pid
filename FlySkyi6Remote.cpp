/*
 * FlySkyi6Remote.cpp
 * ไฟล์การ υλολο ของคลาส FlySkyi6Remote
 */

#include "FlySkyi6Remote.h"

// ค่า Deadband สำหรับแกนสติ๊ก เพื่อป้องกันสัญญาณรบกวน (noise) ตอนที่สติ๊กอยู่ตรงกลาง
const int STICK_DEADBAND = 20;

// Constructor: กำหนดค่าเริ่มต้น
FlySkyi6Remote::FlySkyi6Remote(HardwareSerial& serial_port) : _serial(serial_port) {
  _rxErrors = 0;
  _packets = 0;
  // กำหนดค่าเริ่มต้นของทุกช่องสัญญาณให้เป็น 1500 (ค่ากลาง)
  for (int i = 0; i < IBUS_MAX_CHANNELS; i++) {
    _channels[i] = 1500; 
  }
}

// begin: เริ่มต้นการทำงานของ Serial
void FlySkyi6Remote::begin(long baud_rate) {
  _serial.begin(baud_rate);
}

// read: พยายามอ่านข้อมูลล่าสุดและอัปเดต state
bool FlySkyi6Remote::read(RemoteState& state) {
  // เรียก _parseIbus() เพื่อพยายามถอดรหัส packet ล่าสุด
  if (_parseIbus()) {
    // ถ้าสำเร็จ, ทำการ map ค่าจากช่องสัญญาณต่างๆ ไปยัง struct RemoteState
    state.right_stick_x = _mapStickValue(_channels[0]); // CH1
    state.right_stick_y = _mapStickValue(_channels[1]); // CH2
    state.left_stick_y  = _mapStickValue(_channels[2]); // CH3
    state.left_stick_x  = _mapStickValue(_channels[3]); // CH4

    // สำหรับสวิตช์, ใช้ค่าดิบโดยตรง (1000, 1500, 2000)
    state.switch_A = _channels[4]; // CH5
    state.switch_B = _channels[5]; // CH6
    state.switch_C = _channels[6]; // CH7
    state.switch_D = _channels[7]; // CH8
    state.switch_E = _channels[8]; // CH9
    state.switch_F = _channels[9]; // CH10
    
    return true; // คืนค่า true เพื่อบอกว่ามีข้อมูลใหม่
  }
  return false; // ไม่มีข้อมูลใหม่หรือข้อมูลผิดพลาด
}

// _parseIbus: ตรรกะการถอดรหัสโปรโตคอล i-Bus
bool FlySkyi6Remote::_parseIbus() {
  // ตรวจสอบว่ามีข้อมูลใน buffer พอสำหรับ packet ที่สั้นที่สุดหรือไม่
  // (แม้ว่า i-Bus จะมีขนาดคงที่, การตรวจสอบนี้เป็น safeguard ที่ดี)
  if (_serial.available() < IBUS_PACKET_SIZE) {
    return false;
  }

  // วนลูปเพื่อค้นหาจุดเริ่มต้นของ packet
  while (_serial.available() >= IBUS_PACKET_SIZE) {
    // ตรวจสอบ byte แรก (Header) โดยไม่ดึงออกจาก buffer
    if (_serial.peek() == 0x20) { 
      // อ่านข้อมูลทั้ง packet เข้ามาใน buffer
      _serial.readBytes(_buffer, IBUS_PACKET_SIZE); 
      
      // ตรวจสอบ byte ที่สอง (Command) ซึ่งสำหรับข้อมูลช่องสัญญาณคือ 0x40
      if (_buffer[1] == 0x40) {
        // --- การตรวจสอบ Checksum ---
        // ผลรวมของทุก byte (ยกเว้น 2 byte สุดท้าย) จะต้องเท่ากับ 0xFFFF
        uint16_t checksum_calc = 0xFFFF;
        for (int i = 0; i < IBUS_PACKET_SIZE - 2; i++) {
          checksum_calc -= _buffer[i];
        }
        // ดึงค่า checksum ที่ได้รับจาก packet (เป็น Little Endian)
        uint16_t checksum_received = _buffer[IBUS_PACKET_SIZE - 2] | (_buffer[IBUS_PACKET_SIZE - 1] << 8);

        // เปรียบเทียบ checksum
        if (checksum_calc == checksum_received) {
          _packets++; // เพิ่มจำนวน packet ที่รับสำเร็จ
          // วนลูปเพื่อดึงค่า 16-bit ของแต่ละช่องสัญญาณ
          for (int i = 0; i < IBUS_MAX_CHANNELS; i++) {
            // ข้อมูลเป็นแบบ Little Endian (LSB first)
            _channels[i] = _buffer[2 + (i * 2)] | (_buffer[3 + (i * 2)] << 8);
          }
          return true; // ถอดรหัสสำเร็จ
        } else {
          _rxErrors++; // เพิ่มจำนวนข้อผิดพลาด
        }
      }
    } else {
      // หาก byte แรกไม่ใช่ Header, อ่านทิ้งไป 1 byte เพื่อหา Header ต่อไป
      _serial.read(); 
    }
  }
  return false; // ไม่พบ packet ที่สมบูรณ์
}

// _mapStickValue: แปลงค่าดิบของสติ๊กเป็น -100 ถึง 100
int FlySkyi6Remote::_mapStickValue(int raw_value) {
  // ถ้าค่าอยู่นอกโซน deadband ด้านบวก
  if (raw_value > (1500 + STICK_DEADBAND)) {
    return map(raw_value, 1501 + STICK_DEADBAND, 2000, 1, 100);
  }
  // ถ้าค่าอยู่นอกโซน deadband ด้านลบ
  else if (raw_value < (1500 - STICK_DEADBAND)) {
    return map(raw_value, 1499 - STICK_DEADBAND, 1000, -1, -100);
  }
  // ถ้าค่าอยู่ในโซน deadband, ให้คืนค่าเป็น 0
  return 0; 
}

// getPacketCount: คืนค่าจำนวน packet ที่รับสำเร็จ
uint32_t FlySkyi6Remote::getPacketCount() {
  return _packets;
}

// getErrorCount: คืนค่าจำนวน packet ที่ผิดพลาด
uint32_t FlySkyi6Remote::getErrorCount() {
  return _rxErrors;
}