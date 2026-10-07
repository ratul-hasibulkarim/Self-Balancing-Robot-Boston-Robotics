// Teensy <-> Raspberry Pi link (USB CDC serial, or UART at 1 Mbaud).
// Python twin: companion/robot_brain/link.py  (keep the two in sync!)
//
// Frame:  0xA5 0x5A | type (u8) | len (u8) | payload[len] | crc16-CCITT (LE, over type..payload)
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace link {

enum : uint8_t {
  MSG_CMD = 0x01,        // Pi -> Teensy, 50 Hz
  MSG_CALIBRATE = 0x02,  // Pi -> Teensy, legs on the end stops: store zero offsets
  MSG_EYES = 0x03,       // Pi -> Teensy, eye expression / lights
  MSG_TELEM = 0x81,      // Teensy -> Pi, 100 Hz
  MSG_LOG = 0x82,        // Teensy -> Pi, text
};

#pragma pack(push, 1)
struct Cmd {               // 22 bytes
  uint8_t mode;            // 0 off, 1 balance, 2 crawl
  uint8_t flags;           // bit0 estop
  uint16_t jump_seq;       // increment to request a jump
  int16_t v_mm_s;          // forward speed
  int16_t yaw_mrad_s;      // yaw rate
  uint16_t height_mm;      // leg length target
  int16_t roll_mrad;       // body roll target
  int16_t pitch_mrad;      // body pitch target
  uint16_t jump_h_mm;      // obstacle / step height
  uint8_t jump_mode;       // 0 onto step, 1 over obstacle
  uint16_t jump_dist_mm;   // distance from the wheels to the edge, 0 = now
  uint8_t reserved[3];
};

struct Eyes {
  uint8_t expression;      // 0 normal 1 happy 2 blink 3 angry 4 sleepy 5 alert 6 lost 7 love
  uint8_t headlight;       // 0..255
  uint8_t ir_light;        // 0..255
  uint8_t r, g, b;         // eye colour
};

struct Telem {             // 64 bytes
  uint32_t t_ms;
  uint8_t state;           // bolt::State
  uint8_t flags;           // bit0 contactL bit1 contactR bit2 estop bit3 batt_low bit4 link_lost bit5 motor_fault bit6 calibrated
  int16_t roll_mrad, pitch_mrad, yaw_mrad;
  int16_t gx_mrad_s, gy_mrad_s, gz_mrad_s;
  int16_t v_mm_s;
  int32_t s_mm;            // odometry (wheel travel)
  uint16_t L_mm_x10[2];    // leg lengths, 0.1 mm
  int16_t theta_mrad;
  uint16_t batt_mv;
  uint16_t sonar_mm[4];    // FL, F, FR, Rear (0 = no echo)
  uint16_t cliff_mm[2];    // L, R
  int8_t motor_temp_max;   // deg C
  uint8_t motor_fault_mask;
  int16_t wheel_mNm[2];
  int16_t hip_mNm_max;
  int16_t yaw_odo_mrad;    // integrated gyro yaw (unwrapped, mod 32.767 rad)
  uint8_t reserved[10];
};
#pragma pack(pop)

static_assert(sizeof(Cmd) == 22, "Cmd size");
static_assert(sizeof(Telem) == 64, "Telem size");

inline uint16_t crc16(const uint8_t* d, size_t n, uint16_t crc = 0xFFFF) {
  for (size_t i = 0; i < n; ++i) {
    crc ^= (uint16_t)d[i] << 8;
    for (int b = 0; b < 8; ++b) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

// Encode one frame into buf (needs len + 6 bytes). Returns the frame size.
inline size_t encode(uint8_t type, const void* payload, uint8_t len, uint8_t* buf) {
  buf[0] = 0xA5; buf[1] = 0x5A; buf[2] = type; buf[3] = len;
  memcpy(buf + 4, payload, len);
  const uint16_t c = crc16(buf + 2, len + 2);
  buf[4 + len] = c & 0xFF;
  buf[5 + len] = c >> 8;
  return len + 6;
}

// Byte-wise streaming decoder.
class Decoder {
 public:
  // returns true when a complete, valid frame is available in type()/data()/len()
  bool push(uint8_t b) {
    switch (st_) {
      case 0: st_ = (b == 0xA5) ? 1 : 0; break;
      case 1: st_ = (b == 0x5A) ? 2 : (b == 0xA5 ? 1 : 0); break;
      case 2: type_ = b; st_ = 3; break;
      case 3: len_ = b; n_ = 0; st_ = (len_ <= sizeof(buf_)) ? (len_ ? 4 : 5) : 0; break;
      case 4: buf_[n_++] = b; if (n_ == len_) st_ = 5; break;
      case 5: crc_lo_ = b; st_ = 6; break;
      case 6: {
        st_ = 0;
        uint8_t hdr[2] = {type_, len_};
        uint16_t c = crc16(hdr, 2);
        c = crc16(buf_, len_, c);
        if (c == (uint16_t)(crc_lo_ | (b << 8))) return true;
        ++crc_errors;
        break;
      }
    }
    return false;
  }
  uint8_t type() const { return type_; }
  uint8_t len() const { return len_; }
  const uint8_t* data() const { return buf_; }
  uint32_t crc_errors = 0;

 private:
  uint8_t st_ = 0, type_ = 0, len_ = 0, n_ = 0, crc_lo_ = 0;
  uint8_t buf_[96];
};

}  // namespace link
