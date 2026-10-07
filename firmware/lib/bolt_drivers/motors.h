// CAN motor drivers: DM-J4310-2EC (MIT mode) and LK-TECH MF9025 (torque loop).
// Frame packing is plain C++ so it can be unit-tested on a PC (see firmware/test).
#pragma once
#include <math.h>
#include <stdint.h>

namespace motors {

inline uint32_t float_to_uint(float x, float lo, float hi, int bits) {
  if (x < lo) x = lo;
  if (x > hi) x = hi;
  return (uint32_t)((x - lo) * (float)((1u << bits) - 1) / (hi - lo) + 0.5f);
}
inline float uint_to_float(uint32_t u, float lo, float hi, int bits) {
  return (float)u * (hi - lo) / (float)((1u << bits) - 1) + lo;
}

// ------------------------------------------------------------------ DM-J4310
struct DmState {
  float pos = 0, vel = 0, torque = 0;   // motor shaft (after gearbox)
  int8_t t_mos = 0, t_rotor = 0;
  uint8_t err = 0;                      // 0 = disabled, 1 = enabled, >=8 fault codes
  uint32_t last_rx_ms = 0;
};

// special frames: enable / disable / set zero / clear error
inline void dm_special(uint8_t code, uint8_t out[8]) {
  for (int i = 0; i < 7; ++i) out[i] = 0xFF;
  out[7] = code;          // 0xFC enable, 0xFD disable, 0xFE zero, 0xFB clear error
}

// MIT command: torque feed-forward + optional (kp, kd) around (p, v)
inline void dm_mit(float p, float v, float kp, float kd, float t, float PMAX, float VMAX, float TMAX,
                   uint8_t out[8]) {
  const uint32_t pi = float_to_uint(p, -PMAX, PMAX, 16);
  const uint32_t vi = float_to_uint(v, -VMAX, VMAX, 12);
  const uint32_t kpi = float_to_uint(kp, 0.f, 500.f, 12);
  const uint32_t kdi = float_to_uint(kd, 0.f, 5.f, 12);
  const uint32_t ti = float_to_uint(t, -TMAX, TMAX, 12);
  out[0] = pi >> 8;
  out[1] = pi & 0xFF;
  out[2] = vi >> 4;
  out[3] = ((vi & 0xF) << 4) | (kpi >> 8);
  out[4] = kpi & 0xFF;
  out[5] = kdi >> 4;
  out[6] = ((kdi & 0xF) << 4) | (ti >> 8);
  out[7] = ti & 0xFF;
}

inline void dm_parse(const uint8_t d[8], float PMAX, float VMAX, float TMAX, DmState& s) {
  s.err = d[0] >> 4;
  const uint32_t p = (d[1] << 8) | d[2];
  const uint32_t v = (d[3] << 4) | (d[4] >> 4);
  const uint32_t t = ((d[4] & 0xF) << 8) | d[5];
  s.pos = uint_to_float(p, -PMAX, PMAX, 16);
  s.vel = uint_to_float(v, -VMAX, VMAX, 12);
  s.torque = uint_to_float(t, -TMAX, TMAX, 12);
  s.t_mos = (int8_t)d[6];
  s.t_rotor = (int8_t)d[7];
}

// ------------------------------------------------------------------ LK MF9025
struct LkState {
  float vel = 0;            // rad/s
  float pos = 0;            // rad, multi-turn accumulated from the encoder
  float torque = 0;         // Nm (from iq)
  int8_t temp = 0;
  uint16_t enc_prev = 0;
  bool enc_init = false;
  uint32_t last_rx_ms = 0;
};

inline void lk_simple(uint8_t cmd, uint8_t out[8]) {     // 0x88 on, 0x80 off, 0x81 stop
  for (int i = 0; i < 8; ++i) out[i] = 0;
  out[0] = cmd;
}

inline void lk_torque(float torque_nm, float kt, float iq_fullscale_a, uint8_t out[8]) {
  float amps = torque_nm / kt;
  int32_t iq = (int32_t)lroundf(amps / iq_fullscale_a * 2048.f);
  if (iq > 2000) iq = 2000;
  if (iq < -2000) iq = -2000;
  out[0] = 0xA1; out[1] = out[2] = out[3] = 0;
  out[4] = iq & 0xFF;
  out[5] = (iq >> 8) & 0xFF;
  out[6] = out[7] = 0;
}

inline void lk_parse(const uint8_t d[8], float kt, float iq_fullscale_a, float enc_counts, LkState& s) {
  // reply to 0xA1/0xA2/0x9C: [cmd, temp, iqL, iqH, speedL, speedH, encL, encH]
  s.temp = (int8_t)d[1];
  const int16_t iq = (int16_t)(d[2] | (d[3] << 8));
  const int16_t dps = (int16_t)(d[4] | (d[5] << 8));
  const uint16_t enc = (uint16_t)(d[6] | (d[7] << 8));
  s.torque = iq / 2048.f * iq_fullscale_a * kt;
  s.vel = dps * 0.017453293f;
  if (!s.enc_init) { s.enc_prev = enc; s.enc_init = true; }
  int32_t de = (int32_t)enc - (int32_t)s.enc_prev;
  const int32_t half = (int32_t)(enc_counts / 2);
  if (de > half) de -= (int32_t)enc_counts;
  if (de < -half) de += (int32_t)enc_counts;
  s.enc_prev = enc;
  s.pos += de * 6.2831853f / enc_counts;
}

}  // namespace motors
