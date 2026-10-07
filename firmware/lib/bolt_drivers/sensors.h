// Ultrasonic (interrupt driven, round robin) + Sharp IR cliff sensors + battery.
#pragma once
#include <Arduino.h>

class Sonars {
 public:
  void begin(const uint8_t* trig, const uint8_t* echo) {
    self_ = this;
    for (int i = 0; i < N; ++i) {
      trig_[i] = trig[i];
      echo_[i] = echo[i];
      pinMode(trig_[i], OUTPUT);
      digitalWriteFast(trig_[i], LOW);
      pinMode(echo_[i], INPUT);
    }
    attachInterrupt(echo_[0], isr0, CHANGE);
    attachInterrupt(echo_[1], isr1, CHANGE);
    attachInterrupt(echo_[2], isr2, CHANGE);
    attachInterrupt(echo_[3], isr3, CHANGE);
  }

  // call often; fires one sensor every 30 ms (cross-talk free), ~8 Hz per sensor
  void update(uint32_t now_us) {
    if (now_us - t_fire_ < 30000) return;
    const int done = cur_;
    if (rise_[done] && fall_[done] && fall_[done] > rise_[done]) {
      const uint32_t us = fall_[done] - rise_[done];
      mm[done] = (us < 25000) ? (uint16_t)(us * 0.1715f) : 0;     // 343 m/s, round trip
    } else {
      mm[done] = 0;                                                // no echo = nothing in range
    }
    cur_ = (cur_ + 1) % N;
    rise_[cur_] = fall_[cur_] = 0;
    digitalWriteFast(trig_[cur_], HIGH);
    delayMicroseconds(10);
    digitalWriteFast(trig_[cur_], LOW);
    t_fire_ = now_us;
  }

  static constexpr int N = 4;
  volatile uint16_t mm[N] = {0, 0, 0, 0};

 private:
  static inline Sonars* self_ = nullptr;
  uint8_t trig_[N], echo_[N];
  volatile uint32_t rise_[N] = {0}, fall_[N] = {0};
  uint32_t t_fire_ = 0;
  int cur_ = 0;
  void edge(int i) {
    if (digitalReadFast(echo_[i])) rise_[i] = micros();
    else fall_[i] = micros();
  }
  static void isr0() { self_->edge(0); }
  static void isr1() { self_->edge(1); }
  static void isr2() { self_->edge(2); }
  static void isr3() { self_->edge(3); }
};

// Sharp GP2Y0A21YK0F (10-80 cm): distance ~ 29.988 * V^-1.173 [cm] (power-law fit)
inline uint16_t sharp_mm(int raw, int bits = 12) {
  const float v = raw * 3.3f / (float)((1 << bits) - 1) * 1.5f;   // 1.5 = 10k/20k divider
  if (v < 0.35f) return 800;                                        // nothing (beyond range)
  float cm = 29.988f * powf(v, -1.173f);
  if (cm > 80) cm = 80;
  if (cm < 10) cm = 10;
  return (uint16_t)(cm * 10);
}
