// Expressive LED eyes: two chained 12-LED WS2812B rings behind the smoked face visor.
#pragma once
#include <Arduino.h>
#include <WS2812Serial.h>

class Eyes {
 public:
  static constexpr int LEDS_PER_EYE = 12;
  static constexpr int N = 2 * LEDS_PER_EYE;
  enum Expr : uint8_t { NORMAL = 0, HAPPY, BLINK, ANGRY, SLEEPY, ALERT, LOST, LOVE };

  // display buffer must live in DMAMEM: allocate it in main.cpp (N * 12 bytes)
  Eyes(uint8_t pin, void* display_buf) : leds_(N, display_buf, drawing_, pin, WS2812_GRB) {}
  void begin() { leds_.begin(); leds_.setBrightness(60); }

  void set(uint8_t expr, uint8_t r, uint8_t g, uint8_t b) { expr_ = expr; r_ = r; g_ = g; b_ = b; }

  // call at ~30 Hz
  void update(uint32_t ms) {
    for (int e = 0; e < 2; ++e)
      for (int i = 0; i < LEDS_PER_EYE; ++i) {
        // ring index 0 = top, clockwise; lower half = indices 3..9
        const bool upper = (i <= 2 || i >= 10), lower = (i >= 4 && i <= 8);
        bool on = true;
        uint8_t r = r_, g = g_, b = b_;
        switch (expr_) {
          case HAPPY:  on = upper; break;                         // ^ ^
          case BLINK:  on = ((ms / 120) % 25 == 0) ? lower : true; break;
          case ANGRY:  on = e == 0 ? (i >= 1 && i <= 7) : (i >= 5 && i <= 11); r = 255; g = 40; b = 0; break;
          case SLEEPY: on = lower; r /= 3; g /= 3; b /= 3; break;
          case ALERT:  on = (ms / 150) % 2; r = 255; g = 160; b = 0; break;
          case LOST:   on = ((i + ms / 80) % 12) < 3; r = 160; g = 0; b = 255; break;
          case LOVE:   r = 255; g = 40; b = 120; break;
          default:     if ((ms / 120) % 40 == 0) on = lower; break; // NORMAL with occasional blink
        }
        leds_.setPixel(e * LEDS_PER_EYE + i, on ? ((r << 16) | (g << 8) | b) : 0);
      }
    leds_.show();
  }

 private:
  byte drawing_[N * 3];
  WS2812Serial leds_;
  uint8_t expr_ = NORMAL, r_ = 60, g_ = 230, b_ = 255;
};
