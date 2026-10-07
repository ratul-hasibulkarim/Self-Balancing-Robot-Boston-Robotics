// Minimal ICM-42688-P SPI driver: +-2000 dps, +-16 g, 1 kHz ODR, anti-alias filters on.
#pragma once
#include <Arduino.h>
#include <SPI.h>

class ICM42688 {
 public:
  explicit ICM42688(uint8_t cs) : cs_(cs) {}

  bool begin() {
    pinMode(cs_, OUTPUT);
    digitalWriteFast(cs_, HIGH);
    SPI.begin();
    write(0x76, 0x00);          // bank 0
    write(0x11, 0x01);          // DEVICE_CONFIG: soft reset
    delay(5);
    if (read(0x75) != 0x47) return false;   // WHO_AM_I
    write(0x4F, 0x06);          // GYRO_CONFIG0: 2000 dps, 1 kHz
    write(0x50, 0x06);          // ACCEL_CONFIG0: 16 g, 1 kHz
    write(0x52, 0x44);          // GYRO_ACCEL_CONFIG0: UI filter BW ODR/10 for both
    write(0x4E, 0x0F);          // PWR_MGMT0: gyro + accel low-noise mode
    delay(50);
    return true;
  }

  // gyro rad/s, accel m/s^2, temperature degC (sensor frame)
  bool readAll(float g[3], float a[3], float& temp) {
    uint8_t b[14];
    burst(0x1D, b, 14);
    auto s16 = [&](int i) { return (int16_t)((b[i] << 8) | b[i + 1]); };
    temp = s16(0) / 132.48f + 25.f;
    for (int i = 0; i < 3; ++i) {
      a[i] = s16(2 + 2 * i) * (9.80665f / 2048.f);
      g[i] = s16(8 + 2 * i) * (0.0174532925f / 16.4f);
    }
    return true;
  }

 private:
  SPISettings spi_{10000000, MSBFIRST, SPI_MODE0};
  uint8_t cs_;
  void write(uint8_t r, uint8_t v) {
    SPI.beginTransaction(spi_);
    digitalWriteFast(cs_, LOW);
    SPI.transfer(r & 0x7F);
    SPI.transfer(v);
    digitalWriteFast(cs_, HIGH);
    SPI.endTransaction();
  }
  uint8_t read(uint8_t r) {
    uint8_t v;
    burst(r, &v, 1);
    return v;
  }
  void burst(uint8_t r, uint8_t* d, size_t n) {
    SPI.beginTransaction(spi_);
    digitalWriteFast(cs_, LOW);
    SPI.transfer(r | 0x80);
    for (size_t i = 0; i < n; ++i) d[i] = SPI.transfer(0);
    digitalWriteFast(cs_, HIGH);
    SPI.endTransaction();
  }
};
