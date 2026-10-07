// Host-side test helper: encodes known Telem / Cmd frames with the FIRMWARE code and
// decodes frames given on stdin. Used by companion/tests/test_link.py to prove the
// C++ and Python implementations are byte-compatible.
//   g++ -std=c++17 -I../lib/bolt_link link_host.cpp -o link_host
#include <cstdio>
#include <cstring>
#include "bolt_link.h"
#include "../lib/bolt_drivers/motors.h"

int main(int argc, char** argv) {
  if (argc > 1 && strcmp(argv[1], "encode") == 0) {
    link::Telem t{};
    t.t_ms = 123456; t.state = 1; t.flags = 0x41;
    t.roll_mrad = -12; t.pitch_mrad = 34; t.yaw_mrad = 1571;
    t.v_mm_s = 1500; t.s_mm = -2500000; t.L_mm_x10[0] = 2200; t.L_mm_x10[1] = 2210;
    t.batt_mv = 24123; t.sonar_mm[1] = 1234; t.cliff_mm[0] = 270; t.motor_temp_max = 45;
    t.wheel_mNm[0] = -1500; t.yaw_odo_mrad = -3141;
    uint8_t buf[128];
    size_t n = link::encode(link::MSG_TELEM, &t, sizeof(t), buf);
    for (size_t i = 0; i < n; ++i) printf("%02x", buf[i]);
    printf("\n");
    return 0;
  }
  if (argc > 1 && strcmp(argv[1], "dm") == 0) {           // DM MIT round trip
    uint8_t d[8];
    motors::dm_mit(0, 0, 0, 0.02f, -3.25f, 12.5f, 30.f, 10.f, d);
    for (int i = 0; i < 8; ++i) printf("%02x", d[i]);
    printf("\n");
    return 0;
  }
  // decode: hex string of a Cmd frame on stdin
  char hex[512];
  if (!fgets(hex, sizeof(hex), stdin)) return 1;
  link::Decoder dec;
  for (size_t i = 0; hex[i] && hex[i + 1] && hex[i] != '\n'; i += 2) {
    unsigned b;
    sscanf(hex + i, "%2x", &b);
    if (dec.push((uint8_t)b) && dec.type() == link::MSG_CMD) {
      link::Cmd c;
      memcpy(&c, dec.data(), sizeof(c));
      printf("%u %u %u %d %d %u %d %d %u %u %u\n", c.mode, c.flags, c.jump_seq, c.v_mm_s, c.yaw_mrad_s, c.height_mm,
             c.roll_mrad, c.pitch_mrad, c.jump_h_mm, c.jump_mode, c.jump_dist_mm);
    }
  }
  return 0;
}
