// BOLT firmware - hardware configuration (Teensy 4.1).
// Everything you may need to change for YOUR build is in this file.
#pragma once
#include <stdint.h>

// ----------------------------------------------------------------- loop
constexpr uint32_t CONTROL_HZ = 1000;           // balance loop
constexpr uint32_t TELEMETRY_HZ = 100;          // to the Raspberry Pi
constexpr float CMD_TIMEOUT_S = 0.30f;          // no command from the Pi -> stop & hold balance
constexpr float CMD_LOST_S = 3.0f;              // ... and show "lost link" eyes

// ----------------------------------------------------------------- pins
// CAN buses (each needs its own SN65HVD230 3.3 V transceiver, 120R at the far end)
//   CAN1  TX 22 / RX 23  -> left hip motors  (DM-J4310 x2)
//   CAN2  TX  1 / RX  0  -> right hip motors (DM-J4310 x2)
//   CAN3  TX 31 / RX 30  -> wheel motors     (LK MF9025 x2)
constexpr uint8_t PIN_IMU_CS = 10;              // ICM-42688-P on SPI (11 MOSI, 12 MISO, 13 SCK)
constexpr uint8_t PIN_IMU_INT = 9;
constexpr uint8_t PIN_EYES = 29;                // WS2812B rings (2 x 12 LEDs, chained) via WS2812Serial
constexpr uint8_t PIN_HEADLIGHT = 33;           // PWM -> logic-level MOSFET -> 2 x 1 W white LEDs
constexpr uint8_t PIN_IR_LIGHT = 36;            // PWM -> MOSFET -> 850 nm IR LEDs (night vision)
constexpr uint8_t PIN_BUZZER = 37;
constexpr uint8_t PIN_ESTOP_SENSE = 32;         // HIGH = e-stop loop open (pressed)
constexpr uint8_t PIN_BATT = 16;                // A2: 100k / 6.8k divider from VBAT
constexpr uint8_t PIN_CLIFF_L = 14;             // A0: Sharp GP2Y0A21YK0F (chin, 30 deg fwd of down)
constexpr uint8_t PIN_CLIFF_R = 15;             // A1
// ultrasonic sensors (RCWL-1601 / HC-SR04P - 3.3 V compatible!)
constexpr uint8_t PIN_SONAR_TRIG[4] = {2, 3, 4, 5};        // front-left, front, front-right, rear
constexpr uint8_t PIN_SONAR_ECHO[4] = {24, 25, 26, 27};

// ----------------------------------------------------------------- motors
// DM-J4310-2EC, MIT mode.  Set with the DM debugging tool:
//   CAN ID (slave) 0x01..0x04, master ID 0x11..0x14, 1 Mbit/s, PMAX 12.5, VMAX 30, TMAX 10,
//   CAN timeout 50 ms (motor disables itself if the Teensy stops talking)
struct HipMotorCfg {
  uint8_t bus;        // 1 or 2
  uint16_t id;        // slave id (command)
  uint16_t master;    // feedback id
  float sign;         // +1 / -1 : maps motor positive rotation to +phi (CCW in the x-z plane)
};
constexpr HipMotorCfg HIP[4] = {
    {1, 0x01, 0x11, -1.f},   // left  front  (phi1)
    {1, 0x02, 0x12, -1.f},   // left  rear   (phi4)
    {2, 0x03, 0x13, +1.f},   // right front  (phi1)   right side is mirrored
    {2, 0x04, 0x14, +1.f},   // right rear   (phi4)
};
constexpr float DM_PMAX = 12.5f, DM_VMAX = 30.f, DM_TMAX = 10.f;

// Mechanical calibration pose: legs fully folded against the printed end-stops.
// phi1/phi4 in that pose (from design/kinematics.py at L = L_min - 0.005, theta = 0)
constexpr float CAL_PHI1 = 0.4005f;   // rad
constexpr float CAL_PHI4 = -3.5421f;  // rad (same branch the controller uses: phi4 in (-2pi, 0))

// LK-TECH MF9025v2 on CAN3, IDs 1 and 2 (frame id 0x140 + id), 1 Mbit/s
struct WheelMotorCfg { uint16_t id; float sign; };
constexpr WheelMotorCfg WHEEL[2] = {{1, +1.f}, {2, -1.f}};   // left, right (mirrored)
constexpr float WHEEL_KT = 0.32f;            // Nm/A  - CHECK your motor's datasheet
constexpr float WHEEL_IQ_FULLSCALE_A = 16.5f;// iq = +-2048 <-> +-16.5 A (MF series)
constexpr float WHEEL_ENC_COUNTS = 65536.f;  // 16-bit encoder (some batches: 16384)

// ----------------------------------------------------------------- IMU mounting
// IMU axes -> body axes (x fwd, y left, z up).  Default: chip x fwd, y left, z up.
constexpr int8_t IMU_AXIS_MAP[3] = {0, 1, 2};
constexpr int8_t IMU_AXIS_SIGN[3] = {+1, +1, +1};

// ----------------------------------------------------------------- power
constexpr float BATT_DIVIDER = (100.f + 6.8f) / 6.8f;
constexpr float BATT_LOW_V = 21.0f;          // 6S: 3.5 V/cell -> warn, slow down
constexpr float BATT_CRIT_V = 19.8f;         // 3.3 V/cell -> sit down & power off motors
