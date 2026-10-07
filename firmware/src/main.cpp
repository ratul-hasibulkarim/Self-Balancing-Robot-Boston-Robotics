// BOLT - wheel-legged balancing robot, real-time firmware for Teensy 4.1.
//
//   1 kHz : IMU -> attitude -> whole-body controller (lib/bolt_core) -> 6 CAN motors
//   100 Hz: telemetry to the Raspberry Pi (USB serial, lib/bolt_link)
//   ~30 Hz: eyes / lights, sensors
//
// Safety layers (independent):
//   * hardware e-stop cuts motor power (and is sensed on PIN_ESTOP_SENSE)
//   * motors' own CAN timeout (50 ms, set in the DM tool) -> they go limp if this code hangs
//   * Teensy watchdog (WDOG1) resets the MCU if the loop stalls > 100 ms
//   * command timeout from the Pi -> stop and keep balancing in place
//   * tilt / fall detection in the controller -> limp, protects the electronics
//   * battery under-voltage -> slow down, then sit down and switch the motors off
#include <Arduino.h>
#include <EEPROM.h>
#include <FlexCAN_T4.h>
#include <Watchdog_t4.h>

#include "attitude.h"
#include "bolt_controller.h"
#include "bolt_link.h"
#include "config.h"
#include "eyes.h"
#include "icm42688.h"
#include "motors.h"
#include "robot_config_generated.h"
#include "sensors.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;
WDT_T4<WDT1> wdt;

ICM42688 imu(PIN_IMU_CS);
att::Mahony ahrs;
Sonars sonars;
DMAMEM byte eyes_buf[Eyes::N * 12];
Eyes eyes(PIN_EYES, eyes_buf);

bolt::Controller* ctrl = nullptr;
bolt::Inputs in{};
bolt::Command cmd{};
bolt::Outputs out{};

motors::DmState hip[4];
motors::LkState wheel[2];
float hip_offset[4] = {0, 0, 0, 0};   // motor position at the calibration pose
bool calibrated = false;

link::Decoder rx;
uint32_t last_cmd_ms = 0;
float yaw_unwrapped = 0, yaw_prev = 0;
float gyro_bias[3] = {0, 0, 0};
volatile bool tick = false;
IntervalTimer timer;
uint8_t eyes_expr = Eyes::NORMAL, eye_r = 60, eye_g = 230, eye_b = 255;

// ------------------------------------------------------------------ EEPROM calibration
struct CalBlob { uint32_t magic; float off[4]; };
constexpr uint32_t CAL_MAGIC = 0xB017CA1Bu;

void load_calibration() {
  CalBlob c;
  EEPROM.get(0, c);
  if (c.magic == CAL_MAGIC) {
    memcpy(hip_offset, c.off, sizeof(hip_offset));
    calibrated = true;
  }
}

void store_calibration() {
  // legs are resting on the printed end stops: phi = CAL_PHI
  CalBlob c{CAL_MAGIC, {}};
  for (int i = 0; i < 4; ++i) {
    const float cal_phi = (i % 2 == 0) ? CAL_PHI1 : CAL_PHI4;
    c.off[i] = hip[i].pos - HIP[i].sign * cal_phi;     // phi = sign * (pos - off)  -> off = pos - sign*phi
    hip_offset[i] = c.off[i];
  }
  EEPROM.put(0, c);
  calibrated = true;
}

inline float hip_phi(int i) { return HIP[i].sign * (hip[i].pos - hip_offset[i]); }
inline float hip_dphi(int i) { return HIP[i].sign * hip[i].vel; }

// ------------------------------------------------------------------ CAN helpers
template <typename BUS>
void send(BUS& bus, uint16_t id, const uint8_t d[8]) {
  CAN_message_t m;
  m.id = id;
  m.len = 8;
  memcpy(m.buf, d, 8);
  bus.write(m);
}

void can_send_hip(int i, const uint8_t d[8]) {
  if (HIP[i].bus == 1) send(can1, HIP[i].id, d);
  else send(can2, HIP[i].id, d);
}

void on_can_hip(const CAN_message_t& m) {
  for (int i = 0; i < 4; ++i)
    if (m.id == HIP[i].master) {
      motors::dm_parse(m.buf, DM_PMAX, DM_VMAX, DM_TMAX, hip[i]);
      hip[i].last_rx_ms = millis();
    }
}

void on_can_wheel(const CAN_message_t& m) {
  for (int i = 0; i < 2; ++i)
    if (m.id == 0x140u + WHEEL[i].id && (m.buf[0] == 0xA1 || m.buf[0] == 0x9C)) {
      motors::lk_parse(m.buf, WHEEL_KT, WHEEL_IQ_FULLSCALE_A, WHEEL_ENC_COUNTS, wheel[i]);
      wheel[i].last_rx_ms = millis();
    }
}

void motors_enable(bool on) {
  uint8_t d[8];
  for (int i = 0; i < 4; ++i) {
    motors::dm_special(on ? 0xFC : 0xFD, d);
    can_send_hip(i, d);
  }
  for (int i = 0; i < 2; ++i) {
    motors::lk_simple(on ? 0x88 : 0x80, d);
    send(can3, 0x140 + WHEEL[i].id, d);
  }
}

// ------------------------------------------------------------------ Pi link
void handle_frame(uint8_t type, const uint8_t* p, uint8_t len) {
  if (type == link::MSG_CMD && len == sizeof(link::Cmd)) {
    link::Cmd c;
    memcpy(&c, p, sizeof(c));
    cmd.mode = c.mode;
    cmd.estop = c.flags & 1;
    cmd.jump_seq = c.jump_seq;
    cmd.v = c.v_mm_s * 1e-3f;
    cmd.yaw_rate = c.yaw_mrad_s * 1e-3f;
    cmd.height = c.height_mm * 1e-3f;
    cmd.roll = c.roll_mrad * 1e-3f;
    cmd.pitch = c.pitch_mrad * 1e-3f;
    cmd.jump_height = c.jump_h_mm * 1e-3f;
    cmd.jump_mode = c.jump_mode;
    cmd.jump_dist = c.jump_dist_mm * 1e-3f;
    last_cmd_ms = millis();
  } else if (type == link::MSG_CALIBRATE && out.state == bolt::ST_OFF) {
    store_calibration();
  } else if (type == link::MSG_EYES && len == sizeof(link::Eyes)) {
    link::Eyes e;
    memcpy(&e, p, sizeof(e));
    eyes_expr = e.expression;
    eye_r = e.r; eye_g = e.g; eye_b = e.b;
    analogWrite(PIN_HEADLIGHT, e.headlight);
    analogWrite(PIN_IR_LIGHT, e.ir_light);
  }
}

void send_telemetry(float batt_v, uint16_t cliff[2]) {
  link::Telem t{};
  t.t_ms = millis();
  t.state = (uint8_t)out.state;
  const bool estop = digitalReadFast(PIN_ESTOP_SENSE);
  const bool lost = (millis() - last_cmd_ms) > CMD_LOST_S * 1000;
  uint8_t fault = 0;
  int8_t tmax = -100;
  for (int i = 0; i < 4; ++i) {
    if (hip[i].err >= 8 || millis() - hip[i].last_rx_ms > 100) fault |= 1 << i;
    tmax = max(tmax, hip[i].t_rotor);
  }
  for (int i = 0; i < 2; ++i) {
    if (millis() - wheel[i].last_rx_ms > 100) fault |= 1 << (4 + i);
    tmax = max(tmax, wheel[i].temp);
  }
  t.flags = (out.contact[0] ? 1 : 0) | (out.contact[1] ? 2 : 0) | (estop ? 4 : 0) |
            (batt_v < BATT_LOW_V ? 8 : 0) | (lost ? 16 : 0) | (fault ? 32 : 0) | (calibrated ? 64 : 0);
  t.roll_mrad = in.imu.roll * 1000;
  t.pitch_mrad = in.imu.pitch * 1000;
  t.yaw_mrad = in.imu.yaw * 1000;
  t.gx_mrad_s = in.imu.gx * 1000;
  t.gy_mrad_s = in.imu.gy * 1000;
  t.gz_mrad_s = in.imu.gz * 1000;
  t.v_mm_s = out.ds * 1000;
  t.s_mm = out.s * 1000;
  t.L_mm_x10[0] = out.L[0] * 10000;
  t.L_mm_x10[1] = out.L[1] * 10000;
  t.theta_mrad = 0.5f * (out.theta[0] + out.theta[1]) * 1000;
  t.batt_mv = batt_v * 1000;
  for (int i = 0; i < 4; ++i) t.sonar_mm[i] = sonars.mm[i];
  t.cliff_mm[0] = cliff[0];
  t.cliff_mm[1] = cliff[1];
  t.motor_temp_max = tmax;
  t.motor_fault_mask = fault;
  t.wheel_mNm[0] = out.wheel[0] * 1000;
  t.wheel_mNm[1] = out.wheel[1] * 1000;
  float hm = 0;
  for (float h : out.hip) hm = max(hm, fabsf(h));
  t.hip_mNm_max = hm * 1000;
  t.yaw_odo_mrad = (int16_t)(fmodf(yaw_unwrapped, 32.767f) * 1000);
  uint8_t buf[sizeof(t) + 6];
  const size_t n = link::encode(link::MSG_TELEM, &t, sizeof(t), buf);
  Serial.write(buf, n);
}

// ------------------------------------------------------------------ setup
void on_tick() { tick = true; }

void setup() {
  Serial.begin(2000000);           // USB CDC (baud ignored)
  pinMode(PIN_ESTOP_SENSE, INPUT_PULLUP);
  pinMode(PIN_HEADLIGHT, OUTPUT);
  pinMode(PIN_IR_LIGHT, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  analogReadResolution(12);
  eyes.begin();
  eyes.set(Eyes::SLEEPY, eye_r, eye_g, eye_b);
  eyes.update(0);

  can1.begin(); can1.setBaudRate(1000000); can1.enableFIFO(); can1.enableFIFOInterrupt(); can1.onReceive(on_can_hip);
  can2.begin(); can2.setBaudRate(1000000); can2.enableFIFO(); can2.enableFIFOInterrupt(); can2.onReceive(on_can_hip);
  can3.begin(); can3.setBaudRate(1000000); can3.enableFIFO(); can3.enableFIFOInterrupt(); can3.onReceive(on_can_wheel);

  if (!imu.begin()) {
    while (true) {                 // no IMU: never enable the motors
      eyes.set(Eyes::ANGRY, 255, 0, 0);
      eyes.update(millis());
      tone(PIN_BUZZER, 2000, 100);
      delay(500);
    }
  }
  // gyro bias: robot must be still for 1 s after power-up (it is resting on its kickstand pose)
  float g[3], a[3], tmp, asum[3] = {0, 0, 0};
  for (int n = 0; n < 1000; ++n) {
    imu.readAll(g, a, tmp);
    for (int k = 0; k < 3; ++k) { gyro_bias[k] += g[k] / 1000.f; asum[k] += a[k] / 1000.f; }
    delay(1);
  }
  ahrs.init_from_accel(asum[0], asum[1], asum[2]);

  load_calibration();
  static bolt::Controller c(bolt::default_config());
  ctrl = &c;
  sonars.begin(PIN_SONAR_TRIG, PIN_SONAR_ECHO);
  motors_enable(true);

  WDT_timings_t wcfg;
  wcfg.timeout = 100;              // ms
  wdt.begin(wcfg);
  timer.begin(on_tick, 1000000 / CONTROL_HZ);
  tone(PIN_BUZZER, 1500, 80);
}

// ------------------------------------------------------------------ loop
void loop() {
  can1.events(); can2.events(); can3.events();
  while (Serial.available()) {
    if (rx.push(Serial.read())) handle_frame(rx.type(), rx.data(), rx.len());
  }
  sonars.update(micros());
  if (!tick) return;
  tick = false;
  wdt.feed();
  static uint32_t n = 0;
  ++n;
  const float dt = 1.0f / CONTROL_HZ;

  // ---- IMU / attitude
  float g[3], a[3], temp, gb[3], ab[3];
  imu.readAll(g, a, temp);
  for (int k = 0; k < 3; ++k) {
    const int s = IMU_AXIS_MAP[k];
    gb[k] = IMU_AXIS_SIGN[k] * (g[s] - gyro_bias[s]);
    ab[k] = IMU_AXIS_SIGN[k] * a[s];
  }
  ahrs.update(gb[0], gb[1], gb[2], ab[0], ab[1], ab[2], dt);
  ahrs.euler(in.imu.roll, in.imu.pitch, in.imu.yaw);
  float dyaw = in.imu.yaw - yaw_prev;
  if (dyaw > PI) dyaw -= TWO_PI;
  if (dyaw < -PI) dyaw += TWO_PI;
  yaw_unwrapped += dyaw;
  yaw_prev = in.imu.yaw;
  in.imu.gx = gb[0]; in.imu.gy = gb[1]; in.imu.gz = gb[2];
  // vertical specific force in the world frame (used for free-fall / impact detection)
  const float cr = cosf(in.imu.roll), sr = sinf(in.imu.roll), cp = cosf(in.imu.pitch), sp = sinf(in.imu.pitch);
  in.imu.ax = ab[0]; in.imu.ay = ab[1];
  in.imu.az = -sp * ab[0] + cp * sr * ab[1] + cp * cr * ab[2];
  in.dt = dt;

  // ---- legs / wheels
  for (int s = 0; s < 2; ++s) {
    in.leg[s].phi1 = hip_phi(2 * s);
    in.leg[s].phi4 = hip_phi(2 * s + 1);
    in.leg[s].dphi1 = hip_dphi(2 * s);
    in.leg[s].dphi4 = hip_dphi(2 * s + 1);
    in.leg[s].wheel_vel = WHEEL[s].sign * wheel[s].vel;
  }

  // ---- battery / safety
  static float batt_v = 24.f;
  batt_v += (analogRead(PIN_BATT) * 3.3f / 4095.f * BATT_DIVIDER - batt_v) * 0.002f;
  in.battery_v = batt_v;
  const bool estop_hw = digitalReadFast(PIN_ESTOP_SENSE);
  const float since_cmd = (millis() - last_cmd_ms) * 1e-3f;
  bolt::Command c = cmd;
  if (since_cmd > CMD_TIMEOUT_S) { c.v = 0; c.yaw_rate = 0; c.jump_seq = cmd.jump_seq; }
  if (batt_v < BATT_LOW_V) c.v = constrain(c.v, -0.5f, 0.5f);
  if (batt_v < BATT_CRIT_V || estop_hw || !calibrated) c.mode = bolt::MODE_OFF;
  if (!calibrated) c.estop = 1;

  ctrl->step(in, c, out);

  // ---- motor commands (pure torque; tiny kd for smoothness)
  uint8_t d[8];
  for (int i = 0; i < 4; ++i) {
    const float tq = HIP[i].sign * out.hip[i];
    motors::dm_mit(0, 0, 0, 0.02f, tq, DM_PMAX, DM_VMAX, DM_TMAX, d);
    can_send_hip(i, d);
  }
  for (int i = 0; i < 2; ++i) {
    motors::lk_torque(WHEEL[i].sign * out.wheel[i], WHEEL_KT, WHEEL_IQ_FULLSCALE_A, d);
    send(can3, 0x140 + WHEEL[i].id, d);
  }

  // ---- slower tasks
  static uint16_t cliff[2] = {400, 400};
  if (n % (CONTROL_HZ / TELEMETRY_HZ) == 0) {
    cliff[0] = sharp_mm(analogRead(PIN_CLIFF_L));
    cliff[1] = sharp_mm(analogRead(PIN_CLIFF_R));
    send_telemetry(batt_v, cliff);
  }
  if (n % 33 == 0) {
    uint8_t ex = eyes_expr;
    if (!calibrated) ex = Eyes::ANGRY;
    else if (since_cmd > CMD_LOST_S) ex = Eyes::LOST;
    else if (out.state == bolt::ST_FALLEN) ex = Eyes::SLEEPY;
    else if (out.state >= bolt::ST_CROUCH && out.state <= bolt::ST_LAND) ex = Eyes::HAPPY;
    eyes.set(ex, eye_r, eye_g, eye_b);
    eyes.update(millis());
  }
}
