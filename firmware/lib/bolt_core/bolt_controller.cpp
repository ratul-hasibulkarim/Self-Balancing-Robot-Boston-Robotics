// BOLT whole-body controller - see bolt_controller.h for conventions.
#include "bolt_controller.h"

#include <string.h>

namespace bolt {

namespace {
constexpr float G = 9.81f;
#ifndef JUMP_EDGE_MARGIN
#define JUMP_EDGE_MARGIN 0.05f   // extra take-off distance before a step edge [m]
#endif
inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline float sq(float x) { return x * x; }
inline float approach(float x, float target, float step) {
  if (x < target) return fminf(x + step, target);
  return fmaxf(x - step, target);
}
}  // namespace

Controller::Controller(const RobotConfig& c) : cfg_(c) {
  geom_ = FiveBarGeom{c.l1, c.l2, c.l5};
  reset();
}

void Controller::reset() {
  st_ = ST_OFF;
  t_state_ = 0;
  s_ = s_ref_ = v_ref_ = ds_f_ = 0;
  roll_i_ = 0;
  yaw_i_ = 0;
  L_cmd_ = cfg_.L_nom;
  alpha_init_ = false;
  offground_t_[0] = offground_t_[1] = 0;
  last_jump_seq_ = 0;
  jump_v_ = 0;
  yaw_rate_prev_ = 0;
}

void Controller::gains(float L, float K[2][6]) const {
  L = clampf(L, cfg_.L_min - 0.02f, cfg_.L_max + 0.02f);
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 6; ++j) {
      float acc = 0, p = 1;
      for (int k = 0; k < NK; ++k) { acc += cfg_.K[i][j][k] * p; p *= L; }
      K[i][j] = acc;
    }
}

// Static equilibrium for a commanded body pitch: leg angle that keeps the
// overall COM above the wheels, and the hip torque that holds the body.
void Controller::equilibrium(float pitch_ref, float L, float& theta_eq, float& Tp_eq) const {
  const float xw = cfg_.body_com_x * cosf(pitch_ref) + cfg_.body_com_z * sinf(pitch_ref);
  const float m_eff = cfg_.m_body + 0.5f * cfg_.m_leg;   // legs: ~25% of their mass above the wheel
  theta_eq = asinf(clampf(cfg_.m_body * xw / (m_eff * fmaxf(L, 0.05f)), -0.9f, 0.9f));
  Tp_eq = -cfg_.m_body * G * xw;
}

// [T1, T4] = J^T [F, Tp]; scales F (thrust) first, then everything, to respect torque limits.
float Controller::hip_torques(const LegKin& k, float F, float Tp, float& T1, float& T4) const {
  T1 = k.J[0][0] * F + k.J[1][0] * Tp;
  T4 = k.J[0][1] * F + k.J[1][1] * Tp;
  const float m = fmaxf(fabsf(T1), fabsf(T4));
  float s = 1.f;
  if (m > cfg_.hip_torque_peak) {
    s = cfg_.hip_torque_peak / m;
    T1 *= s;
    T4 *= s;
  }
  return s;
}

void Controller::step(const Inputs& in, const Command& cmd, Outputs& out) {
  memset(&out, 0, sizeof(out));
  const float dt = clampf(in.dt, 1e-4f, 0.02f);
  const ImuMeas& imu = in.imu;
  t_state_ += dt;

  // ---------------------------------------------------------- leg kinematics
  LegKin k[2];
  float Ld[2], thd_b[2], th[2], thd[2], ws[2];
  bool kin_ok = true;
  for (int i = 0; i < 2; ++i) {
    const LegMeas& m = in.leg[i];
    k[i] = five_bar_solve(geom_, m.phi1, m.phi4);
    if (!k[i].ok) { kin_ok = false; continue; }
    Ld[i] = k[i].J[0][0] * m.dphi1 + k[i].J[0][1] * m.dphi4;
    thd_b[i] = k[i].J[1][0] * m.dphi1 + k[i].J[1][1] * m.dphi4;
    th[i] = k[i].theta - imu.pitch;
    thd[i] = thd_b[i] - imu.gy;
    // rear-shin rate (carries the wheel stator)
    float cx = 0, cz = 0, a1 = k[i].alpha_rear_shin;
    five_bar_fk(geom_, m.phi1 + m.dphi1 * 1e-3f, m.phi4 + m.dphi4 * 1e-3f, cx, cz, a1);
    const float alpha_dot = (a1 - k[i].alpha_rear_shin) / 1e-3f;
    ws[i] = cfg_.wheel_radius * (m.wheel_vel + imu.gy - alpha_dot);   // ground speed of wheel i
  }
  if (!kin_ok || cmd.estop) {
    st_ = ST_OFF;
    out.state = st_;
    return;
  }

  const float L = 0.5f * (k[0].L + k[1].L);
  const float theta = 0.5f * (th[0] + th[1]);
  const float dtheta = 0.5f * (thd[0] + thd[1]);
  const float ds_raw = 0.5f * (ws[0] + ws[1]);
  ds_f_ += (ds_raw - ds_f_) * clampf(dt / 0.004f, 0.f, 1.f);
  const float ds = ds_f_;
  s_ += ds * dt;
  az_f_ += (imu.az - az_f_) * clampf(dt / 0.008f, 0.f, 1.f);

  // ---------------------------------------------------------- mode logic
  const bool want_on = cmd.mode == MODE_BALANCE || cmd.mode == MODE_CRAWL;
  const bool tipped = fabsf(imu.pitch) > cfg_.fall_pitch || fabsf(imu.roll) > cfg_.fall_roll;
  auto go = [&](State s) { st_ = s; t_state_ = 0; };

  if (!want_on) {
    if (st_ != ST_OFF) go(ST_OFF);
  } else if (st_ == ST_OFF) {
    s_ref_ = s_; v_ref_ = 0; roll_i_ = 0; L_cmd_ = L;
    last_jump_seq_ = cmd.jump_seq;
    go(fabsf(imu.pitch) < 0.35f && fabsf(imu.roll) < 0.35f ? ST_BALANCE : ST_RECOVER);
  } else if (tipped && st_ != ST_FALLEN && st_ != ST_RECOVER && st_ != ST_FLIGHT) {
    go(ST_FALLEN);
  }

  // ---------------------------------------------------------- references
  const bool crawl = cmd.mode == MODE_CRAWL;
  const float vmax = crawl ? 0.5f : cfg_.v_max;
  const float v_cmd = clampf(cmd.v, -vmax, vmax);
  v_ref_ = approach(v_ref_, v_cmd, cfg_.acc_max * dt);
  const float yaw_ref = clampf(cmd.yaw_rate, -cfg_.yaw_rate_max, cfg_.yaw_rate_max);
  float H = crawl ? cfg_.L_min + 0.005f : clampf(cmd.height, cfg_.L_min, cfg_.L_max - 0.03f);
  // the faster we go, the lower we ride (more robust, less roll-over risk)
  const float speed_frac = fabsf(v_ref_) / cfg_.v_max;
  H = fminf(H, cfg_.L_max - 0.03f - 0.08f * speed_frac);
  L_cmd_ = approach(L_cmd_, H, 0.35f * dt);                    // 0.35 m/s height slew
  const float pitch_ref = crawl ? 0.f : clampf(cmd.pitch, -cfg_.pitch_cmd_max, cfg_.pitch_cmd_max);
  const float roll_ref = clampf(cmd.roll, -cfg_.roll_cmd_max, cfg_.roll_cmd_max);

  // jump request
  if (cmd.jump_seq != last_jump_seq_) {
    last_jump_seq_ = cmd.jump_seq;
    if (st_ == ST_BALANCE && !crawl) {
      // wheel apex clearance = COM rise + leg retraction seen by the wheel
      const float m_tot = cfg_.m_body + 2 * cfg_.m_leg;
      const float retract = (cfg_.L_takeoff - cfg_.L_retract) * cfg_.m_body / m_tot;
      const float h_req = (cmd.jump_dist > 0) ? fmaxf(cmd.jump_height, 0.10f) : cmd.jump_height;
      const float h_com = fmaxf(0.03f, h_req + cfg_.jump_clearance - retract);
      const float v_com = sqrtf(2 * G * h_com);
      jump_v_ = v_com * m_tot / cfg_.m_body;                      // body speed at take-off
      jump_tapex_ = v_com / G;
      // reach take-off speed within the first 40 % of the stroke, then coast:
      // (2F - m_b g) * 0.4 dL = 1/2 m_b v^2   (+10 % for losses)
      const float stroke = 0.4f * (cfg_.L_takeoff - cfg_.L_crouch);
      jump_F_ = fminf(cfg_.thrust_force,
                      1.10f * 0.5f * (0.5f * cfg_.m_body * jump_v_ * jump_v_ / stroke + cfg_.m_body * G));
      crouch_L_ = L_cmd_;
      jump_mode_ = cmd.jump_mode;
      jump_edge_ = cmd.jump_dist;
      jump_h_ = h_req;
      s_jump0_ = s_;
      go(ST_CROUCH);
    }
  }

  float Kg[2][6];
  gains(L, Kg);
  float th_eq, Tp_eq;
  equilibrium(pitch_ref, L, th_eq, Tp_eq);

  // ---------------------------------------------------------- balance law (LQR)
  // the position reference integrates the speed reference (integral action that
  // holds the robot on slopes / with payload).  When the robot is told to START
  // moving or to REVERSE, the reference is re-anchored so an old error (e.g. an
  // overshoot after braking) never causes a "catch-up" surge in the new direction.
  // Stopping keeps the reference, so the robot holds its place on a slope.
  const int dir = (v_cmd > 0.01f) - (v_cmd < -0.01f);
  if (dir != 0 && dir != last_dir_) s_ref_ = s_;
  last_dir_ = dir;
  s_ref_ += v_ref_ * dt;
  float s_err = clampf(s_ - s_ref_, -0.25f, 0.25f);
  s_ref_ = s_ - s_err;
  const float x[6] = {theta - th_eq, dtheta, s_err, ds - v_ref_, imu.pitch - pitch_ref, imu.gy};
  float Tw = 0, Tp = Tp_eq;
  for (int j = 0; j < 6; ++j) {
    Tw -= Kg[0][j] * x[j];
    Tp -= Kg[1][j] * x[j];
  }

  // ---------------------------------------------------------- yaw
  const float yaw_acc = (imu.gz - yaw_rate_prev_) / dt;
  yaw_rate_prev_ = imu.gz;
  const bool grounded = st_ == ST_BALANCE || st_ == ST_CROUCH || st_ == ST_LAND;
  if (grounded) yaw_i_ = clampf(yaw_i_ + cfg_.ki_yaw * (yaw_ref - imu.gz) * dt, -1.2f, 1.2f);
  else yaw_i_ *= 0.99f;
  float Tyaw = cfg_.kp_yaw * (yaw_ref - imu.gz) - cfg_.kd_yaw * yaw_acc + yaw_i_;

  // ---------------------------------------------------------- legs
  const float half_track = 0.5f * cfg_.track;
  float L_ref[2] = {L_cmd_ + half_track * tanf(roll_ref), L_cmd_ - half_track * tanf(roll_ref)};
  roll_i_ = clampf(roll_i_ + cfg_.ki_roll * (roll_ref - imu.roll) * dt, -15.f, 15.f);
  float F_roll = cfg_.kp_roll * (roll_ref - imu.roll) - cfg_.kd_roll * imu.gx + roll_i_;
  const float F_turn = cfg_.m_body * ds * imu.gz * cfg_.com_height_nom / cfg_.track;
  float kp_leg = cfg_.kp_leg, kd_leg = cfg_.kd_leg;
  const float F_ff = 0.5f * cfg_.m_body * G / fmaxf(cosf(theta), 0.5f);
  float F[2], Tpi[2], Twi[2];
  const float sync = cfg_.kp_sync * (th[1] - th[0]) + cfg_.kd_sync * (thd[1] - thd[0]);

  bool wheels_on = true;
  bool hold_angle_only = false;      // in the air: only keep the legs vertical
  float F_override = -1;

  switch (st_) {
    case ST_OFF:
      break;
    case ST_BALANCE:
      break;
    case ST_CROUCH:
      // lower smoothly (0.6 m/s) and wait until the legs are vertical and still
      crouch_L_ = approach(crouch_L_, cfg_.L_crouch, 0.9f * dt);
      L_ref[0] = L_ref[1] = crouch_L_;
      {
        // settled enough: crouched, legs not swinging (a forward lean while
        // accelerating towards the obstacle is fine)
        const bool crouched = (L < cfg_.L_crouch + 0.01f && fabsf(theta - th_eq) < 0.12f &&
                               fabsf(dtheta) < 0.8f) || t_state_ > 1.2f;
        bool fire = crouched;
        if (jump_edge_ > 0) {
          // fire so that the wheel (and the front knee, which sticks out ~2 cm further)
          // is already above the step when it reaches the edge:
          //   d = v (t_thrust + t_clear) + R + margin
          const float remaining = jump_edge_ - (s_ - s_jump0_);
          const float t_thrust = 1.4f * (cfg_.L_takeoff - cfg_.L_crouch) / fmaxf(jump_v_, 0.5f);
          const float t_clear = 0.05f + 0.35f * jump_h_;
          const float d_needed = fmaxf(ds, 0.f) * (t_thrust + t_clear) + cfg_.wheel_radius + JUMP_EDGE_MARGIN;
          const bool low_enough = L < cfg_.L_crouch + 0.03f;
          fire = (crouched || low_enough) && remaining <= d_needed;
          // too close to make it (or overshot): abort instead of hitting the riser
          if (remaining < d_needed - 0.05f || t_state_ > 4.0f) {
            v_ref_ = 0;
            go(ST_BALANCE);
            break;
          }
        }
        if (fire) go(ST_THRUST);
      }
      break;
    case ST_THRUST:
      // full thrust until the planned speed is reached, then hold the speed
      F_override = (0.5f * (Ld[0] + Ld[1]) < jump_v_) ? jump_F_ : 0.5f * cfg_.m_body * G / fmaxf(cosf(theta), 0.7f);
      if (L >= cfg_.L_takeoff || t_state_ > 0.4f) { L_fl_ = L; td_armed_ = false; go(ST_FLIGHT); }
      break;
    case ST_FLIGHT: {
      // tuck the wheels up until the apex, then extend to a landing length that
      // leaves ~7 cm of stroke to absorb the touchdown
      const float L_land = cfg_.L_retract + 0.05f;
      const float target = (jump_mode_ == 1 && t_state_ > jump_tapex_ + 0.03f) ? L_land : cfg_.L_retract;
      L_fl_ = approach(L_fl_, target, 3.0f * dt);
      L_ref[0] = L_ref[1] = L_fl_;
      kp_leg *= 2.5f;
      wheels_on = false;
      hold_angle_only = true;
      F_roll = 0;
      // in free fall nothing loads the leg, so any compression is the ground
      const float Ld_avg = 0.5f * (Ld[0] + Ld[1]);
      if (t_state_ > jump_tapex_ && fabsf(L - L_fl_) < 0.015f && fabsf(Ld_avg) < 0.3f) td_armed_ = true;
      if ((td_armed_ && (L_fl_ - L) > 0.006f && Ld_avg < -0.2f) || t_state_ > 1.2f) {
        L_land_start_ = L;
        s_ref_ = s_; v_ref_ = ds; roll_i_ = 0;
        go(ST_LAND);
      }
      break;
    }
    case ST_LAND: {
      const float a = clampf(t_state_ / 0.35f, 0.f, 1.f);
      L_ref[0] = L_ref[1] = L_land_start_ + (L_cmd_ - L_land_start_) * a;
      kd_leg *= 2.0f;
      if (t_state_ > 0.35f) go(ST_BALANCE);
      break;
    }
    case ST_AIRBORNE:
      wheels_on = false;
      hold_angle_only = true;
      break;
    case ST_FALLEN:
      // limp, protect electronics: torque off except gentle leg damping
      break;
    case ST_RECOVER:
      break;
  }

  if (st_ == ST_OFF || st_ == ST_FALLEN) {
    for (int i = 0; i < 2; ++i) {
      // passive damping on the hips while fallen (no stored energy)
      const float d = (st_ == ST_FALLEN) ? 0.05f : 0.f;
      out.hip[2 * i] = -d * in.leg[i].dphi1;
      out.hip[2 * i + 1] = -d * in.leg[i].dphi4;
      out.L[i] = k[i].L; out.theta[i] = th[i];
    }
    out.state = st_;
    out.s = s_; out.ds = ds;
    return;
  }

  if (st_ == ST_RECOVER) {
    // self-righting from lying on the front/rear bumper (not from lying on a side):
    //   A (0-0.5 s)   fold the legs and swing them towards vertical (in the world)
    //   B (0.5-1.3 s) extend the legs: the body pivots on its bumper and is lifted
    //   C (1.3 s ..)  wheels braked, legs stay vertical in the world, and the hips
    //                 rotate the body upright about the hip axis.  The body COM is only
    //                 ~2 cm from the hip axis (by design), so this needs very little torque
    //   -> once nearly upright, hand over to the balance law.
    const float tA = 0.5f, tB = 1.3f;
    const float ext = clampf((t_state_ - tA) / (tB - tA), 0.f, 1.f);
    const float L_tgt = (cfg_.L_min + 0.01f) + ext * (cfg_.L_nom + 0.04f - cfg_.L_min - 0.01f);
    float tb_target = clampf(imu.pitch, -1.0f, 1.0f);                // legs vertical in the world
    if (t_state_ > tB) {
      // rotate the body: theta_b -> 0 while theta_world is held ~0
      const float a = clampf((t_state_ - tB) / 0.8f, 0.f, 1.f);
      tb_target = (1.f - a) * tb_target;
    }
    for (int i = 0; i < 2; ++i) {
      const float Fi = 0.5f * cfg_.m_body * G * ext + 900.f * (L_tgt - k[i].L) - 30.f * Ld[i];
      const float Ti = 30.f * (tb_target - k[i].theta) - 1.5f * thd_b[i];
      float T1, T4;
      hip_torques(k[i], Fi, Ti, T1, T4);
      out.hip[2 * i] = T1; out.hip[2 * i + 1] = T4;
      out.wheel[i] = clampf(-0.08f * in.leg[i].wheel_vel / cfg_.wheel_radius * cfg_.wheel_radius * 10.f,
                            -1.f, 1.f);                                 // brake the wheels
      out.L[i] = k[i].L; out.theta[i] = th[i];
    }
    if (fabsf(imu.pitch) < 0.25f && fabsf(imu.roll) < 0.3f && t_state_ > 0.2f) {
      s_ref_ = s_; v_ref_ = 0; L_cmd_ = L; roll_i_ = 0;
      go(ST_BALANCE);
    } else if (t_state_ > 3.5f) {
      go(ST_FALLEN);     // give up, wait for the operator (toggle OFF -> BALANCE to retry)
    }
    out.state = st_;
    return;
  }

  // ---------------------------------------------------------- off-ground detection (balance only)
  for (int i = 0; i < 2; ++i) {
    const float ff = (st_ == ST_FLIGHT || st_ == ST_AIRBORNE) ? 0.f : F_ff;   // nothing to carry in the air
    F[i] = (F_override > 0) ? F_override
                            : ff + kp_leg * (L_ref[i] - k[i].L) - kd_leg * Ld[i];
    const float sgn = (i == 0) ? 1.f : -1.f;
    if (F_override <= 0) F[i] += sgn * (F_roll - F_turn);
    F[i] = clampf(F[i], st_ == ST_FLIGHT ? -130.f : -250.f, 450.f);
    // ground reaction: FN = P + m_w (g + zdd_wheel),  zdd_wheel = zdd_body - Ldd cos(theta)
    const float m_w = 0.7f * cfg_.m_leg;
    const float Ldd = (Ld[i] - Ld_prev_[i]) / dt;
    Ld_prev_[i] = Ld[i];
    Ldd_f_[i] += (Ldd - Ldd_f_[i]) * clampf(dt / 0.006f, 0.f, 1.f);
    const float zdd_body = az_f_ - G;
    const float zdd_wheel = zdd_body - Ldd_f_[i] * cosf(th[i]);
    const float F_applied = F_prev_[i];
    out.FN[i] = F_applied * cosf(th[i]) + m_w * (G + zdd_wheel);
    FN_f_[i] += (out.FN[i] - FN_f_[i]) * clampf(dt / 0.006f, 0.f, 1.f);
    if (st_ == ST_BALANCE || st_ == ST_AIRBORNE) {
      offground_t_[i] = (FN_f_[i] < 0.12f * cfg_.m_body * G) ? offground_t_[i] + dt : 0.f;
    } else {
      offground_t_[i] = 0;
    }
    out.contact[i] = offground_t_[i] < 0.03f;
  }
  if (st_ == ST_BALANCE && !out.contact[0] && !out.contact[1]) go(ST_AIRBORNE);
  if (st_ == ST_AIRBORNE && (out.contact[0] || out.contact[1])) {
    s_ref_ = s_; v_ref_ = 0;
    go(ST_BALANCE);
  }

  if (hold_angle_only) {
    // keep each leg vertical in the world, no wheel torque, forget position
    Tp = 0;
    // hold the legs straight down in the BODY frame: almost no reaction torque on
    // the body, so it keeps the attitude it had at take-off
    for (int i = 0; i < 2; ++i) Tpi[i] = -(14.f * k[i].theta + 0.6f * thd_b[i]);
    s_ref_ = s_;
  } else {
    Tpi[0] = 0.5f * Tp + sync;
    Tpi[1] = 0.5f * Tp - sync;
  }
  if (st_ == ST_THRUST || st_ == ST_FLIGHT) {
    // the leg force acts through the hip, a few mm away from the body COM; cancel the
    // resulting pitch moment M = F (bz sin(theta_b) + bx cos(theta_b)) so the body
    // does not start spinning at take-off / during the tuck
    for (int i = 0; i < 2; ++i)
      Tpi[i] -= F[i] * (cfg_.body_com_z * sinf(k[i].theta) + cfg_.body_com_x * cosf(k[i].theta));
  }
  // during the thrust the wheels unload: any wheel torque would just spin the
  // free wheel and kick the body in pitch through the reaction
  const float tw_lim = (st_ == ST_THRUST || st_ == ST_FLIGHT) ? 0.4f : cfg_.wheel_torque_peak;
  Tyaw = clampf(Tyaw, -0.5f * tw_lim, 0.5f * tw_lim);
  Twi[0] = wheels_on ? 0.5f * Tw - Tyaw : 0.f;
  Twi[1] = wheels_on ? 0.5f * Tw + Tyaw : 0.f;

  for (int i = 0; i < 2; ++i) {
    float T1, T4;
    const float sc = hip_torques(k[i], F[i], Tpi[i], T1, T4);
    F_prev_[i] = F[i] * sc;
    out.hip[2 * i] = T1;
    out.hip[2 * i + 1] = T4;
    out.wheel[i] = clampf(Twi[i], -tw_lim, tw_lim);
    out.L[i] = k[i].L;
    out.theta[i] = th[i];
    out.F[i] = F[i];
    out.Tp[i] = Tpi[i];
  }
  out.state = st_;
  out.s = s_;
  out.ds = ds;
  out.v_ref = v_ref_;
  out.pitch_ref = pitch_ref;
}

}  // namespace bolt

// ------------------------------------------------------------------ C API
extern "C" {
void* bolt_create(const bolt::RobotConfig* cfg) { return new bolt::Controller(*cfg); }
void bolt_reset(void* h) { static_cast<bolt::Controller*>(h)->reset(); }
void bolt_step(void* h, const bolt::Inputs* in, const bolt::Command* cmd, bolt::Outputs* out) {
  static_cast<bolt::Controller*>(h)->step(*in, *cmd, *out);
}
void bolt_destroy(void* h) { delete static_cast<bolt::Controller*>(h); }
int bolt_sizes(int* cfg, int* in, int* cmd, int* out) {
  *cfg = sizeof(bolt::RobotConfig);
  *in = sizeof(bolt::Inputs);
  *cmd = sizeof(bolt::Command);
  *out = sizeof(bolt::Outputs);
  return 0;
}
}
