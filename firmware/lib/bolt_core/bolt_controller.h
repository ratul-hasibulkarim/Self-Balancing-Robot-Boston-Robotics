// BOLT whole-body controller (balance + legs + jump + fall handling).
//
// Portable C++ (no Arduino dependencies).  The SAME file is compiled
//   * into the Teensy 4.1 firmware (firmware/src/main.cpp), and
//   * into a shared library loaded by the MuJoCo simulator (sim/controller.py),
// so the controller you flash is the controller that was tested.
//
// Conventions
//   body frame: x forward, y left, z up
//   roll  = rotation about +x (left side up  > 0)
//   pitch = rotation about +y (nose DOWN     > 0)
//   yaw   = rotation about +z (turn left     > 0)
//   leg angles phi1/phi4: see five_bar.h (CCW in the x-z plane)
//   wheel speed: relative to the rear shin, > 0 = rolls the robot forward
//   hip torques: generalised force on phi1/phi4 (motor sign handled by the driver layer)
#pragma once
#include <stdint.h>
#include "five_bar.h"

namespace bolt {

constexpr int NK = 4;   // polynomial order + 1 of the gain schedule K(L)

// ---------------------------------------------------------------- config
struct RobotConfig {
  // geometry
  float l1, l2, l5;            // thigh, shin, hip spacing   [m]
  float wheel_radius;          // [m]
  float track;                 // distance between wheel mid planes [m]
  // lumped masses for feed-forward terms
  float m_body;                // everything rigidly attached to the body [kg]
  float body_com_x, body_com_z;// body COM relative to hip midpoint (body frame) [m]
  float m_leg;                 // one leg (links + wheel + motors) [kg]
  float com_height_nom;        // COM height above ground at L_nom [m]
  // limits
  float hip_torque_peak, hip_torque_cont;  // [Nm]
  float wheel_torque_peak;                 // [Nm]
  float hip_speed_max, wheel_speed_max;    // [rad/s]
  float L_min, L_nom, L_max;               // [m]
  float v_max, acc_max, yaw_rate_max;      // [m/s], [m/s^2], [rad/s]
  float pitch_cmd_max, roll_cmd_max;       // [rad]
  // LQR gain schedule: K[i][j](L) = sum_k K[i][j][k] * L^k,  u = -K x
  // u = [T_wheel_total, T_hip_total],  x = [theta, dtheta, s, ds, pitch, dpitch]
  float K[2][6][NK];
  // leg / roll / yaw loops
  float kp_leg, kd_leg;        // leg length spring/damper  [N/m], [N s/m]
  float kp_roll, kd_roll, ki_roll;
  float kp_yaw, kd_yaw;        // on yaw rate error / yaw accel
  float kp_sync, kd_sync;      // keeps both legs at the same angle
  // jump
  float L_crouch, L_takeoff, L_retract;
  float thrust_force;          // per leg, before torque saturation [N]
  float jump_clearance;        // extra wheel clearance over requested height [m]
  // safety
  float fall_pitch, fall_roll; // [rad]
  float comm_timeout;          // [s] (handled by the caller, documented here)
  float ki_yaw;                // yaw-rate integral (overcomes tyre scrub when turning)
};

// ---------------------------------------------------------------- I/O
struct LegMeas {
  float phi1, phi4;            // [rad]  front / rear thigh angle
  float dphi1, dphi4;          // [rad/s]
  float wheel_vel;             // [rad/s] relative to the rear shin
};

struct ImuMeas {
  float roll, pitch, yaw;      // [rad]
  float gx, gy, gz;            // body rates [rad/s]
  float ax, ay, az;            // specific force, body frame [m/s^2]
};

struct Inputs {
  float dt;                    // [s]
  ImuMeas imu;
  LegMeas leg[2];              // 0 = left, 1 = right
  float battery_v;             // [V]
};

enum Mode : int32_t { MODE_OFF = 0, MODE_BALANCE = 1, MODE_CRAWL = 2 };

struct Command {
  int32_t mode;                // Mode
  int32_t jump_seq;            // increment to request one jump
  int32_t estop;               // 1 = cut torque immediately
  float v;                     // forward speed [m/s]
  float yaw_rate;              // [rad/s]
  float height;                // virtual leg length target [m]
  float roll;                  // body roll target [rad]  (tilt sideways)
  float pitch;                 // body pitch target [rad] (bow / lean)
  float jump_height;           // obstacle / step height to clear [m]
  int32_t jump_mode;           // 0 = onto a step (stay tucked until touchdown)
                               // 1 = over an obstacle / gap (extend after apex for a soft landing)
  float jump_dist;             // wheel-centre distance to the step edge when the jump is
                               // requested [m]; 0 = jump as soon as crouched
};

enum State : int32_t {
  ST_OFF = 0, ST_BALANCE, ST_CROUCH, ST_THRUST, ST_FLIGHT, ST_LAND,
  ST_AIRBORNE, ST_FALLEN, ST_RECOVER
};

struct Outputs {
  float hip[4];                // [L_front, L_rear, R_front, R_rear]  Nm
  float wheel[2];              // [L, R] Nm
  int32_t state;
  int32_t contact[2];
  float L[2], theta[2];        // per leg
  float F[2], Tp[2], FN[2];    // leg force, leg torque, normal-force estimate
  float s, ds, v_ref;          // position / speed estimate and reference
  float pitch_ref;
};

// ---------------------------------------------------------------- controller
class Controller {
 public:
  explicit Controller(const RobotConfig& c);
  void reset();
  void step(const Inputs& in, const Command& cmd, Outputs& out);
  const RobotConfig& config() const { return cfg_; }

 private:
  void gains(float L, float K[2][6]) const;
  void equilibrium(float pitch_ref, float L, float& theta_eq, float& Tp_eq) const;
  float hip_torques(const LegKin& k, float F, float Tp, float& T1, float& T4) const;

  RobotConfig cfg_;
  FiveBarGeom geom_;
  State st_ = ST_OFF;
  float t_state_ = 0;          // time in current state
  float s_ = 0, s_ref_ = 0, v_ref_ = 0, ds_f_ = 0;
  float roll_i_ = 0;
  float L_cmd_ = 0;            // smoothed height target
  float alpha_prev_[2] = {0, 0};
  bool alpha_init_ = false;
  float offground_t_[2] = {0, 0};
  int32_t last_jump_seq_ = 0;
  float jump_v_ = 0;           // planned take-off leg speed
  float jump_F_ = 0;           // planned thrust force per leg
  float az_f_ = 9.81f;         // filtered vertical specific force (free-fall / impact detection)
  float crouch_L_ = 0;         // crouch ramp
  float Ld_prev_[2] = {0, 0}, Ldd_f_[2] = {0, 0};   // leg acceleration (ground-force estimate)
  float FN_f_[2] = {0, 0};
  float F_prev_[2] = {0, 0};   // leg force actually applied last cycle
  float jump_tapex_ = 0;       // predicted time from take-off to apex
  float L_fl_ = 0;             // leg length target in flight
  bool td_armed_ = false;      // touchdown detector armed
  int32_t jump_mode_ = 0;
  float jump_edge_ = 0;        // distance to the edge at request time
  float s_jump0_ = 0;          // odometry at request time
  float jump_h_ = 0;           // requested clearance height
  int last_dir_ = 0;           // sign of the last speed command
  float L_land_start_ = 0;
  float yaw_rate_prev_ = 0;
  float yaw_i_ = 0;
};

}  // namespace bolt

// ---------------------------------------------------------------- C API (simulator)
extern "C" {
void* bolt_create(const bolt::RobotConfig* cfg);
void bolt_reset(void* h);
void bolt_step(void* h, const bolt::Inputs* in, const bolt::Command* cmd, bolt::Outputs* out);
void bolt_destroy(void* h);
int bolt_sizes(int* cfg, int* in, int* cmd, int* out);
}
