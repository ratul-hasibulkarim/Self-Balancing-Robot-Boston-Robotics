"""
ctypes binding to the firmware controller (firmware/lib/bolt_core).

The C++ sources are compiled into sim/build/libbolt.so automatically, so
the simulator always runs the exact controller code that goes on the Teensy.
"""
import ctypes as C
import json
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CORE = os.path.join(ROOT, "firmware", "lib", "bolt_core")
LIB = os.path.join(HERE, "build", "libbolt.so")

F = C.c_float
I32 = C.c_int32


class RobotConfig(C.Structure):
    _fields_ = [(n, F) for n in (
        "l1", "l2", "l5", "wheel_radius", "track",
        "m_body", "body_com_x", "body_com_z", "m_leg", "com_height_nom",
        "hip_torque_peak", "hip_torque_cont", "wheel_torque_peak", "hip_speed_max", "wheel_speed_max",
        "L_min", "L_nom", "L_max", "v_max", "acc_max", "yaw_rate_max", "pitch_cmd_max", "roll_cmd_max")] + \
        [("K", F * 4 * 6 * 2)] + [(n, F) for n in (
            "kp_leg", "kd_leg", "kp_roll", "kd_roll", "ki_roll", "kp_yaw", "kd_yaw", "kp_sync", "kd_sync",
            "L_crouch", "L_takeoff", "L_retract", "thrust_force", "jump_clearance",
            "fall_pitch", "fall_roll", "comm_timeout", "ki_yaw")]


class LegMeas(C.Structure):
    _fields_ = [(n, F) for n in ("phi1", "phi4", "dphi1", "dphi4", "wheel_vel")]


class ImuMeas(C.Structure):
    _fields_ = [(n, F) for n in ("roll", "pitch", "yaw", "gx", "gy", "gz", "ax", "ay", "az")]


class Inputs(C.Structure):
    _fields_ = [("dt", F), ("imu", ImuMeas), ("leg", LegMeas * 2), ("battery_v", F)]


class Command(C.Structure):
    _fields_ = [("mode", I32), ("jump_seq", I32), ("estop", I32)] + \
        [(n, F) for n in ("v", "yaw_rate", "height", "roll", "pitch", "jump_height")] + [("jump_mode", I32), ("jump_dist", F)]


class Outputs(C.Structure):
    _fields_ = [("hip", F * 4), ("wheel", F * 2), ("state", I32), ("contact", I32 * 2),
                ("L", F * 2), ("theta", F * 2), ("F", F * 2), ("Tp", F * 2), ("FN", F * 2),
                ("s", F), ("ds", F), ("v_ref", F), ("pitch_ref", F)]


STATES = ["OFF", "BALANCE", "CROUCH", "THRUST", "FLIGHT", "LAND", "AIRBORNE", "FALLEN", "RECOVER"]


def build_lib(force=False):
    srcs = [os.path.join(CORE, f) for f in ("bolt_controller.cpp", "bolt_controller.h", "five_bar.h")]
    if not force and os.path.exists(LIB) and os.path.getmtime(LIB) > max(os.path.getmtime(s) for s in srcs):
        return
    os.makedirs(os.path.dirname(LIB), exist_ok=True)
    tmp = f"{LIB}.{os.getpid()}.tmp"            # atomic: parallel test workers may race
    subprocess.check_call(["g++", "-O2", "-std=c++17", "-shared", "-fPIC", "-o", tmp,
                           os.path.join(CORE, "bolt_controller.cpp")])
    os.replace(tmp, LIB)


_lib = None


def lib():
    global _lib
    if _lib is None:
        build_lib()
        _lib = C.CDLL(LIB)
        _lib.bolt_create.restype = C.c_void_p
        _lib.bolt_create.argtypes = [C.POINTER(RobotConfig)]
        _lib.bolt_step.argtypes = [C.c_void_p, C.POINTER(Inputs), C.POINTER(Command), C.POINTER(Outputs)]
        _lib.bolt_reset.argtypes = [C.c_void_p]
        _lib.bolt_destroy.argtypes = [C.c_void_p]
        sizes = [C.c_int() for _ in range(4)]
        _lib.bolt_sizes(*[C.byref(s) for s in sizes])
        expect = [C.sizeof(RobotConfig), C.sizeof(Inputs), C.sizeof(Command), C.sizeof(Outputs)]
        assert [s.value for s in sizes] == expect, f"struct size mismatch {[s.value for s in sizes]} vs {expect}"
    return _lib


def make_config(cfg: dict) -> RobotConfig:
    c = RobotConfig()
    for k, v in cfg.items():
        if k == "K":
            for i in range(2):
                for j in range(6):
                    for n in range(4):
                        c.K[i][j][n] = v[i][j][n]
        elif hasattr(c, k):
            setattr(c, k, v)
    return c


def load_config(variant="V3"):
    return json.load(open(os.path.join(HERE, "build", f"config_{variant}.json")))


class Controller:
    def __init__(self, cfg: dict):
        self.cfg_dict = cfg
        self._cfg = make_config(cfg)
        self.h = lib().bolt_create(C.byref(self._cfg))
        self.inp, self.cmd, self.out = Inputs(), Command(), Outputs()

    def step(self):
        lib().bolt_step(self.h, C.byref(self.inp), C.byref(self.cmd), C.byref(self.out))
        return self.out

    def __del__(self):
        try:
            lib().bolt_destroy(self.h)
        except Exception:
            pass
