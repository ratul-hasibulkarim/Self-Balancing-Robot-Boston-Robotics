"""
Single source of truth for the robot geometry and mass budget ("BOLT" the
wheel-legged balancing robot).

Every other tool reads this file:
  * cad/build_parts.py     -> printable STL parts + assembly preview
  * sim/model_builder.py   -> MuJoCo model used for testing
  * sim/lqr_design.py      -> LQR gain schedule -> firmware/lib/core/gains.h

Frame convention (body frame):  x forward, y left, z up.
Origin = midpoint between the front and rear hip-motor axes (the "hip").
Units: metres, kilograms, seconds, newton-metres.

Design iterations (V1 -> V3) are kept here so the simulation report can be
reproduced.  `FINAL` is the design you should print.
"""
from copy import deepcopy

# ---------------------------------------------------------------------------
# Actuators (datasheet values, conservative)
# ---------------------------------------------------------------------------
HIP_MOTOR = dict(            # DM-J4310-2EC  (10:1 QDD, CAN, MIT mode)
    name="DM-J4310-2EC",
    mass=0.300,
    peak_torque=7.0,         # Nm  (short bursts, used for jumping)
    rated_torque=3.0,        # Nm  (continuous)
    max_speed=20.0,          # rad/s at 24 V
    diameter=0.056, length=0.046,
)
WHEEL_MOTOR = dict(          # LK-TECH MF9025v2 direct-drive hub motor (CAN)
    name="LK MF9025v2",
    mass=0.520,
    peak_torque=4.0,         # Nm
    rated_torque=1.8,        # Nm
    max_speed=36.0,          # rad/s no-load at 24 V (~340 rpm)
    diameter=0.096, length=0.034,
)

# ---------------------------------------------------------------------------
# Baseline geometry (V1 = first sketch, "big-head" cute proportions)
# ---------------------------------------------------------------------------
V1 = dict(
    name="V1 big-head",
    # --- 5-bar leg (per side) -------------------------------------------
    hip_spacing=0.090,       # l5: distance between front & rear hip motors
    thigh=0.130,             # l1 = l4
    shin=0.220,              # l2 = l3
    leg_y=0.120,             # lateral position of the leg plane
    wheel_y=0.150,           # lateral position of the wheel mid-plane
    wheel_radius=0.060,
    wheel_width=0.036,
    # --- leg length operating range (hip-mid -> wheel centre) -----------
    L_min=0.120, L_nom=0.200, L_max=0.320,
    # --- body shell -----------------------------------------------------
    body_size=(0.200, 0.200, 0.250),   # x (depth), y (width), z (height)
    body_center_z=0.080,               # shell centre above the hip axis
    # --- internal components: (mass, (x, y, z), (sx, sy, sz)) -----------
    components={
        "battery_6S_5000":   (0.780, (0.000, 0.0, 0.150), (0.150, 0.050, 0.045)),
        "electronics_sled":  (0.420, (-0.020, 0.0, 0.060), (0.140, 0.120, 0.050)),
        "head_sensors":      (0.200, (0.050, 0.0, 0.170), (0.060, 0.150, 0.050)),
        "gps_mast":          (0.060, (-0.030, 0.0, 0.240), (0.040, 0.040, 0.060)),
        "wiring_misc":       (0.150, (0.000, 0.0, 0.060), (0.120, 0.150, 0.080)),
    },
    payload_pos=(0.0, 0.0, 0.230),     # payload carried on the roof
    printed_body_mass=1.10,            # shell + chassis (from CAD estimate)
    printed_leg_mass=0.180,            # per leg: 2 thighs + 2 shins + pins
    wheel_rim_tire_mass=0.140,         # per wheel
)

# V2: battery moved down into the floor between the hips, sensors packed,
#     wider track.  Wheels still small.
V2 = deepcopy(V1)
V2.update(
    name="V2 low-COM",
    leg_y=0.142, wheel_y=0.182,     # track 0.30 m -> 0.364 m
    body_center_z=0.050,
)
V2["components"] = {
    "battery_6S_5000":   (0.780, (0.000, 0.0, -0.010), (0.150, 0.050, 0.045)),
    "electronics_sled":  (0.420, (-0.010, 0.0, 0.055), (0.140, 0.120, 0.040)),
    "head_sensors":      (0.200, (0.060, 0.0, 0.130), (0.050, 0.150, 0.050)),
    "gps_mast":          (0.060, (-0.030, 0.0, 0.200), (0.040, 0.040, 0.060)),
    "wiring_misc":       (0.150, (0.000, 0.0, 0.040), (0.120, 0.150, 0.080)),
}
V2["payload_pos"] = (0.0, 0.0, 0.180)

# V3 (FINAL): bigger 150 mm wheels for rough ground & stair lips, longer
#     shins for jump stroke, payload bay sunk into the body centre directly
#     above the hip axis, battery slung below the hip axis.
V3 = deepcopy(V2)
V3.update(
    name="V3 final",
    thigh=0.140, shin=0.250,
    wheel_radius=0.075, wheel_width=0.040,
    L_min=0.130, L_nom=0.220, L_max=0.350,
    body_size=(0.210, 0.240, 0.230),
    body_center_z=0.045,
)
V3["components"] = {
    "battery_6S_5000":   (0.780, (0.000, 0.0, -0.028), (0.155, 0.052, 0.046)),
    "electronics_sled":  (0.420, (-0.080, 0.0, 0.090), (0.040, 0.120, 0.120)),
    "head_sensors":      (0.200, (0.085, 0.0, 0.110), (0.040, 0.160, 0.050)),
    "gps_mast":          (0.060, (-0.050, 0.0, 0.190), (0.040, 0.040, 0.060)),
    "wiring_misc":       (0.150, (0.000, 0.0, 0.030), (0.120, 0.150, 0.080)),
}
V3["payload_pos"] = (0.010, 0.0, 0.085)   # centre of the sunken payload bay

VARIANTS = {"V1": V1, "V2": V2, "V3": V3}
FINAL = V3

PAYLOAD_MAX = 2.0   # kg, design payload


def get(name="V3"):
    return deepcopy(VARIANTS[name])
