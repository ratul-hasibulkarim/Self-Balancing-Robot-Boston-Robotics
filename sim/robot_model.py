"""
Builds the MuJoCo model (MJCF) of BOLT + test terrain from design/params.py.

The model is a faithful multibody: a floating body, two closed-chain 5-bar
legs per side (closed with an equality constraint), and two wheels with
real tyre/ground contact.  Masses come from the CAD mass report for the final
design (V3) or from the params for earlier iterations.
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "design"))
import kinematics  # noqa: E402
import params as P  # noqa: E402

CAD_ASM = os.path.join(ROOT, "cad", "assembly")
MASS_REPORT = os.path.join(ROOT, "cad", "mass_report.json")
WORK = os.path.join(HERE, "build")
os.makedirs(WORK, exist_ok=True)


# ---------------------------------------------------------------------------
def mass_budget(v):
    """Return the lumped masses used by the model for variant dict v."""
    use_cad = v["name"].startswith("V3") and os.path.exists(MASS_REPORT)
    if use_cad:
        rep = json.load(open(MASS_REPORT))["summary"]
        body_print = rep["body_prints_g"] / 1000.0
        body_print_com = rep["body_prints_com_m"]
        leg_print = rep["per_leg_prints_g"] / 1000.0
        wheel_print = rep["per_wheel_prints_g"] / 1000.0
    else:
        body_print = v["printed_body_mass"]
        body_print_com = [0.0, 0.0, v["body_center_z"]]
        leg_print = v["printed_leg_mass"]
        wheel_print = v["wheel_rim_tire_mass"]
    hw = 0.012           # bearings, pins, screws per joint
    return dict(
        body_print=body_print, body_print_com=body_print_com,
        thigh=leg_print * 0.38 / 2 + hw,
        shin_front=leg_print * 0.18 + 2 * hw,
        shin_rear=leg_print * 0.44 + 2 * hw,
        wheel_stator=P.WHEEL_MOTOR["mass"] * 0.55,
        wheel_rotor=P.WHEEL_MOTOR["mass"] * 0.45,
        wheel_print=wheel_print,
        hip_motor=P.HIP_MOTOR["mass"],
    )


def _fmt(a):
    return " ".join(f"{x:.6g}" for x in a)


def _aa_neg_y(angle):
    """axis-angle string for a rotation in the leg plane (CCW in x-z = about -y)."""
    return f"0 -1 0 {angle:.6f}"


# ---------------------------------------------------------------------------
def terrain_xml(terrain):
    """terrain: dict(kind=..., ...). Returns (asset_xml, world_xml)."""
    kind = terrain.get("kind", "flat")
    asset, world = [], []
    friction = terrain.get("friction", 1.0)
    floor_mat = terrain.get("floor_material", "grid")
    world.append(f'<geom name="floor" type="plane" size="60 60 0.1" material="{floor_mat}" '
                 f'contype="1" conaffinity="2" friction="{friction} 0.02 0.001"/>')
    if kind == "incline":
        a = math.radians(terrain["angle_deg"])
        length = 4.0
        x0 = terrain.get("x0", 0.8)
        cx = x0 + length / 2 * math.cos(a)
        cz = length / 2 * math.sin(a) - 0.05 * math.cos(a)
        world.append(f'<geom name="ramp" type="box" size="{length/2} 1.5 0.05" pos="{cx} 0 {cz}" '
                     f'euler="0 {-a} 0" material="ramp" contype="1" conaffinity="2" friction="{friction} 0.02 0.001"/>')
        # plateau on top
        top = length * math.sin(a)
        px = x0 + length * math.cos(a) + 1.5
        world.append(f'<geom name="plateau" type="box" size="1.5 1.5 {top/2}" pos="{px} 0 {top/2}" '
                     f'material="ramp" contype="1" conaffinity="2" friction="{friction} 0.02 0.001"/>')
    elif kind == "side_slope":
        a = math.radians(terrain["angle_deg"])
        world.append(f'<geom name="slope" type="box" size="3 1.5 0.05" pos="1.5 0 {-0.05 + 0.0}" '
                     f'euler="{a} 0 0" material="ramp" contype="1" conaffinity="2" friction="{friction} 0.02 0.001"/>')
    elif kind == "rough":
        n = 160
        size = 8.0
        rng = np.random.default_rng(terrain.get("seed", 1))
        z = rng.random((n, n))
        # smooth bumps
        from scipy.ndimage import gaussian_filter
        z = gaussian_filter(z, sigma=terrain.get("sigma", 1.6))
        z = (z - z.min()) / (z.max() - z.min())
        z[:, : n // 2 - int(1.2 / size * n)] *= 0.0       # flat start area
        path = os.path.join(WORK, f"hfield_{terrain.get('seed',1)}.png")
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.imsave(path, z, cmap="gray", vmin=0, vmax=1)
        amp = terrain.get("amplitude", 0.03)
        asset.append(f'<hfield name="rough" file="{path}" size="{size/2} {size/2} {amp} 0.01"/>')
        world.append(f'<geom name="rough" type="hfield" hfield="rough" pos="{size/2 - 1.0} 0 0" '
                     f'material="ramp" contype="1" conaffinity="2" friction="{friction} 0.02 0.001"/>')
    elif kind == "steps":
        # list of (x_start, rise, tread)
        x = terrain.get("x0", 1.0)
        h = 0.0
        for i, (rise, tread) in enumerate(terrain["steps"]):
            h += rise
            world.append(f'<geom name="step{i}" type="box" size="{tread/2 + 3:.3f} 1.2 {h/2:.4f}" '
                         f'pos="{x + tread/2 + 3:.3f} 0 {h/2:.4f}" material="ramp" contype="1" conaffinity="2" '
                         f'friction="{friction} 0.02 0.001"/>')
            x += tread
    elif kind == "obstacle_bar":
        # a bar to crawl under: (x, clearance)
        x, clr = terrain["x"], terrain["clearance"]
        world.append(f'<geom name="bar" type="box" size="0.05 1.2 0.05" pos="{x} 0 {clr + 0.05}" '
                     f'material="ramp" contype="1" conaffinity="2"/>')
    elif kind == "gap":
        x, w = terrain["x"], terrain["width"]
        # raise the world floor except the gap: two platforms 0.3 m high
        for name, cx, sx in (("p0", x - 2.0, 2.0), ("p1", x + w + 2.0, 2.0)):
            world.append(f'<geom name="{name}" type="box" size="{sx} 1.2 0.15" pos="{cx} 0 0.15" '
                         f'material="ramp" contype="1" conaffinity="2" friction="{friction} 0.02 0.001"/>')
    for i, ob in enumerate(terrain.get("obstacles", [])):
        x, y, sx, sy, sz = ob
        world.append(f'<geom name="obs{i}" type="box" size="{sx} {sy} {sz}" pos="{x} {y} {sz}" '
                     f'rgba="0.85 0.3 0.25 1" contype="1" conaffinity="2"/>')
    return "\n".join(asset), "\n".join(world)


# ---------------------------------------------------------------------------
def build_mjcf(variant="V3", payload=0.0, terrain=None, visuals=True, start_L=None,
               floor_texture=None, cameras=True, dark=False):
    v = P.get(variant) if isinstance(variant, str) else variant
    terrain = terrain or {"kind": "flat"}
    mb = mass_budget(v)
    l1, l2, l5 = v["thigh"], v["shin"], v["hip_spacing"]
    R, Wd = v["wheel_radius"], v["wheel_width"]
    L0 = start_L or v["L_nom"]
    p1, p4 = kinematics.inverse(L0, 0.0, l1, l2, l5)
    fk = kinematics.forward(p1, p4, l1, l2, l5)
    B, D, C = fk["B"], fk["D"], fk["C"]
    psiF = math.atan2(C[1] - B[1], C[0] - B[0])
    psiR = math.atan2(C[1] - D[1], C[0] - D[0])
    z0 = L0 + R + 0.002
    use_mesh = visuals and v["name"].startswith("V3") and os.path.isdir(CAD_ASM)

    # ---------------- body geoms ----------------
    bs = v["body_size"]
    bz = v["body_center_z"]
    body = []
    body.append(f'<geom name="body_col" type="box" size="{bs[0]/2:.4f} {bs[1]/2:.4f} {bs[2]/2:.4f}" '
                f'pos="0 0 {bz:.4f}" contype="2" conaffinity="0" mass="0" group="3" rgba="1 1 0 0.2"/>')
    com = mb["body_print_com"]
    body.append(f'<geom name="m_shell" type="box" size="{bs[0]/2*0.9:.4f} {bs[1]/2*0.9:.4f} {bs[2]/2*0.9:.4f}" '
                f'pos="{com[0]:.4f} {com[1]:.4f} {com[2]:.4f}" mass="{mb["body_print"]:.4f}" '
                f'contype="0" conaffinity="0" group="4"/>')
    for name, (m, pos, size) in v["components"].items():
        body.append(f'<geom name="m_{name}" type="box" size="{_fmt(np.array(size)/2)}" pos="{_fmt(pos)}" '
                    f'mass="{m}" contype="0" conaffinity="0" group="4"/>')
    hip_y = v["leg_y"] - 0.057
    for s in (1, -1):
        for hx in (l5 / 2, -l5 / 2):
            body.append(f'<geom type="cylinder" size="0.028 0.023" pos="{hx} {s*hip_y:.4f} 0" zaxis="0 1 0" '
                        f'mass="{mb["hip_motor"]}" contype="0" conaffinity="0" group="4"/>')
    if payload > 0:
        pp = v["payload_pos"]
        body.append(f'<geom name="payload" type="box" size="0.05 0.05 0.04" pos="{_fmt(pp)}" mass="{payload}" '
                    f'rgba="0.2 0.6 1 1" contype="0" conaffinity="0" group="1"/>')
    if use_mesh:
        for n, rgba in (("shell_front", "1 0.82 0.25 1"), ("shell_rear", "1 0.82 0.25 1"),
                        ("belly_pan", "0.23 0.23 0.35 1"), ("visor", "0.08 0.09 0.14 1"),
                        ("bumper_front", "1 0.42 0.42 1"), ("bumper_rear", "1 0.42 0.42 1"),
                        ("payload_lid", "1 0.72 0.0 1"), ("electronics_cap", "1 0.72 0.0 1")):
            body.append(f'<geom type="mesh" mesh="{n}" rgba="{rgba}" contype="0" conaffinity="0" mass="0" group="1"/>')
        # glowing eyes
        for s in (1, -1):
            body.append(f'<geom type="ellipsoid" size="0.0025 0.015 0.019" pos="{bs[0]/2 + 0.0005:.4f} {s*0.040} 0.088" '
                        f'rgba="0.35 0.95 1 1" material="glow" contype="0" conaffinity="0" mass="0" group="1"/>')
            body.append(f'<geom type="ellipsoid" size="0.0028 0.005 0.006" pos="{bs[0]/2 + 0.0012:.4f} {s*0.040 + 0.005} 0.096" '
                        f'rgba="1 1 1 1" material="glow" contype="0" conaffinity="0" mass="0" group="1"/>')
        # little smile on the face screen
        for k in range(7):
            a = math.radians(-50 + k * 100 / 6)
            body.append(f'<geom type="sphere" size="0.0022" pos="{bs[0]/2 + 0.0005:.4f} {0.014*math.sin(a):.4f} {0.070 - 0.008*math.cos(a):.4f}" '
                        f'rgba="0.35 0.95 1 1" material="glow" contype="0" conaffinity="0" mass="0" group="1"/>')
    else:
        body.append(f'<geom type="box" size="{bs[0]/2:.4f} {bs[1]/2:.4f} {bs[2]/2:.4f}" pos="0 0 {bz:.4f}" '
                    f'rgba="1 0.82 0.25 1" contype="0" conaffinity="0" mass="0" group="1"/>')
    # GPS puck on its stub (collides: it is the highest point of the robot)
    body.append(f'<geom name="gps_col" type="cylinder" size="0.027 0.010" pos="-0.082 0 {bz + bs[2]/2 + 0.032:.4f}" '
                f'contype="2" conaffinity="0" mass="0" rgba="0.15 0.15 0.15 1" group="1"/>')
    body.append('<site name="imu" pos="-0.08 0 0.09"/>')
    if cameras:
        fz = 0.128
        body.append(f'<camera name="cam_front" pos="{bs[0]/2 + 0.004:.4f} 0 {fz}" '
                    f'xyaxes="0 -1 0 {math.sin(math.radians(10)):.4f} 0 {math.cos(math.radians(10)):.4f}" fovy="120"/>')
        body.append(f'<camera name="cam_rear" pos="{-bs[0]/2 - 0.004:.4f} 0 {fz}" '
                    f'xyaxes="0 1 0 {-math.sin(math.radians(10)):.4f} 0 {math.cos(math.radians(10)):.4f}" fovy="120"/>')
        # ultrasonic beams (3 front, 1 rear) approximated by 3 rays each
        for nm, yaw in (("us_fc", 0), ("us_fl", 45), ("us_fr", -45), ("us_r", 180)):
            for k, dy in enumerate((-12, 0, 12)):
                a = math.radians(yaw + dy)
                x = (bs[0] / 2) * (1 if abs(yaw) < 90 else -1)
                y = 0.0 if yaw in (0, 180) else math.copysign(bs[1] / 2 - 0.03, yaw)
                body.append(f'<site name="{nm}_{k}" pos="{x:.3f} {y:.3f} 0.038" '
                            f'zaxis="{math.cos(a):.4f} {math.sin(a):.4f} {-0.05:.3f}"/>')
        # Sharp IR cliff sensors in the chin, looking 30 deg forward of straight down
        for nm, yy in (("cliff_l", 0.05), ("cliff_r", -0.05)):
            body.append(f'<site name="{nm}" pos="{bs[0]/2 - 0.015:.3f} {yy} {bz - bs[2]/2 + 0.012:.4f}" zaxis="0.5 0 -0.866"/>')
        body.append(f'<light name="headlight" pos="{bs[0]/2:.3f} 0 0.09" dir="1 0 -0.35" diffuse="{1.0 if dark else 0.0} {1.0 if dark else 0.0} {0.9 if dark else 0.0}" '
                    f'cutoff="60" exponent="2" attenuation="1 0 0.02" castshadow="false"/>')

    # ---------------- legs ----------------
    legs = []
    for side, s in (("L", 1), ("R", -1)):
        ly = s * v["leg_y"]
        wy = s * (v["wheel_y"] - v["leg_y"])
        rot_th = 90 if s > 0 else -90
        th_off = (-0.007 if s > 0 else 0.007)
        sf_off = (0.0045 if s > 0 else -0.0045) - (0.0 if s > 0 else 0.0)
        mesh_th = (f'<geom type="mesh" mesh="thigh" euler="{math.radians(rot_th):.5f} 0 0" pos="0 {th_off} 0" '
                   f'rgba="0.23 0.23 0.35 1" contype="0" conaffinity="0" mass="0" group="1"/>') if use_mesh else \
            f'<geom type="capsule" fromto="0 0 0 {l1} 0 0" size="0.012" rgba="0.23 0.23 0.35 1" contype="0" conaffinity="0" mass="0" group="1"/>'

        def shin_vis(name, yoff, rot):
            if use_mesh:
                return (f'<geom type="mesh" mesh="{name}" euler="{math.radians(rot):.5f} 0 0" pos="0 {yoff} 0" '
                        f'rgba="0.36 0.36 0.54 1" contype="0" conaffinity="0" mass="0" group="1"/>')
            return (f'<geom type="capsule" fromto="0 0 0 {l2} 0 0" size="0.010" rgba="0.36 0.36 0.54 1" '
                    f'contype="0" conaffinity="0" mass="0" group="1"/>')
        wheel_vis = ""
        if use_mesh:
            wheel_vis = (f'<geom type="mesh" mesh="wheel_rim" euler="{math.radians(90):.5f} 0 0" rgba="0.15 0.15 0.18 1" contype="0" conaffinity="0" mass="0" group="1"/>'
                         f'<geom type="mesh" mesh="tyre" euler="{math.radians(90):.5f} 0 0" rgba="0.08 0.08 0.08 1" contype="0" conaffinity="0" mass="0" group="1"/>')
        knee_off_F = psiF - p1
        knee_off_R = psiR - p4
        legs.append(f'''
      <body name="thigh_F_{side}" pos="{l5/2} {ly:.4f} 0" axisangle="{_aa_neg_y(p1)}">
        <joint name="hipF_{side}" type="hinge" axis="0 -1 0" damping="0.02" armature="0.004" limited="false"/>
        <geom type="capsule" fromto="0 0 0 {l1} 0 0" size="0.012" mass="{mb['thigh']:.4f}" contype="0" conaffinity="0" group="4"/>
        {mesh_th}
        <body name="shin_F_{side}" pos="{l1} 0 0" axisangle="{_aa_neg_y(knee_off_F)}">
          <joint name="kneeF_{side}" type="hinge" axis="0 -1 0" damping="0.005"/>
          <geom type="capsule" fromto="0 0 0 {l2} 0 0" size="0.010" mass="{mb['shin_front']:.4f}" contype="2" conaffinity="0" group="4"/>
          {shin_vis("shin_front", s * 0.0045 + (0.0 if s > 0 else 0.0), 90 if s > 0 else -90)}
        </body>
      </body>
      <body name="thigh_R_{side}" pos="{-l5/2} {ly:.4f} 0" axisangle="{_aa_neg_y(p4)}">
        <joint name="hipR_{side}" type="hinge" axis="0 -1 0" damping="0.02" armature="0.004" limited="false"/>
        <geom type="capsule" fromto="0 0 0 {l1} 0 0" size="0.012" mass="{mb['thigh']:.4f}" contype="0" conaffinity="0" group="4"/>
        {mesh_th}
        <body name="shin_R_{side}" pos="{l1} 0 0" axisangle="{_aa_neg_y(knee_off_R)}">
          <joint name="kneeR_{side}" type="hinge" axis="0 -1 0" damping="0.005"/>
          <geom type="capsule" fromto="0 0 0 {l2} 0 0" size="0.010" mass="{mb['shin_rear']:.4f}" contype="2" conaffinity="0" group="4"/>
          <geom type="cylinder" size="0.046 0.008" pos="{l2} {s*0.025:.4f} 0" zaxis="0 1 0" mass="{mb['wheel_stator']:.4f}" contype="0" conaffinity="0" group="4"/>
          {shin_vis("shin_rear", s * 0.016, -90 if s > 0 else 90)}
          <body name="wheel_{side}" pos="{l2} {wy:.4f} 0" axisangle="{_aa_neg_y(-psiR)}">
            <joint name="wheel_{side}" type="hinge" axis="0 1 0" damping="0.002" armature="0.0005"/>
            <geom name="tyre_{side}" type="cylinder" size="{R} {Wd/2}" zaxis="0 1 0" mass="{mb['wheel_print']:.4f}"
                  contype="2" conaffinity="0" friction="1.0 0.02 0.001" condim="4" rgba="0.1 0.1 0.1 {0 if use_mesh else 1}" group="{3 if use_mesh else 1}"
                  solref="0.004 1" solimp="0.9 0.95 0.001"/>
            <geom type="cylinder" size="0.044 0.012" zaxis="0 1 0" mass="{mb['wheel_rotor']:.4f}" contype="0" conaffinity="0" group="4"/>
            {wheel_vis}
          </body>
        </body>
      </body>''')

    eq = []
    for side in ("L", "R"):
        eq.append(f'<connect body1="shin_F_{side}" body2="shin_R_{side}" anchor="{l2} 0 0" solref="0.002 1" solimp="0.95 0.99 0.001"/>')

    act = []
    for side in ("L", "R"):
        for j in ("hipF", "hipR"):
            act.append(f'<motor name="{j}_{side}" joint="{j}_{side}" ctrllimited="true" ctrlrange="-{P.HIP_MOTOR["peak_torque"]} {P.HIP_MOTOR["peak_torque"]}"/>')
    for side in ("L", "R"):
        act.append(f'<motor name="wheel_{side}" joint="wheel_{side}" ctrllimited="true" ctrlrange="-{P.WHEEL_MOTOR["peak_torque"]} {P.WHEEL_MOTOR["peak_torque"]}"/>')

    sensors = ['<gyro name="gyro" site="imu"/>', '<accelerometer name="acc" site="imu"/>']
    if cameras:
        for nm in ("us_fc", "us_fl", "us_fr", "us_r"):
            for k in range(3):
                sensors.append(f'<rangefinder name="{nm}_{k}" site="{nm}_{k}" cutoff="4.0"/>')
        for nm in ("cliff_l", "cliff_r"):
            sensors.append(f'<rangefinder name="{nm}" site="{nm}" cutoff="0.6"/>')

    t_asset, t_world = terrain_xml(terrain)
    meshes = ""
    if use_mesh:
        names = ["shell_front", "shell_rear", "belly_pan", "visor", "bumper_front", "bumper_rear",
                 "payload_lid", "electronics_cap", "thigh", "shin_front", "shin_rear", "wheel_rim", "tyre"]
        meshes = "\n".join(f'<mesh name="{n}" file="{os.path.join(CAD_ASM, n + ".stl")}"/>' for n in names)
    floor_tex = (f'<texture name="floortex" type="2d" file="{floor_texture}"/>'
                 f'<material name="linefloor" texture="floortex" texrepeat="1 1" texuniform="false"/>') if floor_texture else ""
    amb = "0.05 0.05 0.05" if dark else "0.35 0.35 0.35"
    dif = "0.02 0.02 0.02" if dark else "0.7 0.7 0.7"

    xml = f'''<mujoco model="BOLT {v['name']}">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.0005" integrator="implicitfast" cone="elliptic" impratio="5"/>
  <visual><global offwidth="1280" offheight="720"/><quality shadowsize="2048"/>
          <headlight ambient="{amb}" diffuse="{dif}"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.82 0.84 0.86" rgb2="0.74 0.76 0.78" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="40 40" reflectance="0.0"/>
    <material name="ramp" rgba="0.55 0.62 0.68 1"/>
    <material name="glow" emission="1"/>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="{'0.02 0.02 0.05' if dark else '0.6 0.75 0.95'}" rgb2="{'0.0 0.0 0.02' if dark else '0.95 0.97 1'}" width="256" height="256"/>
    {floor_tex}
    {meshes}
    {t_asset}
  </asset>
  <worldbody>
    <light name="sun" pos="0 0 6" dir="0.2 0.3 -1" diffuse="{dif}" castshadow="true"/>
    {t_world}
    <body name="base" pos="0 0 {z0:.4f}">
      <freejoint name="root"/>
      {"".join(body)}
      {"".join(legs)}
    </body>
  </worldbody>
  <equality>{"".join(eq)}</equality>
  <actuator>{"".join(act)}</actuator>
  <sensor>{"".join(sensors)}</sensor>
</mujoco>'''
    meta = dict(phi1_0=p1, phi4_0=p4, z0=z0, variant=v, mass=mb)
    return xml, meta


if __name__ == "__main__":
    import mujoco
    for var in ("V1", "V2", "V3"):
        xml, meta = build_mjcf(var)
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        tot = sum(m.body_mass)
        com = d.subtree_com[1]
        print(f"{var}: total mass {tot:.3f} kg   COM above hip {com[2] - d.xpos[1][2]:+.4f} m  "
              f"COM x {com[0]:+.4f}")
