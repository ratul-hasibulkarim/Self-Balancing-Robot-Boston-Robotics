#!/usr/bin/env python3
"""
LQR gain-schedule design for BOLT.

For a grid of leg lengths L the robot is reduced to the classic sagittal
wheel - leg - body model (3 DOF: leg angle theta, wheel travel s, body
pitch phi).  All lumped masses / inertias / COM offsets are measured from
the MuJoCo model (which itself comes from the CAD + params), the model is
linearised around its static equilibrium and an infinite-horizon LQR is
solved.  Each gain is then fitted with a cubic polynomial in L.

    python3 sim/lqr_design.py            # all variants, writes the firmware header for FINAL
"""
import json
import math
import os
import sys

import mujoco
import numpy as np
from scipy.linalg import solve_continuous_are

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "design"))
import kinematics  # noqa: E402
import params as P  # noqa: E402
import robot_model  # noqa: E402

G = 9.81

# State x = [theta, dtheta, s, ds, pitch, dpitch]   input u = [T_wheel_total, T_hip_total]
Q_DIAG = [40.0, 1.0, 120.0, 25.0, 900.0, 4.0]
R_DIAG = [1.0, 0.35]


def set_leg_pose(m, d, meta, L, theta_b=0.0):
    v = meta["variant"]
    l1, l2, l5 = v["thigh"], v["shin"], v["hip_spacing"]
    p1, p4 = kinematics.inverse(L, theta_b, l1, l2, l5)
    fk = kinematics.forward(p1, p4, l1, l2, l5)
    B, D, C = fk["B"], fk["D"], fk["C"]
    psiF = math.atan2(C[1] - B[1], C[0] - B[0])
    psiR = math.atan2(C[1] - D[1], C[0] - D[0])
    fk0 = kinematics.forward(meta["phi1_0"], meta["phi4_0"], l1, l2, l5)
    psiF0 = math.atan2(fk0["C"][1] - fk0["B"][1], fk0["C"][0] - fk0["B"][0])
    psiR0 = math.atan2(fk0["C"][1] - fk0["D"][1], fk0["C"][0] - fk0["D"][0])
    for side in ("L", "R"):
        d.qpos[m.jnt_qposadr[m.joint(f"hipF_{side}").id]] = p1 - meta["phi1_0"]
        d.qpos[m.jnt_qposadr[m.joint(f"hipR_{side}").id]] = p4 - meta["phi4_0"]
        d.qpos[m.jnt_qposadr[m.joint(f"kneeF_{side}").id]] = (psiF - p1) - (psiF0 - meta["phi1_0"])
        d.qpos[m.jnt_qposadr[m.joint(f"kneeR_{side}").id]] = (psiR - p4) - (psiR0 - meta["phi4_0"])
    return p1, p4


def _iyy(m, d, bid):
    """pitch inertia (about world y through the body COM)."""
    R = d.ximat[bid].reshape(3, 3)
    I = R @ np.diag(m.body_inertia[bid]) @ R.T
    return I[1, 1]


def lumped(m, d, meta, L):
    set_leg_pose(m, d, meta, L)
    d.qpos[:3] = [0, 0, 1.0]
    d.qpos[3:7] = [1, 0, 0, 0]
    mujoco.mj_kinematics(m, d)
    mujoco.mj_comPos(m, d)
    base = m.body("base").id
    hip = d.xpos[base].copy()
    mb = m.body_mass[base]
    cb = d.xipos[base] - hip
    Ib = _iyy(m, d, base)
    leg_ids = [m.body(f"{n}_{s}").id for n in ("thigh_F", "thigh_R", "shin_F", "shin_R") for s in "LR"]
    wheel_ids = [m.body(f"wheel_{s}").id for s in "LR"]
    ml = sum(m.body_mass[i] for i in leg_ids)
    cl = sum(m.body_mass[i] * d.xipos[i] for i in leg_ids) / ml
    Il = sum(_iyy(m, d, i) + m.body_mass[i] * ((d.xipos[i][0] - cl[0]) ** 2 + (d.xipos[i][2] - cl[2]) ** 2)
             for i in leg_ids)
    mw = sum(m.body_mass[i] for i in wheel_ids)
    Iw = sum(m.body_inertia[i][1] for i in wheel_ids)    # wheel frames only rotate about y
    wc = d.xpos[wheel_ids[0]]
    lw = (cl[2] - wc[2])            # leg COM height above the wheel axle (leg vertical)
    return dict(mb=mb, bx=cb[0], bz=cb[2], Ib=Ib, ml=ml, lw=lw, Il=Il, mw=mw, Iw=Iw, L=L)


def linearise(p, R_wheel):
    mb, bx, bz, Ib, ml, lw, Il, mw, Iw, L = (p[k] for k in ("mb", "bx", "bz", "Ib", "ml", "lw", "Il", "mw", "Iw", "L"))

    def points(q):
        th, s, ph = q
        W = np.array([s, R_wheel])
        up = np.array([-math.sin(th), math.cos(th)])
        lc = W + lw * up
        H = W + L * up
        bc = H + np.array([bx * math.cos(ph) + bz * math.sin(ph), -bx * math.sin(ph) + bz * math.cos(ph)])
        return W, lc, bc

    def V(q):
        _, lc, bc = points(q)
        return G * (ml * lc[1] + mb * bc[1])

    def gen_grav(q, eps=1e-6):
        g = np.zeros(3)
        for i in range(3):
            dq = np.zeros(3); dq[i] = eps
            g[i] = -(V(q + dq) - V(q - dq)) / (2 * eps)
        return g

    def mass_matrix(q, eps=1e-6):
        Jw, Jl, Jb = (np.zeros((2, 3)) for _ in range(3))
        for i in range(3):
            dq = np.zeros(3); dq[i] = eps
            a, b = points(q + dq), points(q - dq)
            Jw[:, i] = (a[0] - b[0]) / (2 * eps)
            Jl[:, i] = (a[1] - b[1]) / (2 * eps)
            Jb[:, i] = (a[2] - b[2]) / (2 * eps)
        M = mw * Jw.T @ Jw + ml * Jl.T @ Jl + mb * Jb.T @ Jb
        M[0, 0] += Il
        M[2, 2] += Ib
        M[1, 1] += Iw / R_wheel ** 2
        return M

    Bq = np.array([[1.0, 1.0], [1.0 / R_wheel, 0.0], [0.0, 1.0]])
    # equilibrium: theta_eq such that generalized gravity is cancelled by Tp alone
    th = 0.0
    for _ in range(50):
        g = gen_grav(np.array([th, 0, 0]))
        f = g[0] - g[2]                       # theta eq. after Tp = -g_phi
        dg = (gen_grav(np.array([th + 1e-5, 0, 0])) - gen_grav(np.array([th - 1e-5, 0, 0]))) / 2e-5
        th -= f / (dg[0] - dg[2])
    q0 = np.array([th, 0, 0])
    M = mass_matrix(q0)
    K = np.zeros((3, 3))
    for i in range(3):
        dq = np.zeros(3); dq[i] = 1e-5
        K[:, i] = (gen_grav(q0 + dq) - gen_grav(q0 - dq)) / 2e-5
    Minv = np.linalg.inv(M)
    A = np.zeros((6, 6)); B = np.zeros((6, 2))
    idx = [0, 2, 4]
    for r in range(3):
        A[idx[r], idx[r] + 1] = 1.0
    MK = Minv @ K
    MB = Minv @ Bq
    for r in range(3):
        for c in range(3):
            A[idx[r] + 1, idx[c]] = MK[r, c]
        B[idx[r] + 1, :] = MB[r, :]
    return A, B, th


def lqr(A, B, Qd=Q_DIAG, Rd=R_DIAG):
    Q, R = np.diag(Qd), np.diag(Rd)
    X = solve_continuous_are(A, B, Q, R)
    return np.linalg.solve(R, B.T @ X)


def design(variant="V3", Qd=Q_DIAG, Rd=R_DIAG, verbose=True):
    xml, meta = robot_model.build_mjcf(variant, visuals=False, cameras=False)
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    v = meta["variant"]
    Ls = np.linspace(v["L_min"] - 0.01, v["L_max"] + 0.01, 14)
    Ks, lp = [], []
    for L in Ls:
        p = lumped(m, d, meta, L)
        A, B, th = linearise(p, v["wheel_radius"])
        K = lqr(A, B, Qd, Rd)
        cl = np.linalg.eigvals(A - B @ K)
        Ks.append(K)
        lp.append(p)
        if verbose:
            print(f"L={L:.3f}  theta_eq={math.degrees(th):+.2f} deg  slowest pole {max(cl.real):+.2f}  "
                  f"K0={np.round(K[0], 2)}  K1={np.round(K[1], 2)}")
    Ks = np.array(Ks)
    coef = np.zeros((2, 6, 4))
    for i in range(2):
        for j in range(6):
            c = np.polyfit(Ls, Ks[:, i, j], 3)       # highest power first
            coef[i, j] = c[::-1]
    pn = lumped(m, d, meta, v["L_nom"])
    total = sum(m.body_mass)
    mleg = (total - pn["mb"]) / 2
    com_h = (pn["mb"] * (v["L_nom"] + v["wheel_radius"] + pn["bz"]) +
             (total - pn["mb"]) * (v["wheel_radius"] + 0.4 * v["L_nom"])) / total
    cfg = dict(
        l1=v["thigh"], l2=v["shin"], l5=v["hip_spacing"], wheel_radius=v["wheel_radius"],
        track=2 * v["wheel_y"], m_body=float(pn["mb"]), body_com_x=float(pn["bx"]), body_com_z=float(pn["bz"]),
        m_leg=float(mleg), com_height_nom=float(com_h),
        hip_torque_peak=P.HIP_MOTOR["peak_torque"], hip_torque_cont=P.HIP_MOTOR["rated_torque"],
        wheel_torque_peak=P.WHEEL_MOTOR["peak_torque"],
        hip_speed_max=P.HIP_MOTOR["max_speed"], wheel_speed_max=P.WHEEL_MOTOR["max_speed"],
        L_min=v["L_min"], L_nom=v["L_nom"], L_max=v["L_max"],
        v_max=2.0, acc_max=2.5, yaw_rate_max=3.0, pitch_cmd_max=0.45, roll_cmd_max=0.30,
        K=coef.tolist(),
        kp_leg=900.0, kd_leg=45.0, kp_roll=250.0, kd_roll=18.0, ki_roll=120.0,
        kp_yaw=1.2, kd_yaw=0.01, kp_sync=25.0, kd_sync=1.5,
        L_crouch=v["L_min"] + 0.012, L_takeoff=v["L_max"] - 0.012, L_retract=v["L_min"] + 0.02,
        thrust_force=420.0, jump_clearance=0.05,
        fall_pitch=0.85, fall_roll=0.75, comm_timeout=0.3, ki_yaw=6.0,
    )
    out = os.path.join(HERE, "build", f"config_{variant}.json")
    json.dump(cfg, open(out, "w"), indent=1)
    if verbose:
        print(f"lumped @L_nom: {json.dumps({k: round(float(x), 5) for k, x in pn.items()})}")
        print("wrote", out)
    return cfg


def write_firmware_header(cfg, path):
    def f(x):
        t = f"{float(x):.6g}"
        if "." not in t and "e" not in t and "inf" not in t:
            t += ".0"
        return t + "f"
    lines = ["// AUTO-GENERATED by sim/lqr_design.py - do not edit by hand.",
             "// Re-run `python3 sim/lqr_design.py` after changing design/params.py.",
             "#pragma once", '#include "bolt_controller.h"', "", "namespace bolt {", "",
             "inline RobotConfig default_config() {", "  RobotConfig c{};"]
    for k, val in cfg.items():
        if k == "K":
            for i in range(2):
                for j in range(6):
                    lines.append(f"  c.K[{i}][{j}][0] = {f(val[i][j][0])}; c.K[{i}][{j}][1] = {f(val[i][j][1])}; "
                                 f"c.K[{i}][{j}][2] = {f(val[i][j][2])}; c.K[{i}][{j}][3] = {f(val[i][j][3])};")
        else:
            lines.append(f"  c.{k} = {f(val)};")
    lines += ["  return c;", "}", "", "}  // namespace bolt", ""]
    open(path, "w").write("\n".join(lines))


if __name__ == "__main__":
    for var in sys.argv[1:] or ["V1", "V2", "V3"]:
        print(f"===== {var} =====")
        cfg = design(var)
        if var == "V3":
            hdr = os.path.join(ROOT, "firmware", "lib", "bolt_core", "robot_config_generated.h")
            write_firmware_header(cfg, hdr)
            print("wrote", hdr)
