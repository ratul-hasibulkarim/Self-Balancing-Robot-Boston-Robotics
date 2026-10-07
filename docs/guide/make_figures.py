#!/usr/bin/env python3
"""Figures for the beginner's guide (docs/guide/*.md).   python3 docs/guide/make_figures.py"""
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch, Rectangle  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
OUT = os.path.join(HERE, "img")
os.makedirs(OUT, exist_ok=True)
sys.path.insert(0, os.path.join(ROOT, "design"))
import kinematics as K  # noqa: E402
from params import FINAL as P, HIP_MOTOR, WHEEL_MOTOR  # noqa: E402

l1, l2, l5, R = P["thigh"], P["shin"], P["hip_spacing"], P["wheel_radius"]


# ---------------------------------------------------------------- 1. five-bar poses
def fivebar():
    fig, axs = plt.subplots(1, 4, figsize=(16, 5.2), dpi=110)
    poses = [(P["L_min"], 0.0, "crouched  L = 0.13 m"), (P["L_nom"], 0.0, "standing  L = 0.22 m"),
             (P["L_max"], 0.0, "stretched  L = 0.35 m"), (P["L_nom"], math.radians(20), "leg angled  θb = +20°")]
    for ax, (L, th, title) in zip(axs, poses):
        p1, p4 = K.inverse(L, th, l1, l2, l5)
        f = K.forward(p1, p4, l1, l2, l5)
        A, E = (l5 / 2, 0), (-l5 / 2, 0)
        B, D, C = f["B"], f["D"], f["C"]
        ax.add_patch(Rectangle((-0.12, -0.02), 0.24, 0.09, fc="#ffd23f55", ec="#c79a00"))
        ax.text(0, 0.045, "body", ha="center", fontsize=8, color="#8a6d00")
        ax.plot([A[0], B[0]], [A[1], B[1]], color="#1f77b4", lw=5, solid_capstyle="round", label="thighs (driven)")
        ax.plot([E[0], D[0]], [E[1], D[1]], color="#1f77b4", lw=5, solid_capstyle="round")
        ax.plot([B[0], C[0]], [B[1], C[1]], color="#ff7f0e", lw=4, solid_capstyle="round", label="shins (passive)")
        ax.plot([D[0], C[0]], [D[1], C[1]], color="#ff7f0e", lw=4, solid_capstyle="round")
        ax.plot([0, C[0]], [0, C[1]], "--", color="#2ca02c", lw=1.6, label="virtual leg L, θb")
        ax.add_patch(Circle(C, R, fc="#22222233", ec="#222"))
        offs = {"A": (4, 14), "E": (-30, 14), "B": (5, 5), "D": (-38, 5), "C": (8, -12)}
        for (pt, nm, col) in ((A, "A", "#1f77b4"), (E, "E", "#1f77b4"), (B, "B knee", "#444"),
                              (D, "D knee", "#444"), (C, "C foot / wheel axle", "#2ca02c")):
            ax.plot(*pt, "o", color=col, ms=7, zorder=5)
            ax.annotate(nm, pt, textcoords="offset points", xytext=offs[nm[0]], fontsize=8, weight="bold")
        ax.text(0.0, 0.13, f"φ1 = {math.degrees(p1):.0f}°   φ4 = {math.degrees(p4):.0f}°", fontsize=8.5,
                color="#1f77b4", ha="center")
        ax.text(0.0, -0.43, "A = front hip motor, E = rear hip motor", fontsize=7.5, ha="center", color="#555")
        ax.set_title(title, fontsize=10)
        ax.set_aspect("equal")
        ax.set_xlim(-0.26, 0.26)
        ax.set_ylim(-0.45, 0.17)
        ax.grid(alpha=0.25)
    axs[0].legend(fontsize=7.5, loc="lower left")
    fig.suptitle("The 5-bar leg (seen from the robot's right side, front = right). Two motors at A and E drive the thighs;\n"
                 "moving them TOGETHER changes the leg length, moving them the SAME WAY swings the leg.", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fivebar_poses.png"))
    plt.close(fig)


# ---------------------------------------------------------------- 2. leg force vs length
def leg_force():
    Ls = np.linspace(P["L_min"], P["L_max"], 60)
    dLdphi, Fmax = [], []
    for L in Ls:
        p1, p4 = K.inverse(L, 0, l1, l2, l5)
        J = K.jacobian(p1, p4, l1, l2, l5)
        # straight push: dphi1 = -dphi4 ; F dL = T1 dphi1 + T4 dphi4  with |T| = peak
        g = abs(J[0][0]) + abs(J[0][1])
        dLdphi.append(abs(J[0][0]))
        Fmax.append(2 * HIP_MOTOR["peak_torque"] / g)
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.2), dpi=110)
    ax[0].plot(Ls, np.array(dLdphi) * 1000, lw=2)
    ax[0].set_xlabel("virtual leg length L [m]"); ax[0].set_ylabel("dL/dφ  [mm per rad]")
    ax[0].set_title("Leg 'gear ratio': how much L moves per motor radian")
    ax[1].plot(Ls, Fmax, lw=2, color="#d62728", label="max leg force (both motors at 7 Nm)")
    ax[1].axhline(P["components"]["battery_6S_5000"][0] * 0 + 4.2 * 9.81 / 2, color="#555", ls="--",
                  label="force to just hold the body (≈ 21 N per leg)")
    ax[1].set_xlabel("virtual leg length L [m]"); ax[1].set_ylabel("force along the leg [N]")
    ax[1].set_title("Why it can jump: 4-8x the holding force per leg")
    ax[1].legend(fontsize=8)
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "leg_force.png"))
    plt.close(fig)


# ---------------------------------------------------------------- 3. pendulum model
def pendulum():
    fig, ax = plt.subplots(figsize=(7, 7), dpi=110)
    th, ph = math.radians(-14), math.radians(18)
    W = np.array([0.0, R])
    Lv = 0.24
    H = W + Lv * np.array([-math.sin(th), math.cos(th)])
    ax.axhline(0, color="#777", lw=2)
    ax.add_patch(Circle(W, R, fc="#33333333", ec="#222", lw=2))
    ax.plot(*zip(W, H), color="#5c5c8a", lw=6, solid_capstyle="round")
    # body as a rotated rectangle around H
    c, s = math.cos(ph), math.sin(ph)
    rect = np.array([[-0.1, -0.06], [0.1, -0.06], [0.1, 0.16], [-0.1, 0.16], [-0.1, -0.06]])
    rot = np.array([[c, s], [-s, c]])
    pts = rect @ rot.T + H
    ax.fill(pts[:, 0], pts[:, 1], fc="#ffd23f88", ec="#c79a00", lw=2)
    com = H + np.array([0.0, 0.024]) @ rot.T
    ax.plot(*com, "o", color="#c62828", ms=10)
    ax.annotate("body COM", com, xytext=(com[0] + 0.05, com[1] + 0.05), fontsize=9, arrowprops=dict(arrowstyle="->"))
    ax.plot(*H, "s", color="#2e7d32", ms=9)
    ax.annotate("hip", H, xytext=(H[0] - 0.12, H[1] - 0.02), fontsize=9)
    ax.plot([W[0], W[0]], [W[1], W[1] + 0.3], ":", color="#555")
    ax.annotate("", xy=(W[0] + 0.07 * math.sin(-th) * -1, W[1] + 0.12), xytext=(W[0], W[1] + 0.13),
                arrowprops=dict(arrowstyle="<-", color="#2ca02c"))
    ax.text(W[0] - 0.05, W[1] + 0.16, "θ  leg angle\n(foot ahead > 0)", fontsize=9, color="#2ca02c", ha="right")
    ax.plot([H[0], H[0]], [H[1], H[1] + 0.25], ":", color="#555")
    ax.text(H[0] + 0.06, H[1] + 0.25, "φ  body pitch\n(nose down > 0)", fontsize=9, color="#c79a00")
    ax.add_patch(FancyArrowPatch((W[0], -0.04), (W[0] + 0.18, -0.04), arrowstyle="->", mutation_scale=15, color="#1f77b4"))
    ax.text(W[0] + 0.09, -0.07, "s, ds/dt  wheel travel / speed", fontsize=9, color="#1f77b4", ha="center")
    ax.add_patch(FancyArrowPatch((W[0] + 0.1, W[1]), (W[0] + 0.05, W[1] + 0.09), connectionstyle="arc3,rad=0.5",
                                 arrowstyle="->", mutation_scale=14, color="#d62728"))
    ax.text(W[0] + 0.11, W[1] + 0.03, "T_wheel", color="#d62728", fontsize=9)
    ax.add_patch(FancyArrowPatch((H[0] + 0.05, H[1] - 0.04), (H[0] + 0.0, H[1] - 0.07), connectionstyle="arc3,rad=0.5",
                                 arrowstyle="->", mutation_scale=14, color="#d62728"))
    ax.text(H[0] + 0.06, H[1] - 0.09, "T_hip (Tp)", color="#d62728", fontsize=9)
    ax.set_aspect("equal"); ax.set_xlim(-0.3, 0.35); ax.set_ylim(-0.1, 0.6); ax.axis("off")
    ax.set_title("The balance model: wheel + virtual leg + body (3 bodies, 3 angles/positions)\n"
                 "state x = [θ, dθ/dt, s, ds/dt, φ, dφ/dt]   inputs u = [T_wheel, T_hip]", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "pendulum_model.png"))
    plt.close(fig)


# ---------------------------------------------------------------- 4. LQR gains vs L
def gains():
    cfg = json.load(open(os.path.join(ROOT, "sim", "build", "config_V3.json")))
    Kc = np.array(cfg["K"])
    Ls = np.linspace(P["L_min"], P["L_max"], 50)
    names = ["θ", "dθ/dt", "s", "ds/dt", "φ", "dφ/dt"]
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.2), dpi=110)
    for i, (ax, uname) in enumerate(zip(axs, ("wheel torque gains  K[0][j]", "hip torque gains  K[1][j]"))):
        for j in range(6):
            ax.plot(Ls, [sum(Kc[i, j, k] * L ** k for k in range(4)) for L in Ls], label=names[j], lw=2)
        ax.set_title(uname); ax.set_xlabel("leg length L [m]"); ax.grid(alpha=0.3); ax.legend(fontsize=8, ncol=3)
    fig.suptitle("Gain scheduling: the LQR gains are re-computed for every leg length (cubic fit of 14 designs)", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "lqr_gains.png"))
    plt.close(fig)


# ---------------------------------------------------------------- 5. motor envelope
def motors():
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.2), dpi=110)
    for ax, m, extra in ((axs[0], HIP_MOTOR, None), (axs[1], WHEEL_MOTOR, R)):
        w = np.linspace(0, m["max_speed"], 50)
        ax.fill_between(w, 0, m["peak_torque"] * (1 - w / m["max_speed"]), color="#1f77b433", label="available (peak)")
        ax.plot(w, m["rated_torque"] * np.ones_like(w) * (w < m["max_speed"] * (1 - m["rated_torque"] / m["peak_torque"])),
                "--", color="#555", label="continuous rating")
        ax.set_xlabel("speed [rad/s]"); ax.set_ylabel("torque [Nm]"); ax.grid(alpha=0.3)
        ax.set_title(m["name"])
        if extra:
            for v, col in ((2.0, "#2ca02c"), (2.4, "#d62728")):
                ax.axvline(v / extra, color=col, lw=2, label=f"{v} m/s on 150 mm wheels")
            ax.axvline(2.0 / 0.06, color="#9467bd", lw=2, ls=":", label="2.0 m/s on 120 mm wheels (V1/V2)")
        ax.legend(fontsize=8)
    fig.suptitle("Motor torque-speed envelope used in the simulator (back-EMF limit): faster = less torque left for balancing", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "motor_envelope.png"))
    plt.close(fig)


# ---------------------------------------------------------------- 6. architecture
def architecture():
    fig, ax = plt.subplots(figsize=(14, 7.5), dpi=110)
    ax.set_xlim(0, 28); ax.set_ylim(0, 15); ax.axis("off")

    def b(x, y, w, h, t, sub="", fc="#eef3ff"):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.05,rounding_size=0.25", fc=fc, ec="#334", lw=1.3))
        ax.text(x + w / 2, y + h - 0.35, t, ha="center", va="top", fontsize=10, weight="bold")
        if sub:
            ax.text(x + w / 2, y + h - 0.95, sub, ha="center", va="top", fontsize=8.2)

    def arr(a, c, t=""):
        ax.add_patch(FancyArrowPatch(a, c, arrowstyle="->", mutation_scale=14, lw=1.5, color="#445"))
        if t:
            ax.text((a[0] + c[0]) / 2, (a[1] + c[1]) / 2 + 0.15, t, fontsize=8, ha="center", color="#445")
    b(0.3, 12, 5, 2.4, "Phone app", "joystick, buttons\nWebSocket JSON", "#fff4d6")
    b(6.3, 12, 5, 2.4, "Mission Planner", "waypoints, arm, modes\nMAVLink over UDP", "#fff4d6")
    b(0.3, 6.2, 11, 4.8, "Raspberry Pi 5 - 'brain' (20 Hz)",
      "modes: MANUAL / LINE / EXPLORE / MISSION / RTL\nperception: line follower, sonar grid, step locator\n"
      "navigation: A* grid + A* road graph, pure pursuit, HALT\nGPS + odometry pose, actions: jump/step/dodge")
    b(0.3, 0.4, 11, 4.6, "Teensy 4.1 - real-time (1000 Hz)",
      "IMU -> attitude (Mahony filter)\n5-bar kinematics -> virtual leg\nLQR balance + leg springs + roll + yaw\n"
      "jump state machine, safety; eyes / lights / buzzer")
    b(13, 10.4, 6, 2.0, "2 fisheye cameras", "CSI -> Pi", "#ffeef8")
    b(13, 7.8, 6, 2.0, "GPS + compass", "UART / I2C -> Pi", "#e8fbfb")
    b(13, 5.2, 6, 2.0, "IMU, sonars, IR cliff", "SPI / GPIO / ADC -> Teensy", "#efffe9")
    b(21, 2.6, 6.5, 2.2, "4 hip motors DM-J4310", "CAN1 + CAN2", "#fff3e6")
    b(21, 0.2, 6.5, 2.2, "2 wheel motors MF9025", "CAN3", "#fff3e6")
    arr((2.8, 12), (2.8, 11.0), "commands"); arr((8.8, 12), (8.8, 11.0), "missions")
    arr((4, 6.2), (4, 5.0), "Command 50 Hz"); arr((8, 5.0), (8, 6.2), "Telemetry 100 Hz")
    arr((13, 11.4), (11.3, 10.4)); arr((13, 8.8), (11.3, 8.8)); arr((13, 6.0), (11.3, 4.6))
    arr((11.3, 3.7), (21, 3.7), "torque commands 1 kHz"); arr((11.3, 1.3), (21, 1.3), "torque commands 1 kHz")
    arr((21, 3.0), (11.3, 3.0), "angle / speed feedback")
    ax.set_title("How BOLT's pieces talk to each other", fontsize=13, weight="bold")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "architecture.png"))
    plt.close(fig)


# ---------------------------------------------------------------- 7. jump phases (schematic)
def jump_phases():
    fig, ax = plt.subplots(figsize=(13, 3.6), dpi=110)
    ax.set_xlim(0, 13); ax.set_ylim(-0.3, 3.2); ax.axis("off")
    phases = [("BALANCE", 1.0, 1.4), ("CROUCH", 1.0, 0.8), ("THRUST", 1.0, 1.9), ("FLIGHT\n(tuck)", 1.9, 0.8),
              ("FLIGHT\n(apex)", 2.3, 0.8), ("LAND", 1.0, 0.95), ("BALANCE", 1.0, 1.4)]
    x = 0.8
    for name, hip, leg in phases:
        wheel_y = hip - leg
        ax.add_patch(Rectangle((x - 0.35, hip), 0.7, 0.5, fc="#ffd23f", ec="#c79a00"))
        ax.plot([x, x], [hip, wheel_y + 0.2], color="#5c5c8a", lw=4)
        ax.add_patch(Circle((x, wheel_y + 0.2), 0.2, fc="#222"))
        ax.text(x, -0.25, name, ha="center", fontsize=9)
        x += 1.8
    ax.axhline(0.0, color="#777")
    ax.set_title("Jump state machine: crouch -> push hard (thrust) -> tuck the wheels up -> fall onto the step -> absorb -> balance",
                 fontsize=10.5)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "jump_phases.png"))
    plt.close(fig)


if __name__ == "__main__":
    for f in (fivebar, leg_force, pendulum, gains, motors, architecture, jump_phases):
        f()
    print("figures ->", OUT)
