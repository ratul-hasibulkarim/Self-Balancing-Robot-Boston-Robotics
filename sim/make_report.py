#!/usr/bin/env python3
"""
Builds the figures for docs/04_simulation_report.md from the test-suite results
and a few dedicated runs (renders, GIFs, time series).

    python3 sim/test_suite.py V1 V2 V3      # first
    python3 sim/make_report.py              # -> docs/images/*
"""
import json
import math
import os
import sys

os.environ.setdefault("MUJOCO_GL", "osmesa")
import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import controller as ctl  # noqa: E402
import robot_model  # noqa: E402
from simulator import BoltSim, save_gif  # noqa: E402

IMG = os.path.join(ROOT, "docs", "images")
os.makedirs(IMG, exist_ok=True)
RES = os.path.join(HERE, "results")
COL = {"V1": "#9aa0c3", "V2": "#5fa8d3", "V3": "#f4a300"}


def load():
    return {v: json.load(open(os.path.join(RES, f"{v}.json"))) for v in ("V1", "V2", "V3")
            if os.path.exists(os.path.join(RES, f"{v}.json"))}


# ------------------------------------------------------------------ comparison chart
def comparison(R):
    metrics = [
        ("Push recovery fwd [N s]", lambda r: r["push_recovery"]["max_impulse_Ns"]["fwd"]),
        ("Push recovery back [N s]", lambda r: r["push_recovery"]["max_impulse_Ns"]["back"]),
        ("Top speed held [m/s]", lambda r: r["top_speed"]["top_speed_ok"]),
        ("Max slope climb+hold [deg]", lambda r: r["incline"]["max_slope_deg"]),
        ("Bumps at 1.5 m/s [cm]", lambda r: 100 * r["rough_terrain"]["max_bump_amplitude_at_1p5ms"]),
        ("Max payload [kg]", lambda r: r["payload"]["max_payload_kg"]),
        ("Jump wheel clearance [cm]", lambda r: 100 * r["jump_flat"]["max_wheel_clearance_m"]),
        ("Step-up, 100 % success [cm]", lambda r: 100 * r["step_up"]["max_step_m"]),
        ("Lowest crawl bar [cm] (lower = better)", lambda r: 100 * r["crawl"]["lowest_bar_passed_m"]),
    ]
    fig, axs = plt.subplots(3, 3, figsize=(13, 9.5), dpi=110)
    for ax, (name, f) in zip(axs.flat, metrics):
        vs = [v for v in R]
        vals = []
        for v in vs:
            try:
                vals.append(float(f(R[v])))
            except Exception:
                vals.append(0.0)
        bars = ax.bar(vs, vals, color=[COL[v] for v in vs])
        for b, x in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, b.get_height(), f"{x:g}", ha="center", va="bottom", fontsize=9)
        ax.set_title(name, fontsize=10)
        ax.spines[["top", "right"]].set_visible(False)
        ax.set_ylim(0, max(vals + [1]) * 1.2)
    fig.suptitle("Design iterations in simulation  (V1 big-head -> V2 low-COM -> V3 final)", fontsize=13, weight="bold")
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "iterations_comparison.png"))
    plt.close(fig)


# ------------------------------------------------------------------ COM diagram
def com_diagram():
    fig, axs = plt.subplots(1, 3, figsize=(13, 5), dpi=110, sharey=True)
    for ax, var in zip(axs, ("V1", "V2", "V3")):
        xml, meta = robot_model.build_mjcf(var, visuals=False, cameras=False)
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        v = meta["variant"]
        hip_z = d.xpos[1][2]
        bs, bz = v["body_size"], v["body_center_z"]
        ax.add_patch(plt.Rectangle((-bs[0] / 2, hip_z + bz - bs[2] / 2), bs[0], bs[2], fc="#ffd23f55", ec="#c79a00"))
        for name, (mass, pos, size) in v["components"].items():
            ax.add_patch(plt.Rectangle((pos[0] - size[0] / 2, hip_z + pos[2] - size[2] / 2), size[0], size[2],
                                       fc="#5c5c8a55", ec="#5c5c8a"))
            ax.text(pos[0], hip_z + pos[2], name.replace("_", " ").split(" ")[0], fontsize=7, ha="center", va="center")
        pp = v["payload_pos"]
        ax.add_patch(plt.Rectangle((pp[0] - 0.05, hip_z + pp[2] - 0.04), 0.1, 0.08, fc="none", ec="#1e88e5", ls="--"))
        ax.text(pp[0], hip_z + pp[2] + 0.045, "payload", color="#1e88e5", fontsize=7, ha="center")
        # legs/wheel
        R = v["wheel_radius"]
        ax.add_patch(plt.Circle((0, R), R, fc="#222", ec="#000"))
        ax.plot([0, 0], [R, hip_z], color="#5c5c8a", lw=3)
        body_com = d.xipos[1]
        tot_com = d.subtree_com[1]
        ax.plot(body_com[0], body_com[2], "o", color="#c62828", ms=8, label="body COM")
        ax.plot(tot_com[0], tot_com[2], "X", color="#000", ms=10, label="whole-robot COM")
        ax.plot(0, hip_z, "s", color="#2e7d32", ms=7, label="hip axis")
        ax.axhline(0, color="#999")
        ax.set_aspect("equal")
        ax.set_xlim(-0.25, 0.25)
        ax.set_title(f"{v['name']}\nbody COM {100*(body_com[2]-hip_z):+.1f} cm vs hip, whole COM {100*tot_com[2]:.1f} cm high", fontsize=9.5)
        ax.grid(alpha=0.25)
    axs[0].legend(fontsize=8, loc="upper left")
    axs[0].set_ylabel("height above ground [m]")
    fig.suptitle("Centre-of-mass placement: battery slung under the hip axis, payload bay centred on it", fontsize=12, weight="bold")
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "com_iterations.png"))
    plt.close(fig)


# ------------------------------------------------------------------ time series
PUSH_N = 90


def ts_push_and_step():
    fig, axs = plt.subplots(2, 2, figsize=(13, 7.5), dpi=110)
    # push recovery
    sim = BoltSim("V3")
    sim.run(1.0)
    sim.push([PUSH_N, 0, 0], 0.1)
    sim.run(3.0)
    t = np.array([l["t"] for l in sim.log])
    axs[0, 0].plot(t, np.degrees([l["pitch"] for l in sim.log]), label="body pitch [deg]")
    axs[0, 0].plot(t, np.degrees([l["theta"] for l in sim.log]), label="leg angle [deg]")
    axs[0, 0].plot(t, [l["v"] * 10 for l in sim.log], label="speed x10 [m/s]")
    axs[0, 0].axvspan(1.0, 1.1, color="#ff4d5e33", label=f"{PUSH_N / 10:g} N s shove")
    axs[0, 0].set_title(f"Push recovery (V3): {PUSH_N} N for 0.1 s")
    axs[0, 0].legend(fontsize=8)
    # step up
    x0 = 1.8
    sim = BoltSim("V3", terrain={"kind": "steps", "x0": x0, "steps": [(0.16, 3.0)]})

    def pol(t_, s, c):
        x = s.state()["x"]
        c.v = 1.0 if t_ > 0.3 else 0
        if c.jump_seq == 0 and x > x0 - 0.9:
            c.jump_height, c.jump_seq, c.jump_mode, c.jump_dist = 0.16, 1, 0, x0 - x
        if x > x0 + 0.8:
            c.v = 0
    sim.run(4.0, policy=pol, log_every=5)
    L = sim.log
    t = np.array([l["t"] for l in L])
    ax = axs[0, 1]
    ax.plot(t, [l["z"] for l in L], label="hip height [m]")
    ax.plot(t, [l["wheel_low"] for l in L], label="wheel bottom [m]")
    ax.plot(t, [l["L"] for l in L], label="leg length [m]")
    ax.axhline(0.16, color="#999", ls="--", label="step top (16 cm)")
    st = [l["state"] for l in L]
    for i in range(1, len(st)):
        if st[i] != st[i - 1]:
            ax.axvline(t[i], color="#ccc", lw=0.8)
            ax.text(t[i], 0.62, ctl.STATES[st[i]], rotation=90, fontsize=7, va="top")
    ax.set_title("Jump onto a 16 cm step at 1 m/s (V3)")
    ax.legend(fontsize=8, loc="center left")
    ax.set_xlim(1.0, 3.5)
    # incline
    sim = BoltSim("V3", terrain={"kind": "incline", "angle_deg": 20, "x0": 0.8})
    sim.run(9.0, policy=lambda t_, s, c: setattr(c, "v", 0.6 if 0.3 < t_ < 6 else 0))
    t = np.array([l["t"] for l in sim.log])
    axs[1, 0].plot(t, [l["z"] for l in sim.log], label="hip height [m]")
    axs[1, 0].plot(t, [l["v"] for l in sim.log], label="speed [m/s]")
    axs[1, 0].plot(t, np.degrees([l["pitch"] for l in sim.log]) / 10, label="body pitch [deg/10]")
    axs[1, 0].plot(t, np.degrees([l["theta"] for l in sim.log]) / 10, label="leg angle [deg/10]")
    axs[1, 0].set_title("Climb a 20 deg ramp, stop and hold on it (V3)")
    axs[1, 0].legend(fontsize=8)
    # payload pitch disturbance
    ax = axs[1, 1]
    for m in (0.0, 1.0, 2.0, 3.0):
        sim = BoltSim("V3", payload=m)
        sim.run(5.0, policy=lambda t_, s, c: setattr(c, "v", 1.0 if 0.5 < t_ < 3 else 0))
        t = np.array([l["t"] for l in sim.log])
        ax.plot(t, np.degrees([l["theta"] for l in sim.log]), label=f"payload {m:g} kg")
    ax.set_title("Leg angle while accelerating/braking with payload (V3)")
    ax.set_ylabel("leg angle [deg]")
    ax.legend(fontsize=8)
    for a in axs.flat:
        a.set_xlabel("time [s]")
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "timeseries.png"))
    plt.close(fig)


# ------------------------------------------------------------------ renders and GIFs
def renders():
    sim = BoltSim("V3", render=True, width=900, height=720)
    sim.run(0.6)
    from PIL import Image
    for name, az, el, dist in (("hero", 150, -12, 1.25), ("front", 180, -4, 1.15), ("side", 90, -4, 1.15),
                               ("rear_top", -40, -35, 1.25)):
        sim.cam.azimuth, sim.cam.elevation, sim.cam.distance = az, el, dist
        Image.fromarray(sim.render_frame()).save(os.path.join(IMG, f"render_{name}.png"))
    # poses: crawl, tall, tilt, lean
    tiles = []
    for name, cmd in (("crawl", dict(mode=2)), ("tall", dict(height=0.32)), ("bow (pitch +23 deg)", dict(pitch=0.4)),
                      ("lean (roll 14 deg)", dict(roll=0.25))):
        s = BoltSim("V3", render=True, width=420, height=420)

        def pol(t, s_, c, cmd=cmd):
            if t > 0.3:
                for k, v in cmd.items():
                    setattr(c, k, v)
        s.run(3.0, policy=pol)
        s.cam.azimuth, s.cam.elevation, s.cam.distance = (150 if "roll" not in name else 180), -8, 1.3
        img = s.render_frame()
        tiles.append((name, img))
        s.renderer.close()          # several OSMesa contexts in one process can corrupt frames
    fig, axs = plt.subplots(1, 4, figsize=(14, 4), dpi=100)
    for ax, (n, im) in zip(axs, tiles):
        ax.imshow(im)
        ax.set_title(n)
        ax.axis("off")
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "postures.png"))
    plt.close(fig)


def gif(name, sim, duration, policy, cam=(150, -10, 1.8), track=True, fps=20):
    sim.cam.azimuth, sim.cam.elevation, sim.cam.distance = cam
    sim.cam_track = track
    sim.frames = []
    sim.run(duration, policy=policy, fps=fps, stop_on_fall=False)
    save_gif(sim.frames, os.path.join(IMG, f"{name}.gif"), fps=fps, scale=0.5)
    sim.renderer.close()


def gifs():
    x0 = 1.8
    sim = BoltSim("V3", render=True, width=640, height=420, terrain={"kind": "steps", "x0": x0, "steps": [(0.16, 3.0)]})

    def pol(t, s, c):
        x = s.state()["x"]
        c.v = 1.0 if t > 0.3 else 0
        if c.jump_seq == 0 and x > x0 - 0.9:
            c.jump_height, c.jump_seq, c.jump_mode, c.jump_dist = 0.16, 1, 0, x0 - x
        if x > x0 + 0.8:
            c.v = 0
    gif("jump_step_16cm", sim, 4.5, pol, cam=(120, -8, 2.2))
    sim = BoltSim("V3", render=True, width=640, height=420, terrain={"kind": "rough", "amplitude": 0.08, "seed": 3})
    gif("rough_terrain", sim, 5.0, lambda t, s, c: setattr(c, "v", 1.5 if t > 0.3 else 0), cam=(130, -12, 2.0))
    sim = BoltSim("V3", render=True, width=640, height=420, terrain={"kind": "flat", "obstacles": [(1.8, 0, 0.05, 1.0, 0.07)]})

    def hop(t, s, c):
        x = s.state()["x"]
        c.v = 1.0 if t > 0.3 else 0
        if c.jump_seq == 0 and x > 1.0:
            c.jump_height, c.jump_seq, c.jump_mode, c.jump_dist = 0.14, 1, 1, 1.75 - x
        if x > 2.8:
            c.v = 0
    gif("hop_obstacle", sim, 4.0, hop, cam=(100, -8, 2.2))
    sim = BoltSim("V3", render=True, width=640, height=420, terrain={"kind": "obstacle_bar", "x": 1.5, "clearance": 0.42})

    def crawl(t, s, c):
        c.mode = 2 if t > 0.3 else 1
        c.v = 0.45 if t > 1.2 else 0
    gif("crawl_under_bar", sim, 6.0, crawl, cam=(100, -6, 2.0))


if __name__ == "__main__":
    what = sys.argv[1:] or ["comparison", "com", "ts", "renders", "gifs"]
    R = load()
    if "comparison" in what and R:
        comparison(R)
    if "com" in what:
        com_diagram()
    if "ts" in what:
        ts_push_and_step()
    if "renders" in what:
        renders()
    if "gifs" in what:
        gifs()
    print("figures in", IMG)
