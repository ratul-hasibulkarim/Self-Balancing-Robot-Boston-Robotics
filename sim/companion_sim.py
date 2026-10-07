#!/usr/bin/env python3
"""
Software-in-the-loop: the real companion brain (companion/robot_brain) driving the
MuJoCo robot, which runs the real firmware controller.

  sim robot  <- link.Command --  Brain (behaviors.py)  <- app / MAVLink
             -- Telemetry, rendered camera, virtual GPS, sonar, cliff sensors -->

    python3 sim/companion_sim.py --demo line        # vision line following (+ dark run)
    python3 sim/companion_sim.py --demo obstacles   # A* detours + HALT when boxed in
    python3 sim/companion_sim.py --demo mission     # Mission Planner (MAVLink) GPS mission over roads
    python3 sim/companion_sim.py --demo app         # serve the phone app against the sim robot
"""
import argparse
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "companion"))
os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco  # noqa: E402

from simulator import BoltSim, quat_to_rpy  # noqa: E402
from robot_brain.behaviors import Brain, RobotIO, Settings  # noqa: E402
from robot_brain.gps import Fix, LocalFrame  # noqa: E402
from robot_brain.link import Command, Telemetry  # noqa: E402

OUT = os.path.join(HERE, "results")
LAT0, LON0 = 23.7275, 90.3889          # simulated origin (any place on Earth works)


class SimRobotIO(RobotIO):
    def __init__(self, sim: BoltSim, cam_size=(320, 240), gps_noise=0.3, seed=0):
        self.sim = sim
        self.frame_ll = LocalFrame(LAT0, LON0)
        self.rng = np.random.default_rng(seed)
        self.gps_noise = gps_noise
        self.cam_renderer = mujoco.Renderer(sim.m, cam_size[1], cam_size[0])
        self.vopt = mujoco.MjvOption()
        self.vopt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
        self._frame_t = -1
        self._frame = None
        self.yaw_unwrapped, self._yaw_prev = 0.0, None
        self.last_eyes = (0, 0, 0)
        self.gps_bias = np.zeros(2)

    def now(self):
        return self.sim.t

    def telemetry(self):
        s, o = self.sim.state(), self.sim.ctrl.out
        if self._yaw_prev is None:
            self._yaw_prev = s["yaw"]
        d = (s["yaw"] - self._yaw_prev + math.pi) % (2 * math.pi) - math.pi
        self.yaw_unwrapped += d
        self._yaw_prev = s["yaw"]
        son = self.sim.sonar()
        flags = (1 if o.contact[0] else 0) | (2 if o.contact[1] else 0) | 64
        return Telemetry(t_ms=int(self.sim.t * 1000), state=o.state, flags=flags, roll=s["roll"], pitch=s["pitch"],
                         yaw=s["yaw"], gz=float(self.sim.d.sensordata[2]), v=o.ds, s=o.s, L=(o.L[0], o.L[1]),
                         theta=o.theta[0], battery_v=24.0,
                         sonar=tuple(0.0 if son[k] >= 3.99 else son[k] for k in ("us_fl", "us_fc", "us_fr", "us_r")),
                         cliff=tuple(self.sim.cliff()), yaw_odo=self.yaw_unwrapped)

    def send(self, cmd: Command):
        c = self.sim.cmd
        c.mode, c.estop, c.jump_seq = cmd.mode, int(cmd.estop), cmd.jump_seq
        c.v, c.yaw_rate, c.height, c.roll, c.pitch = cmd.v, cmd.yaw_rate, cmd.height, cmd.roll, cmd.pitch
        c.jump_height, c.jump_mode, c.jump_dist = cmd.jump_height, cmd.jump_mode, cmd.jump_dist

    def eyes(self, expression=0, headlight=0, ir=0, rgb=(60, 230, 255)):
        self.last_eyes = (expression, headlight, ir)
        lid = self.sim.m.light("headlight").id
        k = 3.0 * headlight / 255.0          # 2 x 1 W LEDs ~ a bright spot light
        self.sim.m.light_diffuse[lid] = [k, k, 0.9 * k]

    def frame(self, cam="front"):
        if self.sim.t - self._frame_t >= 0.099:         # 10 fps camera
            self.cam_renderer.update_scene(self.sim.d, camera="cam_front" if cam == "front" else "cam_rear",
                                           scene_option=self.vopt)
            rgb = self.cam_renderer.render()
            self._frame = np.ascontiguousarray(rgb[:, :, ::-1])   # -> BGR like OpenCV
            self._frame_t = self.sim.t
        return self._frame

    def gps(self):
        s = self.sim.state()
        self.gps_bias = 0.995 * self.gps_bias + self.rng.normal(0, self.gps_noise * 0.1, 2)   # slow wander
        lat, lon = self.frame_ll.to_ll(s["x"] + self.gps_bias[0], s["y"] + self.gps_bias[1])
        vx = self.sim.d.qvel[0]
        vy = self.sim.d.qvel[1]
        spd = math.hypot(vx, vy)
        course = (90 - math.degrees(math.atan2(vy, vx))) % 360
        return Fix(lat, lon, 10.0, 1, 14, 0.7, spd, course, self.sim.t)


def run_brain(sim, brain, duration, record=None, every=0.1, on_tick=None):
    """Advance the sim; brain ticks at 20 Hz. Returns trajectory list."""
    traj = []
    n_tick = int(round(Brain.TICK / sim.CTRL_DT))
    steps = int(round(duration / sim.CTRL_DT))
    for i in range(steps):
        if i % n_tick == 0:
            brain.step()
            if on_tick:
                on_tick(sim, brain)
            s = sim.state()
            traj.append((sim.t, s["x"], s["y"], s["yaw"], brain.mode, brain.status, brain.nav.state))
        sim.step()
        if record is not None and sim.renderer is not None and i % int(round(every / sim.CTRL_DT)) == 0:
            record.append(sim.render_frame())
        if sim.fallen():
            print("robot fell!")
            break
    return traj


# ====================================================================== demo: line following
def make_line_texture(path, size_m=12.0, px=2400, color=(220, 30, 30), dark=False):
    import cv2
    img = np.full((px, px, 3), 60 if dark else 205, np.uint8)
    k = px / size_m
    def P(x, y):  # world (x, y) -> pixel (col, row); texture row 0 is +y edge
        return int((x + size_m / 2) * k), int((size_m / 2 - y) * k)
    # closed track: straight, S-bend, big curves
    pts = []
    for t in np.linspace(0, 2 * math.pi, 600):
        x = 3.6 * math.cos(t)
        y = 2.2 * math.sin(t) + 0.6 * math.sin(3 * t) * (1 if math.cos(t) > 0 else 0.3)
        pts.append(P(x, y))            # starts at the right-most point (3.6, 0), runs counter-clockwise
    cv2.polylines(img, [np.array(pts, np.int32)], True, color[::-1], thickness=int(0.05 * k), lineType=cv2.LINE_AA)
    cv2.imwrite(path, img)
    world = [((x / k) - size_m / 2, size_m / 2 - (y / k)) for x, y in pts]
    return world


def demo_line(dark=False, duration=40.0, gif=True):
    os.makedirs(OUT, exist_ok=True)
    tex = os.path.join(HERE, "build", f"line_{'dark' if dark else 'day'}.png")
    track = make_line_texture(tex, dark=dark)
    sim = BoltSim("V3", render=gif, width=480, height=320, terrain={"kind": "flat"}, dark=dark)
    # replace floor material by the line texture on a 12 x 12 m patch
    xml_patch = None
    del xml_patch
    sim = _with_line_patch(sim, tex, dark, gif)
    io = SimRobotIO(sim)
    # start on the track, heading along it (track starts at the right-most point going "up")
    _place(sim, track[0][0], track[0][1], math.pi / 2)
    brain = Brain(io, Settings(line_speed=0.7, auto_lights=True))
    brain.set_mode("LINE", color="red")
    frames = [] if gif else None
    if gif:
        sim.cam.distance, sim.cam.elevation, sim.cam.azimuth = 2.2, -35, 200
    traj = run_brain(sim, brain, duration, frames, every=0.1)
    tr = np.array(track)
    err = [np.min(np.hypot(tr[:, 0] - x, tr[:, 1] - y)) for _, x, y, *_ in traj]
    dist = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(traj, traj[1:]))
    res = dict(dark=dark, duration=duration, distance_m=round(dist, 1), mean_err_m=round(float(np.mean(err)), 3),
               max_err_m=round(float(np.max(err)), 3), final_mode=brain.mode, status=brain.status,
               headlight_on=io.last_eyes[1] > 0)
    _plot_xy(os.path.join(OUT, f"line_{'dark' if dark else 'day'}.png"), traj, track=track,
             title=f"Line following ({'dark, headlight' if dark else 'daylight'}) - mean error {res['mean_err_m']*100:.1f} cm")
    if gif and frames:
        from simulator import save_gif
        save_gif(frames[::2], os.path.join(OUT, f"line_{'dark' if dark else 'day'}.gif"), fps=10, scale=0.7)
        cam = io.frame("front")
        import cv2
        cv2.imwrite(os.path.join(OUT, f"line_cam_{'dark' if dark else 'day'}.png"), cam)
    return res


def _with_line_patch(sim, tex, dark, render):
    """Rebuild the sim with a textured 12 x 12 m floor patch."""
    import robot_model
    xml, meta = robot_model.build_mjcf("V3", terrain={"kind": "flat"}, visuals=render, dark=dark,
                                       floor_texture=tex)
    xml = xml.replace('<body name="base"', '<geom name="linepatch" type="plane" size="6 6 0.1" pos="0 0 0.0005" '
                      'material="linefloor" contype="0" conaffinity="0"/>\n    <body name="base"', 1)
    new = BoltSim.__new__(BoltSim)
    new.__dict__.update(sim.__dict__)
    new.m = mujoco.MjModel.from_xml_string(xml)
    new.d = mujoco.MjData(new.m)
    new.meta = meta
    if render:
        new.renderer = mujoco.Renderer(new.m, sim.renderer.height, sim.renderer.width) if sim.renderer else None
    j = new.m.joint
    new.qadr = {n: new.m.jnt_qposadr[j(n).id] for n in sim.qadr}
    new.vadr = {n: new.m.jnt_dofadr[j(n).id] for n in sim.vadr}
    new.base = new.m.body("base").id
    new.us_ids = {nm: [new.m.sensor(f"{nm}_{k}").id for k in range(3)] for nm in ("us_fc", "us_fl", "us_fr", "us_r")}
    new.log, new.frames, new.t, new.prev_out = [], [], 0.0, None
    import controller as ctl
    new.ctrl = ctl.Controller(new.cfg)
    new.cmd = new.ctrl.cmd
    new.cmd.mode, new.cmd.height = 1, 0.22
    mujoco.mj_forward(new.m, new.d)
    return new


def _place(sim, x, y, yaw):
    sim.d.qpos[0], sim.d.qpos[1] = x, y
    sim.d.qpos[3:7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    mujoco.mj_forward(sim.m, sim.d)


def _plot_xy(path, traj, track=None, obstacles=(), roads=None, waypoints=None, title=""):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig, ax = plt.subplots(figsize=(7, 6), dpi=110)
    if roads:
        for a, b in roads:
            ax.plot([a[0], b[0]], [a[1], b[1]], color="#bbb", lw=7, solid_capstyle="round", zorder=0)
    if track is not None:
        tr = np.array(track)
        ax.plot(tr[:, 0], tr[:, 1], color="#d62728", lw=2, label="line")
    for (x, y, sx, sy, _) in obstacles:
        ax.add_patch(Rectangle((x - sx, y - sy), 2 * sx, 2 * sy, color="#555"))
    t = np.array([(p[1], p[2]) for p in traj])
    ax.plot(t[:, 0], t[:, 1], color="#1f77b4", lw=1.6, label="robot")
    ax.plot(t[0, 0], t[0, 1], "go", label="start")
    ax.plot(t[-1, 0], t[-1, 1], "ks", label="end")
    if waypoints:
        w = np.array(waypoints)
        ax.plot(w[:, 0], w[:, 1], "r*", ms=14, label="waypoints")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("x / east [m]")
    ax.set_ylabel("y / north [m]")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


# ====================================================================== demo: obstacles + halt
def demo_obstacles(duration=45.0):
    os.makedirs(OUT, exist_ok=True)
    results = {}
    scenes = {
        # goal 8 m ahead behind a wall with a gap at the side -> must detour
        "detour": [(3.0, 0.0, 0.1, 1.2, 0.4), (5.5, 1.6, 0.6, 0.1, 0.4), (6.0, -1.5, 0.3, 0.3, 0.4)],
        # goal inside a closed box -> no path -> must HALT and report
        "boxed_in": [(3.0, 0.0, 0.1, 2.5, 0.4), (-1.5, 0.0, 0.1, 2.5, 0.4), (0.75, 2.5, 2.35, 0.1, 0.4),
                     (0.75, -2.5, 2.35, 0.1, 0.4)],
    }
    for name, obs in scenes.items():
        sim = BoltSim("V3", terrain={"kind": "flat", "obstacles": obs})
        io = SimRobotIO(sim)
        brain = Brain(io, Settings(auto_lights=False))
        brain.set_mode("GUIDED")
        goal_xy = (8.0, 0.0)
        # GUIDED target without MAVLink: put it straight into the navigator via a tiny shim
        brain.mav = type("M", (), {"guided_target": io.frame_ll.to_ll(*goal_xy), "params": {}, "mode": "GUIDED",
                                   "statustext": lambda *a, **k: None, "current": 0})()
        traj = run_brain(sim, brain, duration)
        s = sim.state()
        reached = math.hypot(s["x"] - goal_xy[0], s["y"] - goal_xy[1]) < 1.2
        halted = brain.status.startswith("HALT")
        results[name] = dict(reached_goal=reached, halted=halted, status=brain.status, final_mode=brain.mode,
                             time_s=round(traj[-1][0], 1), x=round(s["x"], 2), y=round(s["y"], 2))
        _plot_xy(os.path.join(OUT, f"obstacles_{name}.png"), traj, obstacles=obs, waypoints=[goal_xy],
                 title=f"{name}: {'reached goal' if reached else ''} {'HALT - ' + brain.status if halted else ''}")
    return results


# ====================================================================== demo: Mission Planner mission over roads
def make_road_grid(frame, blocks=3, spacing=12.0):
    """A small street grid around the origin as GeoJSON (what you'd export from OSM)."""
    feats, segs = [], []
    n = blocks + 1
    for i in range(n):
        for j in range(n):
            x, y = i * spacing - 6, j * spacing - 6
            if i < n - 1:
                segs.append(((x, y), (x + spacing, y)))
            if j < n - 1:
                segs.append(((x, y), (x, y + spacing)))
    for a, b in segs:
        la, lo = frame.to_ll(*a)
        lb, lob = frame.to_ll(*b)
        feats.append({"type": "Feature", "properties": {"highway": "residential"},
                      "geometry": {"type": "LineString", "coordinates": [[lo, la], [lob, lb]]}})
    return {"type": "FeatureCollection", "features": feats}, segs


def demo_mission(duration=240.0, port=14571):
    """Mission Planner is emulated by a pymavlink GCS that uploads a waypoint mission over UDP,
    then starts it.  One street is blocked by a parked car -> the robot re-routes."""
    from pymavlink import mavutil
    from robot_brain.mission.mavlink_vehicle import MavlinkVehicle
    from robot_brain.navigation.planner import RoadGraph
    os.makedirs(OUT, exist_ok=True)
    frame = LocalFrame(LAT0, LON0)
    gj, segs = make_road_grid(frame)
    # a broken-down van blocking the street x = 18 between y = 6 and 18 (on the shortest route)
    obstacles = [(18.0, 12.0, 1.45, 1.2, 0.6)]
    # buildings fill the blocks (keeps the robot on the streets)
    for i in range(3):
        for j in range(3):
            cx, cy = -6 + 12 * i + 6, -6 + 12 * j + 6
            obstacles.append((cx, cy, 4.4, 4.4, 1.0))
    sim = BoltSim("V3", terrain={"kind": "flat", "obstacles": obstacles})
    _place(sim, -6.0, -6.0, 0.0)
    io = SimRobotIO(sim, gps_noise=0.3)
    roads = RoadGraph.from_geojson(gj)
    mav = MavlinkVehicle(f"udpout:127.0.0.1:{port}")
    brain = Brain(io, Settings(cruise_speed=1.5, auto_lights=False), roads, mav)
    brain.set_mode("HOLD")          # robot already standing (operator pressed "Stand" in the app)
    # ---- the "Mission Planner" side
    gcs = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}", source_system=255, source_component=190)
    wps_xy = [(18.0, -6.0), (18.0, 30.0), (-6.0, 30.0)]        # far corners: forces multi-segment routes
    home = frame.to_ll(-6.0, -6.0)
    items = [home] + [frame.to_ll(*w) for w in wps_xy]
    log = []

    def gcs_service(sim_, brain_):
        """Run the GCS protocol steps as messages arrive (non-blocking)."""
        while True:
            m = gcs.recv_match(blocking=False)
            if m is None:
                break
            t = m.get_type()
            if t == "HEARTBEAT" and not gcs_state["hb"]:
                gcs_state["hb"] = True
                gcs.target_system, gcs.target_component = m.get_srcSystem(), m.get_srcComponent()
                gcs.mav.mission_count_send(gcs.target_system, gcs.target_component, len(items))
                log.append("GCS: vehicle found, uploading mission")
            elif t in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
                lat, lon = items[m.seq]
                gcs.mav.mission_item_int_send(gcs.target_system, gcs.target_component, m.seq,
                                              mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                              mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0,
                                              int(lat * 1e7), int(lon * 1e7), 0)
            elif t == "MISSION_ACK" and not gcs_state["started"]:
                log.append("GCS: mission accepted -> ARM + MISSION_START")
                gcs.mav.command_long_send(gcs.target_system, gcs.target_component,
                                          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
                gcs.mav.command_long_send(gcs.target_system, gcs.target_component,
                                          mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)
                gcs_state["started"] = True
            elif t == "STATUSTEXT":
                log.append("VEHICLE: " + m.text)
            elif t == "GLOBAL_POSITION_INT":
                gcs_state["pos"] = (m.lat / 1e7, m.lon / 1e7)
        # mirror state into the MAVLink endpoint (what main.py does)
        ll = brain_.ll()
        if ll:
            mav.lat, mav.lon = ll
        time.sleep(0.002)          # let the UDP threads run

    gcs_state = {"hb": False, "started": False, "pos": None}
    traj = run_brain(sim, brain, duration, on_tick=gcs_service)
    end = sim.state()
    done = brain.status.startswith("mission complete") or (brain.mode == "HOLD" and brain.mission_idx >= len(brain.mission) > 0)
    res = dict(mission_items=len(items), completed=done, final_status=brain.status, time_s=round(traj[-1][0], 1),
               end_xy=(round(end["x"], 1), round(end["y"], 1)), gcs_saw_position=gcs_state["pos"] is not None,
               log=log[-8:])
    road_xy = [(a, b) for a, b in segs]
    _plot_xy(os.path.join(OUT, "mission_roads.png"), traj, obstacles=obstacles, roads=road_xy, waypoints=wps_xy,
             title="Mission Planner mission: A* over the street graph (blocked street -> re-route)")
    mav.close()
    return res


# ====================================================================== demo: app against the sim
def demo_app(port=8080, seconds=None):
    import asyncio
    import threading
    from robot_brain.app_server import AppServer
    sim = BoltSim("V3", terrain={"kind": "flat", "obstacles": [(3.0, 0.5, 0.2, 0.2, 0.3)]})
    io = SimRobotIO(sim)
    brain = Brain(io, Settings(auto_lights=False))
    brain.set_mode("MANUAL")
    lock = threading.Lock()

    def loop():
        n_tick = int(round(Brain.TICK / sim.CTRL_DT))
        while True:
            t0 = time.time()
            with lock:
                brain.step()
                for _ in range(n_tick):
                    sim.step()
                if sim.fallen():
                    brain.mode = "IDLE"
            time.sleep(max(0, Brain.TICK - (time.time() - t0)))
    threading.Thread(target=loop, daemon=True).start()
    server = AppServer(brain, io, port=port, lock=lock)
    aloop = asyncio.new_event_loop()
    asyncio.set_event_loop(aloop)
    aloop.run_until_complete(server.start())
    print(f"app on http://localhost:{port}/  (simulated robot)")
    if seconds:
        aloop.run_until_complete(asyncio.sleep(seconds))
    else:
        aloop.run_forever()
    return brain, sim


if __name__ == "__main__":
    import json
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", default="all", choices=["line", "obstacles", "mission", "app", "all"])
    ap.add_argument("--no-gif", action="store_true")
    a = ap.parse_args()
    results = {}
    if a.demo in ("line", "all"):
        results["line_day"] = demo_line(False, gif=not a.no_gif)
        results["line_dark"] = demo_line(True, duration=25, gif=not a.no_gif)
        print(json.dumps(results, indent=1))
    if a.demo in ("obstacles", "all"):
        results["obstacles"] = demo_obstacles()
        print(json.dumps(results["obstacles"], indent=1))
    if a.demo in ("mission", "all"):
        results["mission"] = demo_mission()
        print(json.dumps(results["mission"], indent=1))
    if a.demo == "app":
        demo_app()
    if a.demo == "all":
        json.dump(results, open(os.path.join(OUT, "companion_demos.json"), "w"), indent=1, default=str)
