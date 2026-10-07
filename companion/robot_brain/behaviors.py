"""
BOLT behaviour manager ("the brain") - runs on the Raspberry Pi at 20 Hz.

Inputs  : telemetry from the Teensy, cameras, GPS, operator (app / Mission Planner)
Output  : one link.Command per tick (+ eye / light requests)

Modes
  IDLE      motors off, sitting
  MANUAL    drive from the app (joystick), actions on demand
  LINE      follow a coloured line with the front camera (stops for obstacles, halts if blocked / line lost)
  EXPLORE   wander, avoid obstacles
  MISSION   execute the Mission Planner / app mission (GPS waypoints, shortest routes, actions)
  GUIDED    go to one point ("Fly to here")
  RTL       go back home
  HOLD      stand still, balancing

Everything that talks to hardware goes through a RobotIO object (real: io_real.py,
simulated: sim/companion_sim.py) so the same brain runs on the robot and in MuJoCo.
"""
import math
import time
from dataclasses import dataclass, field

from .gps import Fix, LocalFrame, haversine
from .link import Command, Telemetry
from .navigation.planner import LocalNavigator, OccupancyGrid, RoadGraph, pure_pursuit
from .perception.line_follower import LineFollower
from .perception.obstacles import SONAR_ANGLES, SONAR_ORDER, ObstacleTracker
from .perception.step_detector import StepDetector

EXPR = {"normal": 0, "happy": 1, "blink": 2, "angry": 3, "sleepy": 4, "alert": 5, "lost": 6, "love": 7}


class RobotIO:
    """Interface the brain needs. Implemented by io_real.RealRobotIO and the simulator."""
    def now(self) -> float: return time.time()
    def telemetry(self) -> Telemetry: raise NotImplementedError
    def send(self, cmd: Command): raise NotImplementedError
    def eyes(self, expression=0, headlight=0, ir=0, rgb=(60, 230, 255)): pass
    def frame(self, cam="front"): return None          # BGR numpy image or None
    def gps(self) -> Fix: return Fix()


@dataclass
class Pose:
    x: float = 0.0           # east  [m] in the local frame
    y: float = 0.0           # north [m]
    yaw: float = 0.0         # rad, CCW from east (ENU)
    yaw_offset: float = 0.0  # ENU yaw = IMU yaw + offset
    s_prev: float = None
    gps_ok: bool = False


@dataclass
class Settings:
    cruise_speed: float = 1.2
    line_speed: float = 0.8
    line_color: str = "red"
    height: float = 0.22
    crawl_height: float = 0.135
    wp_radius: float = 1.0
    auto_lights: bool = True
    step_tread_default: float = 0.6


class Brain:
    TICK = 0.05

    def __init__(self, io: RobotIO, settings: Settings = None, road_graph: RoadGraph = None, mav=None):
        self.io = io
        self.cfg = settings or Settings()
        self.roads = road_graph
        self.mav = mav
        self.mode = "IDLE"
        self.cmd = Command(mode=0, height=self.cfg.height)
        self.pose = Pose()
        self.frame0: LocalFrame = None
        self.home_ll = None
        self.obs = ObstacleTracker()
        self.view = None
        self.grid = OccupancyGrid()
        self.nav = LocalNavigator(self.grid)
        self.line = LineFollower(self.cfg.line_color)
        self.step_det = StepDetector()
        self.drive_v = 0.0
        self.drive_w = 0.0
        self.last_drive_t = -1e9
        self.action = None            # active one-shot behaviour (generator)
        self.action_name = ""
        self.status = "ready"
        self.alerts = []
        self.route_ll = []            # current global route (lat, lon)
        self.route_xy = []
        self.mission = []             # list of dicts {command, lat, lon, params}
        self.mission_idx = 0
        self.mission_wait_until = None
        self.jump_counts = {}
        self.line_lost_t = None
        self.blocked_since = None
        self.crawl = False
        self.lights = (0, 0)
        self._lights_on = False
        self.expression = "normal"
        self.t = 0.0
        if mav is not None:
            mav.on_mode = self._mav_mode
            mav.on_arm = lambda armed: (self.mode == "IDLE" or not armed) and self.set_mode("HOLD" if armed else "IDLE")
            mav.on_mission_changed = self.load_mav_mission

    # ================================================================ operator API (app / GCS)
    def set_mode(self, mode, **kw):
        mode = mode.upper()
        self.action = None
        self.nav.reset()
        self.blocked_since = None
        self.line_lost_t = None
        if mode == "LINE" and "color" in kw:
            self.cfg.line_color = kw["color"]
            self.line.set_color(kw["color"])
        if mode == "MISSION":
            self.mission_idx = kw.get("start", self.mission_idx if self.mission_idx < len(self.mission) else 0)
            self.route_ll = []
        if mode == "RTL":
            self.route_ll = []
        self.mode = mode
        self.status = f"mode {mode}"
        if self.mav and mode in ("MISSION", "GUIDED", "RTL", "HOLD", "MANUAL"):
            self.mav.mode = {"MISSION": "AUTO"}.get(mode, mode)
        return True

    def _mav_mode(self, name):
        self.set_mode({"AUTO": "MISSION"}.get(name, name))

    def drive(self, v, w):
        self.drive_v, self.drive_w, self.last_drive_t = v, w, self.t

    def set_posture(self, height=None, pitch=None, roll=None, crawl=None):
        if height is not None:
            self.cfg.height = height
        if pitch is not None:
            self.cmd.pitch = pitch
        if roll is not None:
            self.cmd.roll = roll
        if crawl is not None:
            self.crawl = crawl

    def estop(self):
        self.cmd.estop = True
        self.mode = "IDLE"
        self.action = None
        self.status = "E-STOP"

    def release_estop(self):
        self.cmd.estop = False
        self.status = "e-stop released"

    def start_action(self, name, **kw):
        """jump | hop | step_up | stairs | dodge_left | dodge_right | sit | stand | calibrate"""
        gens = {"jump": self._act_jump, "hop": self._act_hop, "step_up": self._act_step_up,
                "stairs": self._act_stairs, "dodge_left": lambda **k: self._act_dodge(+1),
                "dodge_right": lambda **k: self._act_dodge(-1), "sit": self._act_sit, "stand": self._act_stand}
        if name not in gens:
            return False
        if name not in ("sit", "stand") and self.mode == "IDLE":
            return False
        self.action = gens[name](**kw)
        self.action_name = name
        self.status = f"action {name}"
        return True

    def set_mission(self, waypoints):
        """App mission: [{'lat':..,'lon':..}, {'action':'jump','height':0.1}, ...]"""
        self.mission = []
        for w in waypoints:
            if "lat" in w:
                self.mission.append(dict(command=16, lat=w["lat"], lon=w["lon"], params=(w.get("hold", 0), 0, 0, 0)))
            elif w.get("action") == "jump":
                self.mission.append(dict(command=31010, params=(w.get("height", 0.12), w.get("mode", 1), 0, 0)))
        self.mission_idx = 0

    def load_mav_mission(self, items):
        # item 0 is "home" in ArduPilot missions
        self.mission = [dict(command=i.command, lat=i.lat, lon=i.lon, params=i.params, seq=i.seq) for i in items[1:]]
        if items:
            self.home_ll = (items[0].lat, items[0].lon) if items[0].lat else self.home_ll
        self.mission_idx = 0

    # ================================================================ helpers
    def _alert(self, text, level="warn"):
        self.alerts.append(dict(t=self.t, text=text, level=level))
        self.alerts = self.alerts[-20:]
        self.status = text
        if self.mav:
            self.mav.statustext(f"BOLT: {text}")

    def _update_pose(self, tel: Telemetry, fix: Fix):
        p = self.pose
        if p.s_prev is None:
            p.s_prev = tel.s
        ds = tel.s - p.s_prev
        p.s_prev = tel.s
        p.yaw = tel.yaw_odo + p.yaw_offset
        p.x += ds * math.cos(p.yaw)
        p.y += ds * math.sin(p.yaw)
        if fix is not None and fix.valid:
            if self.frame0 is None:
                self.frame0 = LocalFrame(fix.lat, fix.lon)
                self.home_ll = self.home_ll or (fix.lat, fix.lon)
                gx, gy = 0.0, 0.0
                p.x, p.y = 0.0, 0.0
            gx, gy = self.frame0.to_xy(fix.lat, fix.lon)
            k = 0.05 if fix.hdop < 2 else 0.02        # complementary filter: odometry short-term, GPS long-term
            p.x += k * (gx - p.x)
            p.y += k * (gy - p.y)
            # heading from GPS course while driving straight and fast enough
            if fix.speed > 0.6 and abs(tel.gz) < 0.15:
                gps_yaw = math.pi / 2 - math.radians(fix.course)
                err = (gps_yaw - p.yaw + math.pi) % (2 * math.pi) - math.pi
                p.yaw_offset += 0.05 * err
            p.gps_ok = True

    def ll(self):
        if self.frame0 is None:
            return None
        return self.frame0.to_ll(self.pose.x, self.pose.y)

    def _pose_tuple(self):
        return (self.pose.x, self.pose.y, self.pose.yaw)

    # ================================================================ main tick
    def step(self):
        self.t = self.io.now()
        tel = self.io.telemetry()
        fix = self.io.gps()
        self._update_pose(tel, fix)
        self.view = self.obs.update(tel.sonar, tel.cliff, 0.5 * (tel.L[0] + tel.L[1]) or self.cfg.height)
        free = [(SONAR_ANGLES[k], self.view.ranges[k]) for k in SONAR_ORDER]
        self.grid.recenter(self.pose.x, self.pose.y)
        self.grid.integrate((self.pose.x, self.pose.y), self.pose.yaw, ObstacleTracker.points(self.view), free)

        c = self.cmd
        c.mode = 0 if self.mode == "IDLE" else (2 if self.crawl else 1)
        c.height = self.cfg.crawl_height if self.crawl else self.cfg.height
        v, w = 0.0, 0.0

        if self.action is not None:
            try:
                v, w = next(self.action)
            except StopIteration:
                self.action = None
                self.status = f"{self.action_name} done"
        elif self.mode == "MANUAL":
            if self.t - self.last_drive_t < 0.5:          # app keep-alive
                v, w = self.drive_v, self.drive_w
        elif self.mode == "LINE":
            v, w = self._line_follow()
        elif self.mode == "EXPLORE":
            v, w = self._explore()
        elif self.mode in ("MISSION", "GUIDED", "RTL"):
            v, w = self._mission()

        # ---- reflexes (always on, except mid-jump)
        if self.action_name not in ("jump", "hop", "step_up", "stairs") or self.action is None:
            if self.view.cliff and v > 0:
                v = -0.2
                self._alert("drop-off ahead! backing up")
            if self.view.front_min < 0.30 and v > 0:
                v = 0.0
            if self.view.rear < 0.25 and v < 0:
                v = 0.0
        if tel.battery_low:
            v = max(-0.5, min(0.5, v))

        c.v, c.yaw_rate = v, w
        self._lights(tel)
        self.io.send(c)
        return c

    # ================================================================ modes
    def _lights(self, tel):
        expr = self.expression
        if self.mode == "IDLE":
            expr = "sleepy"
        elif self.status.startswith("HALT") or "blocked" in self.status:
            expr = "alert"
        head, ir = self.lights
        if self.cfg.auto_lights:
            img = self.io.frame("front")
            if img is not None:
                floor = img[img.shape[0] // 2:]
                # hysteresis: once the lights are on the image gets brighter
                dark = float(floor.mean()) < (70 if self._lights_on else 45)
                self._lights_on = dark
                head, ir = (220, 255) if dark else (0, 0)
        self.io.eyes(EXPR.get(expr, 0), head, ir)

    def _line_follow(self):
        img = self.io.frame("front")
        if img is None:
            return 0.0, 0.0
        res = self.line.process(img)
        if self.view.front_min < 0.6:              # obstacle on the line: wait, then halt
            self.blocked_since = self.blocked_since or self.t
            if self.t - self.blocked_since > 5.0:
                self._alert("HALT: line blocked by an obstacle", "error")
                self.mode = "HOLD"
            return 0.0, 0.0
        self.blocked_since = None
        if not res.found:
            self.line_lost_t = self.line_lost_t or self.t
            if self.t - self.line_lost_t > 4.0:
                self._alert("HALT: line lost", "error")
                self.mode = "HOLD"
                return 0.0, 0.0
            return 0.1, 0.9 * (1 if self.line._prev < 0 else -1)   # search towards where it was
        self.line_lost_t = None
        w = self.line.steer(res, self.TICK)
        v = self.cfg.line_speed * max(0.3, 1.0 - 1.2 * abs(res.offset)) * (0.6 if res.junction else 1.0)
        return v, w

    def _explore(self):
        # head for a point 5 m ahead; the local navigator detours around obstacles
        if not hasattr(self, "_explore_goal") or self.nav.state in ("ARRIVED", "HALT"):
            if self.nav.state == "HALT":
                self._alert("HALT: boxed in, no free direction", "error")
                self.mode = "HOLD"
                return 0.0, 0.0
            self._explore_goal = (self.pose.x + 5 * math.cos(self.pose.yaw), self.pose.y + 5 * math.sin(self.pose.yaw))
            self.nav.reset()
        return self.nav.step(self.t, self._pose_tuple(), self._explore_goal, self.view.front_min, 0.8)

    # ---------------------------------------------------------------- missions
    def _goto_ll(self, lat, lon, speed):
        """Follow the shortest road route to (lat, lon); detour locally; reroute / halt if blocked."""
        here = self.ll()
        if here is None:
            self.status = "waiting for GPS fix"
            return 0.0, 0.0, False
        if not self.route_ll or self.route_ll[-1] != (lat, lon):
            route = self.roads.shortest_path(here, (lat, lon)) if self.roads else [here, (lat, lon)]
            if route is None:
                self._alert("HALT: no route to the goal", "error")
                self.mode = "HOLD"
                return 0.0, 0.0, False
            self.route_ll = route
            self.route_xy = [self.frame0.to_xy(a, b) for a, b in route]
            self._route_i = 1
            self.nav.reset()
        # current route target = next route vertex beyond the waypoint radius
        while self._route_i < len(self.route_xy) - 1 and \
                math.hypot(self.route_xy[self._route_i][0] - self.pose.x, self.route_xy[self._route_i][1] - self.pose.y) < 1.5:
            self._route_i += 1
        goal = self.route_xy[self._route_i]
        if self._route_i == len(self.route_xy) - 1 and haversine(*here, lat, lon) < self.cfg.wp_radius:
            return 0.0, 0.0, True
        v, w = self.nav.step(self.t, self._pose_tuple(), goal, self.view.front_min, speed)
        if self.nav.state == "HALT":
            # local detour impossible: block this road segment and look for an alternative route
            if self.roads:
                self.roads.block_edge_near(*here, radius_m=4.0)
                alt = self.roads.shortest_path(here, (lat, lon))
                if alt is not None:
                    self._alert("road blocked - taking an alternative route")
                    self.route_ll = []
                    self.nav.reset()
                    return 0.0, 0.0, False
            self._alert("HALT: no alternative path", "error")
            self.mode = "HOLD"
            return 0.0, 0.0, False
        return v, w, False

    def _mission(self):
        speed = self.mav.params.get("CRUISE_SPEED", self.cfg.cruise_speed) if self.mav else self.cfg.cruise_speed
        if self.mode == "GUIDED":
            tgt = self.mav.guided_target if self.mav else None
            if tgt is None:
                return 0.0, 0.0
            v, w, done = self._goto_ll(*tgt, speed)
            if done:
                self.mode = "HOLD"
                self.status = "arrived"
            return v, w
        if self.mode == "RTL":
            if self.home_ll is None:
                return 0.0, 0.0
            v, w, done = self._goto_ll(*self.home_ll, speed)
            if done:
                self.mode = "HOLD"
                self.status = "home"
            return v, w
        # ---- AUTO mission
        if self.mission_idx >= len(self.mission):
            self.mode = "HOLD"
            self._alert("mission complete", "info")
            return 0.0, 0.0
        it = self.mission[self.mission_idx]
        cmd, p = it["command"], it.get("params", (0, 0, 0, 0))
        if self.mav:
            self.mav.current = it.get("seq", self.mission_idx + 1)
        if self.mission_wait_until is not None:
            if self.t < self.mission_wait_until:
                return 0.0, 0.0
            self.mission_wait_until = None
            self.mission_idx += 1
            return 0.0, 0.0
        if cmd == 16:                                  # NAV_WAYPOINT
            v, w, done = self._goto_ll(it["lat"], it["lon"], speed)
            if done:
                self.status = f"reached waypoint {self.mission_idx + 1}"
                self.route_ll = []
                self.mission_wait_until = self.t + p[0]
            return v, w
        if cmd in (19, 112):                           # LOITER_TIME / CONDITION_DELAY
            self.mission_wait_until = self.t + p[0]
            return 0.0, 0.0
        if cmd == 20:                                  # RTL
            self.mode = "RTL"
            return 0.0, 0.0
        if cmd == 178:                                 # DO_CHANGE_SPEED
            if self.mav:
                self.mav.params["CRUISE_SPEED"] = p[1]
            self.cfg.cruise_speed = p[1]
            self.mission_idx += 1
            return 0.0, 0.0
        if cmd == 177:                                 # DO_JUMP (loop)
            k = self.mission_idx
            n = self.jump_counts.get(k, 0)
            if n < int(p[1]):
                self.jump_counts[k] = n + 1
                self.mission_idx = max(0, int(p[0]) - 1)
            else:
                self.mission_idx += 1
            return 0.0, 0.0
        if cmd == 31010:                               # BOLT jump
            self.start_action("hop" if int(p[1]) == 1 else "jump", height=p[0])
            self.mission_idx += 1
            return 0.0, 0.0
        if cmd == 31011:                               # BOLT height / crawl
            self.crawl = bool(p[1])
            if p[0] > 0:
                self.cfg.height = p[0]
            self.mission_idx += 1
            return 0.0, 0.0
        if cmd == 31012:                               # BOLT tilt
            self.cmd.pitch, self.cmd.roll = math.radians(p[0]), math.radians(p[1])
            self.mission_idx += 1
            return 0.0, 0.0
        self.mission_idx += 1                          # unknown item: skip
        return 0.0, 0.0

    # ================================================================ one-shot actions (generators -> (v, w))
    def _wait_state(self, states, timeout):
        t0 = self.t
        while self.io.telemetry().state_name not in states and self.t - t0 < timeout:
            yield self.cmd.v, 0.0

    def _act_jump(self, height=0.12, **kw):
        """Jump in place / while moving (keeps the current speed)."""
        v = self.drive_v if self.mode == "MANUAL" else self.cmd.v
        c = self.cmd
        c.jump_height, c.jump_mode, c.jump_dist = height, 1, 0.0
        c.jump_seq = (c.jump_seq + 1) & 0xFFFF
        yield v, 0.0
        yield from self._wait_state(("CROUCH", "THRUST", "FLIGHT"), 0.5)
        t0 = self.t
        while self.io.telemetry().state_name != "BALANCE" and self.t - t0 < 3:
            yield v, 0.0

    def _act_hop(self, height=0.10, speed=1.0, **kw):
        """Hop OVER an obstacle in front (bar, curb-like box): approach at speed and let the
        controller time the take-off from the measured distance."""
        yield from self._approach_and_jump(height, speed, mode=1)

    def _act_step_up(self, height=0.15, speed=1.0, **kw):
        """Jump UP onto a step / curb in front."""
        yield from self._approach_and_jump(height, speed, mode=0)

    def _measure_edge(self, height, n=5):
        """Stand still and measure the wheel-to-edge distance from n fresh camera frames
        (median).  Yields while measuring; result in self._edge_d (None if not seen)."""
        vals, t0 = [], self.t
        last = None
        while len(vals) < n and self.t - t0 < 2.0:
            img = self.io.frame("front")
            if img is not None and img is not last:
                last = img
                tel = self.io.telemetry()
                est = self.step_det.estimate(img, 0.5 * (tel.L[0] + tel.L[1]), body_pitch=tel.pitch, step_height=height)
                if est.found:
                    vals.append(est.wheel_to_edge)
            yield 0.0, 0.0
        vals.sort()
        ok = len(vals) >= 3 and vals[-1] - vals[0] < 0.05
        self._edge_d = vals[len(vals) // 2] if ok else None

    def _approach_and_jump(self, height, speed, mode, runup=1.6, request_at=0.9):
        """1) locate the edge with the camera: stop-and-measure, creeping 10 cm at a time,
           until it is 0.5-0.75 m away (where the estimate is accurate to ~2 cm),
        2) back up to `runup` metres by odometry, 3) run up to speed and hand the remaining
           distance to the controller ~0.9 m before the edge; it times the take-off itself."""
        d = None
        crept = 0.0
        for _ in range(12):                                   # 1) locate (creeps at most ~0.6 m)
            for _ in range(6):
                yield 0.0, 0.0                                # let it settle
            yield from self._measure_edge(height)
            d = self._edge_d
            if d is not None and 0.5 <= d <= 0.75:
                break
            if crept > 0.6:
                break
            s0 = self.io.telemetry().s
            if d is None:
                step = 0.15                                   # nothing reliable yet: move closer
            elif d < 0.5:
                step = -0.15                                  # too close: back off
            else:
                step = min(0.6, max(0.08, d - 0.65))          # seen far away: close in to ~0.65 m
            while abs(self.io.telemetry().s - s0) < abs(step):
                yield (0.25 if step > 0 else -0.25), 0.0
            crept += max(step, 0.0)
        if d is None or not 0.5 <= d <= 0.75:
            self._alert("step not found: place BOLT 0.5-0.8 m in front of it, facing it, and retry")
            return
        s_loc = self.io.telemetry().s
        d_now = lambda: d - (self.io.telemetry().s - s_loc)     # noqa: E731
        t0 = self.t
        while d_now() < runup - 0.03 and self.t - t0 < 8:       # 2) back up for the run-up
            yield -0.35, 0.0
        for _ in range(15):
            yield 0.0, 0.0
        t0 = self.t
        while d_now() > request_at and self.t - t0 < 4:          # 3) run up to speed
            yield speed, 0.0
        c = self.cmd
        c.jump_height, c.jump_mode, c.jump_dist = height, mode, max(0.0, d_now())
        c.jump_seq = (c.jump_seq + 1) & 0xFFFF
        yield speed, 0.0
        yield from self._wait_state(("CROUCH",), 0.3)
        t0 = self.t
        landed = False
        while self.t - t0 < 5:
            st = self.io.telemetry().state_name
            landed = landed or st == "LAND"
            if st == "BALANCE" and self.t - t0 > 0.2:
                break
            yield (0.5 if landed else speed), 0.0
        if not landed:
            self._alert("jump aborted (too close / too fast)")
        else:
            self.status = "jump done"

    def _act_stairs(self, rise=0.15, tread=None, steps=3, speed=0.8, **kw):
        """Climb a short flight: run-up -> jump -> brake -> reverse to the edge -> crouched run-up -> ...
        Verified in simulation for treads >= 0.6 m (see docs/04_simulation_report.md)."""
        tread = tread or self.cfg.step_tread_default
        for k in range(steps):
            yield from self._approach_and_jump(rise, speed, mode=0)
            # brake and reverse to just past the edge we landed on (odometry)
            s0 = self.io.telemetry().s
            for _ in range(10):
                yield 0.0, 0.0
            back = max(0.0, self.io.telemetry().s - s0 - 0.15)
            s1 = self.io.telemetry().s
            while s1 - self.io.telemetry().s < back:
                yield -0.25, 0.0
            self.cmd.height = self.cfg.crawl_height + 0.005

    def _act_dodge(self, side):
        """Quick sidestep-like dodge: snap-turn 50 deg, burst forward, turn back."""
        yaw0 = self.pose.yaw
        t0 = self.t
        while abs(((self.pose.yaw - yaw0) + math.pi) % (2 * math.pi) - math.pi) < math.radians(50) and self.t - t0 < 1.0:
            yield 0.3, 3.0 * side
        t1 = self.t
        while self.t - t1 < 0.5:
            yield 1.5, 0.0
        t2 = self.t
        while abs(((self.pose.yaw - yaw0) + math.pi) % (2 * math.pi) - math.pi) > math.radians(5) and self.t - t2 < 1.0:
            yield 0.6, -2.5 * side
        yield 0.0, 0.0

    def _act_sit(self, **kw):
        self.cfg.height = self.cfg.crawl_height
        t0 = self.t
        while self.t - t0 < 1.5:
            yield 0.0, 0.0
        self.mode = "IDLE"

    def _act_stand(self, **kw):
        self.cfg.height = 0.22
        self.mode = "MANUAL"
        t0 = self.t
        while self.t - t0 < 1.0:
            yield 0.0, 0.0

    # ================================================================ for the app
    def snapshot(self):
        tel = self.io.telemetry()
        ll = self.ll()
        return dict(
            mode=self.mode, status=self.status, action=self.action_name if self.action else "",
            telemetry=tel.as_dict(), pose=dict(x=self.pose.x, y=self.pose.y, yaw=self.pose.yaw),
            gps=dict(lat=ll[0], lon=ll[1]) if ll else None, home=self.home_ll,
            sonar={k: round(v, 2) for k, v in (self.view.ranges.items() if self.view else [])},
            cliff=bool(self.view and self.view.cliff), nav=self.nav.state,
            route=self.route_ll, mission_idx=self.mission_idx, mission_len=len(self.mission),
            alerts=self.alerts[-5:], height=self.cfg.height, crawl=self.crawl,
            pitch=self.cmd.pitch, roll=self.cmd.roll, line_color=self.cfg.line_color)
