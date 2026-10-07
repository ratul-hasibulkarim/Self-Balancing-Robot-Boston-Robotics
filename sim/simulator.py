"""
BOLT closed-loop simulator: MuJoCo plant + firmware controller (via ctypes)
+ sensor noise/latency + motor torque-speed limits.
"""
import math
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controller as ctl  # noqa: E402
import robot_model  # noqa: E402
import params as P  # noqa: E402  (design/ is on the path via robot_model)


def quat_to_rpy(q):
    w, x, y, z = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def motor_limit(tau, omega, peak, w_max):
    """Linear torque-speed envelope (back-EMF). Braking torque is not limited."""
    if tau * omega > 0:
        avail = peak * max(0.0, 1.0 - abs(omega) / w_max)
        return max(-avail, min(avail, tau))
    return max(-peak, min(peak, tau))


class BoltSim:
    CTRL_DT = 0.001          # 1 kHz, same as the firmware loop

    def __init__(self, variant="V3", payload=0.0, terrain=None, cfg=None, seed=0,
                 noise=True, render=False, width=640, height=400, floor_texture=None, dark=False,
                 start_L=None, cam_track=True, payload_offset=None):
        self.variant = variant
        xml, self.meta = robot_model.build_mjcf(variant, payload=payload, terrain=terrain,
                                                visuals=render, floor_texture=floor_texture,
                                                dark=dark, start_L=start_L)
        if payload_offset is not None and payload > 0:
            xml = xml.replace('name="payload" type="box" size="0.05 0.05 0.04" pos="',
                              f'name="payload" type="box" size="0.05 0.05 0.04" pos="{payload_offset} ', 1)
        self.m = mujoco.MjModel.from_xml_string(xml)
        self.d = mujoco.MjData(self.m)
        self.cfg = dict(cfg or ctl.load_config(variant))
        self.ctrl = ctl.Controller(self.cfg)
        self.rng = np.random.default_rng(seed)
        self.noise = noise
        self.nsub = int(round(self.CTRL_DT / self.m.opt.timestep))
        self.t = 0.0
        self.log = []
        self.frames = []
        self.cmd = self.ctrl.cmd
        self.cmd.mode = 1
        self.cmd.height = self.meta["variant"]["L_nom"] if start_L is None else start_L
        self.prev_out = None
        self.ext_force = None
        self.renderer = None
        self.cam_track = cam_track
        if render:
            self.renderer = mujoco.Renderer(self.m, height, width)
            self.cam = mujoco.MjvCamera()
            self.cam.distance = 1.6
            self.cam.elevation = -12
            self.cam.azimuth = 135
            self.vopt = mujoco.MjvOption()
            self.vopt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
        j = self.m.joint
        self.qadr = {n: self.m.jnt_qposadr[j(n).id] for n in
                     ("hipF_L", "hipR_L", "hipF_R", "hipR_R", "wheel_L", "wheel_R")}
        self.vadr = {n: self.m.jnt_dofadr[j(n).id] for n in self.qadr}
        self.base = self.m.body("base").id
        self.us_ids = {nm: [self.m.sensor(f"{nm}_{k}").id for k in range(3)]
                       for nm in ("us_fc", "us_fl", "us_fr", "us_r")} if self.m.nsensor > 2 else {}
        mujoco.mj_forward(self.m, self.d)

    # ------------------------------------------------------------- sensors
    def sonar(self):
        """Ultrasonic emulation: min of 3 rays in a ~25 deg cone, 4 m max, 1 cm noise."""
        out = {}
        for nm, ids in self.us_ids.items():
            vals = [self.d.sensordata[self.m.sensor_adr[i]] for i in ids]
            vals = [x for x in vals if x >= 0]
            r = min(vals) if vals else 4.0
            out[nm] = min(4.0, r + (self.rng.normal(0, 0.01) if self.noise else 0))
        return out

    def cliff(self):
        out = []
        for nm in ("cliff_l", "cliff_r"):
            v = self.d.sensordata[self.m.sensor_adr[self.m.sensor(nm).id]]
            out.append(0.6 if v < 0 else min(0.6, v + (self.rng.normal(0, 0.003) if self.noise else 0)))
        return out

    def _fill_inputs(self):
        d, inp = self.d, self.ctrl.inp
        n = self.rng.normal if self.noise else (lambda a, b: 0.0)
        roll, pitch, yaw = quat_to_rpy(d.qpos[3:7])
        gyro = d.sensordata[0:3]
        acc = d.sensordata[3:6]
        inp.dt = self.CTRL_DT
        inp.imu.roll = roll + n(0, 0.002)
        inp.imu.pitch = pitch + n(0, 0.002)
        inp.imu.yaw = yaw
        inp.imu.gx, inp.imu.gy, inp.imu.gz = (gyro[0] + n(0, 0.004), gyro[1] + n(0, 0.004), gyro[2] + n(0, 0.004))
        inp.imu.ax, inp.imu.ay, inp.imu.az = acc
        p10, p40 = self.meta["phi1_0"], self.meta["phi4_0"]
        for i, s in enumerate("LR"):
            leg = inp.leg[i]
            leg.phi1 = p10 + d.qpos[self.qadr[f"hipF_{s}"]] + n(0, 0.0004)
            leg.phi4 = p40 + d.qpos[self.qadr[f"hipR_{s}"]] + n(0, 0.0004)
            leg.dphi1 = d.qvel[self.vadr[f"hipF_{s}"]] + n(0, 0.01)
            leg.dphi4 = d.qvel[self.vadr[f"hipR_{s}"]] + n(0, 0.01)
            leg.wheel_vel = d.qvel[self.vadr[f"wheel_{s}"]] + n(0, 0.02)
        inp.battery_v = 23.5

    def _apply(self, out):
        d = self.d
        hm, wm = P.HIP_MOTOR, P.WHEEL_MOTOR
        names = ("hipF_L", "hipR_L", "hipF_R", "hipR_R")
        for k, n in enumerate(names):
            d.ctrl[k] = motor_limit(out.hip[k], d.qvel[self.vadr[n]], hm["peak_torque"], hm["max_speed"])
        for k, n in enumerate(("wheel_L", "wheel_R")):
            d.ctrl[4 + k] = motor_limit(out.wheel[k], d.qvel[self.vadr[n]], wm["peak_torque"], wm["max_speed"])

    # ------------------------------------------------------------- stepping
    def step(self):
        self._fill_inputs()
        out = self.ctrl.step()
        # 1 control period of latency (CAN round trip)
        if self.prev_out is not None:
            self._apply(self.prev_out)
        self.prev_out = ctl.Outputs.from_buffer_copy(out)
        if self.ext_force is not None:
            f, t_end = self.ext_force
            if self.t < t_end:
                self.d.xfrc_applied[self.base, :3] = f
            else:
                self.d.xfrc_applied[self.base, :] = 0
                self.ext_force = None
        for _ in range(self.nsub):
            mujoco.mj_step(self.m, self.d)
        self.t += self.CTRL_DT
        return out

    def push(self, force_xyz, duration=0.1):
        self.ext_force = (np.array(force_xyz, dtype=float), self.t + duration)

    def state(self):
        d = self.d
        roll, pitch, yaw = quat_to_rpy(d.qpos[3:7])
        vel_world = d.qvel[0:3]
        Rw = d.xmat[self.base].reshape(3, 3)
        v_body = Rw.T @ vel_world
        return dict(t=self.t, x=d.qpos[0], y=d.qpos[1], z=d.qpos[2], roll=roll, pitch=pitch, yaw=yaw,
                    v=v_body[0], vz=vel_world[2])

    def fallen(self):
        s = self.state()
        # body touching the ground or tipped over
        return abs(s["pitch"]) > 1.0 or abs(s["roll"]) > 0.9 or self.ctrl.out.state == 7

    def run(self, duration, policy=None, log_every=10, fps=30, stop_on_fall=True):
        n = int(round(duration / self.CTRL_DT))
        frame_every = int(round(1.0 / fps / self.CTRL_DT)) if self.renderer else 0
        for i in range(n):
            if policy:
                policy(self.t, self, self.cmd)
            out = self.step()
            if i % log_every == 0:
                s = self.state()
                s.update(state=out.state, L=0.5 * (out.L[0] + out.L[1]), Lr=out.L[1], Ll=out.L[0],
                         theta=0.5 * (out.theta[0] + out.theta[1]), v_ref=out.v_ref, ds=out.ds,
                         hipT=max(abs(x) for x in out.hip), wheelT=max(abs(x) for x in out.wheel),
                         wheel_low=self.wheel_bottom_z())
                self.log.append(s)
            if frame_every and i % frame_every == 0:
                self.frames.append(self.render_frame())
            if stop_on_fall and self.fallen():
                return False
        return True

    def wheel_bottom_z(self):
        return min(self.d.xpos[self.m.body(f"wheel_{s}").id][2] for s in "LR") - self.meta["variant"]["wheel_radius"]

    def render_frame(self, camera=None):
        if camera is not None:
            self.renderer.update_scene(self.d, camera=camera, scene_option=self.vopt)
        else:
            if self.cam_track:
                self.cam.lookat[:] = self.d.xpos[self.base] + np.array([0, 0, -0.05])
            self.renderer.update_scene(self.d, camera=self.cam, scene_option=self.vopt)
        return self.renderer.render().copy()


def save_gif(frames, path, fps=30, scale=1.0):
    from PIL import Image
    imgs = [Image.fromarray(f) for f in frames]
    if scale != 1.0:
        imgs = [im.resize((int(im.width * scale), int(im.height * scale))) for im in imgs]
    imgs[0].save(path, save_all=True, append_images=imgs[1:], duration=int(1000 / fps), loop=0, optimize=True)


if __name__ == "__main__":
    sim = BoltSim("V3")
    ok = sim.run(3.0)
    s = sim.state()
    print("balanced 3 s:", ok, {k: round(v, 3) for k, v in s.items()}, "state", ctl.STATES[sim.ctrl.out.state])
