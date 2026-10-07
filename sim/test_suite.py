#!/usr/bin/env python3
"""
BOLT simulation test suite.  Runs every requirement-driven scenario against a
design variant and reports pass/fail + performance numbers.

    python3 sim/test_suite.py V3            # one variant
    python3 sim/test_suite.py V1 V2 V3      # design iteration comparison
                                            # -> sim/results/<variant>.json + summary.md
"""
import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controller as ctl  # noqa: E402
from simulator import BoltSim  # noqa: E402

RESULTS = os.path.join(HERE, "results")
os.makedirs(RESULTS, exist_ok=True)


def L_nom(sim):
    return sim.meta["variant"]["L_nom"]


# ---------------------------------------------------------------- scenarios
def t_static(var):
    sim = BoltSim(var)
    ok = sim.run(5.0)
    p = np.array([l["pitch"] for l in sim.log[100:]])
    x = np.array([l["x"] for l in sim.log[100:]])
    return dict(ok=ok, pitch_rms_deg=float(np.degrees(np.sqrt(np.mean(p ** 2)))), drift_m=float(np.ptp(x)))


def _push_ok(var, impulse, direction=1, payload=0.0):
    """Shove the body; pass only if it recovers CLEANLY: never tips, legs never swing
    beyond +-45 deg, and after 4 s it is back to a calm, normal stance."""
    sim = BoltSim(var, payload=payload)
    sim.run(1.0)
    sim.push([direction * impulse / 0.1, 0, 0], 0.1)
    if not sim.run(4.0):
        return False
    after = [l for l in sim.log if l["t"] > 1.0]
    worst_leg = max(abs(l["theta"]) for l in after)
    end = sim.log[-1]
    return bool(sim.ctrl.out.state == 1 and worst_leg < math.radians(45) and abs(end["pitch"]) < 0.1
            and abs(end["theta"]) < 0.15 and abs(end["v"]) < 0.3)


def t_push(var, payload=0.0):
    out = {}
    for d, name in ((1, "fwd"), (-1, "back")):
        lo, hi = 2.0, 40.0
        if not _push_ok(var, lo, d, payload):
            out[name] = 0.0
            continue
        for _ in range(7):
            mid = 0.5 * (lo + hi)
            if _push_ok(var, mid, d, payload):
                lo = mid
            else:
                hi = mid
        out[name] = round(lo, 1)
    return dict(ok=min(out.values()) >= 8.0, max_impulse_Ns=out)


def t_speed(var):
    """Accelerate to v_max, cruise, emergency stop."""
    res = {}
    for v in (1.0, 2.0, 2.4):
        sim = BoltSim(var, cfg=dict(ctl.load_config(var), v_max=v))

        def pol(t, s, c, v=v):
            c.v = v if 0.3 < t < 5.0 else 0.0
        ok = sim.run(7.5, policy=pol)
        vmax = max(l["v"] for l in sim.log)
        pmax = max(abs(l["pitch"]) for l in sim.log)
        # pass = survived AND tracked the command (no runaway past the motors' speed limit)
        res[f"{v}"] = dict(ok=bool(ok and vmax < 1.2 * v and math.degrees(pmax) < 10), v_reached=round(vmax, 2),
                           pitch_max_deg=round(math.degrees(pmax), 1))
    top = max([float(k) for k, r in res.items() if r["ok"]] or [0])
    return dict(ok=res["2.0"]["ok"], top_speed_ok=top, runs=res)


def t_turn(var):
    sim = BoltSim(var)

    def pol(t, s, c):
        c.v = 1.5 if t > 0.3 else 0
        c.yaw_rate = 2.0 if t > 1.5 else 0
    ok = sim.run(5.0, policy=pol)
    rmax = max(abs(l["roll"]) for l in sim.log)
    return dict(ok=ok, roll_max_deg=round(math.degrees(rmax), 1))


def t_incline(var):
    res = {}
    for a in (10, 15, 20, 25, 30):
        sim = BoltSim(var, terrain={"kind": "incline", "angle_deg": a, "x0": 0.8})

        def pol(t, s, c):
            c.v = 0.6 if t > 0.3 else 0
        ok = sim.run(9.0, policy=pol)
        top = 4.0 * math.sin(math.radians(a))
        reached = ok and sim.state()["z"] > top + 0.2
        # hold position on the slope: stop half way and stand 3 s
        sim2 = BoltSim(var, terrain={"kind": "incline", "angle_deg": a, "x0": 0.8})

        def pol2(t, s, c):
            c.v = 0.6 if 0.3 < t < 4.0 else 0
        ok2 = sim2.run(7.0, policy=pol2)
        xs = [l["x"] for l in sim2.log if l["t"] > 5.0]
        hold = ok2 and (max(xs) - min(xs) < 0.15 if xs else False)
        res[a] = dict(climb=bool(reached), hold=bool(hold))
    best = max([a for a, r in res.items() if r["climb"] and r["hold"]] or [0])
    return dict(ok=best >= 15, max_slope_deg=best, runs=res)


def t_side_slope(var):
    res = {}
    for a in (8, 12, 16):
        sim = BoltSim(var, terrain={"kind": "side_slope", "angle_deg": a})

        def pol(t, s, c):
            c.v = 0.5 if t > 0.5 else 0
        ok = sim.run(5.0, policy=pol)
        rolls = [abs(l["roll"]) for l in sim.log if l["t"] > 3.0]
        res[a] = dict(ok=ok, body_roll_deg=round(math.degrees(max(rolls)) if rolls else 99, 1))
    best = max([a for a, r in res.items() if r["ok"] and r["body_roll_deg"] < 4] or [0])
    return dict(ok=best >= 12, max_side_slope_leveled_deg=best, runs=res)


def t_rough(var):
    res = {}
    for amp in (0.05, 0.08, 0.11, 0.14):
        for v in (0.8, 1.5):
            sim = BoltSim(var, terrain={"kind": "rough", "amplitude": amp, "seed": 3})

            def pol(t, s, c, v=v):
                c.v = v if t > 0.3 else 0
            ok = sim.run(4.0 / v + 1.0, policy=pol)
            res[f"{amp}@{v}"] = bool(ok and sim.state()["x"] > 3.0)
    best = max([float(k.split("@")[0]) for k, r in res.items() if r and k.endswith("1.5")] or [0])
    return dict(ok=res.get("0.05@1.5", False), max_bump_amplitude_at_1p5ms=best, runs=res)


def t_payload(var):
    res = {}
    for m in (0.0, 1.0, 2.0, 3.0):
        sim = BoltSim(var, payload=m)

        def pol(t, s, c):
            c.v = 1.0 if 0.5 < t < 4 else 0
        ok = sim.run(6.0, policy=pol)
        push = _push_ok(var, 8.0, 1, payload=m)
        res[m] = dict(drive=ok, push8Ns=push)
    best = max([m for m, r in res.items() if r["drive"] and r["push8Ns"]] or [0])
    return dict(ok=best >= 2.0, max_payload_kg=best, runs=res)


def t_tilt(var):
    res = {}
    for pitch in (-0.4, -0.2, 0.2, 0.4):
        sim = BoltSim(var)

        def pol(t, s, c, p=pitch):
            c.pitch = p if t > 0.5 else 0
        ok = sim.run(4.0, policy=pol)
        res[f"pitch {pitch:+.1f}"] = dict(ok=ok, achieved_deg=round(math.degrees(sim.state()["pitch"]), 1))
    for roll in (-0.25, 0.25):
        sim = BoltSim(var)

        def pol(t, s, c, r=roll):
            c.roll = r if t > 0.5 else 0
        ok = sim.run(4.0, policy=pol)
        res[f"roll {roll:+.2f}"] = dict(ok=ok, achieved_deg=round(math.degrees(sim.state()["roll"]), 1))
    for h in ("min", "max"):
        sim = BoltSim(var)
        v = sim.meta["variant"]

        def pol(t, s, c, h=h):
            c.height = (v["L_min"] if h == "min" else v["L_max"]) if t > 0.5 else v["L_nom"]
        ok = sim.run(4.0, policy=pol)
        res[f"height {h}"] = dict(ok=ok, hip_height_m=round(sim.state()["z"], 3))
    return dict(ok=all(r["ok"] for r in res.values()), runs=res)


def t_crawl(var):
    """Crawl mode under a bar: find the lowest bar the robot passes."""
    res = {}
    for clr in (0.50, 0.45, 0.42, 0.40, 0.38):
        sim = BoltSim(var, terrain={"kind": "obstacle_bar", "x": 1.5, "clearance": clr}, start_L=None)

        def pol(t, s, c):
            c.mode = 2 if t > 0.3 else 1
            c.v = 0.4 if t > 1.5 else 0
        ok = sim.run(8.0, policy=pol)
        res[clr] = bool(ok and sim.state()["x"] > 1.8)
    best = min([c for c, r in res.items() if r] or [9])
    return dict(ok=best <= 0.45, lowest_bar_passed_m=best, runs=res)


def _jump_flat(var, h):
    sim = BoltSim(var)

    def pol(t, s, c):
        if t > 1.0 and c.jump_seq == 0:
            c.jump_height = h
            c.jump_seq = 1
            c.jump_mode = 1
    ok = sim.run(4.0, policy=pol)
    return ok and sim.ctrl.out.state == 1, max(l["wheel_low"] for l in sim.log)


def t_jump(var):
    res = {}
    for h in (0.10, 0.20, 0.30):
        ok, lift = _jump_flat(var, h)
        res[h] = dict(ok=ok, wheel_clearance_m=round(lift, 3))
    best = max([r["wheel_clearance_m"] for r in res.values() if r["ok"]] or [0])
    return dict(ok=best > 0.15, max_wheel_clearance_m=best, runs=res)


def _step(var, rise, v, d):
    x0 = 1.8
    sim = BoltSim(var, terrain={"kind": "steps", "x0": x0, "steps": [(rise, 3.0)]})

    def pol(t, s, c):
        x = s.state()["x"]
        c.v = v if t > 0.3 else 0
        if c.jump_seq == 0 and x > x0 - d:
            c.jump_height, c.jump_seq, c.jump_mode, c.jump_dist = rise, 1, 0, x0 - x
        if x > x0 + 0.8:
            c.v = 0
    ok = sim.run(5.0, policy=pol)
    s = sim.state()
    return bool(ok and s["x"] > x0 + 0.1 and abs(s["z"] - (rise + L_nom(sim) + sim.meta["variant"]["wheel_radius"])) < 0.05)


def t_step(var):
    res = {}
    for rise in (0.08, 0.12, 0.16, 0.20, 0.22):
        trials = [_step(var, rise, 1.0, d) for d in (0.7, 0.9, 1.1)]
        res[rise] = sum(trials) / len(trials)
    best = max([r for r, p in res.items() if p >= 0.99] or [0])
    return dict(ok=best >= 0.16, max_step_m=best, success_rate=res)


def _stairs(var, rise, tread, n=3, vj=0.8, back=0.15):
    """Stair policy (same as companion/robot_brain/behaviors.py StairClimber):
    run up -> jump (controller times the take-off from the edge distance) -> brake ->
    reverse to just past the edge we landed on -> crouched run-up -> next jump."""
    x0 = 1.8
    sim = BoltSim(var, terrain={"kind": "steps", "x0": x0, "steps": [(rise, tread)] * (n - 1) + [(rise, 3.0)]})
    edges = [x0 + i * tread for i in range(n)]
    st = {"k": 0, "phase": "approach", "t0": 0, "landed": False, "seq": 0}
    Lc, Ln = sim.cfg["L_crouch"], sim.cfg["L_nom"]

    def jump(c, x):
        k = st["k"]
        st["seq"] += 1
        c.jump_height, c.jump_mode, c.jump_dist, c.jump_seq = rise, 0, edges[k] - x, st["seq"]
        st["k"] = k + 1
        st["phase"] = "jumping"
        st["landed"] = False

    def pol(t, s, c):
        x = s.state()["x"]
        o = s.ctrl.out
        k, ph = st["k"], st["phase"]
        if ph == "approach":
            c.v = vj if t > 0.3 else 0
            if o.state == 1 and edges[0] - x < 0.9:
                jump(c, x)
        elif ph == "jumping":
            if o.state == 5:
                c.v = 0.4 if k < n else 0.5               # landing: ease off, brake after
                st["landed"] = True
            if o.state == 1 and st["landed"]:
                st["phase"] = "reposition" if k < n else "done"
                c.height = Ln
            elif o.state == 1 and s.ctrl.out.v_ref == 0.0 and not st["landed"] and t > 0.5:
                st["k"] = k - 1                            # controller aborted the jump: retry
                st["phase"] = "reposition"
                c.v = 0.0
        elif ph == "reposition":
            target = edges[k - 1] + back if k > 0 else edges[0] - 1.0
            c.v = -0.25 if x > target + 0.03 else 0.0
            if x <= target + 0.03:
                c.height = Lc
                st["t0"] = st["t0"] or t
                if t - st["t0"] > 0.5 and abs(s.state()["v"]) < 0.05:
                    st["phase"] = "runup"
            else:
                st["t0"] = 0
        elif ph == "runup":
            c.v, c.height = vj, Lc
            if o.state == 1:
                jump(c, x)
        else:
            c.v = 0.5 if x < edges[-1] + 0.6 else 0
            c.height = Ln
    ok = sim.run(16.0, policy=pol)
    return bool(ok and abs(sim.state()["z"] - L_nom(sim) - sim.meta["variant"]["wheel_radius"] - n * rise) < 0.05)


def t_stairs(var):
    res = {}
    for tread in (0.30, 0.45, 0.60):
        for rise in (0.12, 0.17):
            res[f"{rise}x{tread}"] = _stairs(var, rise, tread)
    ok_treads = [float(k.split("x")[1]) for k, r in res.items() if r]
    return dict(ok=any(res.values()), min_tread_m=min(ok_treads) if ok_treads else None, runs=res)


def t_obstacle_hop(var):
    """Hop over a 10 cm x 10 cm bar lying on the floor (jump_mode 1)."""
    res = {}
    for h in (0.06, 0.10, 0.14):
        sim = BoltSim(var, terrain={"kind": "flat", "obstacles": [(1.8, 0, 0.05, 1.0, h / 2)]})

        def pol(t, s, c, h=h):
            x = s.state()["x"]
            c.v = 1.0 if t > 0.3 else 0
            if c.jump_seq == 0 and x > 1.0:
                c.jump_height, c.jump_seq, c.jump_mode, c.jump_dist = h, 1, 1, 1.75 - x
            if x > 2.8:
                c.v = 0
        ok = sim.run(6.0, policy=pol)
        res[h] = bool(ok and sim.state()["x"] > 2.0)
    best = max([h for h, r in res.items() if r] or [0])
    return dict(ok=best >= 0.10, max_obstacle_m=best, runs=res)


def t_recover(var):
    """Tipped onto the front / rear bumper (motors were off): can it self-right?
    (Lying on a side or upside down is not recoverable with this leg layout.)"""
    res = {}
    for direction in (1, -1):
        sim = BoltSim(var)
        sim.run(1.0)
        sim.cmd.mode = 0
        sim.push([direction * 6, 0, 0], 0.3)
        sim.run(2.0, stop_on_fall=False)
        lying = abs(sim.state()["pitch"]) > 0.8
        sim.cmd.mode = 1
        sim.run(5.0, stop_on_fall=False)
        res["front" if direction > 0 else "back"] = dict(lying=lying, recovered=sim.ctrl.out.state == 1 and abs(sim.state()["pitch"]) < 0.2)
    return dict(ok=all(r["recovered"] for r in res.values()), experimental=True, runs=res)


TESTS = {
    "static_balance": t_static,
    "push_recovery": t_push,
    "top_speed": t_speed,
    "turning": t_turn,
    "incline": t_incline,
    "side_slope": t_side_slope,
    "rough_terrain": t_rough,
    "payload": t_payload,
    "tilt_height": t_tilt,
    "crawl": t_crawl,
    "jump_flat": t_jump,
    "step_up": t_step,
    "stairs": t_stairs,
    "obstacle_hop": t_obstacle_hop,
    "fall_recovery": t_recover,
}


def _run(args):
    var, name = args
    t0 = time.time()
    try:
        r = TESTS[name](var)
    except Exception as e:      # a crash is a failure, not a suite abort
        r = dict(ok=False, error=repr(e))
    r["runtime_s"] = round(time.time() - t0, 1)
    return var, name, r


def main(variants, only=None):
    ctl.build_lib(True)
    names = only or list(TESTS)
    jobs = [(v, n) for v in variants for n in names]
    with ProcessPoolExecutor(os.cpu_count()) as ex:
        results = list(ex.map(_run, jobs))
    table = {}
    for var, name, r in results:
        table.setdefault(var, {})[name] = r
    for var in variants:
        path = os.path.join(RESULTS, f"{var}.json")
        old = json.load(open(path)) if (only and os.path.exists(path)) else {}
        old.update(table[var])
        json.dump(old, open(path, "w"), indent=1, default=str)
        print(f"\n===== {var} =====")
        for n, r in table[var].items():
            print(f"  {'PASS' if r.get('ok') else 'FAIL'}  {n:15s} {json.dumps({k: v for k, v in r.items() if k not in ('ok',)}, default=str)[:180]}")


if __name__ == "__main__":
    args = sys.argv[1:]
    only = None
    if "--only" in args:
        i = args.index("--only")
        only = args[i + 1].split(",")
        args = args[:i] + args[i + 2:]
    main(args or ["V3"], only)
