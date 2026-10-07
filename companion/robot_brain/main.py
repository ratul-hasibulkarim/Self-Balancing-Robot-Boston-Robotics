"""
BOLT companion computer entry point (Raspberry Pi 5).

    python3 -m robot_brain.main --config config.yaml           # real robot
    python3 sim/companion_sim.py --demo app                     # same brain + app against MuJoCo

Runs: brain tick (20 Hz) in a thread, the app web server + MAVLink endpoint.
"""
import argparse
import asyncio
import os
import threading
import time

import yaml

from .behaviors import Brain, Settings
from .navigation.planner import RoadGraph


def run(io, cfg, block=True):
    s = cfg.get("settings", {})
    settings = Settings(**{k: v for k, v in s.items() if hasattr(Settings, k)})
    roads = None
    if cfg.get("road_graph") and os.path.exists(cfg["road_graph"]):
        roads = RoadGraph.from_geojson(cfg["road_graph"])
        print(f"road graph: {len(roads.nodes)} nodes")
    mav = None
    if cfg.get("mavlink", {}).get("enabled", True):
        from .mission.mavlink_vehicle import MavlinkVehicle
        mav = MavlinkVehicle(cfg.get("mavlink", {}).get("url", "udpout:255.255.255.255:14550"))
    brain = Brain(io, settings, roads, mav)
    lock = threading.Lock()

    def brain_loop():
        nxt = time.monotonic()
        while True:
            with lock:
                brain.step()
                if mav:                                    # mirror state to Mission Planner
                    ll = brain.ll()
                    tel = io.telemetry()
                    if ll:
                        mav.lat, mav.lon = ll
                    import math
                    mav.heading = (math.pi / 2 - brain.pose.yaw) % (2 * math.pi)
                    mav.roll, mav.pitch, mav.speed, mav.battery_v = tel.roll, tel.pitch, abs(tel.v), tel.battery_v
                    fix = io.gps()
                    mav.fix_type, mav.sats, mav.alt = (3 if fix.valid else 0), fix.sats, fix.alt
            nxt += Brain.TICK
            time.sleep(max(0.0, nxt - time.monotonic()))

    threading.Thread(target=brain_loop, daemon=True).start()
    from .app_server import AppServer
    server = AppServer(brain, io, port=cfg.get("app_port", 8080), lock=lock)
    if block:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(server.start())
        print(f"BOLT app: http://<robot-ip>:{server.port}/")
        loop.run_forever()
    return brain, server


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=os.path.join(os.path.dirname(__file__), "..", "config.yaml"))
    args = ap.parse_args()
    cfg = yaml.safe_load(open(args.config))
    from .io_real import RealRobotIO
    io = RealRobotIO(cfg.get("teensy_port", "/dev/ttyACM0"), cfg.get("gps_port", "/dev/ttyAMA2"))
    run(io, cfg)


if __name__ == "__main__":
    main()
