#!/usr/bin/env python3
"""
BOLT bench tool (run on the Pi with the brain service STOPPED):

    python3 tools/bolt_cli.py status              # live telemetry
    python3 tools/bolt_cli.py calibrate           # legs folded on the end stops -> store zero offsets
    python3 tools/bolt_cli.py sonar               # sensor check
    python3 tools/bolt_cli.py eyes happy          # LED check
    python3 tools/bolt_cli.py balance [seconds]   # hold the robot upright, it balances in place
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from robot_brain.link import EXPRESSIONS, Command, SerialLink  # noqa: E402


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    port = os.environ.get("BOLT_PORT", "/dev/ttyACM0")
    lk = SerialLink(port)
    time.sleep(0.3)
    cmd = sys.argv[1]
    if cmd == "status":
        while True:
            t = lk.telemetry
            print(f"\r{t.state_name:8s} batt {t.battery_v:5.2f} V  pitch {t.pitch*57.3:+6.1f}  roll {t.roll*57.3:+6.1f}  "
                  f"L {t.L[0]*100:4.1f}/{t.L[1]*100:4.1f} cm  v {t.v:+5.2f}  temp {t.motor_temp_max:3d}C  "
                  f"faults {t.motor_fault_mask:06b}  cal {'yes' if t.calibrated else 'NO'}  estop {'ON' if t.estop else 'off'}", end="")
            time.sleep(0.1)
    elif cmd == "calibrate":
        input("Fold BOTH legs fully against the printed end stops (wheels up, robot on its belly) and press Enter...")
        lk.calibrate()
        time.sleep(0.5)
        print("calibrated:", lk.telemetry.calibrated)
    elif cmd == "sonar":
        while True:
            t = lk.telemetry
            print(f"\rsonar FL {t.sonar[0]:.2f} F {t.sonar[1]:.2f} FR {t.sonar[2]:.2f} R {t.sonar[3]:.2f}   "
                  f"cliff L {t.cliff[0]:.2f} R {t.cliff[1]:.2f}  (m)", end="")
            time.sleep(0.1)
    elif cmd == "eyes":
        e = sys.argv[2] if len(sys.argv) > 2 else "happy"
        lk.send_eyes(EXPRESSIONS.index(e), headlight=80)
        time.sleep(2)
        lk.send_eyes(0)
    elif cmd == "balance":
        secs = float(sys.argv[2]) if len(sys.argv) > 2 else 10
        input("Hold the robot upright, wheels on the floor. Enter = motors ON (Ctrl-C = off)...")
        c = Command(mode=1, height=0.22)
        t0 = time.time()
        try:
            while time.time() - t0 < secs:
                lk.send_command(c)
                time.sleep(0.02)
        finally:
            for _ in range(10):
                lk.send_command(Command(mode=0))
                time.sleep(0.02)
    lk.close()


if __name__ == "__main__":
    main()
