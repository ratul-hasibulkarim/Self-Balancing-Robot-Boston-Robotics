#!/usr/bin/env python3
"""
Draws the BOLT wiring diagrams (SVG + PNG) into electronics/:
    power_distribution.svg   battery -> anti-spark -> fuse -> e-stop -> motor bus / logic
    signal_wiring.svg        Teensy 4.1 + Raspberry Pi 5 + every sensor / bus with pin numbers
The pin numbers come from firmware/src/config.h - keep both in sync.
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
C = dict(pwr="#d62728", gnd="#222222", can="#ff7f0e", spi="#1f77b4", uart="#9467bd", i2c="#17becf",
         sig="#2ca02c", usb="#8c564b", csi="#e377c2", led="#bcbd22", logic="#ff9896")


def box(ax, x, y, w, h, title, lines=(), fc="#f5f6fa", ec="#333", fs=9, tfs=10.5):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.12", fc=fc, ec=ec, lw=1.4))
    ax.text(x + w / 2, y + h - 0.12, title, ha="center", va="top", fontsize=tfs, weight="bold")
    for i, t in enumerate(lines):
        ax.text(x + 0.14, y + h - 0.78 - i * 0.31, t, ha="left", va="top", fontsize=fs, family="monospace")


def wire(ax, pts, color, label=None, lw=2.2, ls="-", lpos=0.5, lfs=8, loff=(0, 0.12)):
    xs, ys = zip(*pts)
    ax.plot(xs, ys, color=color, lw=lw, ls=ls, solid_capstyle="round", zorder=1)
    if label:
        i = max(0, min(len(pts) - 2, int(lpos * (len(pts) - 1))))
        (x0, y0), (x1, y1) = pts[i], pts[i + 1]
        ax.text((x0 + x1) / 2 + loff[0], (y0 + y1) / 2 + loff[1], label, color=color, fontsize=lfs, ha="center",
                va="bottom", bbox=dict(fc="white", ec="none", pad=0.5, alpha=0.85), zorder=3)


def legend(ax, x, y, items):
    for i, (name, col) in enumerate(items):
        ax.plot([x, x + 0.5], [y - i * 0.32] * 2, color=col, lw=3)
        ax.text(x + 0.62, y - i * 0.32, name, va="center", fontsize=8.5)


# ---------------------------------------------------------------------------
def power():
    fig, ax = plt.subplots(figsize=(15, 9.5))
    ax.set_xlim(0, 30); ax.set_ylim(-0.4, 19); ax.axis("off")
    ax.set_title("BOLT - power distribution (6S LiPo, 22.2 V nominal / 25.2 V full)", fontsize=14, weight="bold")
    box(ax, 0.5, 13.5, 4.2, 3.6, "BATTERY", ["6S1P LiPo 5000 mAh", "50C, XT90-S", "slide-in tray (rear)", "~780 g"], fc="#ffe8e8")
    box(ax, 5.6, 14.3, 3.2, 2.0, "XT90-S", ["anti-spark", "connector"], fc="#fff")
    box(ax, 9.6, 14.3, 3.2, 2.0, "MAIN FUSE", ["40 A blade", "(ATO/ATC)"], fc="#fff")
    box(ax, 13.6, 13.8, 4.4, 3.0, "E-STOP", ["mushroom, NC", "2-pole 32 A DC", "pole 1: motor bus", "pole 2: sense loop"], fc="#ffd6d6")
    box(ax, 19.0, 12.6, 5.0, 5.4, "MOTOR BUS / PDB", ["6x XT30 outputs", "each with 10 A fuse", "1000 uF 50 V low-ESR", "TVS SMBJ28A (regen)", "ACS758-50B current", "sensor (optional)"], fc="#fff2e0")
    # motors
    for i, (lbl, y) in enumerate((("DM-J4310 hip L-front  (CAN1 id 1)", 17.6), ("DM-J4310 hip L-rear   (CAN1 id 2)", 16.4),
                                  ("DM-J4310 hip R-front  (CAN2 id 3)", 15.2), ("DM-J4310 hip R-rear   (CAN2 id 4)", 14.0),
                                  ("MF9025 wheel left     (CAN3 id 1)", 12.8), ("MF9025 wheel right    (CAN3 id 2)", 11.6))):
        ax.text(25.2, y, lbl, fontsize=8.5, family="monospace", va="center",
                bbox=dict(fc="#eef3ff", ec="#557", boxstyle="round,pad=0.3"))
        wire(ax, [(24.0, 15.3), (24.6, y), (25.1, y)], C["pwr"], lw=1.6)
    wire(ax, [(4.7, 15.3), (5.6, 15.3)], C["pwr"])
    wire(ax, [(8.8, 15.3), (9.6, 15.3)], C["pwr"])
    wire(ax, [(12.8, 15.3), (13.6, 15.3)], C["pwr"])
    wire(ax, [(18.0, 15.3), (19.0, 15.3)], C["pwr"])
    # logic branch (always on, before the e-stop)
    box(ax, 9.6, 8.8, 3.6, 2.2, "LOGIC FUSE", ["5 A", "(after main fuse)"], fc="#fff")
    wire(ax, [(11.2, 14.3), (11.2, 11.0)], C["pwr"], "always-on logic", lpos=0.5, loff=(-1.4, 0))
    box(ax, 3.9, 4.2, 5.6, 3.4, "BUCK #1  5 V / 5 A", ["e.g. Pololu D36V50F5", "or Mateksys BEC 12S-PRO", "-> Raspberry Pi 5", "   (USB-C PD not needed:", "   5 V on GPIO pins 2/4)"], fc="#e9ffe9")
    box(ax, 14.2, 4.2, 5.4, 3.4, "BUCK #2  5 V / 3 A", ["sensors + LEDs:", "4x RCWL-1601 sonar", "2x Sharp GP2Y0A21", "2x WS2812 rings, IR LEDs", "headlights (via MOSFETs)"], fc="#e9ffe9")
    wire(ax, [(11.4, 8.8), (11.4, 8.2), (6.9, 8.2), (6.9, 7.6)], C["pwr"])
    wire(ax, [(11.4, 8.2), (16.3, 8.2), (16.3, 7.6)], C["pwr"])
    box(ax, 4.4, 0.5, 5.0, 2.6, "RASPBERRY PI 5 (8 GB)", ["active cooler", "powers Teensy 4.1 via USB", "(VIN-VUSB pad left joined)"], fc="#eaf5ff")
    wire(ax, [(6.9, 4.2), (6.9, 3.1)], C["pwr"], "5 V")
    box(ax, 10.6, 0.5, 4.4, 2.6, "TEENSY 4.1", ["5 V from Pi USB", "VBAT sense: 100k/6.8k", "-> pin 16 (A2)"], fc="#eaf5ff")
    wire(ax, [(9.4, 1.8), (10.6, 1.8)], C["usb"], "USB")
    # vbat sense
    wire(ax, [(2.6, 13.5), (2.6, 0.1), (12.8, 0.1), (12.8, 0.5)], C["sig"], "VBAT sense (divider at the Teensy)", lpos=0.5, lw=1.4, ls="--")
    # estop sense
    wire(ax, [(13.85, 13.8), (13.85, 3.1)], C["sig"], "e-stop sense -> pin 32", lw=1.4, ls="--", lpos=0.5, loff=(1.4, 0))
    # ground note
    ax.text(20.3, 8.6, "GROUND: one star point on the PDB.\nMotor GND, buck GNDs, Pi GND and Teensy GND\nall meet there. CAN-GND wire runs with CAN-H/L.",
            fontsize=9, va="top", bbox=dict(fc="#f7f7f7", ec="#999", boxstyle="round,pad=0.4"))
    ax.text(20.3, 5.9, "WIRE GAUGES: battery -> PDB 12 AWG silicone\nPDB -> each motor 16 AWG,  logic 20 AWG\nsignals 26-28 AWG, CAN twisted pair",
            fontsize=9, va="top", bbox=dict(fc="#f7f7f7", ec="#999", boxstyle="round,pad=0.4"))
    ax.text(20.3, 3.2, "WHY: the e-stop only kills MOTOR power; the Pi + Teensy stay on\nso the app keeps telemetry and the robot reports why it stopped.\nCap + TVS absorb regen spikes from hard braking / jump landings.",
            fontsize=9, va="top", bbox=dict(fc="#fff8e0", ec="#cc9", boxstyle="round,pad=0.4"))
    legend(ax, 0.6, 11.4, [("battery / motor power", C["pwr"]), ("USB", C["usb"]), ("sense signal", C["sig"])])
    for ext in ("svg", "png"):
        fig.savefig(os.path.join(HERE, f"power_distribution.{ext}"), dpi=130, bbox_inches="tight")
    plt.close(fig)


# ---------------------------------------------------------------------------
def signals():
    fig, ax = plt.subplots(figsize=(17, 12))
    ax.set_xlim(0, 34.5); ax.set_ylim(0, 24.5); ax.axis("off")
    ax.set_title("BOLT - signal wiring (Teensy 4.1 real-time + Raspberry Pi 5 brain)", fontsize=14, weight="bold")
    # Teensy
    box(ax, 12.0, 6.0, 7.0, 12.5, "TEENSY 4.1  (1 kHz control)", [
        "22 CAN1_TX  23 CAN1_RX", " 1 CAN2_TX   0 CAN2_RX", "31 CAN3_TX  30 CAN3_RX",
        "11 MOSI 12 MISO 13 SCK", "10 IMU_CS    9 IMU_INT",
        " 2/24 SONAR FL trig/echo", " 3/25 SONAR F  trig/echo", " 4/26 SONAR FR trig/echo", " 5/27 SONAR R  trig/echo",
        "14 (A0) CLIFF L  15 (A1) CLIFF R", "16 (A2) VBAT sense", "29 WS2812 eyes (DIN)",
        "33 HEADLIGHT pwm  36 IR pwm", "37 BUZZER   32 E-STOP sense", "USB  -> Raspberry Pi 5", "3V3 / GND to all 3.3 V parts"],
        fc="#eaf5ff", fs=8.6)
    # CAN transceivers + motors (left)
    for i, (nm, mot, y) in enumerate((("CAN1", "DM-J4310 L-front id1 / L-rear id2", 20.5),
                                      ("CAN2", "DM-J4310 R-front id3 / R-rear id4", 16.8),
                                      ("CAN3", "MF9025 wheel L id1 / wheel R id2", 13.1))):
        box(ax, 6.3, y - 1.25, 3.9, 2.5, f"SN65HVD230 ({nm})", ["3.3 V CAN xcvr", "120R at bus end"], fc="#fff3e6", fs=8, tfs=9.5)
        box(ax, 0.3, y - 1.25, 5.4, 2.5, nm + " motors", [mot.split(" / ")[0], mot.split(" / ")[1], "1 Mbit/s CANH/L/GND"], fc="#fff3e6", fs=8)
        wire(ax, [(5.7, y - 0.9), (6.3, y - 0.9)], C["can"])
        wire(ax, [(10.2, y), (11.0 + 0.25 * i, y), (11.0 + 0.25 * i, 16.5 - i * 0.3), (12.0, 16.5 - i * 0.3)], C["can"])
    # IMU
    box(ax, 6.3, 7.4, 3.6, 2.4, "ICM-42688-P", ["IMU on the sled,", "on rubber grommets", "SPI 10 MHz"], fc="#e8f1ff", fs=8)
    wire(ax, [(9.9, 8.6), (12.0, 8.6)], C["spi"], "SPI")
    # sonar
    box(ax, 0.3, 7.0, 5.2, 3.6, "4x RCWL-1601 sonar", ["3.3 V-compatible HC-SR04", "FL/F/FR in the face,", "R in the rear shell", "VCC 5 V (buck #2)", "ECHO is 3.3 V: OK"], fc="#efffe9", fs=8)
    wire(ax, [(5.5, 9.0), (6.0, 9.0), (6.0, 11.3), (12.0, 11.3)], C["sig"], "trig/echo x4", lpos=0.9)
    box(ax, 0.3, 3.3, 5.2, 3.0, "2x Sharp GP2Y0A21YK0F", ["chin, 30 deg fwd of down", "(drop-off / step detect)", "Vout via 10k/20k divider"], fc="#efffe9", fs=8)
    wire(ax, [(5.5, 4.8), (11.4, 4.8), (11.4, 10.4), (12.0, 10.4)], C["sig"], "A0 / A1", lpos=0.2)
    # right side: LEDs etc.
    box(ax, 21.0, 15.4, 5.6, 2.6, "Eyes: 2x WS2812B ring", ["12 LEDs each, chained", "5 V, 330R in DIN line", "behind smoked visor"], fc="#fbfbe0", fs=8)
    wire(ax, [(19.0, 15.0), (20.0, 15.0), (20.0, 16.6), (21.0, 16.6)], C["led"], "pin 29")
    box(ax, 21.0, 11.8, 5.6, 3.0, "Lights (MOSFET AO3400)", ["2x 1 W white (headlights)", "4x 850 nm IR (night)", "gate 100R, 10k pull-down"], fc="#fbfbe0", fs=8)
    wire(ax, [(19.0, 13.0), (21.0, 13.0)], C["led"], "33 / 36")
    box(ax, 21.0, 9.2, 5.6, 2.0, "Buzzer + E-stop sense", ["piezo on pin 37", "NC contact pin 32 -> GND"], fc="#fbfbe0", fs=8)
    wire(ax, [(19.0, 10.2), (21.0, 10.2)], C["sig"], "37 / 32")
    # Raspberry Pi
    box(ax, 12.0, 0.4, 7.0, 4.6, "RASPBERRY PI 5  (brain, 20 Hz)", ["USB-A  -> Teensy (link + flashing)", "CAM0 / CAM1 -> fisheye front/rear", "GPIO4/5 UART2 -> GPS", "GPIO2/3 I2C1  -> compass", "Wi-Fi -> phone app / Mission Planner"], fc="#eaf5ff", fs=8.4)
    wire(ax, [(15.5, 6.0), (15.5, 5.0)], C["usb"], "USB")
    box(ax, 21.0, 4.4, 5.6, 3.0, "2x Arducam IMX219 NoIR", ["M12 220 deg fisheye", "front (10 deg down) + rear", "=> 360 deg with 2 cameras"], fc="#ffeef8", fs=8)
    wire(ax, [(19.0, 3.8), (20.2, 3.8), (20.2, 5.9), (21.0, 5.9)], C["csi"], "CSI x2")
    box(ax, 21.0, 0.4, 5.6, 3.2, "GPS: Holybro M10", ["u-blox M10 + IST8310", "on the cap stub (top)", "UART 115200, I2C compass"], fc="#e8fbfb", fs=8)
    wire(ax, [(19.0, 1.4), (21.0, 1.4)], C["uart"], "TX/RX")
    wire(ax, [(19.0, 2.4), (21.0, 2.4)], C["i2c"], "SDA/SCL")
    box(ax, 28.0, 1.2, 5.6, 4.8, "Operator", ["phone: BOLT app", "  http://bolt.local:8080", "laptop: Mission Planner", "  UDP 14550 (MAVLink)", "Pi = Wi-Fi AP or client"], fc="#f2f2f2", fs=8)
    wire(ax, [(26.6, 3.6), (28.0, 3.6)], "#777", "Wi-Fi", ls=":")
    legend(ax, 30.4, 23.4, [("CAN bus", C["can"]), ("SPI", C["spi"]), ("digital / analog", C["sig"]), ("LED / PWM", C["led"]),
                            ("USB", C["usb"]), ("CSI camera", C["csi"]), ("UART", C["uart"]), ("I2C", C["i2c"])])
    ax.text(19.6, 23.6, "RULES\n- Teensy 4.1 pins are 3.3 V ONLY (not 5 V tolerant):\n  use 3.3 V sonars (RCWL-1601) and divide Sharp outputs.\n- One CAN bus per leg + one for the wheels keeps each\n  bus at ~50 % load at 1 kHz.\n- Keep CAN pairs twisted, away from motor phase wires.\n- Electronics sled connects through ONE 12-pin\n  connector (JST-GH/Molex) + XT30 -> pull-out in seconds.",
            fontsize=9, va="top", bbox=dict(fc="#fff8e0", ec="#cc9", boxstyle="round,pad=0.4"))
    for ext in ("svg", "png"):
        fig.savefig(os.path.join(HERE, f"signal_wiring.{ext}"), dpi=130, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    power()
    signals()
    print("wrote", os.listdir(HERE))
