#!/usr/bin/env python3
"""
Parametric CAD for BOLT, the wheel-legged balancing robot.

Generates every printable part as an STL (already in its best print
orientation) plus an assembly preview, a mass/COM report and part renders.

    python3 cad/build_parts.py            # -> cad/stl/*.stl, cad/assembly.stl,
                                          #    cad/mass_report.json, docs/images/*.png

Requires: manifold3d, trimesh, numpy, matplotlib (pip install manifold3d trimesh matplotlib)

All dimensions in millimetres.  Geometry is driven by design/params.py (FINAL).
Bolt patterns for the motors are parameters below - CHECK THEM AGAINST THE
DRAWING OF THE MOTOR BATCH YOU RECEIVE before printing.
"""
import json
import math
import os
import sys

import numpy as np
import trimesh
from manifold3d import CrossSection, Manifold

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "design"))
import kinematics  # noqa: E402
from params import FINAL as P, HIP_MOTOR, WHEEL_MOTOR  # noqa: E402

SEG = 64          # circle resolution

# ---------------------------------------------------------------------------
# Mechanical interface parameters (mm)
# ---------------------------------------------------------------------------
HIP_PCD = 40.0            # DM-J4310 output flange bolt circle
HIP_BOLTS = 4             # M3
HIP_STATOR_PCD = 50.0     # DM-J4310 rear housing bolt circle (to side plate)
WHEEL_STATOR_PCD = 30.0   # MF9025 stator (fixed side) bolt circle, M4
WHEEL_ROTOR_PCD = 80.0    # MF9025 rotor face bolt circle, M3 x 6
BEARING_OD, BEARING_W, PIN_D = 22.2, 7.2, 8.2   # 608-2RS + M8 shoulder bolt
M3_CLR, M3_INSERT = 3.4, 4.2                    # clearance / heat-set insert
MAGNET_D, MAGNET_H = 6.3, 3.2                   # 6x3 mm N52 disc magnets

BX = P["body_size"][0] * 500        # half depth   (105)
BY = P["body_size"][1] * 500        # half width   (120)
BZ0 = (P["body_center_z"] - P["body_size"][2] / 2) * 1000   # bottom (-70)
BZ1 = (P["body_center_z"] + P["body_size"][2] / 2) * 1000   # top    (160)
R_SHELL = 42.0
WALL = 3.0
SPLIT_Z = 5.0           # belly pan / upper shell split height

L5 = P["hip_spacing"] * 1000
L1 = P["thigh"] * 1000
L2 = P["shin"] * 1000
LEG_Y = P["leg_y"] * 1000
WHEEL_Y = P["wheel_y"] * 1000
WR = P["wheel_radius"] * 1000
WW = P["wheel_width"] * 1000

PLATE_Y0 = BY - 12      # side plate inner face (108)
PLATE_T = 8.0

THIGH_T = 10.0
SHIN_T = 10.0
THIGH_Y = 125.0                  # inner face of thighs
FRONT_SHIN_Y = THIGH_Y + THIGH_T + 1.5
REAR_SHIN_Y = FRONT_SHIN_Y + SHIN_T + 1.5


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl(r, h, axis="z", center=True, r2=None, seg=SEG):
    m = Manifold.cylinder(h, r, r if r2 is None else r2, seg, center)
    if axis == "x":
        m = m.rotate([0, 90, 0])
    elif axis == "y":
        m = m.rotate([-90, 0, 0])
    return m


def rounded_box(x0, x1, y0, y1, z0, z1, r):
    s = Manifold.sphere(r, 32)
    pts = []
    for x in (x0 + r, x1 - r):
        for y in (y0 + r, y1 - r):
            for z in (z0 + r, z1 - r):
                pts.append(s.translate([x, y, z]))
    return Manifold.batch_hull(pts)


def rounded_plate_xz(x0, x1, z0, z1, r, y0, t):
    """Plate lying in the x-z plane, thickness t from y0 to y0+t."""
    cs = CrossSection.square([x1 - x0 - 2 * r, z1 - z0 - 2 * r]).translate(
        [x0 + r, z0 + r]).offset(r, CrossSection.JoinType.Round if hasattr(
            CrossSection, "JoinType") else 1)
    m = Manifold.extrude(cs, t)            # extruded along +z, profile in x-y
    # map profile (x, y) -> (x, z), extrusion z -> y
    return m.rotate([90, 0, 0]).translate([0, y0 + t, 0])


def bolt_circle(pcd, n, d, h, axis="y", start_deg=45.0, center=(0, 0, 0)):
    holes = []
    for i in range(n):
        a = math.radians(start_deg + 360.0 * i / n)
        if axis == "y":
            off = [center[0] + pcd / 2 * math.cos(a), center[1], center[2] + pcd / 2 * math.sin(a)]
        else:
            off = [center[0] + pcd / 2 * math.cos(a), center[1] + pcd / 2 * math.sin(a), center[2]]
        holes.append(cyl(d / 2, h, axis).translate(off))
    return Manifold.batch_boolean(holes, 0) if hasattr(Manifold, "batch_boolean") else sum(holes[1:], holes[0])


def union(parts):
    out = parts[0]
    for p in parts[1:]:
        out = out + p
    return out


def link2d(length, r_a, r_b, width):
    """2-D capsule-like link from (0,0) to (length,0) in the x-y plane."""
    a = CrossSection.circle(r_a, SEG)
    b = CrossSection.circle(r_b, SEG).translate([length, 0])
    bar = CrossSection.square([length, width]).translate([0, -width / 2])
    return CrossSection.batch_hull([a, b, bar]) if hasattr(CrossSection, "batch_hull") else (a + b + bar).hull()


# ---------------------------------------------------------------------------
# BODY
# ---------------------------------------------------------------------------
def envelope(offset=0.0):
    return rounded_box(-BX - offset, BX + offset, -BY - offset, BY + offset,
                       BZ0 - offset, BZ1 + offset, R_SHELL + offset)


def hip_cutouts():
    """Openings in the shell for the thighs (both sides, both hips)."""
    cuts = []
    for side in (1, -1):
        slot = Manifold.batch_hull([
            cyl(34, 40, "y").translate([L5 / 2, side * BY, 0]),
            cyl(34, 40, "y").translate([-L5 / 2, side * BY, 0]),
        ])
        cuts.append(slot)
    return union(cuts)


BAY = dict(x0=-55, x1=65, y0=-62, y1=62, z0=30, z1=BZ1 + 5)
SLED = dict(x0=-100, x1=-60, y0=-62, y1=62, z0=30)


def shell_upper():
    outer = envelope()
    inner = envelope(-WALL)
    shell = outer - inner
    shell = shell - box(-200, 200, -200, 200, -200, SPLIT_Z)         # keep upper part
    shell = shell - hip_cutouts()
    for (px, pz) in end_stop_points():                  # clearance for the end-stop pegs
        shell = shell - cyl(5.5, 2 * BY + 40, "y").translate([px, 0, pz])
    # roof openings: payload bay + electronics cartridge
    shell = shell - box(BAY["x0"], BAY["x1"], BAY["y0"], BAY["y1"], 100, 300)
    shell = shell - box(SLED["x0"], SLED["x1"], SLED["y0"], SLED["y1"], 100, 300)
    # ---- face (front, x = +BX) ----
    eye_z, eye_y = 85.0, 46.0
    for s in (1, -1):
        shell = shell - cyl(20, 40, "x").translate([BX, s * eye_y, eye_z])      # eyes
    shell = shell - cyl(8.5, 40, "x").translate([BX, 0, 128])                   # fisheye cam
    for a in range(4):                                                          # IR LEDs
        ang = math.radians(45 + 90 * a)
        shell = shell - cyl(2.6, 40, "x").translate([BX, 15 * math.cos(ang), 128 + 15 * math.sin(ang)])
    for s in (1, -1):                                                           # mouth sonar
        shell = shell - cyl(8.5, 40, "x").translate([BX, s * 13, 38])
    # corner sonars at +-45 deg
    for s in (1, -1):
        c = cyl(8.5, 40, "x").rotate([0, 0, s * 45]).translate(
            [BX - R_SHELL * (1 - math.cos(math.radians(45))), s * (BY - R_SHELL * (1 - math.sin(math.radians(45)))), 38])
        shell = shell - c
    # ---- rear (x = -BX) ----
    shell = shell - cyl(8.5, 40, "x").translate([-BX, 0, 128])                  # rear cam
    for s in (1, -1):
        shell = shell - cyl(8.5, 40, "x").translate([-BX, s * 13, 38])          # rear sonar
    # ---- magnet bosses on the split line (inside) ----
    bosses = []
    for (x, y) in ((70, 105), (-70, 105), (70, -105), (-70, -105), (100, 0), (-100, 0)):
        bosses.append(cyl(6, 8, "z", center=False).translate([x * 0.98, y * 0.98, SPLIT_Z]))
    shell = shell + (union(bosses) ^ envelope(-0.5))
    for (x, y) in ((70, 105), (-70, 105), (70, -105), (-70, -105), (100, 0), (-100, 0)):
        shell = shell - cyl(MAGNET_D / 2, MAGNET_H, "z", center=False).translate([x * 0.98, y * 0.98, SPLIT_Z - 0.01])
    # vent slots on the sides above the hips (cooling for the motors/electronics)
    for i in range(5):
        for s in (1, -1):
            shell = shell - box(-50 + i * 22, -38 + i * 22, s * BY - 10, s * BY + 10, 70, 110)
    return shell


def shell_front():
    return shell_upper() - box(-300, 0, -300, 300, -300, 300)


def shell_rear():
    return shell_upper() - box(0, 300, -300, 300, -300, 300)


def visor():
    """Clear PETG face visor glued into the eye windows (cute eyes glow through)."""
    v = envelope(1.5) - envelope()
    v = v ^ box(BX - 30, BX + 5, -75, 75, 60, 110)
    return v


def belly_pan():
    outer = envelope()
    inner = envelope(-WALL)
    pan = (outer - inner) ^ box(-300, 300, -300, 300, -300, SPLIT_Z)
    pan = pan - hip_cutouts()
    # battery door at the rear: open slot for the battery tray
    pan = pan - box(-BX - 10, -60, -34, 34, BZ0 + 2, -2)
    # skid ribs (TPU skid glued below)
    for s in (1, -1):
        pan = pan + (box(-80, 80, s * 40 - 3, s * 40 + 3, BZ0 - 1, BZ0 + 4) ^ envelope(-0.2))
    # drain/vent holes
    for i in range(-3, 4):
        pan = pan - cyl(3, 20, "z").translate([i * 20, 0, BZ0])
    for (x, y) in ((70, 105), (-70, 105), (70, -105), (-70, -105), (100, 0), (-100, 0)):
        boss = cyl(6, 10, "z", center=False).translate([x * 0.98, y * 0.98, SPLIT_Z - 10]) ^ envelope(-0.5)
        pan = pan + boss
        pan = pan - cyl(MAGNET_D / 2, MAGNET_H, "z", center=False).translate(
            [x * 0.98, y * 0.98, SPLIT_Z - MAGNET_H + 0.01])
    return pan


def end_stop_points():
    """Where the thighs rest when the legs are folded to the calibration pose
    (L = L_min - 5 mm, theta = 0) -> firmware CAL_PHI1 / CAL_PHI4."""
    p1, p4 = kinematics.inverse(P["L_min"] - 0.005, 0.0, P["thigh"], P["shin"], P["hip_spacing"])
    pts = []
    along = 40.0
    half = 26.0 - (26.0 - 13.0) / L1 * along          # half-width of the thigh outline there
    for (hx, phi, ccw) in ((L5 / 2, p1, True), (-L5 / 2, p4, False)):
        ux, uz = math.cos(phi), math.sin(phi)
        nx, nz = (-uz, ux) if ccw else (uz, -ux)       # side the thigh moves towards when folding
        pts.append((hx + along * ux + (half + 4.0) * nx, along * uz + (half + 4.0) * nz))
    return pts


def side_plate(side=1):
    """Structural side plate (PETG-CF / PA-CF, 8 mm). Carries both hip motors."""
    y0 = PLATE_Y0 if side > 0 else -PLATE_Y0 - PLATE_T
    plate = rounded_plate_xz(-85, 85, BZ0 + 8, 88, 14, y0, PLATE_T)
    for hx in (L5 / 2, -L5 / 2):
        plate = plate - cyl(21, 40, "y").translate([hx, y0, 0])                    # rotor clearance
        plate = plate - bolt_circle(HIP_STATOR_PCD, 4, M3_CLR, 40, "y", 45, (hx, y0, 0))
    # lightening windows
    plate = plate - Manifold.batch_hull([cyl(10, 40, "y").translate([0, y0, 45]),
                                         cyl(10, 40, "y").translate([0, y0, -40])])
    for x in (-62, 62):
        plate = plate - Manifold.batch_hull([cyl(7, 40, "y").translate([x, y0, 50]),
                                             cyl(7, 40, "y").translate([x, y0, -38])])
    # calibration end-stop pegs (thighs rest on them when the legs are folded)
    for (px, pz) in end_stop_points():
        y_peg0 = y0 + PLATE_T if side > 0 else y0 - (THIGH_Y + 8 - PLATE_Y0 - PLATE_T)
        plate = plate + cyl(4.0, THIGH_Y + 8 - PLATE_Y0 - PLATE_T, "y", center=False).translate([px, y_peg0, pz])
    # edge holes for floor / top frame (heat-set inserts go into the frame parts)
    for x in (-70, -25, 25, 70):
        plate = plate - cyl(M3_CLR / 2, 40, "y").translate([x, y0, BZ0 + 16])
        plate = plate - cyl(M3_CLR / 2, 40, "y").translate([x, y0, 78])
    return plate


def floor_plate():
    """Floor of the chassis with integrated battery rails + side flanges."""
    z0 = BZ0 + 8
    base = box(-85, 85, -PLATE_Y0, PLATE_Y0, z0, z0 + 5)
    flanges = []
    for s in (1, -1):
        flanges.append(box(-85, 85, s * PLATE_Y0 - (8 if s > 0 else 0), s * PLATE_Y0 + (0 if s > 0 else 8), z0, z0 + 16))
    rails = [box(-85, 85, s * 30 - 2, s * 30 + 2, z0 + 5, z0 + 11) for s in (1, -1)]
    stop = box(78, 85, -30, 30, z0 + 5, z0 + 20)                       # front stop
    m = union([base] + flanges + rails + [stop])
    for x in (-70, -25, 25, 70):
        for s in (1, -1):
            m = m - cyl(M3_INSERT / 2, 30, "y").translate([x, s * PLATE_Y0, BZ0 + 16])
    for x in range(-60, 61, 30):          # weight relief
        for y in (-75, 75):
            m = m - cyl(10, 30, "z").translate([x, y, z0])
    return m


def top_frame():
    z0 = 72
    m = box(-85, 85, -PLATE_Y0, PLATE_Y0, z0, z0 + 6)
    for s in (1, -1):
        m = m + box(-85, 85, s * PLATE_Y0 - (8 if s > 0 else 0), s * PLATE_Y0 + (0 if s > 0 else 8), z0 - 2, 88)
    m = m - box(BAY["x0"] + 3, BAY["x1"] - 3, BAY["y0"] + 3, BAY["y1"] - 3, 0, 200)     # payload bay drop-in
    m = m - box(SLED["x0"] + 12, SLED["x1"], SLED["y0"], SLED["y1"], 0, 200)              # sled slot
    # sled guide fingers
    for s in (1, -1):
        m = m + box(-85, -60, s * 66 - 4, s * 66 + 4, z0 - 30, z0 + 6)
        m = m - box(-82, -78, s * 66 - 5, s * 66 + 5, z0 - 31, z0 + 7)        # guide slot (sled edge 4 mm)
    for x in (-70, -25, 25, 70):
        for s in (1, -1):
            m = m - cyl(M3_INSERT / 2, 30, "y").translate([x, s * PLATE_Y0, 78])
    for (x, y) in ((40, 90), (-40, 90), (40, -90), (-40, -90)):
        m = m - cyl(M3_INSERT / 2, 20, "z").translate([x, y, z0])                         # roof-rack inserts
    return m


def battery_tray():
    """Slide-in tray for a 6S 5000 mAh pack (155x52x46). Snap latch at the rear."""
    z0 = BZ0 + 13
    m = box(-85, 78, -28, 28, z0, z0 + 3)
    for s in (1, -1):
        m = m + box(-85, 78, s * 28 - (3 if s > 0 else 0), s * 28 + (0 if s > 0 else 3), z0, z0 + 25)
    m = m + box(-90, -85, -34, 34, z0 - 3, z0 + 45)                          # rear door / handle
    m = m - box(-91, -84, -10, 10, z0 + 30, z0 + 40)                           # finger hole
    m = m - box(-91, -84, -8, 8, z0 + 8, z0 + 24)                              # XT60 pass-through
    for x in (-50, 0, 50):                                                     # strap slots
        m = m - box(x - 10, x + 10, -40, 40, z0 + 8, z0 + 12)
    # latch tongue (flexes, catches the belly pan edge)
    m = m + box(-95, -90, -6, 6, z0 + 10, z0 + 42)
    m = m + box(-100, -95, -6, 6, z0 + 38, z0 + 42)
    return m


def payload_bay():
    b = BAY
    outer = box(b["x0"], b["x1"], b["y0"], b["y1"], b["z0"], 148)
    inner = box(b["x0"] + 2.4, b["x1"] - 2.4, b["y0"] + 2.4, b["y1"] - 2.4, b["z0"] + 2.4, 200)
    m = outer - inner
    m = m + (box(b["x0"] - 6, b["x1"] + 6, b["y0"] - 6, b["y1"] + 6, 78, 81) -
             box(b["x0"] + 1, b["x1"] - 1, b["y0"] + 1, b["y1"] - 1, 0, 200))    # rests on top frame
    for x in (b["x0"] + 25, b["x1"] - 25):                                        # tie-down eyelets
        for s in (1, -1):
            m = m - cyl(2.5, 10, "y").translate([x, s * (b["y1"] - 1), b["z0"] + 10])
    for i in range(4):
        m = m - box(b["x0"] + 15 + i * 25, b["x0"] + 30 + i * 25, -40, 40, b["z0"] - 1, b["z0"] + 3)  # drain
    return m


def payload_lid():
    b = BAY
    lid = envelope() ^ box(b["x0"] - 4, b["x1"] + 4, b["y0"] - 4, b["y1"] + 4, BZ1 - 30, 300)
    lid = lid - envelope(-WALL) - box(-300, 300, -300, 300, -300, 148)
    lid = lid + box(b["x0"] + 0.4, b["x1"] - 0.4, b["y0"] + 0.4, b["y1"] - 0.4, 144, 150) - \
        box(b["x0"] + 2.6, b["x1"] - 2.6, b["y0"] + 2.6, b["y1"] - 2.6, 140, 152)
    lid = lid - cyl(MAGNET_D / 2, 30, "z").translate([b["x1"] - 10, 0, 150])
    lid = lid - box(b["x0"] - 5, b["x0"] + 8, -15, 15, BZ1 - 6, 300)            # finger notch
    return lid


def electronics_sled():
    """Vertical, tool-less cartridge: Raspberry Pi 5 on the front face,
    Teensy carrier + power board on the rear face.  Pull up by the handle."""
    x0 = -82
    m = box(x0, x0 + 4, -61, 61, SLED["z0"], 150)
    m = m + box(x0 - 2, x0 + 6, -61, 61, SLED["z0"], SLED["z0"] + 6)           # bottom stiffener / connector ledge
    m = m + box(x0 - 2, x0 + 6, -61, -57, SLED["z0"], 150) + box(x0 - 2, x0 + 6, 57, 61, SLED["z0"], 150)
    # Raspberry Pi 5 holes (58 x 49, M2.5) on +x face, standoffs
    for (y, z) in ((-29, 52), (29, 52), (-29, 101), (29, 101)):
        m = m + cyl(3.2, 6, "x", center=False).translate([x0 + 4, y, z])
        m = m - cyl(1.15, 30, "x").translate([x0, y, z])
    # Teensy carrier board (70 x 55, M3) on -x face
    for (y, z) in ((-32, 45), (32, 45), (-32, 97), (32, 97)):
        m = m + cyl(3.5, 6, "x", center=False).translate([x0 - 6, y, z])
        m = m - cyl(1.4, 30, "x").translate([x0, y, z])
    # thumb-screw holes: the sled is CLAMPED rigidly to the top frame (the IMU sits on
    # the Teensy carrier and must see the true body motion - no rubber here)
    for (y, z) in ((-12, 125), (12, 125)):
        m = m - cyl(1.7, 30, "x").translate([x0, y, z])
    # cable-tie slots
    for z in (70, 120):
        for y in (-50, 50):
            m = m - box(x0 - 3, x0 + 7, y - 2, y + 2, z, z + 5)
    return m


def electronics_cap():
    """Roof cap over the sled slot with the GPS 'antenna' mast (cute ahoge)."""
    cap = envelope() ^ box(SLED["x0"] - 4, SLED["x1"] + 4, SLED["y0"] - 4, SLED["y1"] + 4, BZ1 - 40, 300)
    cap = cap - envelope(-WALL) - box(-300, 300, -300, 300, -300, 150)
    cap = cap + box(SLED["x0"] + 0.4, SLED["x1"] - 0.4, SLED["y0"] + 0.4, SLED["y1"] - 0.4, 146, 152) - \
        box(SLED["x0"] + 2.6, SLED["x1"] - 2.6, SLED["y0"] + 2.6, SLED["y1"] - 2.6, 140, 154)
    # mast: slightly curved, tilted back
    # short stub (keeps the crawl height low) with the GPS puck seat on top
    mast = Manifold.batch_hull([cyl(9, 2, "z").translate([-80, 0, BZ1 - 6]),
                                cyl(7, 2, "z").translate([-82, 0, BZ1 + 18])])
    cap = cap + mast
    cap = cap + cyl(26, 4, "z").translate([-82, 0, BZ1 + 20])                    # GPS puck seat (M10 puck, 50 mm)
    cap = cap - cyl(3, 200, "z").translate([-81, 0, BZ1])                       # cable bore
    cap = cap - cyl(MAGNET_D / 2, 30, "z").translate([SLED["x0"] + 10, 0, 152])
    return cap


def bumper(front=True):
    """TPU 95A crash bumper wrapping the lower face corners ('cheeks')."""
    band = envelope(7) - envelope(0.2)
    band = band ^ box(-300, 300, -300, 300, 18, 58)
    band = band ^ (box(BX - 50, 300, -300, 300, -300, 300) if front else box(-300, -BX + 50, -300, 300, -300, 300))
    for s in (1, -1):
        band = band - cyl(9, 60, "x").translate([0, s * 13, 38])
        if front:
            band = band - cyl(9, 80, "x").rotate([0, 0, s * 45]).translate([BX - 12, s * (BY - 12), 38])
    band = band - hip_cutouts()
    return band


def camera_mount():
    """Bracket for an Arducam 220-degree M12 fisheye board (25x24, M2 21x21?):
    tilted 10 deg down. Print 2 (front + rear)."""
    m = box(0, 4, -16, 16, -16, 16)
    for y in (-10.5, 10.5):
        for z in (-10.5, 10.5):
            m = m - cyl(1.1, 20, "x").translate([0, y, z])
    m = m - cyl(8, 20, "x")
    foot = box(-20, 0, -16, 16, -18, -14).rotate([0, -10, 0])
    m = m + foot
    for y in (-10, 10):
        m = m - cyl(1.6, 20, "z").translate([-12, y, -16])
    return m


def sonar_bracket():
    """HC-SR04 holder (board 45x20, transducers 16 mm, 26 mm apart)."""
    m = box(0, 3, -24, 24, -12, 12)
    for y in (-13, 13):
        m = m - cyl(8.2, 10, "x").translate([0, y, 0])
    m = m + box(-12, 0, -24, -20, -12, 12) + box(-12, 0, 20, 24, -12, 12)
    for y in (-22, 22):
        m = m - cyl(1.6, 30, "z").translate([-6, y, 0])
    return m


# ---------------------------------------------------------------------------
# LEGS (built flat in the x-y plane = print orientation)
# ---------------------------------------------------------------------------
def thigh():
    prof = link2d(L1, 26, 13, 20)
    m = Manifold.extrude(prof, THIGH_T)
    m = m - cyl(10, 40, "z").translate([0, 0, 0])                              # flange spigot
    m = m - bolt_circle(HIP_PCD, HIP_BOLTS, M3_CLR, 40, "z", 45, (0, 0, 0))
    m = m - cyl(PIN_D / 2, 40, "z").translate([L1, 0, 0])                       # knee pin (press-fit M8)
    m = m - Manifold.batch_hull([cyl(5, 40, "z").translate([38, 0, 0]), cyl(4, 40, "z").translate([L1 - 25, 0, 0])])
    # counter-bores for bolt heads
    m = m - bolt_circle(HIP_PCD, HIP_BOLTS, 6.0, 6, "z", 45, (0, 0, THIGH_T))
    return m


def shin(rear=False):
    prof = link2d(L2, 14, 34 if rear else 14, 20)
    m = Manifold.extrude(prof, SHIN_T)
    m = m - cyl(BEARING_OD / 2, BEARING_W * 2, "z").translate([0, 0, SHIN_T])  # knee 608 bearing pocket
    m = m - cyl(PIN_D / 2 + 0.6, 40, "z")
    m = m - Manifold.batch_hull([cyl(5, 40, "z").translate([30, 0, 0]), cyl(5, 40, "z").translate([L2 - 50, 0, 0])])
    if rear:
        # foot carries the wheel motor stator + an M8 hollow axle stub for the front shin
        m = m + cyl(16, 12, "z", center=False).translate([L2, 0, -12])        # axle boss (inboard)
        m = m - cyl(5.2, 60, "z").translate([L2, 0, 0])                         # cable bore
        m = m - bolt_circle(WHEEL_STATOR_PCD, 4, 4.4, 60, "z", 45, (L2, 0, 0))
    else:
        m = m - cyl(16.3, 40, "z").translate([L2, 0, 0])                       # rides on rear-shin boss
        m = m + (cyl(22, SHIN_T, "z", center=False).translate([L2, 0, 0]) - cyl(16.3, 40, "z").translate([L2, 0, 0]))
    return m


def wheel_rim():
    """Cup-shaped rim bolted ONLY to the MF9025 output face; the motor body sits
    inside the cup with 1.5 mm radial clearance.  Tyre is separate TPU."""
    rim_r = WR - 15
    m = cyl(rim_r, WW - 4, "z")
    m = m - cyl(49.5, WW, "z").translate([0, 0, 3])                            # motor pocket (96 mm motor)
    for i in range(12):                                                         # lightening pockets in the ring
        a = math.radians(15 + 30 * i)
        m = m - cyl(2.4, WW - 10, "z").translate([55 * math.cos(a), 55 * math.sin(a), 1])
    m = m - cyl(18, 40, "z")                                                     # centre bore
    m = m - bolt_circle(WHEEL_ROTOR_PCD, 6, M3_CLR, 40, "z", 0, (0, 0, 0))
    # tyre retaining lips
    m = m + (cyl(rim_r + 3, 2.5, "z").translate([0, 0, (WW - 4) / 2 - 1.25]) - cyl(rim_r - 1, 10, "z"))
    m = m + (cyl(rim_r + 3, 2.5, "z").translate([0, 0, -(WW - 4) / 2 + 1.25]) - cyl(rim_r - 1, 10, "z"))
    return m


def tyre():
    """TPU 95A tyre (or 85A for more grip), chevron tread."""
    m = cyl(WR, WW, "z") - cyl(WR - 15 + 0.3, WW + 2, "z")
    m = m - (cyl(WR - 11.8, WW - 3.6, "z") - cyl(WR - 17, WW, "z"))            # grooves for rim lips
    n = 36
    grooves = []
    for i in range(n):
        a = 360.0 * i / n
        g = box(WR - 2.5, WR + 2, -1.2, 1.2, -WW / 2 - 1, -2).rotate([0, 0, a])
        g2 = box(WR - 2.5, WR + 2, -1.2, 1.2, 2, WW / 2 + 1).rotate([0, 0, a + 5])
        grooves += [g, g2]
    m = m - union(grooves)
    return m


# ---------------------------------------------------------------------------
# mass properties
# ---------------------------------------------------------------------------
def to_trimesh(m):
    mesh = m.to_mesh()
    return trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3],
                           faces=np.asarray(mesh.tri_verts), process=False)


MATERIAL = {      # g/cm^3 effective density (material density x fill factor)
    "PETG": 1.27 * 0.55,
    "PETG-CF": 1.30 * 0.65,
    "PA-CF": 1.15 * 0.70,
    "TPU": 1.21 * 0.60,
    "PETG-clear": 1.27 * 1.0,
}

# part name -> (builder, qty, material, print note)
PARTS = {
    "shell_front":      (shell_front, 1, "PETG", "split face on bed, 3 walls, 15% gyroid, tree supports (eye/camera holes)"),
    "shell_rear":       (shell_rear, 1, "PETG", "split face on bed, 3 walls, 15% gyroid, tree supports"),
    "belly_pan":        (belly_pan, 1, "PETG-CF", "split face on bed, 4 walls, 25% gyroid"),
    "visor":            (visor, 1, "PETG-clear", "100% infill, 0.12 mm layers, glue into eye windows"),
    "side_plate_L":     (lambda: side_plate(1), 1, "PA-CF", "flat, 6 walls, 40% gyroid (structural)"),
    "side_plate_R":     (lambda: side_plate(-1), 1, "PA-CF", "flat, 6 walls, 40% gyroid (structural)"),
    "floor_plate":      (floor_plate, 1, "PA-CF", "flat, 5 walls, 40% gyroid"),
    "top_frame":        (top_frame, 1, "PA-CF", "upside-down, 5 walls, 35% gyroid"),
    "battery_tray":     (battery_tray, 1, "PETG", "flat, 4 walls, 30%"),
    "payload_bay":      (payload_bay, 1, "PETG", "open side up, 3 walls, 20%"),
    "payload_lid":      (payload_lid, 1, "PETG", "outer face down on textured PEI"),
    "electronics_sled": (electronics_sled, 1, "PETG-CF", "flat on the Pi side, 4 walls, 30%"),
    "electronics_cap":  (electronics_cap, 1, "PETG", "upright, supports under puck seat"),
    "bumper_front":     (lambda: bumper(True), 1, "TPU", "TPU 95A, 3 walls, 25% gyroid, slow 25 mm/s"),
    "bumper_rear":      (lambda: bumper(False), 1, "TPU", "TPU 95A, 3 walls, 25% gyroid"),
    "camera_mount":     (camera_mount, 2, "PETG", "flat"),
    "sonar_bracket":    (sonar_bracket, 4, "PETG", "flat"),
    "thigh":            (thigh, 4, "PA-CF", "flat, 100% infill or 8 walls (jump loads!)"),
    "shin_front":       (lambda: shin(False), 2, "PA-CF", "flat, DIAGONAL on a 256 mm bed, 8 walls, 50% gyroid"),
    "shin_rear":        (lambda: shin(True), 2, "PA-CF", "flat (boss down, supports), DIAGONAL on a 256 mm bed, 8 walls, 50%"),
    "wheel_rim":        (wheel_rim, 2, "PETG-CF", "flat, 6 walls, 40%"),
    "tyre":             (tyre, 2, "TPU", "TPU 95A (85A for grip), 4 walls, 20% gyroid"),
}


def assembly_transforms():
    """Place leg parts at the nominal standing pose (L = L_nom, theta = 0)."""
    p1, p4 = kinematics.inverse(P["L_nom"], 0.0, P["thigh"], P["shin"], P["hip_spacing"])
    fk = kinematics.forward(p1, p4, P["thigh"], P["shin"], P["hip_spacing"])
    B = np.array(fk["B"]) * 1000
    D = np.array(fk["D"]) * 1000
    C = np.array(fk["C"]) * 1000
    return p1, p4, B, D, C


def leg_part_pose(part, length, start_xz, end_xz, y):
    """part is built along +x in x-y plane with thickness along +z.
    Rotate so x->direction in x-z plane and z -> +y."""
    d = np.array(end_xz) - np.array(start_xz)
    ang = math.degrees(math.atan2(d[1], d[0]))
    m = part.rotate([90, 0, 0])            # (x,y,z) -> (x,-z,y): thickness now along -y
    m = m.rotate([0, -ang, 0])             # rotate in x-z plane
    return m.translate([start_xz[0], y + (THIGH_T if False else 0), start_xz[1]])


def build_assembly(parts):
    p1, p4, B, D, C = assembly_transforms()
    A = np.array([L5 / 2, 0])
    E = np.array([-L5 / 2, 0])
    pieces = []
    for name in ("shell_front", "shell_rear", "belly_pan", "visor", "side_plate_L", "side_plate_R",
                 "floor_plate", "top_frame", "battery_tray", "payload_bay", "payload_lid",
                 "electronics_sled", "electronics_cap", "bumper_front", "bumper_rear"):
        pieces.append((name, parts[name]))
    for s in (1, -1):
        def place(part, start, end, y, thick):
            m = leg_part_pose(part, None, start, end, 0)
            # after rotate([90,0,0]) thickness spans y in [-thick, 0]
            if s > 0:
                return m.translate([0, y + thick, 0])
            return m.mirror([0, 1, 0]).translate([0, -(y + thick), 0])
        pieces.append(("thigh", place(parts["thigh"], A, B, THIGH_Y, THIGH_T)))
        pieces.append(("thigh", place(parts["thigh"], E, D, THIGH_Y, THIGH_T)))
        pieces.append(("shin_front", place(parts["shin_front"], B, C, FRONT_SHIN_Y, SHIN_T)))
        pieces.append(("shin_rear", place(parts["shin_rear"], D, C, REAR_SHIN_Y, SHIN_T)))
        wheel = (parts["wheel_rim"] + parts["tyre"]).rotate([90, 0, 0]).translate([C[0], s * WHEEL_Y, C[1]])
        pieces.append(("wheel", wheel))
        motor_w = cyl(48, 34, "y").translate([C[0], s * (WHEEL_Y - 2), C[1]])
        pieces.append(("motor_MF9025", motor_w))
        for hx in (L5 / 2, -L5 / 2):
            pieces.append(("motor_DM4310", cyl(28, 46, "y").translate([hx, s * (PLATE_Y0 - 23), 0])))
    return pieces


def main():
    out_dir = os.path.join(HERE, "stl")
    os.makedirs(out_dir, exist_ok=True)

    parts, report = {}, {}
    total_printed = 0.0
    for name, (fn, qty, mat, note) in PARTS.items():
        m = fn()
        parts[name] = m
        tm = to_trimesh(m)
        vol_cm3 = abs(m.volume()) / 1000.0
        grams = vol_cm3 * MATERIAL[mat]
        total_printed += grams * qty
        bb = tm.bounds
        report[name] = dict(qty=qty, material=mat, volume_cm3=round(vol_cm3, 1),
                            est_mass_g=round(grams, 1), bbox_mm=[round(x, 1) for x in (bb[1] - bb[0])],
                            watertight=bool(tm.is_watertight), print=note)
    # write STLs in print orientation (lay large flat face on the bed)
    print_rot = {"shell_front": [0, 90, 0], "shell_rear": [0, -90, 0], "belly_pan": [180, 0, 0],
                 "side_plate_L": [90, 0, 0], "side_plate_R": [-90, 0, 0], "top_frame": [180, 0, 0],
                 "payload_lid": [180, 0, 0], "electronics_sled": [0, 90, 0], "camera_mount": [0, -90, 0],
                 "sonar_bracket": [0, 90, 0], "visor": [0, 90, 0], "electronics_cap": [0, 0, 0]}
    for name, m in parts.items():
        pm = m.rotate(print_rot.get(name, [0, 0, 0]))
        bb = pm.bounding_box()
        pm = pm.translate([-(bb[0] + bb[3]) / 2, -(bb[1] + bb[4]) / 2, -bb[2]])
        to_trimesh(pm).export(os.path.join(out_dir, f"{name}.stl"))
        report[name]["print_bbox_mm"] = [round(x, 1) for x in (np.array(pm.bounding_box()[3:]) - np.array(pm.bounding_box()[:3]))]

    pieces = build_assembly(parts)
    asm = trimesh.util.concatenate([to_trimesh(m) for _, m in pieces])
    asm.export(os.path.join(HERE, "assembly_nominal_pose.stl"))

    # mass / COM of the body shell + chassis prints (for the simulator)
    body_names = ["shell_front", "shell_rear", "belly_pan", "visor", "side_plate_L", "side_plate_R",
                  "floor_plate", "top_frame", "battery_tray", "payload_bay", "payload_lid",
                  "electronics_sled", "electronics_cap", "bumper_front", "bumper_rear"]
    msum, mcom = 0.0, np.zeros(3)
    for n in body_names:
        tm = to_trimesh(parts[n])
        g = report[n]["est_mass_g"] * report[n]["qty"]
        msum += g
        mcom += g * np.array(tm.center_mass if tm.is_watertight else tm.centroid)
    body_com = (mcom / msum / 1000).tolist()
    leg_g = sum(report[n]["est_mass_g"] * report[n]["qty"] for n in ("thigh", "shin_front", "shin_rear")) / 2
    wheel_g = report["wheel_rim"]["est_mass_g"] + report["tyre"]["est_mass_g"]
    summary = dict(printed_total_g=round(total_printed, 0), body_prints_g=round(msum, 0),
                   body_prints_com_m=[round(x, 4) for x in body_com],
                   per_leg_prints_g=round(leg_g, 0), per_wheel_prints_g=round(wheel_g, 0))
    json.dump(dict(parts=report, summary=summary), open(os.path.join(HERE, "mass_report.json"), "w"), indent=2)

    # body-frame meshes for the simulator / renderer (metres)
    asm_dir = os.path.join(HERE, "assembly")
    os.makedirs(asm_dir, exist_ok=True)
    for name in body_names:
        tm = to_trimesh(parts[name]); tm.apply_scale(0.001)
        tm.export(os.path.join(asm_dir, f"{name}.stl"))
    for name in ("thigh", "shin_front", "shin_rear", "wheel_rim", "tyre"):
        tm = to_trimesh(parts[name]); tm.apply_scale(0.001)
        tm.export(os.path.join(asm_dir, f"{name}.stl"))
    print(json.dumps(summary, indent=2))
    for n, r in report.items():
        print(f"{n:18s} x{r['qty']} {r['material']:10s} {r['est_mass_g']:7.1f} g  print bbox {r['print_bbox_mm']}  wt={r['watertight']}")


if __name__ == "__main__":
    main()
