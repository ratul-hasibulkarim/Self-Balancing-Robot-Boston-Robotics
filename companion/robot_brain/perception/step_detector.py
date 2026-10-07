"""
Distance to a step / curb / obstacle base from the front camera (ground-plane geometry).

The front camera sits ~0.13 m above the hip, looking 10 deg down.  The bottom edge of
a step face is where the floor ends: the lowest strong horizontal edge in the centre
of the image.  Its image row gives the depression angle below the horizon, and with
the known camera height:

    d_ground = h_cam / tan(depression)

The sonar can't do this job: it is mounted ~0.33 m above the floor and its beam
passes over a 15 cm step.

Camera model: pinhole for the simulator (fovy 120 deg).  For the real 220 deg fisheye,
undistort first with cv2.fisheye (calibrate once with a checkerboard) or pass `row_to_angle`.
"""
import math
from dataclasses import dataclass

import cv2
import numpy as np

CAM_X_AHEAD_OF_HIP = 0.105      # m
CAM_Z_ABOVE_HIP = 0.128         # m
CAM_TILT = math.radians(10)     # looking down


@dataclass
class StepEstimate:
    found: bool
    distance: float = 0.0       # horizontal distance camera -> edge [m]
    wheel_to_edge: float = 0.0  # what the controller wants as jump_dist [m]
    row: int = -1
    strength: float = 0.0


class StepDetector:
    def __init__(self, fovy_deg=120.0, min_strength=10.0, band=0.3):
        self.fovy = math.radians(fovy_deg)
        self.min_strength = min_strength
        self.band = band                 # use the central 30 % of columns
        self.row_to_angle = None         # optional callable(row, h) -> angle below the optical axis

    def _edges(self, bgr):
        """Rows of straight horizontal brightness edges (local maxima), bottom-up."""
        h, w = bgr.shape[:2]
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        c0, c1 = int(w * (0.5 - self.band / 2)), int(w * (0.5 + self.band / 2))
        band = gray[:, c0:c1]
        k = 2
        step = band[2 * k:, :] - band[:-2 * k, :]                 # index i <-> image row i + k
        mag = np.abs(np.median(step, axis=1))
        consistent = (np.abs(step) > 0.5 * self.min_strength).mean(axis=1)
        start = int(h * 0.52)                                      # below the horizon only
        rows = []
        for i in range(len(mag) - 2, max(start - k, 1), -1):
            if mag[i] > self.min_strength and consistent[i] > 0.75 and mag[i] >= mag[i - 1] and mag[i] >= mag[i + 1]:
                rows.append((i + k, float(mag[i])))
        return rows

    def _depression(self, r, h, body_pitch):
        f = (h / 2) / math.tan(self.fovy / 2)
        alpha = self.row_to_angle(r, h) if self.row_to_angle else math.atan((r + 0.5 - h / 2) / f)
        return CAM_TILT + body_pitch + alpha

    def estimate(self, bgr, leg_length, wheel_radius=0.075, body_pitch=0.0, step_height=None) -> StepEstimate:
        """Distance to the base of the nearest step/obstacle ahead.
        With step_height known (operator slider / mission item) the riser's top and bottom
        edges are matched as a pair, which removes the top-vs-base ambiguity."""
        h = bgr.shape[0]
        h_cam = leg_length + wheel_radius + CAM_Z_ABOVE_HIP
        edges = self._edges(bgr)
        if not edges:
            return StepEstimate(False)
        def dist(r, z):
            dep = self._depression(r, h, body_pitch)
            return (h_cam - z) / math.tan(dep) if dep > math.radians(2) and h_cam > z else None
        if step_height:
            best = None
            for rl, ml in edges:                          # lower edge = base (floor level)
                db = dist(rl, 0.0)
                for ru, mu in edges:                      # upper edge = top of the riser
                    if ru >= rl:
                        continue
                    dt = dist(ru, step_height)
                    if db and dt and abs(db - dt) < 0.12 * db + 0.03:
                        d = 0.5 * (db + dt)
                        # the riser's own edges are the high-contrast ones; faint floor
                        # texture (tile joints, cracks) must not win
                        if best is None or ml + mu > best[2]:
                            best = (d, rl, ml + mu)
            if best:
                d, r, m = best
                return StepEstimate(True, d, d + CAM_X_AHEAD_OF_HIP, r, m)
            # single edge: assume it is the top edge (gives the SHORTER distance -> jumps early, never into the riser)
            r, m = max(edges, key=lambda e: e[1])
            d = dist(r, step_height)
            return StepEstimate(d is not None, d or 0.0, (d or 0.0) + CAM_X_AHEAD_OF_HIP, r, m)
        r, m = edges[0]                                   # nearest edge, assumed at floor level
        d = dist(r, 0.0)
        return StepEstimate(d is not None, d or 0.0, (d or 0.0) + CAM_X_AHEAD_OF_HIP, r, m)
