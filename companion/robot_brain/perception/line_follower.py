"""
Coloured-line follower (vision).  Works with the front fisheye camera looking
~10 deg down; only the lower part of the image (the floor just ahead) is used.

    lf = LineFollower(color="red")
    steer = lf.process(bgr_frame)      # -> LineResult(found, offset, angle, ...)

Colours are HSV ranges; add your own in COLORS or pass hsv_ranges=[...].
In the dark the robot switches its headlight on; for very dark scenes the
IR illuminator + NoIR camera still sees high-contrast tape (use "dark"/"white").
"""
from dataclasses import dataclass

import cv2
import numpy as np

COLORS = {   # list of (low HSV, high HSV) - OpenCV hue is 0..180
    "red":    [((0, 90, 60), (10, 255, 255)), ((170, 90, 60), (180, 255, 255))],
    "orange": [((10, 100, 80), (22, 255, 255))],
    "yellow": [((22, 80, 80), (35, 255, 255))],
    "green":  [((40, 70, 50), (85, 255, 255))],
    "blue":   [((95, 90, 50), (130, 255, 255))],
    "black":  [((0, 0, 0), (180, 255, 60))],
    "white":  [((0, 0, 190), (180, 40, 255))],
}


@dataclass
class LineResult:
    found: bool = False
    offset: float = 0.0       # -1 (line far left) .. +1 (far right), at the near row
    angle: float = 0.0        # rad, + = line bends to the right ahead
    coverage: float = 0.0     # fraction of ROI pixels on the line
    junction: bool = False    # line much wider than usual (crossing / T)


class LineFollower:
    def __init__(self, color="red", hsv_ranges=None, roi_top=0.55, min_coverage=0.004):
        self.ranges = hsv_ranges or COLORS[color]
        self.roi_top = roi_top
        self.min_coverage = min_coverage
        self.kp, self.kd, self.k_ang = 6.0, 0.3, 0.4      # tuned in sim/companion_sim.py (line demo)
        self._prev = 0.0
        self.last_mask = None

    def set_color(self, color):
        self.ranges = COLORS[color]

    def mask(self, bgr):
        h = bgr.shape[0]
        roi = bgr[int(h * self.roi_top):, :]
        hsv = cv2.cvtColor(cv2.GaussianBlur(roi, (5, 5), 0), cv2.COLOR_BGR2HSV)
        m = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in self.ranges:
            m |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        self.last_mask = m
        return m

    def process(self, bgr) -> LineResult:
        m = self.mask(bgr)
        hh, ww = m.shape
        cov = float(np.count_nonzero(m)) / m.size
        if cov < self.min_coverage:
            return LineResult(found=False, coverage=cov)
        # centroid in a near band and a far band -> offset and heading of the line
        def centroid(band):
            ys, xs = np.nonzero(band)
            return (xs.mean() / ww * 2 - 1) if len(xs) > 15 else None
        near = centroid(m[int(hh * 0.6):, :])
        far = centroid(m[: int(hh * 0.4), :])
        if near is None and far is None:
            return LineResult(found=False, coverage=cov)
        if near is None:
            near = far
        if far is None:
            far = near
        angle = float(np.arctan2(far - near, 0.8))
        widths = np.count_nonzero(m, axis=1)
        junction = widths.max() > 0.5 * ww
        return LineResult(True, float(near), angle, cov, bool(junction))

    def steer(self, res: LineResult, dt=0.05):
        """Yaw-rate command (rad/s, + = turn left) from a LineResult."""
        if not res.found:
            return 0.0
        err = res.offset + self.k_ang * res.angle
        d = (err - self._prev) / dt
        self._prev = err
        return -(self.kp * err + self.kd * d)
