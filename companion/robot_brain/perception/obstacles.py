"""
Obstacle perception from the ultrasonic ring + Sharp IR cliff sensors.

Sonars: FL (+45 deg), F (0), FR (-45 deg), Rear (180 deg), ~25 deg cones, 0.03-4 m.
Each reading is filtered (median of 5) and turned into a polar "free space"
histogram used by the local planner.  Cliff sensors look down/forward and
report the distance to the floor; a sudden increase means a drop-off.
"""
import collections
import math
from dataclasses import dataclass

SONAR_ANGLES = {"fl": math.radians(45), "f": 0.0, "fr": math.radians(-45), "r": math.pi}
SONAR_ORDER = ("fl", "f", "fr", "r")
MAX_RANGE = 4.0


@dataclass
class ObstacleView:
    ranges: dict              # name -> filtered range [m] (MAX_RANGE if nothing)
    front_min: float
    rear: float
    cliff: bool               # drop-off ahead
    step_ahead: bool          # something low and close ahead (candidate step / curb)


class ObstacleTracker:
    # cliff sensors sit 0.058 m below the hip axis, 30 deg forward of vertical
    SENSOR_BELOW_HIP = 0.058
    WHEEL_R = 0.075

    def __init__(self, cliff_margin=0.08):
        self.hist = {k: collections.deque(maxlen=5) for k in SONAR_ORDER}
        self.cliff_margin = cliff_margin

    def expected_floor(self, leg_length):
        """distance the cliff sensors should read on flat ground at this ride height"""
        return (leg_length + self.WHEEL_R - self.SENSOR_BELOW_HIP) / math.cos(math.radians(30))

    def update(self, sonar_m, cliff_m, leg_length=0.22) -> ObstacleView:
        floor = self.expected_floor(leg_length)
        out = {}
        for k, r in zip(SONAR_ORDER, sonar_m):
            self.hist[k].append(r if r > 0.02 else MAX_RANGE)
            vals = sorted(self.hist[k])
            out[k] = vals[len(vals) // 2]
        cliff = all(c > floor + self.cliff_margin for c in cliff_m)
        step = any(c < floor - 0.06 for c in cliff_m)
        return ObstacleView(out, min(out["fl"], out["f"], out["fr"]), out["r"], cliff, step)

    @staticmethod
    def points(view: ObstacleView):
        """Obstacle points in the robot frame (x fwd, y left) for the local map."""
        pts = []
        for k, r in view.ranges.items():
            if r < MAX_RANGE - 0.05:
                a = SONAR_ANGLES[k]
                for da in (-0.2, 0.0, 0.2):        # spread over the cone
                    pts.append((r * math.cos(a + da), r * math.sin(a + da)))
        return pts
