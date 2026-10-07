"""
Navigation: shortest-path planning (A*) on
  * a road network (GeoJSON from OpenStreetMap / drawn in QGIS)  -> global route between GPS waypoints
  * a rolling occupancy grid built from the sonars                 -> detours around obstacles
plus a pure-pursuit path follower and the "no alternative path -> HALT" logic.
"""
import heapq
import json
import math
from dataclasses import dataclass, field

import numpy as np


# ====================================================================== A* on a grid
class OccupancyGrid:
    """Rolling log-odds grid in the local ENU frame (metres)."""

    def __init__(self, size_m=40.0, res=0.10, origin=(0.0, 0.0)):
        self.res = res
        self.n = int(size_m / res)
        self.origin = np.array(origin, float) - size_m / 2
        self.logodds = np.zeros((self.n, self.n), np.float32)

    def recenter(self, x, y):
        """Keep the robot inside the grid (shift by whole cells when it nears the border)."""
        cx, cy = self.idx(x, y)
        m = self.n // 4
        if m < cx < self.n - m and m < cy < self.n - m:
            return
        sx, sy = cx - self.n // 2, cy - self.n // 2
        self.logodds = np.roll(self.logodds, (-sx, -sy), axis=(0, 1))
        if sx > 0: self.logodds[-sx:, :] = 0
        if sx < 0: self.logodds[:-sx, :] = 0
        if sy > 0: self.logodds[:, -sy:] = 0
        if sy < 0: self.logodds[:, :-sy] = 0
        self.origin += np.array([sx, sy]) * self.res

    def idx(self, x, y):
        return int((x - self.origin[0]) / self.res), int((y - self.origin[1]) / self.res)

    def xy(self, i, j):
        return self.origin[0] + (i + 0.5) * self.res, self.origin[1] + (j + 0.5) * self.res

    def inside(self, i, j):
        return 0 <= i < self.n and 0 <= j < self.n

    def integrate(self, robot_xy, robot_yaw, hits_robot_frame, free_rays=()):
        """hits: obstacle points (x fwd, y left) in the robot frame; free_rays: (angle, range)."""
        c, s = math.cos(robot_yaw), math.sin(robot_yaw)
        rx, ry = robot_xy
        for a, r in free_rays:                         # clear along each ray
            for k in np.arange(0.2, max(r - 0.15, 0.2), self.res):
                i, j = self.idx(rx + k * math.cos(robot_yaw + a), ry + k * math.sin(robot_yaw + a))
                if self.inside(i, j):
                    self.logodds[i, j] = max(self.logodds[i, j] - 0.4, -4)
        for px, py in hits_robot_frame:
            wx, wy = rx + c * px - s * py, ry + s * px + c * py
            i, j = self.idx(wx, wy)
            if self.inside(i, j):
                self.logodds[i, j] = min(self.logodds[i, j] + 0.9, 6)

    def blocked(self, inflate_m=0.28):
        occ = self.logodds > 1.0
        k = int(math.ceil(inflate_m / self.res))
        if k <= 0 or not occ.any():
            return occ
        out = occ.copy()
        ii, jj = np.nonzero(occ)
        for di in range(-k, k + 1):
            for dj in range(-k, k + 1):
                if di * di + dj * dj <= k * k:
                    a, b = np.clip(ii + di, 0, self.n - 1), np.clip(jj + dj, 0, self.n - 1)
                    out[a, b] = True
        return out


def astar_grid(blocked, start, goal, max_expand=200000):
    """8-connected A* on a boolean grid. Returns list of (i, j) or None."""
    n0, n1 = blocked.shape
    if not (0 <= goal[0] < n0 and 0 <= goal[1] < n1):
        return None
    if blocked[goal]:
        return None
    h = lambda a: math.hypot(a[0] - goal[0], a[1] - goal[1])   # noqa: E731
    openq = [(h(start), 0.0, start)]
    came, g = {start: None}, {start: 0.0}
    steps = ((1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414))
    expanded = 0
    while openq:
        _, gc, cur = heapq.heappop(openq)
        if cur == goal:
            path = []
            while cur is not None:
                path.append(cur)
                cur = came[cur]
            return path[::-1]
        if gc > g.get(cur, 1e18):
            continue
        expanded += 1
        if expanded > max_expand:
            return None
        for di, dj, c in steps:
            nb = (cur[0] + di, cur[1] + dj)
            if not (0 <= nb[0] < n0 and 0 <= nb[1] < n1) or blocked[nb]:
                continue
            ng = gc + c
            if ng < g.get(nb, 1e18):
                g[nb], came[nb] = ng, cur
                heapq.heappush(openq, (ng + h(nb), ng, nb))
    return None


# ====================================================================== A* on a road graph
@dataclass
class RoadGraph:
    nodes: dict = field(default_factory=dict)    # id -> (lat, lon)
    adj: dict = field(default_factory=dict)      # id -> [(id, metres)]

    @classmethod
    def from_geojson(cls, path_or_dict, snap_m=0.5):
        from ..gps import haversine
        gj = path_or_dict if isinstance(path_or_dict, dict) else json.load(open(path_or_dict))
        g = cls()
        index = {}

        def node(lat, lon):
            key = (round(lat / (snap_m * 9e-6)), round(lon / (snap_m * 9e-6)))
            if key not in index:
                index[key] = len(g.nodes)
                g.nodes[index[key]] = (lat, lon)
                g.adj[index[key]] = []
            return index[key]
        for feat in gj["features"]:
            geom = feat["geometry"]
            lines = [geom["coordinates"]] if geom["type"] == "LineString" else geom["coordinates"]
            oneway = feat.get("properties", {}).get("oneway") in ("yes", True, 1)
            for line in lines:
                ids = [node(lat, lon) for lon, lat, *_ in line]
                for a, b in zip(ids, ids[1:]):
                    if a == b:
                        continue
                    d = haversine(*g.nodes[a], *g.nodes[b])
                    g.adj[a].append((b, d))
                    if not oneway:
                        g.adj[b].append((a, d))
        return g

    def nearest(self, lat, lon):
        from ..gps import haversine
        return min(self.nodes, key=lambda k: haversine(lat, lon, *self.nodes[k]))

    def block_edge_near(self, lat, lon, radius_m=3.0):
        """Mark road segments passing within radius_m of a blocked spot as impassable."""
        from ..gps import LocalFrame
        fr = LocalFrame(lat, lon)

        def seg_dist(a, b):
            ax, ay = fr.to_xy(*self.nodes[a])
            bx, by = fr.to_xy(*self.nodes[b])
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 == 0 else max(0.0, min(1.0, -(ax * dx + ay * dy) / L2))
            return math.hypot(ax + t * dx, ay + t * dy)
        n = 0
        for a in self.adj:
            keep = [(b, d) for b, d in self.adj[a] if seg_dist(a, b) > radius_m]
            n += len(self.adj[a]) - len(keep)
            self.adj[a] = keep
        return n

    def shortest_path(self, start_ll, goal_ll):
        """A* (haversine heuristic). Returns [(lat, lon), ...] or None if unreachable."""
        from ..gps import haversine
        s, t = self.nearest(*start_ll), self.nearest(*goal_ll)
        h = lambda k: haversine(*self.nodes[k], *self.nodes[t])   # noqa: E731
        openq = [(h(s), 0.0, s)]
        came, g = {s: None}, {s: 0.0}
        while openq:
            _, gc, cur = heapq.heappop(openq)
            if cur == t:
                out = []
                while cur is not None:
                    out.append(self.nodes[cur])
                    cur = came[cur]
                return [tuple(start_ll)] + out[::-1] + [tuple(goal_ll)]
            if gc > g.get(cur, 1e18):
                continue
            for nb, d in self.adj[cur]:
                ng = gc + d
                if ng < g.get(nb, 1e18):
                    g[nb], came[nb] = ng, cur
                    heapq.heappush(openq, (ng + h(nb), ng, nb))
        return None


# ====================================================================== path following
def pure_pursuit(pose, path_xy, lookahead=0.8, v_max=1.2, k_yaw=1.8):
    """pose (x, y, yaw); path [(x, y)...] -> (v, yaw_rate, index_of_target, distance_to_end)."""
    x, y, yaw = pose
    if not path_xy:
        return 0.0, 0.0, 0, 0.0
    d = [math.hypot(px - x, py - y) for px, py in path_xy]
    i0 = int(np.argmin(d))
    tgt = len(path_xy) - 1
    for i in range(i0, len(path_xy)):
        if d[i] >= lookahead:
            tgt = i
            break
    tx, ty = path_xy[tgt]
    err = math.atan2(ty - y, tx - x) - yaw
    err = (err + math.pi) % (2 * math.pi) - math.pi
    remain = d[-1]
    v = v_max * max(0.0, math.cos(err)) ** 2 * min(1.0, remain / 1.0 + 0.15)
    return v, k_yaw * err, tgt, remain


class LocalNavigator:
    """Goal-seeking with obstacle detours over the sonar occupancy grid.

    Every replan_s the A* route from the robot to the (local) goal is recomputed
    on the inflated grid.  If no route exists the robot stops, turns on the spot
    to look around (the sonars only see the front), and replans; when still no
    route exists after a full turn it HALTS and reports 'no alternative path'.
    """

    def __init__(self, grid: OccupancyGrid, replan_s=0.5, stop_dist=0.45):
        self.grid = grid
        self.replan_s = replan_s
        self.stop_dist = stop_dist
        self.path = []
        self.t_plan = -1e9
        self.state = "GO"          # GO | SCAN | HALT | ARRIVED
        self.scan_yaw0 = None
        self.scan_turned = 0.0
        self.last_yaw = None
        self.reason = ""

    def plan(self, pose, goal_xy):
        blocked = self.grid.blocked()
        s = self.grid.idx(pose[0], pose[1])
        gi = self.grid.idx(*goal_xy)
        if not self.grid.inside(*gi):         # goal outside the local map: aim at the border point
            dx, dy = goal_xy[0] - pose[0], goal_xy[1] - pose[1]
            k = (self.grid.n * self.grid.res * 0.4) / max(math.hypot(dx, dy), 1e-6)
            gi = self.grid.idx(pose[0] + dx * k, pose[1] + dy * k)
        blocked[max(s[0] - 2, 0):s[0] + 3, max(s[1] - 2, 0):s[1] + 3] = False   # never block ourselves
        cells = astar_grid(blocked, s, gi)
        if cells is None:
            return None
        return [self.grid.xy(i, j) for i, j in cells[::3]] + [self.grid.xy(*cells[-1])]

    def step(self, t, pose, goal_xy, front_min, v_max=1.2):
        """Returns (v, yaw_rate)."""
        if math.hypot(goal_xy[0] - pose[0], goal_xy[1] - pose[1]) < 0.4:
            self.state = "ARRIVED"
            return 0.0, 0.0
        if self.state == "HALT":
            return 0.0, 0.0
        if self.state == "SCAN":
            dyaw = (pose[2] - self.last_yaw + math.pi) % (2 * math.pi) - math.pi
            self.scan_turned += abs(dyaw)
            self.last_yaw = pose[2]
            p = self.plan(pose, goal_xy)
            if p is not None and len(p) > 1:
                self.path, self.state, self.t_plan = p, "GO", t
            elif self.scan_turned > 2 * math.pi:
                self.state, self.reason = "HALT", "no alternative path"
                return 0.0, 0.0
            return 0.0, 0.8
        if t - self.t_plan > self.replan_s or not self.path:
            p = self.plan(pose, goal_xy)
            self.t_plan = t
            if p is None:
                self.state, self.scan_turned, self.last_yaw = "SCAN", 0.0, pose[2]
                self.path = []
                return 0.0, 0.0
            self.path = p
        v, w, _, _ = pure_pursuit(pose, self.path, v_max=v_max)
        if front_min < self.stop_dist:            # reflex: never drive into something
            v = min(v, 0.0)
        elif front_min < 1.2:
            v = min(v, v_max * (front_min - self.stop_dist) / (1.2 - self.stop_dist) + 0.1)
        return v, w

    def reset(self):
        self.state, self.path, self.reason = "GO", [], ""
