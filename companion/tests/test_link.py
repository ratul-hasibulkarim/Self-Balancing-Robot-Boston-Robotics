"""C++ (firmware) <-> Python (companion) link compatibility + unit tests for the brain modules.
    cd companion && python3 -m pytest -q tests
"""
import math
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
FW = os.path.join(os.path.dirname(os.path.dirname(HERE)), "firmware")

from robot_brain import link  # noqa: E402
from robot_brain.gps import Fix, LocalFrame, haversine, make_nmea, parse_nmea  # noqa: E402
from robot_brain.navigation.planner import OccupancyGrid, RoadGraph, astar_grid, pure_pursuit  # noqa: E402


@pytest.fixture(scope="module")
def host_bin(tmp_path_factory):
    out = tmp_path_factory.mktemp("bin") / "link_host"
    subprocess.check_call(["g++", "-std=c++17", "-I", os.path.join(FW, "lib", "bolt_link"),
                           os.path.join(FW, "test", "link_host.cpp"), "-o", str(out)])
    return str(out)


def test_telemetry_from_firmware(host_bin):
    hexs = subprocess.check_output([host_bin, "encode"]).decode().strip()
    frames = list(link.Decoder().push(bytes.fromhex(hexs)))
    assert len(frames) == 1 and frames[0][0] == link.MSG_TELEM
    t = link.Telemetry.unpack(frames[0][1])
    assert t.t_ms == 123456 and t.state_name == "BALANCE" and t.contact_l and t.calibrated
    assert abs(t.pitch - 0.034) < 1e-9 and abs(t.yaw - 1.571) < 1e-9
    assert abs(t.v - 1.5) < 1e-9 and abs(t.s + 2500.0) < 1e-6
    assert t.L == (0.22, 0.221) and abs(t.battery_v - 24.123) < 1e-9
    assert abs(t.sonar[1] - 1.234) < 1e-9 and t.motor_temp_max == 45 and abs(t.yaw_odo + 3.141) < 1e-9


def test_command_to_firmware(host_bin):
    c = link.Command(mode=2, estop=True, jump_seq=7, v=-1.25, yaw_rate=0.5, height=0.18, roll=-0.1, pitch=0.2,
                     jump_height=0.15, jump_mode=1, jump_dist=0.9)
    frame = link.encode(link.MSG_CMD, c.pack()).hex()
    out = subprocess.run([host_bin], input=frame + "\n", capture_output=True, text=True).stdout.split()
    assert list(map(int, out)) == [2, 1, 7, -1250, 500, 180, -100, 200, 150, 1, 900]


def test_decoder_resyncs_after_garbage():
    good = link.encode(link.MSG_CMD, link.Command(mode=1).pack())
    bad = bytearray(good)
    bad[6] ^= 0xFF
    d = link.Decoder()
    frames = list(d.push(b"\x00\xA5\x13" + bytes(bad) + good))
    assert len(frames) == 1 and d.crc_errors == 1


def test_dm_mit_packing(host_bin):
    from_fw = subprocess.check_output([host_bin, "dm"]).decode().strip()
    # p=0, v=0, kp=0, kd=0.02, t=-3.25 with PMAX 12.5 VMAX 30 TMAX 10
    p = round((0 + 12.5) * 65535 / 25)
    v = round((0 + 30) * 4095 / 60)
    kd = round(0.02 * 4095 / 5)
    t = round((-3.25 + 10) * 4095 / 20)
    exp = bytes([p >> 8, p & 0xFF, v >> 4, ((v & 0xF) << 4), 0, kd >> 4, ((kd & 0xF) << 4) | (t >> 8), t & 0xFF]).hex()
    assert from_fw == exp


def test_nmea_roundtrip():
    f = Fix()
    for s in make_nmea(23.7281234, 90.3895678, speed=1.2, course=45):
        assert parse_nmea(s, f)
    assert abs(f.lat - 23.7281234) < 2e-6 and abs(f.lon - 90.3895678) < 2e-6
    assert f.valid and abs(f.speed - 1.2) < 0.01 and abs(f.course - 45) < 0.1


def test_local_frame():
    fr = LocalFrame(23.7275, 90.3889)
    lat, lon = fr.to_ll(100.0, -50.0)
    x, y = fr.to_xy(lat, lon)
    assert abs(x - 100) < 1e-6 and abs(y + 50) < 1e-6
    assert abs(haversine(23.7275, 90.3889, lat, lon) - math.hypot(100, 50)) < 0.2


def test_astar_grid_detour_and_no_path():
    import numpy as np
    g = np.zeros((40, 40), bool)
    g[20, 0:35] = True                       # wall with a gap at the end
    p = astar_grid(g, (5, 5), (35, 5))
    assert p is not None and p[-1] == (35, 5) and all(not g[c] for c in p)
    g[20, :] = True                          # close the gap -> no path
    assert astar_grid(g, (5, 5), (35, 5)) is None


def test_road_graph_shortest_and_reroute():
    fr = LocalFrame(0.0, 0.0)
    def ln(a, b):
        (la, lo), (lb, lob) = fr.to_ll(*a), fr.to_ll(*b)
        return {"type": "Feature", "properties": {}, "geometry": {"type": "LineString", "coordinates": [[lo, la], [lob, lb]]}}
    # square block: A(0,0) B(100,0) C(100,100) D(0,100) + a long detour E(200,50)
    pts = dict(A=(0, 0), B=(100, 0), C=(100, 100), D=(0, 100), E=(200, 50))
    feats = [ln(pts[a], pts[b]) for a, b in ("AB", "BC", "CD", "DA", "BE", "EC")]
    g = RoadGraph.from_geojson({"type": "FeatureCollection", "features": feats})
    route = g.shortest_path(fr.to_ll(*pts["A"]), fr.to_ll(*pts["C"]))
    assert route is not None and len(route) >= 3
    length = sum(haversine(*a, *b) for a, b in zip(route, route[1:]))
    assert abs(length - 200) < 1.0                      # A-B-C or A-D-C
    assert g.block_edge_near(*fr.to_ll(100, 50), radius_m=5) == 2       # B-C blocked (both directions)
    assert g.block_edge_near(*fr.to_ll(0, 50), radius_m=5) == 2         # D-A blocked too
    route = g.shortest_path(fr.to_ll(*pts["A"]), fr.to_ll(*pts["C"]))  # only A-B-E-C remains
    length = sum(haversine(*a, *b) for a, b in zip(route, route[1:]))
    assert abs(length - (100 + 2 * math.hypot(100, 50))) < 1.0
    g.block_edge_near(*fr.to_ll(150, 25), radius_m=5)                  # and B-E: no way at all
    assert g.shortest_path(fr.to_ll(*pts["A"]), fr.to_ll(*pts["C"])) is None


def test_pure_pursuit_turns_towards_path():
    v, w, _, _ = pure_pursuit((0, 0, 0), [(0, 0), (1, 1), (2, 2)])
    assert w > 0 and v >= 0


def test_occupancy_grid_inflation():
    g = OccupancyGrid(size_m=10, res=0.1)
    for _ in range(3):
        g.integrate((0, 0), 0.0, [(2.0, 0.0)])
    b = g.blocked(inflate_m=0.3)
    i, j = g.idx(2.0, 0.0)
    assert b[i, j] and b[i, j + 2] and not b[i, j + 6]
