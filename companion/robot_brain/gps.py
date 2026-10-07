"""
GPS (u-blox M8N/M10 "Pixhawk / Mission Planner" puck) reader + geodesy helpers.

The puck's UART goes to the Pi (UART2: GPIO4 TX -> GPS RX, GPIO5 RX <- GPS TX).
Configure the receiver once with u-center: 10 Hz, 115200 baud, NMEA GGA+RMC+GSA,
GPS+Galileo+BeiDou, "Automotive" dynamic model, SBAS on.
"""
import math
import threading
import time
from dataclasses import dataclass

R_EARTH = 6378137.0


@dataclass
class Fix:
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    fix_quality: int = 0      # 0 none, 1 GPS, 2 DGPS, 4 RTK fixed, 5 RTK float
    sats: int = 0
    hdop: float = 99.0
    speed: float = 0.0        # m/s
    course: float = 0.0       # deg, true
    t: float = 0.0

    @property
    def valid(self):
        return self.fix_quality > 0 and self.hdop < 5.0


def _nmea_checksum_ok(line: str) -> bool:
    if "*" not in line:
        return False
    body, cs = line[1:].split("*", 1)
    c = 0
    for ch in body:
        c ^= ord(ch)
    try:
        return c == int(cs[:2], 16)
    except ValueError:
        return False


def _deg(v: str, hemi: str) -> float:
    if not v:
        return 0.0
    dot = v.index(".")
    d = float(v[: dot - 2]) + float(v[dot - 2:]) / 60.0
    return -d if hemi in ("S", "W") else d


def parse_nmea(line: str, fix: Fix) -> bool:
    """Update `fix` from one NMEA sentence. Returns True if it was used."""
    line = line.strip()
    if not line.startswith("$") or not _nmea_checksum_ok(line):
        return False
    f = line.split("*")[0].split(",")
    kind = f[0][3:]
    if kind == "GGA" and len(f) >= 10:
        fix.lat, fix.lon = _deg(f[2], f[3]), _deg(f[4], f[5])
        fix.fix_quality = int(f[6] or 0)
        fix.sats = int(f[7] or 0)
        fix.hdop = float(f[8] or 99)
        fix.alt = float(f[9] or 0)
        fix.t = time.time()
        return True
    if kind == "RMC" and len(f) >= 9:
        if f[2] == "A":
            fix.lat, fix.lon = _deg(f[3], f[4]), _deg(f[5], f[6])
            fix.speed = float(f[7] or 0) * 0.514444
            fix.course = float(f[8] or 0)
        return True
    return False


def make_nmea(lat, lon, alt=10.0, sats=12, hdop=0.8, speed=0.0, course=0.0):
    """Generate GGA + RMC sentences (used by the simulator's virtual GPS)."""
    def dm(x, lat_):
        a = abs(x)
        d = int(a)
        m = (a - d) * 60
        if lat_:
            return f"{d:02d}{m:07.4f}", "N" if x >= 0 else "S"
        return f"{d:03d}{m:07.4f}", "E" if x >= 0 else "W"

    def wrap(body):
        c = 0
        for ch in body:
            c ^= ord(ch)
        return f"${body}*{c:02X}"
    la, ns = dm(lat, True)
    lo, ew = dm(lon, False)
    t = time.strftime("%H%M%S.00", time.gmtime())
    gga = wrap(f"GPGGA,{t},{la},{ns},{lo},{ew},1,{sats:02d},{hdop:.1f},{alt:.1f},M,0.0,M,,")
    rmc = wrap(f"GPRMC,{t},A,{la},{ns},{lo},{ew},{speed / 0.514444:.2f},{course:.1f},010126,,,A")
    return [gga, rmc]


# ------------------------------------------------------------------ geodesy (local tangent plane)
class LocalFrame:
    """Equirectangular ENU frame around an origin: accurate to cm over a few km."""

    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = lat0, lon0
        self.k_lat = math.pi / 180 * R_EARTH
        self.k_lon = math.pi / 180 * R_EARTH * math.cos(math.radians(lat0))

    def to_xy(self, lat, lon):
        return ((lon - self.lon0) * self.k_lon, (lat - self.lat0) * self.k_lat)   # east, north

    def to_ll(self, x, y):
        return (self.lat0 + y / self.k_lat, self.lon0 + x / self.k_lon)


def haversine(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R_EARTH * math.asin(math.sqrt(a))


def bearing(lat1, lon1, lat2, lon2):
    """Initial bearing in radians, 0 = north, clockwise."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return math.atan2(y, x)


class GpsReader:
    """Reads NMEA from a serial port in a background thread."""

    def __init__(self, port="/dev/ttyAMA2", baud=115200):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.fix = Fix()
        self._run = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        buf = b""
        while self._run:
            buf += self.ser.read(512)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                parse_nmea(line.decode(errors="ignore"), self.fix)

    def close(self):
        self._run = False
