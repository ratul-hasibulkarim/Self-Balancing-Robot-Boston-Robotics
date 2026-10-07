"""
Teensy <-> Raspberry Pi link.  Byte-for-byte twin of firmware/lib/bolt_link/bolt_link.h.

Frame: 0xA5 0x5A | type u8 | len u8 | payload | crc16-CCITT (LE) over type..payload
"""
import struct
import threading
import time
from dataclasses import dataclass, field

MSG_CMD, MSG_CALIBRATE, MSG_EYES, MSG_TELEM, MSG_LOG = 0x01, 0x02, 0x03, 0x81, 0x82

CMD_FMT = "<BBHhhHhhHBH3x"              # 22 bytes
EYES_FMT = "<BBBBBB"
TELEM_FMT = "<IBBhhhhhhhi2HhH4H2HbBhhhh10x"   # 64 bytes
assert struct.calcsize(CMD_FMT) == 22
assert struct.calcsize(TELEM_FMT) == 64

STATES = ["OFF", "BALANCE", "CROUCH", "THRUST", "FLIGHT", "LAND", "AIRBORNE", "FALLEN", "RECOVER"]
EXPRESSIONS = ["normal", "happy", "blink", "angry", "sleepy", "alert", "lost", "love"]


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(msg_type: int, payload: bytes) -> bytes:
    body = bytes([msg_type, len(payload)]) + payload
    c = crc16(body)
    return b"\xA5\x5A" + body + bytes([c & 0xFF, c >> 8])


class Decoder:
    def __init__(self):
        self.st, self.buf, self.crc_errors = 0, bytearray(), 0

    def push(self, data: bytes):
        """Feed bytes, yield (type, payload) for every valid frame."""
        for b in data:
            st = self.st
            if st == 0:
                self.st = 1 if b == 0xA5 else 0
            elif st == 1:
                self.st = 2 if b == 0x5A else (1 if b == 0xA5 else 0)
            elif st == 2:
                self.type, self.st = b, 3
            elif st == 3:
                self.len, self.buf = b, bytearray()
                self.st = 4 if b else 5
            elif st == 4:
                self.buf.append(b)
                if len(self.buf) == self.len:
                    self.st = 5
            elif st == 5:
                self.crc_lo, self.st = b, 6
            elif st == 6:
                self.st = 0
                if crc16(bytes([self.type, self.len]) + bytes(self.buf)) == (self.crc_lo | (b << 8)):
                    yield self.type, bytes(self.buf)
                else:
                    self.crc_errors += 1


@dataclass
class Command:
    mode: int = 0                 # 0 off, 1 balance, 2 crawl
    estop: bool = False
    jump_seq: int = 0
    v: float = 0.0                # m/s
    yaw_rate: float = 0.0         # rad/s
    height: float = 0.22          # m
    roll: float = 0.0             # rad
    pitch: float = 0.0            # rad
    jump_height: float = 0.0      # m
    jump_mode: int = 0            # 0 onto step, 1 over obstacle
    jump_dist: float = 0.0        # m

    def pack(self) -> bytes:
        c = lambda x, lo, hi: int(max(lo, min(hi, round(x))))  # noqa: E731
        return struct.pack(CMD_FMT, self.mode, 1 if self.estop else 0, self.jump_seq & 0xFFFF,
                           c(self.v * 1000, -32000, 32000), c(self.yaw_rate * 1000, -32000, 32000),
                           c(self.height * 1000, 0, 65535), c(self.roll * 1000, -32000, 32000),
                           c(self.pitch * 1000, -32000, 32000), c(self.jump_height * 1000, 0, 65535),
                           self.jump_mode, c(self.jump_dist * 1000, 0, 65535))

    @classmethod
    def unpack(cls, b: bytes):
        f = struct.unpack(CMD_FMT, b)
        return cls(mode=f[0], estop=bool(f[1] & 1), jump_seq=f[2], v=f[3] / 1000, yaw_rate=f[4] / 1000,
                   height=f[5] / 1000, roll=f[6] / 1000, pitch=f[7] / 1000, jump_height=f[8] / 1000,
                   jump_mode=f[9], jump_dist=f[10] / 1000)


@dataclass
class Telemetry:
    t_ms: int = 0
    state: int = 0
    flags: int = 0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    v: float = 0.0
    s: float = 0.0
    L: tuple = (0.0, 0.0)
    theta: float = 0.0
    battery_v: float = 0.0
    sonar: tuple = (0.0, 0.0, 0.0, 0.0)     # m, FL F FR Rear (0 = nothing in range)
    cliff: tuple = (0.4, 0.4)               # m
    motor_temp_max: int = 0
    motor_fault_mask: int = 0
    wheel_torque: tuple = (0.0, 0.0)
    hip_torque_max: float = 0.0
    yaw_odo: float = 0.0
    rx_time: float = field(default_factory=time.time)

    @property
    def state_name(self):
        return STATES[self.state] if self.state < len(STATES) else str(self.state)

    contact_l = property(lambda s: bool(s.flags & 1))
    contact_r = property(lambda s: bool(s.flags & 2))
    estop = property(lambda s: bool(s.flags & 4))
    battery_low = property(lambda s: bool(s.flags & 8))
    link_lost = property(lambda s: bool(s.flags & 16))
    motor_fault = property(lambda s: bool(s.flags & 32))
    calibrated = property(lambda s: bool(s.flags & 64))

    def pack(self) -> bytes:
        return struct.pack(TELEM_FMT, self.t_ms, self.state, self.flags,
                           *[int(round(x * 1000)) for x in (self.roll, self.pitch, self.yaw, self.gx, self.gy, self.gz, self.v)],
                           int(round(self.s * 1000)), *[int(round(x * 10000)) for x in self.L],
                           int(round(self.theta * 1000)), int(round(self.battery_v * 1000)),
                           *[int(round(x * 1000)) for x in self.sonar], *[int(round(x * 1000)) for x in self.cliff],
                           self.motor_temp_max, self.motor_fault_mask,
                           *[int(round(x * 1000)) for x in self.wheel_torque], int(round(self.hip_torque_max * 1000)),
                           int(round((self.yaw_odo % 32.767) * 1000)) if self.yaw_odo >= 0 else -int(round((-self.yaw_odo % 32.767) * 1000)))

    @classmethod
    def unpack(cls, b: bytes):
        f = struct.unpack(TELEM_FMT, b)
        return cls(t_ms=f[0], state=f[1], flags=f[2], roll=f[3] / 1000, pitch=f[4] / 1000, yaw=f[5] / 1000,
                   gx=f[6] / 1000, gy=f[7] / 1000, gz=f[8] / 1000, v=f[9] / 1000, s=f[10] / 1000,
                   L=(f[11] / 10000, f[12] / 10000), theta=f[13] / 1000, battery_v=f[14] / 1000,
                   sonar=tuple(x / 1000 for x in f[15:19]), cliff=(f[19] / 1000, f[20] / 1000),
                   motor_temp_max=f[21], motor_fault_mask=f[22], wheel_torque=(f[23] / 1000, f[24] / 1000),
                   hip_torque_max=f[25] / 1000, yaw_odo=f[26] / 1000)

    def as_dict(self):
        d = {k: getattr(self, k) for k in ("t_ms", "state", "roll", "pitch", "yaw", "v", "s", "L", "theta",
                                           "battery_v", "sonar", "cliff", "motor_temp_max", "motor_fault_mask")}
        d.update(state_name=self.state_name, contact=[self.contact_l, self.contact_r], estop=self.estop,
                 battery_low=self.battery_low, link_lost=self.link_lost, motor_fault=self.motor_fault,
                 calibrated=self.calibrated)
        return d


class SerialLink:
    """Talks to the Teensy over USB serial (/dev/ttyACM0) in a background thread."""

    def __init__(self, port="/dev/ttyACM0", baud=2_000_000):
        import serial  # pyserial
        self.ser = serial.Serial(port, baud, timeout=0.01)
        self.dec = Decoder()
        self.telemetry = Telemetry()
        self.on_telemetry = None
        self.lock = threading.Lock()
        self._run = True
        self.th = threading.Thread(target=self._reader, daemon=True)
        self.th.start()

    def _reader(self):
        while self._run:
            data = self.ser.read(256)
            for t, p in self.dec.push(data):
                if t == MSG_TELEM and len(p) == 64:
                    tel = Telemetry.unpack(p)
                    with self.lock:
                        self.telemetry = tel
                    if self.on_telemetry:
                        self.on_telemetry(tel)

    def send_command(self, cmd: Command):
        self.ser.write(encode(MSG_CMD, cmd.pack()))

    def send_eyes(self, expression=0, headlight=0, ir=0, rgb=(60, 230, 255)):
        self.ser.write(encode(MSG_EYES, struct.pack(EYES_FMT, expression, headlight, ir, *rgb)))

    def calibrate(self):
        self.ser.write(encode(MSG_CALIBRATE, b""))

    def close(self):
        self._run = False
        self.ser.close()
