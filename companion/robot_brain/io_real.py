"""
Real-hardware RobotIO: Teensy over USB serial, two CSI fisheye cameras (Picamera2),
u-blox GPS on UART.
"""
import threading
import time

from .behaviors import RobotIO
from .gps import Fix, GpsReader
from .link import Command, SerialLink, Telemetry


class Cameras:
    """Front + rear CSI cameras (Raspberry Pi 5 has two CSI ports).
    Falls back to OpenCV V4L2 devices if Picamera2 is not available."""

    def __init__(self, size=(640, 480), fps=20):
        self.frames = {"front": None, "rear": None}
        self._cams = {}
        try:
            from picamera2 import Picamera2
            for name, idx in (("front", 0), ("rear", 1)):
                try:
                    cam = Picamera2(idx)
                    cam.configure(cam.create_video_configuration(main={"size": size, "format": "BGR888"},
                                                                  controls={"FrameRate": fps}))
                    cam.start()
                    self._cams[name] = ("picam", cam)
                except Exception as e:
                    print(f"camera {name} not available: {e}")
        except ImportError:
            import cv2
            for name, idx in (("front", 0), ("rear", 2)):
                cap = cv2.VideoCapture(idx)
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
                    self._cams[name] = ("cv", cap)
        self._run = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self._run:
            for name, (kind, cam) in self._cams.items():
                try:
                    if kind == "picam":
                        self.frames[name] = cam.capture_array()
                    else:
                        ok, f = cam.read()
                        if ok:
                            self.frames[name] = f
                except Exception:
                    pass
            time.sleep(0.03)


class RealRobotIO(RobotIO):
    def __init__(self, serial_port="/dev/ttyACM0", gps_port="/dev/ttyAMA2", cameras=True):
        self.link = SerialLink(serial_port)
        try:
            self.gps_reader = GpsReader(gps_port)
        except Exception as e:
            print("GPS not available:", e)
            self.gps_reader = None
        self.cams = Cameras() if cameras else None
        self._last_eyes = None

    def now(self):
        return time.monotonic()

    def telemetry(self) -> Telemetry:
        return self.link.telemetry

    def send(self, cmd: Command):
        self.link.send_command(cmd)

    def eyes(self, expression=0, headlight=0, ir=0, rgb=(60, 230, 255)):
        key = (expression, headlight, ir, rgb)
        if key != self._last_eyes:
            self.link.send_eyes(expression, headlight, ir, rgb)
            self._last_eyes = key

    def frame(self, cam="front"):
        return self.cams.frames.get(cam) if self.cams else None

    def gps(self) -> Fix:
        return self.gps_reader.fix if self.gps_reader else Fix()
