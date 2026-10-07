"""
Web server for the BOLT controller app (app/ folder, a PWA you "install" on the phone).

    GET  /                 the app
    GET  /video/front.mjpg live camera (MJPEG), also /video/rear.mjpg
    WS   /ws               JSON both ways (see PROTOCOL below)

PROTOCOL (app -> robot)
  {"type":"drive","v":0.8,"w":-0.3}                   joystick (send >= 5 Hz, else the robot stops)
  {"type":"mode","mode":"MANUAL|LINE|EXPLORE|MISSION|GUIDED|RTL|HOLD|IDLE","color":"red"}
  {"type":"action","name":"jump|hop|step_up|stairs|dodge_left|dodge_right|sit|stand", ...params}
  {"type":"posture","height":0.22,"pitch":0.1,"roll":0,"crawl":false}
  {"type":"lights","headlight":0..255,"ir":0..255,"auto":true}   {"type":"eyes","expression":"happy"}
  {"type":"mission","waypoints":[{"lat":..,"lon":..}, {"action":"jump","height":0.1}], "start":true}
  {"type":"estop"}  {"type":"estop_release"}  {"type":"settings", ...}
(robot -> app) {"type":"state", ...Brain.snapshot()} at 10 Hz
"""
import asyncio
import json
import os

from aiohttp import web

APP_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "app")


class AppServer:
    def __init__(self, brain, io, host="0.0.0.0", port=8080, lock=None):
        self.brain, self.io = brain, io
        self.host, self.port = host, port
        self.lock = lock
        self.clients = set()
        self.app = web.Application()
        self.app.add_routes([web.get("/ws", self.ws), web.get("/video/{cam}.mjpg", self.mjpeg),
                             web.get("/", self.index)])
        self.app.router.add_static("/", APP_DIR)

    async def index(self, request):
        return web.FileResponse(os.path.join(APP_DIR, "index.html"))

    # ------------------------------------------------------------------ websocket
    def handle(self, msg):
        b = self.brain
        t = msg.get("type")
        if t == "drive":
            b.drive(float(msg.get("v", 0)), float(msg.get("w", 0)))
        elif t == "mode":
            b.set_mode(msg["mode"], **({"color": msg["color"]} if "color" in msg else {}))
        elif t == "action":
            params = {k: v for k, v in msg.items() if k not in ("type", "name")}
            b.start_action(msg["name"], **params)
        elif t == "posture":
            b.set_posture(height=msg.get("height"), pitch=msg.get("pitch"), roll=msg.get("roll"), crawl=msg.get("crawl"))
        elif t == "lights":
            b.cfg.auto_lights = bool(msg.get("auto", False))
            b.lights = (int(msg.get("headlight", 0)), int(msg.get("ir", 0)))
        elif t == "eyes":
            b.expression = msg.get("expression", "normal")
        elif t == "mission":
            b.set_mission(msg.get("waypoints", []))
            if msg.get("start"):
                b.set_mode("MISSION", start=0)
        elif t == "estop":
            b.estop()
        elif t == "estop_release":
            b.release_estop()
        elif t == "settings":
            for k in ("cruise_speed", "line_speed", "wp_radius", "step_tread_default"):
                if k in msg:
                    setattr(b.cfg, k, float(msg[k]))

    async def ws(self, request):
        ws = web.WebSocketResponse(heartbeat=2.0)
        await ws.prepare(request)
        self.clients.add(ws)
        try:
            async for m in ws:
                if m.type == web.WSMsgType.TEXT:
                    try:
                        msg = json.loads(m.data)
                        if self.lock:
                            with self.lock:
                                self.handle(msg)
                        else:
                            self.handle(msg)
                    except Exception as e:
                        await ws.send_json({"type": "error", "text": str(e)})
        finally:
            self.clients.discard(ws)
            if not self.clients:
                self.brain.drive(0, 0)          # nobody is driving any more
        return ws

    async def broadcast_loop(self):
        while True:
            if self.clients:
                snap = json.dumps({"type": "state", **self.brain.snapshot()}, default=float)
                for ws in list(self.clients):
                    try:
                        await ws.send_str(snap)
                    except Exception:
                        self.clients.discard(ws)
            await asyncio.sleep(0.1)

    # ------------------------------------------------------------------ video
    async def mjpeg(self, request):
        import cv2
        cam = request.match_info["cam"]
        resp = web.StreamResponse(headers={"Content-Type": "multipart/x-mixed-replace; boundary=frame",
                                           "Cache-Control": "no-cache"})
        await resp.prepare(request)
        try:
            while True:
                f = self.io.frame(cam)
                if f is not None:
                    ok, jpg = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    if ok:
                        await resp.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n")
                await asyncio.sleep(0.07)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return resp

    async def start(self):
        runner = web.AppRunner(self.app)
        await runner.setup()
        await web.TCPSite(runner, self.host, self.port).start()
        asyncio.ensure_future(self.broadcast_loop())
