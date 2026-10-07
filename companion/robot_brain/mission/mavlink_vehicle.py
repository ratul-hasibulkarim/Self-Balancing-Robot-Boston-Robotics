"""
MAVLink "rover" endpoint so ArduPilot Mission Planner (or QGroundControl) can
see BOLT on the map, upload/download waypoint missions, arm, switch modes
(MANUAL / HOLD / AUTO / RTL / GUIDED) and "Fly to here".

Connect Mission Planner: top-right -> UDP -> port 14550 (the robot sends to the
GCS address in config.yaml, or broadcasts on the local network).

Supported mission items
    16  NAV_WAYPOINT           goto lat/lon (routed over the road graph with A*), param1 = hold s
    19  NAV_LOITER_TIME        wait param1 s
    20  NAV_RETURN_TO_LAUNCH   go home
    112 CONDITION_DELAY        wait param1 s
    177 DO_JUMP                repeat from item param1, param2 times
    178 DO_CHANGE_SPEED        param2 = m/s
    31010 USER_1  "BOLT jump"  param1 = height m, param2 = 0 onto / 1 over
    31011 USER_2  "BOLT height" param1 = leg length m (0 = default), param2 = 1 crawl
    31012 USER_3  "BOLT tilt"  param1 = pitch deg, param2 = roll deg
(In Mission Planner use the "USER_1..3" commands via the command drop-down, or
type the numeric id in a mission file.)
"""
import threading
import time
from dataclasses import dataclass

from pymavlink import mavutil

mavlink = mavutil.mavlink

MODES = {0: "MANUAL", 4: "HOLD", 10: "AUTO", 11: "RTL", 15: "GUIDED"}
MODE_IDS = {v: k for k, v in MODES.items()}
CMD_JUMP, CMD_HEIGHT, CMD_TILT = 31010, 31011, 31012


@dataclass
class MissionItem:
    seq: int
    command: int
    frame: int = mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    params: tuple = (0, 0, 0, 0)
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    autocontinue: int = 1

    def to_msg(self, mav, target_sys, target_comp):
        return mav.mission_item_int_encode(target_sys, target_comp, self.seq, self.frame, self.command, 0,
                                           self.autocontinue, *self.params, int(self.lat * 1e7),
                                           int(self.lon * 1e7), self.alt, mavlink.MAV_MISSION_TYPE_MISSION)


class MavlinkVehicle:
    def __init__(self, url="udpout:127.0.0.1:14550", sysid=1, compid=1):
        self.conn = mavutil.mavlink_connection(url, source_system=sysid, source_component=compid,
                                               dialect="ardupilotmega")
        self.mav = self.conn.mav
        self.mode = "HOLD"
        self.armed = False
        self.mission = []
        self.current = 0
        self.home = None
        self.guided_target = None
        # live state (set by the behaviour manager)
        self.lat = self.lon = 0.0
        self.alt = 0.0
        self.heading = 0.0      # rad, 0 = north, clockwise
        self.roll = self.pitch = 0.0
        self.speed = 0.0
        self.battery_v = 0.0
        self.fix_type, self.sats = 0, 0
        self.params = {"WP_RADIUS": 1.0, "CRUISE_SPEED": 1.2, "BOLT_JUMP_H": 0.15, "BOLT_HEIGHT": 0.22,
                       "BOLT_LINE_SPD": 0.6}
        # callbacks
        self.on_mode = None        # f(new_mode_name)
        self.on_arm = None         # f(bool)
        self.on_mission_changed = None
        self._upload = None
        self._gcs_sys, self._gcs_comp = 255, 190
        self._run = True
        self._lock = threading.Lock()
        threading.Thread(target=self._rx_loop, daemon=True).start()
        threading.Thread(target=self._tx_loop, daemon=True).start()

    # ------------------------------------------------------------------ outgoing
    def statustext(self, text, severity=mavlink.MAV_SEVERITY_INFO):
        self.mav.statustext_send(severity, text.encode()[:50])

    def _tx_loop(self):
        n = 0
        while self._run:
            try:
                with self._lock:
                    if n % 5 == 0:
                        base = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (mavlink.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0)
                        self.mav.heartbeat_send(mavlink.MAV_TYPE_GROUND_ROVER, mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                                                base, MODE_IDS.get(self.mode, 4), mavlink.MAV_STATE_ACTIVE)
                        mv = int(self.battery_v * 1000)
                        self.mav.sys_status_send(0, 0, 0, 500, mv, -1, -1, 0, 0, 0, 0, 0, 0)
                        self.mav.mission_current_send(self.current)
                    t = int(time.time() * 1000) & 0xFFFFFFFF
                    import math
                    hdg = int((math.degrees(self.heading) % 360) * 100)
                    self.mav.global_position_int_send(t, int(self.lat * 1e7), int(self.lon * 1e7), int(self.alt * 1000),
                                                      0, int(self.speed * 100 * math.cos(self.heading)),
                                                      int(self.speed * 100 * math.sin(self.heading)), 0, hdg)
                    self.mav.attitude_send(t, self.roll, -self.pitch, self.heading, 0, 0, 0)
                    self.mav.gps_raw_int_send(t * 1000, self.fix_type, int(self.lat * 1e7), int(self.lon * 1e7),
                                              int(self.alt * 1000), 80, 65535, int(self.speed * 100), hdg, self.sats)
                    self.mav.vfr_hud_send(self.speed, self.speed, hdg // 100, 0, self.alt, 0)
            except OSError:
                pass
            n += 1
            time.sleep(0.2)

    # ------------------------------------------------------------------ incoming
    def _ack(self, cmd, result=mavlink.MAV_RESULT_ACCEPTED):
        self.mav.command_ack_send(cmd, result)

    def set_mode(self, name):
        if name not in MODE_IDS:
            return False
        self.mode = name
        if self.on_mode:
            self.on_mode(name)
        return True

    def _rx_loop(self):
        while self._run:
            m = self.conn.recv_match(blocking=True, timeout=0.5)
            if m is None:
                continue
            try:
                with self._lock:
                    self._handle(m)
            except Exception as e:      # never let a bad packet kill the link
                print("mavlink handler error:", e)

    def _handle(self, m):
        t = m.get_type()
        if t == "HEARTBEAT" and m.type == mavlink.MAV_TYPE_GCS:
            self._gcs_sys, self._gcs_comp = m.get_srcSystem(), m.get_srcComponent()
        elif t == "PARAM_REQUEST_LIST":
            for i, (k, v) in enumerate(self.params.items()):
                self.mav.param_value_send(k.encode(), float(v), mavlink.MAV_PARAM_TYPE_REAL32, len(self.params), i)
        elif t == "PARAM_REQUEST_READ":
            k = m.param_id if m.param_index < 0 else list(self.params)[m.param_index]
            if k in self.params:
                self.mav.param_value_send(k.encode(), float(self.params[k]), mavlink.MAV_PARAM_TYPE_REAL32,
                                          len(self.params), list(self.params).index(k))
        elif t == "PARAM_SET":
            k = m.param_id.strip("\x00")
            self.params[k] = m.param_value
            self.mav.param_value_send(k.encode(), float(m.param_value), mavlink.MAV_PARAM_TYPE_REAL32,
                                      len(self.params), list(self.params).index(k))
        elif t == "SET_MODE":
            self.set_mode(MODES.get(m.custom_mode, "HOLD"))
        elif t == "COMMAND_LONG":
            self._command(m)
        # ---- mission upload
        elif t == "MISSION_COUNT":
            self._upload = {"count": m.count, "items": [None] * m.count}
            if m.count == 0:
                self.mission = []
                self.mav.mission_ack_send(m.get_srcSystem(), m.get_srcComponent(), mavlink.MAV_MISSION_ACCEPTED)
            else:
                self.mav.mission_request_int_send(m.get_srcSystem(), m.get_srcComponent(), 0)
        elif t in ("MISSION_ITEM_INT", "MISSION_ITEM"):
            scale = 1e7 if t == "MISSION_ITEM_INT" else 1.0
            item = MissionItem(m.seq, m.command, m.frame, (m.param1, m.param2, m.param3, m.param4),
                               m.x / scale, m.y / scale, m.z, m.autocontinue)
            if m.current == 2:                      # "Fly to here" in GUIDED
                self.guided_target = (item.lat, item.lon)
                self.set_mode("GUIDED")
                self.mav.mission_ack_send(m.get_srcSystem(), m.get_srcComponent(), mavlink.MAV_MISSION_ACCEPTED)
                return
            if self._upload is None:
                return
            self._upload["items"][m.seq] = item
            nxt = next((i for i, x in enumerate(self._upload["items"]) if x is None), None)
            if nxt is None:
                self.mission = self._upload["items"]
                self._upload = None
                self.current = 1 if len(self.mission) > 1 else 0
                self.mav.mission_ack_send(m.get_srcSystem(), m.get_srcComponent(), mavlink.MAV_MISSION_ACCEPTED)
                self.statustext(f"BOLT: mission with {len(self.mission)} items received")
                if self.on_mission_changed:
                    self.on_mission_changed(self.mission)
            else:
                self.mav.mission_request_int_send(m.get_srcSystem(), m.get_srcComponent(), nxt)
        # ---- mission download
        elif t == "MISSION_REQUEST_LIST":
            self.mav.mission_count_send(m.get_srcSystem(), m.get_srcComponent(), len(self.mission))
        elif t in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            if m.seq < len(self.mission):
                self.mav.send(self.mission[m.seq].to_msg(self.mav, m.get_srcSystem(), m.get_srcComponent()))
        elif t == "MISSION_CLEAR_ALL":
            self.mission, self.current = [], 0
            self.mav.mission_ack_send(m.get_srcSystem(), m.get_srcComponent(), mavlink.MAV_MISSION_ACCEPTED)
        elif t == "MISSION_SET_CURRENT":
            self.current = m.seq
        elif t == "SET_POSITION_TARGET_GLOBAL_INT":
            self.guided_target = (m.lat_int / 1e7, m.lon_int / 1e7)

    def _command(self, m):
        c = m.command
        if c == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            self.armed = m.param1 > 0.5
            if self.on_arm:
                self.on_arm(self.armed)
            self._ack(c)
        elif c == mavlink.MAV_CMD_DO_SET_MODE:
            self._ack(c, mavlink.MAV_RESULT_ACCEPTED if self.set_mode(MODES.get(int(m.param2), "")) else mavlink.MAV_RESULT_DENIED)
        elif c == mavlink.MAV_CMD_MISSION_START:
            self.current = int(m.param1) if m.param1 > 0 else (1 if len(self.mission) > 1 else 0)
            self.set_mode("AUTO")
            self._ack(c)
        elif c == mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
            self.set_mode("RTL")
            self._ack(c)
        elif c in (mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                   mavlink.MAV_CMD_REQUEST_MESSAGE, mavlink.MAV_CMD_DO_SET_HOME):
            if c == mavlink.MAV_CMD_DO_SET_HOME and m.param1 == 0:
                self.home = (m.param5, m.param6)
            self._ack(c)
        else:
            self._ack(c, mavlink.MAV_RESULT_UNSUPPORTED)

    def close(self):
        self._run = False
