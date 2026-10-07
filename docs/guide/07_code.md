# Chapter 7 — How the code works

The software has four layers. Each one only talks to the layer next to it:

```
app/ (phone)  ⇄  companion/ (Raspberry Pi, Python, 20 Hz)  ⇄  firmware/ (Teensy, C++, 1000 Hz)  ⇄  motors & sensors
                                   ↑ sim/ replaces the hardware below the Pi, or below the controller
```

## 7.1 Firmware: `firmware/src/main.cpp`

**`setup()`** runs once at power‑up:
1. starts USB serial and the pins (e‑stop sense, lights, buzzer);
2. starts the three CAN buses at 1 Mbit/s, each with a receive callback (`on_can_hip`, `on_can_wheel`) that decodes motor replies in the background;
3. starts the IMU. If it isn't found, the motors are never enabled (eyes turn angry, buzzer beeps);
4. averages 1000 gyro samples (1 s, robot still) for the **gyro bias** and sets the starting attitude from gravity;
5. loads the leg calibration from EEPROM, creates the controller with the generated gains (`bolt::default_config()`);
6. enables the motors, starts the **watchdog** (100 ms) and a hardware timer that sets `tick = true` every millisecond.

**`loop()`** runs as fast as possible. It handles CAN events, incoming Pi frames and sonars, and when `tick` is set, one **control cycle**:

```
1. read IMU (14 bytes over SPI), subtract bias, rotate into body axes
2. Mahony filter -> roll, pitch, yaw; unwrap yaw; vertical acceleration
3. legs: motor positions -> φ1, φ4 per leg (sign + calibration offset), speeds; wheel speeds
4. battery filter, e-stop, command timeout (no command for 0.3 s -> v = 0)
5. ctrl->step(in, cmd, out)            <- the shared controller (7.2)
6. send 4 hip torques (DM "MIT" frames) and 2 wheel torques (LK 0xA1 frames)
7. every 10th cycle: cliff sensors + telemetry to the Pi;  every 33rd: eyes
```

### Motor drivers: `firmware/lib/bolt_drivers/motors.h`
**DM‑J4310 "MIT" frame.** Five numbers are squeezed into 8 bytes: position p (16 bits), velocity v (12), stiffness kp (12), damping kd (12), feed‑forward torque t (12). Each real number is mapped linearly onto the integer range: `u = (x − min)/(max − min)·(2^bits − 1)`. BOLT sends pure torque (p = v = kp = 0, kd = 0.02). Example, T = −1.83 Nm with TMAX = 10:

```
p = 0x8000, v = 0x800, kp = 0x000, kd = 0x010, t = (−1.83+10)/20·4095 = 0x689
bytes: 80 00 80 00 00 01 06 89
```
The reply holds the motor's position, speed, torque and temperatures, unpacked by `dm_parse`.

**LK MF9025.** Command byte `0xA1` = torque control. Torque → current (`/ WHEEL_KT`) → a 16‑bit value where ±2048 means ±16.5 A. The reply has temperature, current, speed (degrees per second) and a 16‑bit encoder, which `lk_parse` unwraps into a continuous angle.

### Other drivers
* `icm42688.h`: register setup (2000 °/s, 16 g, 1 kHz, anti‑alias filters) and a 14‑byte burst read.
* `attitude.h`: the Mahony filter (chapter 2.15).
* `sensors.h`: interrupt‑driven sonar timing, the Sharp voltage‑to‑distance curve.
* `eyes.h`: draws expressions (happy = upper half of the rings, sleepy = lower half, blinking…) on the 24 LEDs.

## 7.2 The controller: `firmware/lib/bolt_core/bolt_controller.cpp`

`Controller::step(Inputs, Command, Outputs)` is the heart of BOLT. In order:

1. **Leg kinematics** (`five_bar_solve`): L, θ_b, Jacobian for each leg. Leg speeds L̇ and θ̇ = J·φ̇. World leg angle θ = θ_b − pitch. Wheel ground speed with the shin‑rotation correction (chapter 2.14). Odometry s.
2. **Mode logic**: OFF → BALANCE (if upright) or RECOVER; any state → FALLEN if tipped.
3. **References**: speed ramped by the acceleration limit, height slewed at 0.35 m/s (and lowered at high speed), pitch/roll targets clamped. A new `jump_seq` from the Pi starts the **jump planner** (chapter 2.13), which computes take‑off speed, thrust force and apex time, then switches to CROUCH.
4. **Balance law**: gains for the current L, equilibrium for the commanded pitch, state error x, then `T_wheel = −K₀·x` and `T_hip = T_eq − K₁·x`. The position reference is re‑anchored only when starting or reversing.
5. **Yaw**: PI on yaw rate → ±T_yaw on the wheels.
6. **Legs**: target lengths (roll lean), roll PD+I, turn compensation, gravity feed‑forward.
7. **State machine** `switch (st_)`: CROUCH / THRUST / FLIGHT / LAND adjust targets, gains and limits (chapter 2.13).
8. **Contact estimate** per leg (normal force from leg force + wheel inertia), airborne detection.
9. **Leg torque split**: average leg torque ± a sync term that keeps both legs at the same angle; in flight, hold the legs in the body frame; during thrust and flight add the anti‑spin feed‑forward.
10. **Output**: `[T1, T4] = Jᵀ·[F, T_p]` with torque limits, wheel torques with limits, plus telemetry fields.

The file has no Arduino code, which is why it compiles both for the PC (simulation) and the Teensy. The bottom of the file exports a small C API (`bolt_create`, `bolt_step`…) for Python ctypes.

`five_bar.h` holds the kinematics. `robot_config_generated.h` is written by `sim/lqr_design.py`; never edit it by hand.

## 7.3 The Pi ↔ Teensy protocol: `firmware/lib/bolt_link/bolt_link.h` and `companion/robot_brain/link.py`
Every message is a frame:

```
A5 5A | type | length | payload ... | CRC16 (2 bytes, little-endian)
```
The two sync bytes mark the start. The CRC (a checksum) detects corrupted bytes, which are then ignored. The decoder is a small state machine fed one byte at a time, so it re‑synchronises after garbage. A real command frame (mode = balance, v = 1.0 m/s, yaw = 0.5 rad/s, height 0.22 m):

```
A5 5A 01 16 | 01 00 00 00 E8 03 F4 01 DC 00 00 00 00 00 00 00 00 00 00 00 00 00 | BB 81
            mode flags seq   v=1000 yaw=500 h=220 ...                               CRC
```
Units are integers (mm/s, mrad, mm) to keep frames small. The C++ struct and the Python `struct` format string must stay identical. `companion/tests/test_link.py` compiles a C++ helper and checks byte‑for‑byte compatibility in both directions.

## 7.4 The brain: `companion/robot_brain/`

| File | What it does |
|---|---|
| `main.py` | starts everything: brain thread (20 Hz), app web server, MAVLink endpoint, copies the robot state to Mission Planner |
| `io_real.py` | real hardware: Teensy link, two Picamera2 cameras, GPS reader |
| `behaviors.py` | **the Brain**: modes, actions, reflexes, pose estimate |
| `perception/line_follower.py` | coloured line → steering |
| `perception/obstacles.py` | sonar filtering, obstacle points, drop‑off detection |
| `perception/step_detector.py` | camera → distance to a step edge (experimental) |
| `navigation/planner.py` | occupancy grid, A* on the grid, road graph A*, pure pursuit, local navigator with halt |
| `mission/mavlink_vehicle.py` | pretends to be an ArduPilot rover for Mission Planner |
| `gps.py` | NMEA parsing, lat/lon ↔ metres |
| `app_server.py` | serves the app, WebSocket commands, MJPEG video |
| `link.py` | the protocol from 7.3 |

### `Brain.step()` — one tick (every 50 ms)
```
telemetry + GPS fix            -> update pose (odometry + IMU yaw, slowly corrected by GPS)
sonars + cliff sensors         -> filtered ranges, obstacle points into the occupancy grid
if an action is running        -> take (v, w) from the action generator
else by mode: MANUAL (joystick) | LINE | EXPLORE | MISSION / GUIDED / RTL
reflexes                       -> drop-off ahead: back up; < 0.3 m to an obstacle: no forward speed
lights                         -> dark camera image: headlight + IR on
send Command to the Teensy
```
**Actions are Python generators.** `_act_dodge` is a function that `yield`s (speed, turn) once per tick: turn 50°, burst forward, turn back. The brain calls `next()` every tick. This makes multi‑step behaviours easy to write and read, with no threads or state machines.

### Line following (`line_follower.py`)
1. Keep the lower 45 % of the image (the floor just ahead), blur, convert to HSV (hue/saturation/value), so a colour is a range of hue.
2. Mask pixels in the colour's HSV range, remove specks.
3. Find the line's centre in a near band and a far band → **offset** (where the line is) and **angle** (where it's going).
4. Yaw rate = −(6.0·error + 0.3·d(error)/dt), with error = offset + 0.4·angle. Speed drops in curves.
5. Line lost for 4 s, or blocked for 5 s → HALT and alert.

### Obstacle avoidance and halting (`planner.py`)
* **Occupancy grid**: the world around the robot as 10 cm cells, each holding a "log‑odds" number. A sonar hit adds evidence of an obstacle; the cells along the beam lose evidence (they're free). It scrolls with the robot.
* **Inflation**: cells within 28 cm of an obstacle count as blocked, so the robot's centre never gets close enough to hit.
* **A\***: the classic shortest‑path search. It expands cells in order of `cost so far + straight‑line distance to goal`, so it finds the shortest path while exploring as little as possible.
* **Pure pursuit**: drive towards a point ~0.8 m ahead on the path; turn rate ∝ heading error.
* **`LocalNavigator`**: replans every 0.5 s. If A* finds nothing, turn on the spot (sonars only see forward) and keep trying. If a full 360° turn finds nothing → **HALT, "no alternative path"**. On missions the brain first marks that road blocked and asks the road graph for another route.

### GPS missions (`planner.RoadGraph` + `mavlink_vehicle.py`)
* Roads are loaded from GeoJSON (export OpenStreetMap streets for your area). Points closer than 0.5 m are merged into graph nodes; edges carry their length.
* `shortest_path` runs A* with the great‑circle (haversine) distance as the heuristic.
* MAVLink handshake with Mission Planner:
  1. heartbeat ("I'm a rover");
  2. GCS sends `MISSION_COUNT`; the robot requests each item (`MISSION_REQUEST_INT`) and stores it, then sends `MISSION_ACK`;
  3. `ARM` + `MISSION_START`: the brain switches to MISSION;
  4. the robot sends position, attitude, GPS, battery and status texts several times per second.

### App server (`app_server.py`) and app (`app/`)
The Pi serves `app/` as a website. The page opens a **WebSocket** (a two‑way connection) and sends small JSON messages such as `{"type":"drive","v":0.8,"w":-0.3}` ten times a second while you touch the joystick. Ten times a second the server sends back the whole brain state (`Brain.snapshot()`) to update the screen. Video is MJPEG: an endless HTTP response of JPEG images the `<img>` tag displays. `manifest.webmanifest` + `sw.js` (service worker) make it installable and usable offline on the robot's own Wi‑Fi.

## 7.5 The simulation code: `sim/`
| File | Role |
|---|---|
| `robot_model.py` | builds the MuJoCo XML from params + CAD |
| `lqr_design.py` | controller design → gains → firmware header |
| `controller.py` | compiles and wraps the C++ controller (ctypes) |
| `simulator.py` | `BoltSim`: sensors with noise, latency, motor limits, logging, rendering |
| `test_suite.py` | the 15 tests, parallel runner, results JSON |
| `companion_sim.py` | `SimRobotIO`: the real brain driving the simulated robot; demos |
| `make_report.py` | report figures and GIFs |

## 7.6 Tests you can run
```bash
python3 -m pytest -q companion/tests     # protocol byte-compatibility, GPS, A*, re-routing, grid
python3 sim/test_suite.py V3             # physics requirements
firmware/tools/check_build.sh            # firmware compiles + links for the Teensy (needs arm-gcc + Teensy core)
```

## 7.7 Recipes: how to change things
| I want to… | Change | Then run |
|---|---|---|
| make the body stiffer / softer in pitch | `Q_DIAG` (5th weight) in `sim/lqr_design.py` | `lqr_design.py`, test suite, reflash |
| softer suspension | `kp_leg`, `kd_leg` in `lqr_design.py` | same |
| allow faster driving | `v_max` in `lqr_design.py` (check the motor envelope!) | same |
| support a different line colour | add an HSV range to `COLORS` in `line_follower.py` | — |
| add a new app button | button in `app/index.html`, handler in `app/app.js` sending `{"type":"action","name":"x"}`, a generator `_act_x` in `behaviors.py` registered in `start_action` | test with `sim/companion_sim.py --demo app` |
| add a mission command | a new `cmd` branch in `Brain._mission()` | — |
| change pins | `firmware/src/config.h` + `electronics/make_diagrams.py` | rebuild firmware, regenerate diagrams |
| change motor direction | the `sign` fields in `config.h` | check with `bolt_cli.py status` |
| re‑tune on the real robot | change gains in the sim first, run the suite, then flash | — |
