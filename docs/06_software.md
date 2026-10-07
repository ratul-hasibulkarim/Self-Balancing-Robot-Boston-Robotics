# 6 · Software — control code, brain and controller app

```
 phone app (PWA) ──Wi‑Fi/WebSocket──┐            Mission Planner ──UDP 14550 MAVLink──┐
                                     ▼                                                ▼
 ┌────────────────────── Raspberry Pi 5 : companion/robot_brain (Python, 20 Hz) ───────────────────────┐
 │ app_server.py   behaviors.py (Brain: modes, actions, safety reflexes)   mission/mavlink_vehicle.py   │
 │ perception/line_follower.py  perception/obstacles.py  navigation/planner.py (A*, road graph, halt)   │
 │ gps.py (NMEA, ENU)  io_real.py (Teensy link, Picamera2 x2, GPS)                                      │
 └───────────────────────────────── link.py  ⇅ USB, 50 Hz cmd / 100 Hz telemetry ──────────────────────┘
 ┌──────────────────── Teensy 4.1 : firmware/ (C++, 1 kHz) ───────────────────────────────────────────┐
 │ main.cpp: IMU → Mahony → bolt_core::Controller → torques → 3 CAN buses; sensors; eyes; watchdogs     │
 │ lib/bolt_core: five_bar.h, bolt_controller.cpp  (← the SAME code runs in the MuJoCo simulator)       │
 └─────────────── CAN1/CAN2: 4 × DM‑J4310 hips ─────────── CAN3: 2 × MF9025 wheels ────────────────────┘
```

## 6.1 Real‑time controller (`firmware/lib/bolt_core`)

Every millisecond:

1. **Leg kinematics** (`five_bar.h`): from the two thigh angles φ1, φ4 compute the foot position, the *virtual leg* (length L, angle θb relative to the body) and the Jacobian J = ∂(L, θb)/∂(φ1, φ4).
2. **State estimate**: world leg angle θ = θb − pitch, wheel ground speed from the wheel motor speed corrected for the shin's own rotation, odometry s.
3. **Balance — gain‑scheduled LQR.** State `x = [θ, θ̇, s − s_ref, ṡ − v_ref, pitch − pitch_ref, pitch_rate]`, input `u = [wheel torque, hip torque]`, `u = u_eq − K(L)·x`. K is solved offline for 14 leg lengths on the linearised wheel‑leg‑body model whose masses/inertias are measured from the MuJoCo model (`sim/lqr_design.py`), and fitted with cubic polynomials in L → `robot_config_generated.h`. `u_eq` / θ_eq hold a commanded body pitch (bow / lean back) statically.
4. **Legs — virtual model control.** Per leg a spring‑damper on L (+ gravity feed‑forward), roll PD+I (left/right legs differentially → level body on side slopes, or a commanded lean), turn compensation (outer leg pushes more in curves), and a sync term that keeps both legs at the same angle. Leg force F and leg torque Tp are mapped to the two hip motors with `τ = Jᵀ [F, Tp]` (torque‑limited).
5. **Yaw**: PI on yaw rate → differential wheel torque.
6. **Mode / state machine**

| State | What happens |
|---|---|
| BALANCE | normal driving; position reference integrates the speed command (holds position on slopes), re‑anchored when you start/reverse (no catch‑up surges) |
| CROUCH | lower to the crouch length; if a step edge distance was given, roll on and **fire exactly when** `remaining ≤ v·(t_thrust + t_clear) + R + margin` |
| THRUST | constant leg force sized from energy so the take‑off speed gives the requested clearance (accelerate in the first 40 % of the stroke, then coast); wheel torque limited; a feed‑forward hip torque cancels the moment of the leg force about the body COM |
| FLIGHT | tuck legs, hold them straight down in the body frame (no body spin); jump‑over mode extends after apex for a soft landing; touchdown = leg gets compressed while armed |
| LAND | ramp back to ride height with double damping, balance law on |
| AIRBORNE | picked up / wheels unloaded → wheels stop, legs hold |
| FALLEN | tilt beyond limits → torque off (limp, protects gears and electronics) |
| RECOVER | experimental self‑righting (not working yet, see limits) |

Jump planning: the wheel clearance at the apex is the COM rise plus the leg retraction seen by the wheel, so the requested height `h` gives the COM rise `h_com = h + clearance − (L_takeoff − L_retract)·m_body/m_total` → take‑off speed `v = √(2 g h_com)·m_total/m_body` → thrust force. In simulation the planned and achieved clearances agree within ~1 cm up to the motor limit (~22 cm).

### Safety layers
Motor CAN timeout (50 ms) · Teensy watchdog (100 ms) · Pi command timeout 300 ms → stop & hold · e‑stop (hardware power cut + sense pin) · fall detection → limp · battery low → 0.5 m/s cap, critical → sit & off · uncalibrated → motors never enabled.

## 6.2 Build and flash the firmware
```bash
cd firmware
pio run -t upload              # PlatformIO (VS Code extension or CLI), board teensy41
# or from the Raspberry Pi over the existing USB cable:
pio run && teensy_loader_cli --mcu=TEENSY41 -w -v .pio/build/teensy41/firmware.hex
```
`firmware/tools/check_build.sh` compiles and links the firmware with plain `arm-none-eabi-g++` against the Teensy core sources (no PlatformIO needed) — that's how this repository was verified (62 KB code).

## 6.3 Companion brain (`companion/`)
```bash
cd companion
python3 -m robot_brain.main --config config.yaml       # or: sudo systemctl start bolt
python3 -m pytest -q tests                             # link byte-compat with the firmware, GPS, A*, re-routing
```
**Modes** (app "Auto" tab or Mission Planner): `MANUAL`, `LINE` (colour line following), `EXPLORE` (wander & avoid), `MISSION` (= Mission Planner AUTO), `GUIDED` (fly‑to‑here), `RTL`, `HOLD`, `IDLE`.

**Actions** (app buttons / mission items): `jump` (in place or while moving, height slider), `hop` (over an obstacle ahead), `step_up` (onto a curb/step), `stairs` (repeated step‑ups, experimental), `dodge_left/right`, `sit`, `stand`; posture: height, crawl, tilt (pitch), lean (roll).

`hop` / `step_up` locate the edge with the **front camera** (`perception/step_detector.py`: lowest straight horizontal edges + ground‑plane geometry, riser top/base matched using the step height from the slider; ≤ 2 cm error at 0.5–0.9 m in simulation). The sonar can't do this: it sits 0.33 m high and its beam passes over low steps. Procedure: stop‑and‑measure (creeping if needed) until the edge is 0.5–0.75 m away → back up 1.6 m by odometry → run up at 1 m/s → hand the remaining distance to the controller at 0.9 m, which times the take‑off itself. **Experimental**: start the action with BOLT 0.5–0.8 m in front of the step, facing it; it refuses with a message rather than guessing when it can't locate the edge.

**Line following** (`perception/line_follower.py`): HSV mask of the lower image (red, orange, yellow, green, blue, black, white or custom ranges) → line offset + heading from two bands → PD yaw‑rate, speed reduced in curves. Obstacle on the line → stop, wait 5 s, then HALT. Line lost → search for 4 s, then HALT. When the image floor is dark the brain switches on the headlight and the 850 nm IR illuminator automatically.

**Obstacles & halting** (`navigation/planner.py`): sonar returns go into a rolling 40 × 40 m log‑odds grid (10 cm). Every 0.5 s an 8‑connected A* on the inflated grid plans the detour, pure pursuit follows it, a reflex stops at 0.3 m. If A* finds no route: turn on the spot to scan (sonars only see forward); still nothing after a full turn → on a mission, mark that road segment blocked and re‑route over the road graph; no route at all → **HALT** + status text to the app and Mission Planner.

**GPS missions** (`mission/mavlink_vehicle.py` + `behaviors.py`): BOLT is a MAVLink *Rover* (heartbeat, position, attitude, GPS, battery, mission protocol, arm, mode changes, fly‑to‑here, a few params). Between waypoints it plans the **shortest route on the road graph (A* with haversine heuristic)**. Pose = wheel odometry + IMU yaw fused with GPS (complementary filter; heading corrected from the GPS course when driving straight).

## 6.4 Controller app (`app/`)
A Progressive Web App served by the robot: open `http://bolt.local:8080` on the phone → "Add to Home screen" → it runs full‑screen like a native app (Android & iOS), works offline on the robot's own Wi‑Fi.

* **Drive tab**: joystick (expo curve), max‑speed slider, **Jump / Hop over / Step up / Stairs / Dodge ◀ ▶ / Crawl / Stand‑Sit**, sliders for height, tilt (pitch), lean (roll), jump height; **Level** button.
* **Auto tab**: Manual / Hold / Explore & avoid / Return home; line‑following colour picker + speed; sonar radar.
* **Mission tab**: add the current position / typed lat‑lon / jump steps, mini‑map with the planned A* route, start / pause.
* **Setup tab**: auto lights, headlight & IR sliders, eye expressions, robot address, sit, release e‑stop.
* Always visible: link dot, state, mode, battery, big **STOP**, live front/rear camera with artificial horizon, alerts as toasts. Keyboard on desktop: W/A/S/D, Space = jump, C = crawl, X = stop.

Protocol (JSON over `/ws`) is documented at the top of `companion/robot_brain/app_server.py`; the app was tested in headless Chromium against the simulated robot (drive, jump state sequence, crawl, waypoints, e‑stop, no JS errors).

## 6.5 Simulation workflow (`sim/`)
```bash
pip install mujoco numpy scipy matplotlib trimesh manifold3d opencv-python-headless pymavlink aiohttp pyyaml
python3 cad/build_parts.py            # CAD -> STL + mass report (feeds the sim)
python3 sim/lqr_design.py             # gains for V1..V3, writes the firmware header
python3 sim/test_suite.py V1 V2 V3    # 15 requirement tests per design -> sim/results/*.json
python3 sim/companion_sim.py --demo all   # brain-in-the-loop: line, obstacles/halt, Mission Planner mission
python3 sim/companion_sim.py --demo app   # drive the simulated robot from the real app at http://localhost:8080
python3 sim/make_report.py            # figures for docs/04
```

## 6.6 Bring‑up checklist (real robot)
1. Calibrate and check signs (§3.8). 2. `bolt_cli.py balance` held by hand, then let go. 3. App: drive slowly, then height/tilt/lean sliders. 4. Small jumps (0.05 m) on grass/carpet before anything else. 5. Step‑ups with foam blocks before hard curbs. 6. Line following on tape, then missions in an empty car park.

**Tuning knobs**: LQR weights `Q_DIAG`, `R_DIAG` in `sim/lqr_design.py` (more weight on pitch = stiffer body; more on s/ṡ = tighter position) → regenerate the header; leg spring/damper `kp_leg/kd_leg`; roll `kp/kd/ki_roll`; yaw `kp/ki_yaw`; jump `thrust_force`, `jump_clearance`, `JUMP_EDGE_MARGIN`. Change them in the sim first, run the suite, then flash.
