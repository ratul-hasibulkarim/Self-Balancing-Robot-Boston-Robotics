# Chapter 5 — How the simulations were done, and how the design was improved

## 5.1 What a physics simulator does
MuJoCo stores the robot as rigid **bodies** connected by **joints**, each with mass and inertia. Every 0.5 ms it:

1. finds which shapes touch (tyre ↔ ground, shin ↔ step edge…);
2. computes contact forces with friction;
3. adds motor torques, gravity and constraint forces;
4. solves Newton's laws for the accelerations and steps positions and speeds forward.

Repeat 2000 times per simulated second and you get the robot's motion. The time step must be much shorter than the fastest thing that happens. Here that is tyre contact and the 1 kHz control loop.

## 5.2 Building the robot model (`sim/robot_model.py`)
The script writes an MJCF (MuJoCo XML) file from `design/params.py`:

* **base body** with a free joint (it can fly, which is needed for jumps). Its mass is built from boxes for every component: shell prints at their CAD centre of mass, battery, sled, sensors, four hip motors. One box approximates the body's collisions.
* **per side:** `thigh_F` (hinge at A) → `shin_F` (hinge at B); `thigh_R` (hinge at E) → `shin_R` (hinge at D) → `wheel` (hinge at C).
* **closing the loop:** a tree can't contain a loop, so the front shin's tip is tied to the rear shin's tip with an **equality constraint** (`<connect>`). That turns the two open chains into the real 5‑bar. The legs are placed in the exact closed pose from the inverse kinematics at start, so the constraint starts satisfied. This was checked: foot positions agree to 1e‑17 m.
* **contacts:** cylinders for the tyres (friction 1.0), capsules for the shins (so a knee can hit a step), the GPS puck (crawl height). Robot parts collide with the world but not with each other.
* **actuators:** torque motors on the 4 hip joints and 2 wheel joints.
* **sensors:** gyro and accelerometer at the IMU position; ultrasonics are 3 rays per sensor spread over its cone; IR cliff sensors are rays 30° forward of down; two cameras where the fisheyes are.
* **visuals:** the CAD meshes, so renders look like the real robot.
* **terrains:** flat, ramp, side slope, random height‑field (rough), steps/stairs, bars, gaps, obstacles, a textured floor with a coloured line.

## 5.3 Making the simulation honest
A simulation that is too perfect gives false confidence. These real‑world effects are added (`sim/simulator.py`):

| Effect | Implementation |
|---|---|
| motor speed limits | torque clipped to the back‑EMF envelope (chapter 2.12) for every motor |
| bus latency | the torque computed this millisecond is applied next millisecond |
| sensor noise | IMU angles ±0.002 rad, gyro ±0.004 rad/s, encoders, wheel speed |
| sonar behaviour | minimum of 3 rays, 4 m max, 1 cm noise |
| GPS | random slow wander of the position (±0.3 m) |
| camera | images rendered from the robot's camera and processed by the real OpenCV code |

## 5.4 One controller for both worlds
The controller (`firmware/lib/bolt_core/bolt_controller.cpp`) is written in plain C++ with no Arduino calls. `sim/controller.py` compiles it into `libbolt.so` and calls it through **ctypes**, using Python structures that mirror the C++ structs byte for byte (sizes are checked at load time). The loop each millisecond:

```
read simulated IMU + joint encoders  ->  Inputs struct
bolt_step(controller, Inputs, Command, Outputs)     # the firmware code
apply motor torque limits  ->  MuJoCo actuators  ->  2 physics steps
```
On the Teensy, `main.cpp` does exactly the same with real sensors and CAN motors.

## 5.5 Designing the controller from the model
`sim/lqr_design.py` (details in chapter 2):

1. builds the MuJoCo model and poses the legs at 14 lengths;
2. measures the lumped masses, COM offsets and inertias of body, legs and wheels from MuJoCo;
3. builds the 3‑body wheel–leg–body equations and linearises them numerically;
4. solves LQR at each length and fits cubic polynomials;
5. writes `sim/build/config_<variant>.json` (for the sim) and `firmware/lib/bolt_core/robot_config_generated.h` (for the Teensy).

So a change in the CAD flows automatically into the gains.

## 5.6 The test suite (`sim/test_suite.py`)
Each requirement became a function that sets up a scene, drives the robot with a scripted "operator", and measures pass/fail:

| Test | Scene and pass criterion |
|---|---|
| static_balance | stand 5 s with sensor noise; report pitch RMS and drift |
| push_recovery | shove the body for 0.1 s; binary search for the biggest impulse it recovers from **cleanly** (no fall, legs never past ±45°, calm stance after 4 s) |
| top_speed | 1.0 / 2.0 / 2.4 m/s; must track (< 120 % of command) with < 10° pitch |
| turning | 1.5 m/s with 2 rad/s yaw; report body roll |
| incline | 10–30° ramps: climb to the plateau **and** stop half way and hold for 3 s |
| side_slope | 8–16° sideways slope; body must stay within 4° of level |
| rough_terrain | random bumps 5–14 cm at 0.8 and 1.5 m/s; must cross 4 m |
| payload | 0–3 kg in the bay; drive and survive an 8 N·s push |
| tilt_height | pitch ±0.2/0.4 rad, roll ±0.25 rad, min/max height; measure what is achieved |
| crawl | crawl mode under bars at 0.38–0.50 m |
| jump_flat | requested heights 0.1/0.2/0.3 m; measure wheel clearance |
| step_up | steps 8–22 cm, run‑up at 1 m/s, jump requested at 0.7/0.9/1.1 m; success rate |
| stairs | 3 steps, treads 0.30/0.45/0.60 m, using the stair policy |
| obstacle_hop | hop over 6/10/14 cm bars |
| fall_recovery | tip it over with motors off, then try to self‑right |

The 45 test runs (15 tests × 3 designs) run in parallel on 4 cores in under a minute. Results go to `sim/results/<variant>.json`.

## 5.7 Iterating the design: V1 → V2 → V3

![iterations](../images/iterations_comparison.png)
![COM](../images/com_iterations.png)

**V1 "big head".** The first sketch went for the cartoon look: battery and sensors up in the head, 120 mm wheels, narrow 300 mm track. Body COM 74 mm above the hip. Result: it balanced, but recovered cleanly only from 8.2 N·s pushes, and at 2 m/s it **ran away and fell**. The motors ran out of speed (chapter 2.12).

**V2 "low COM".** The battery moved under the hip axis and the track widened to 364 mm. Body COM fell to +29 mm, push recovery rose to 9.1 N·s, and it crawled under a lower bar (0.40 m). It still ran away at 2 m/s, because the wheels were the same.

**V3 final.** 150 mm wheels (2 m/s now uses 74 % of the motor speed instead of 93 %), longer shins for more jump stroke, the payload bay sunk onto the hip axis, and the real CAD masses. **Safe top speed doubled to 2.0 m/s**, push recovery reached 10 N·s, jump clearance 22 cm. Trade‑off: the bigger wheel gives less pushing force per Nm, so the steepest ramp dropped from 30° to 25° (still above the 20° target), and the robot gained 0.7 kg.

The lesson: change **one idea per iteration** and keep the tests identical, so you know what each change bought.

## 5.8 Iterating the controller: the bug diary
Most of the work was here. The tests found each problem; detailed logs found the cause.

| # | What the test showed | Cause found in the logs | Fix |
|---|---|---|---|
| 1 | first jump falls over | crouch too abrupt (leg swung 15°), thrust stopped early, landing never detected | ramp the crouch, wait until legs are vertical, plan thrust from energy |
| 2 | "landed" while still 20 cm in the air | the accelerometer feels the legs' internal pull, not just the ground | detect touchdown by **leg compression** after apex |
| 3 | body spins nose‑down after take‑off | leg force 5 mm off the body COM, and the wheel spinning up as it unloads | feed‑forward hip torque cancelling the moment; wheel torque limited in thrust; legs held in the body frame in flight |
| 4 | wheel clips the step lip | legs extended for landing too early | two jump modes: "onto a step" stays tucked |
| 5 | step jumps only work in a 5 cm window | crouch takes a variable time | give the controller the edge distance; it fires when `remaining ≤ v·(t_thrust + t_clear) + R + margin` |
| 6 | knee hits the step edge | the front knee sticks out in front of the wheel | include it in the take‑off distance |
| 7 | after landing the robot reverses at 2× the commanded speed | stored position error makes it "catch up" | re‑anchor the position reference when starting or reversing, keep it when stopping (so it still holds on slopes) |
| 8 | that fix broke ramp climbing | anchoring on every change removed the slope‑holding integral | anchor only on start / reverse |
| 9 | turns at half the commanded rate | tyre scrub vs a P‑only yaw loop | yaw‑rate PI |
| 10 | push test said 21 N·s | the robot "survived" with legs flailing through 120° | stricter test → honest 10 N·s |
| 11 | camera rendered garbage | several OpenGL contexts in one process | close each renderer after use |

## 5.9 Software‑in‑the‑loop: testing the brain
`sim/companion_sim.py` connects the real Raspberry Pi code to the simulated robot through `SimRobotIO`. That class returns telemetry, rendered camera frames, sonar/cliff readings and a virtual GPS, and accepts the same `Command` messages the Teensy would.

* **Line following.** A floor texture with a red track with S‑bends. The camera image goes through the real `LineFollower`. Gains were tuned by a parallel sweep. Result: 2.6 cm mean error, by day and in a dark scene where the brain switches the headlight on.
* **Obstacles.** A wall with a gap: the occupancy grid + A* finds the detour. A closed box: it scans 360° and halts with "no alternative path".
* **Mission Planner.** A pymavlink "ground station" uploads a 4‑item mission over real UDP, arms and starts it. The robot follows the shortest street routes on a street grid. A broken‑down van blocks the shortest street, and the robot detours around the block.
* **App.** A headless Chromium opens the real app, drives, jumps, crawls, adds a waypoint and presses E‑STOP.

## 5.10 What the simulation can't promise
Real tyres, gearbox backlash, flexing prints, cable drag, real lighting and fisheye distortion are only approximated. Treat the numbers as "the design is capable of this", then re‑tune on the real robot (chapter 7 has the recipes).
