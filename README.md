# BOLT — a cute, 3D‑printable wheel‑legged balancing robot

**BOLT** balances on two wheels at the ends of two legs, like Boston Dynamics' *Handle* or ETH's *Ascento*. It drives fast, leans, crawls, carries a payload and jumps onto curbs. It follows coloured lines (day or night), avoids obstacles and halts when no path exists. It runs GPS missions from **Mission Planner** over the shortest road route, and you drive it from a **phone app**.

| | | |
|---|---|---|
| ![](docs/images/render_hero.png) | ![](docs/images/jump_step_16cm.gif) | ![](docs/images/postures.png) |

> Everything here is designed and **tested in simulation** (MuJoCo + the real firmware controller + the real Raspberry Pi code). It has not been built yet. Expect to re‑tune gains on hardware. Read the limits section before you buy motors.

> **New to robotics?** Read the [beginner's guide](docs/guide/README.md). It covers how every part was designed, the physics, the linkages, how the simulations and design iterations were done, and how the electronics and code work.

## What's in this repository

| Deliverable | Where | |
|---|---|---|
| **1. 3D‑printable parts (solid parts)** | `cad/stl/*.stl` (22 parts, print‑oriented) · parametric generator `cad/build_parts.py` · assembly `cad/assembly_nominal_pose.stl` | [print & build guide](docs/05_print_and_build.md) |
| **2. Simulation + test‑driven design iterations** | `sim/` — MuJoCo model, LQR design, 15‑test suite, 3 design iterations, brain‑in‑the‑loop demos | [simulation report](docs/04_simulation_report.md) |
| **3. Control code** | `firmware/` — Teensy 4.1, 1 kHz balance + 5‑bar legs + jump state machine, CAN motor drivers · `companion/` — Raspberry Pi 5 brain (line following, A* routing, obstacle halt, MAVLink) | [software guide](docs/06_software.md) |
| **4. Circuit diagrams and settings** | `electronics/*.svg/.png` · pin maps, motor/Pi/GPS/Mission Planner settings | [electronics & settings](docs/03_electronics_and_settings.md) |
| **5. Controller app** | `app/` — installable phone app (PWA) served by the robot | [software guide §6.4](docs/06_software.md#64-controller-app-app) |
| Design rationale, COM, requirement traceability | | [design overview](docs/01_design.md) · [bill of materials](docs/02_bom.md) |
| **Full explanation for beginners** | physics, mechanism, CAD, simulation & iterations, electronics, code walkthrough | [docs/guide](docs/guide/README.md) |

## The robot in numbers (final design V3, simulation)

* 6.4 kg, 0.49 m tall, 0.40 m wide. Ride height is adjustable from 0.21 to 0.39 m (hip).
* 4 × DM‑J4310 hip actuators drive two 5‑bar legs; 2 × MF9025 hub motors drive the 150 mm wheels. Power is a 6S 5000 mAh LiPo.
* Teensy 4.1 runs the 1 kHz balance loop. Raspberry Pi 5 handles vision, navigation, the app and MAVLink. Two 220° fisheye cameras give 360° coverage.
* Speed: **2.0 m/s** tracked cleanly. Turns at 2 rad/s while doing 1.5 m/s.
* Slopes: **25°** ramps climbed and held. **16°** side slopes with the body kept level.
* Rough ground: **8 cm** bumps at 1.5 m/s. Recovers cleanly from a **10 N·s** shove.
* Payload: **3 kg**, in a bay centred on the hip axis.
* Jumps: **22 cm** wheel clearance. Steps up onto **20 cm** steps (100 % at a 1 m/s run‑up). Hops over **14 cm** obstacles.
* Posture: tilts **±23°**, leans **±15°**, crawls under a **0.42 m** bar.
* Line following: **2.6 cm** mean error, day and night (auto headlight/IR).
* Navigation: A* detours around obstacles, re‑routes around blocked roads, and **halts** if no path exists.

## Limits — read this first

* **Multi‑step stairs don't work reliably.** Single steps and curbs up to 20 cm work. House stairs (25–30 cm treads) fail: there isn't enough run‑up between steps for these hip motors. The fix is stronger hip actuators; see [§4.6](docs/04_simulation_report.md).
* **Self‑righting** from lying flat doesn't work yet. The robot goes limp and you stand it up.
* **Camera‑based step locating is experimental.** Start "Step up" with BOLT 0.5–0.8 m in front of the step.
* Datasheet values to **verify on your parts**: motor bolt patterns, the MF9025 torque constant, and CAN details. They are parameters in the code.

## Quick start

```bash
pip install mujoco numpy scipy matplotlib trimesh manifold3d opencv-python-headless pymavlink aiohttp pyyaml pytest
python3 cad/build_parts.py                    # STLs + mass report
python3 sim/lqr_design.py                     # controller gains (also writes the firmware header)
python3 sim/test_suite.py V3                  # run the 15 requirement tests
python3 sim/companion_sim.py --demo app       # drive the simulated robot from the real app: http://localhost:8080
python3 -m pytest -q companion/tests          # firmware<->Pi link byte compatibility, GPS, A*, re-routing
cd firmware && pio run                        # build the Teensy firmware
```

## Repository map
```
design/      params.py (single source of truth, V1..V3), kinematics.py (5-bar)
cad/         build_parts.py -> stl/, assembly/, mass_report.json
sim/         robot_model.py, lqr_design.py, controller.py (ctypes), simulator.py,
             test_suite.py, companion_sim.py, make_report.py, results/
firmware/    src/main.cpp, src/config.h, lib/bolt_core (controller, shared with sim),
             lib/bolt_drivers (CAN motors, IMU, sonar, eyes), lib/bolt_link, tools/check_build.sh
companion/   robot_brain/ (behaviors, perception, navigation, mission, app_server, link, gps),
             config.yaml, tools/bolt_cli.py, systemd/, tests/
app/         index.html, app.js, style.css, manifest + service worker (PWA)
electronics/ make_diagrams.py -> power_distribution.*, signal_wiring.*
docs/        01_design .. 06_software, images/, guide/ (beginner's guide, 8 chapters)
```
