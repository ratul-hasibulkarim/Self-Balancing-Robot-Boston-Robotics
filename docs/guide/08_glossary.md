# Chapter 8 — Glossary and learning path

## Glossary
| Term | Meaning |
|---|---|
| **A\*** | shortest‑path search that uses an estimate of the remaining distance to search efficiently |
| **actuator** | something that makes motion (here: motors) |
| **back‑EMF** | the voltage a spinning motor generates against its supply; limits torque at high speed |
| **buck converter** | efficient circuit that steps a voltage down (22 V → 5 V) |
| **CAN bus** | robust 2‑wire differential bus used for the motors |
| **COM** | centre of mass, the point where the robot's weight effectively acts |
| **controller (LQR)** | the rule that computes torques from the measured state; LQR = linear‑quadratic regulator, finds the optimal gains |
| **CRC** | checksum that detects corrupted messages |
| **ctypes** | Python module used to call C/C++ code; how the sim runs the firmware controller |
| **DOF** | degrees of freedom, the number of independent ways something can move |
| **equality constraint** | in MuJoCo, a rule that keeps two points together; closes the 5‑bar loop |
| **feed‑forward** | adding the torque you *know* you'll need, instead of waiting for an error |
| **five‑bar linkage** | 5 links in a loop with 2 motors; BOLT's leg |
| **gain scheduling** | switching or interpolating controller gains as conditions (here leg length) change |
| **GeoJSON** | text format for map shapes (roads as lines of lat/lon points) |
| **IMU** | inertial measurement unit: gyroscope + accelerometer |
| **inverse kinematics** | finding joint angles that put the foot where you want it |
| **Jacobian** | table of how much each output (L, θ) changes per joint movement; maps forces to torques by its transpose |
| **Lagrangian mechanics** | deriving equations of motion from kinetic and potential energy |
| **linearisation** | approximating the equations near a pose as straight‑line (linear) relations |
| **log‑odds grid** | map whose cells accumulate evidence of being occupied or free |
| **Mahony filter** | fuses gyro and accelerometer into an attitude estimate |
| **MAVLink** | messaging protocol used by drones/rovers and Mission Planner |
| **MJCF** | MuJoCo's XML model format |
| **MuJoCo** | the physics simulator used here |
| **NMEA** | text format GPS receivers speak |
| **non‑minimum phase** | a system that first moves the "wrong" way (roll back before going forward) |
| **odometry** | estimating travel from wheel rotation |
| **PWM** | switching a signal on and off fast; the on‑fraction sets average power |
| **pure pursuit** | path follower that steers towards a point a fixed distance ahead on the path |
| **PWA** | progressive web app: a web page installable like a phone app |
| **Riccati equation** | the equation whose solution gives the LQR gains |
| **SIL** | software‑in‑the‑loop: real software driving a simulated robot |
| **singularity** | linkage pose where it loses the ability to move/push in some direction |
| **SPI / I²C / UART** | common chip‑to‑chip communication buses |
| **state (x)** | the minimal set of numbers that describes the robot's motion now |
| **TVS diode** | clamps voltage spikes |
| **virtual leg** | the imaginary straight line from hip to wheel that the controller reasons about |
| **virtual work** | power in = power out → relation between joint torques and leg forces |
| **watchdog** | timer that resets the processor if the program stops responding |

## A learning path for a beginner
1. **Programming**: Python basics, then C/C++ basics (Arduino is a good start).
2. **Electronics**: Ohm's law, voltage dividers, using a multimeter, a few Arduino sensor projects (ultrasonic, IMU, LEDs, a MOSFET).
3. **Mechanics**: free‑body diagrams, torque = force × distance, levers. Then planar linkages: draw this repo's 5‑bar on paper and compute one pose by hand (chapter 3.2).
4. **Dynamics and control**: build a simple two‑wheeled balancing robot with a PID controller first. Then learn state‑space and LQR (Brian Douglas' "Control Systems" videos are excellent), and re‑derive chapter 2.3.
5. **Simulation**: MuJoCo's tutorial notebooks; then read `sim/robot_model.py` and change one parameter in `design/params.py` to see what happens in `sim/test_suite.py`.
6. **Robotics software**: OpenCV basics (colour thresholding), A* on a grid (many visual tutorials), then ArduPilot/MAVLink concepts.
7. **Build in stages**: one leg on a bench, then both legs with the robot held, then balancing with a safety tether, then jumps on soft ground.
