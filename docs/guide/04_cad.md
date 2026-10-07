# Chapter 4 — How the CAD was made (parts designed by code)

## 4.1 The idea: constructive solid geometry
Instead of clicking in a CAD program, every part is built in Python (`cad/build_parts.py`) from simple solids:

* **add** (`a + b`): glue two solids together;
* **subtract** (`a − b`): drill holes and pockets;
* **intersect** (`a ^ b`): keep only the overlap, for example "the part of the shell below the split line";
* **hull**: shrink‑wrap several shapes (eight spheres → a rounded box; two circles → a link with round ends).

The geometry kernel is **manifold3d**, which guarantees closed, watertight solids that every slicer accepts. **trimesh** exports STL and computes volume and centre of mass.

Why code? Every dimension comes from `design/params.py`, so the parts always match the kinematics and the simulator, and changing the leg length re‑generates everything in about 2 seconds.

## 4.2 Helper functions (top of the file)
| Helper | Makes |
|---|---|
| `box(x0,x1,y0,y1,z0,z1)` | a block from coordinates |
| `cyl(r, h, axis)` | a cylinder along x, y or z (holes, bosses, motors) |
| `rounded_box(…, r)` | hull of 8 spheres: the cute rounded body |
| `rounded_plate_xz` | a flat plate with rounded corners (side plates) |
| `bolt_circle(pcd, n, d…)` | n holes on a circle (motor flanges) |
| `link2d(length, r_a, r_b, w)` | 2‑D outline of a link with round ends, later extruded (thighs, shins) |

## 4.3 How each part is shaped and why
* **Shell (`shell_upper` → front/rear halves).** The outer envelope is a rounded box of 210 × 240 × 230 mm with 42 mm corner radius (the friendly "toaster" look). The shell is the envelope minus the same box shrunk by 3 mm (`envelope(-WALL)`). Then holes are cut: hip slots for the thighs, roof openings for the payload bay and the electronics cartridge, eye windows, the fisheye camera hole and IR LED holes, sonar "mouth" and "cheek" holes angled ±45°, end‑stop peg clearances, vent slots. Magnet bosses are added on the split line. It is cut at x = 0 into two halves so each fits a 256 mm bed.
* **Belly pan.** The lower part of the same envelope with a rear opening for the battery, skid ribs and drain holes.
* **Side plates.** 8 mm structural plates with clearance holes for the motor rotors, motor bolt circles, lightening windows to save weight, edge holes to the floor and top frame, and the **calibration pegs** computed by `end_stop_points()` from the kinematics.
* **Floor plate / top frame.** Tie the side plates into a box. The floor has battery rails and a stop. The top frame has the payload‑bay drop‑in, the sled slot with guide fingers, and heat‑set insert holes for a roof rack.
* **Payload bay / lid.** A 120 × 124 mm box hanging from the top frame, centred on the hip axis, with tie‑down eyelets and drain slots. The lid is cut from the envelope so it follows the roof curve.
* **Electronics sled / cap.** A vertical plate with Pi 5 and Teensy‑carrier standoffs, cable‑tie slots and thumb‑screw holes. The cap carries the short GPS stub and puck seat.
* **Legs.** `link2d` outlines extruded 10 mm. Thigh: motor flange spigot, bolt circle with counter‑bores, knee pin hole, lightening slot. Shins: 608 bearing pocket at the knee. The rear shin has the wheel‑motor stator bolt circle and an axle boss. The front shin has a ring that rides on that boss.
* **Wheel rim + tyre.** The rim is a cup bolted to the motor's output face, with retaining lips. The tyre is a TPU ring with grooves that match the lips and a chevron tread.
* **Bumpers, visor, brackets.** The bumper is a band between envelope+7 mm and the envelope, height‑limited to the "cheeks". The visor is a 1.5 mm skin over the face.

## 4.4 Print orientation
Each part is rotated so its biggest flat face sits on the bed (`print_rot`), then moved so it rests on z = 0 and is centred. Structural plates print flat, so the layers run along the load direction.

## 4.5 Weighing the robot before it exists
For every part the script computes `volume × material density × fill factor` (fill factor ≈ walls + infill, for example PA‑CF 0.70) and the centre of mass. It writes `cad/mass_report.json`. The simulator reads the body total (1.39 kg of prints) and its COM, the per‑leg and per‑wheel print masses. Motors, battery, Pi etc. are added as known catalogue masses at their positions from `params.py`. When you build the real robot, weigh the parts and correct `params.py`. The controller is regenerated from it.

## 4.6 Assembly outputs
* `cad/stl/*.stl` — print‑ready parts;
* `cad/assembly/*.stl` — parts in robot coordinates (metres), used by the simulator as visual meshes;
* `cad/assembly_nominal_pose.stl` — the whole robot standing, legs posed with the inverse kinematics.

## 4.7 Changing the design
* Longer legs: edit `thigh` / `shin` in `design/params.py`, re‑run `cad/build_parts.py`, `sim/lqr_design.py` and the test suite.
* Different motor: edit `HIP_PCD`, `HIP_STATOR_PCD`, `WHEEL_*_PCD` at the top of `cad/build_parts.py` and the motor data in `params.py`.
* Different look: `R_SHELL` (roundness), eye positions in `shell_upper`, colours in `sim/robot_model.py`.
