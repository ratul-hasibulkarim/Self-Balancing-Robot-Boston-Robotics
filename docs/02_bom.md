# 2 · Bill of materials

Prices are rough 2026 street prices (USD) for orientation only.

## Actuation & power
| Qty | Part | Notes | ~USD |
|---|---|---|---|
| 4 | **DM‑J4310‑2EC** quasi‑direct‑drive actuator (Damiao) | hips; 10:1, 3 Nm cont. / 7 Nm peak, CAN, MIT mode, dual encoder | 4 × 100 |
| 2 | **LK‑TECH MF9025v2** hub motor with driver (CAN version) | wheels; direct drive, ~4 Nm peak | 2 × 140 |
| 1 | 6S 5000 mAh 50C LiPo, XT90‑S | e.g. CNHL / Tattu; fits 155 × 52 × 46 mm | 70 |
| 1 | XT90‑S anti‑spark pair, 40 A blade fuse + holder, 5 A fuse | | 10 |
| 1 | Mushroom e‑stop, 2‑pole, ≥ 32 A DC | pole 1 motor bus, pole 2 sense loop | 15 |
| 1 | Small PDB / bus bar, 6 × XT30, 6 × 10 A fuses | or a 3D‑printed bus bar + inline fuses | 20 |
| 1 | 1000 µF 50 V low‑ESR capacitor + SMBJ28A TVS | motor bus protection | 3 |
| 1 | 5 V 5 A buck (Pololu D36V50F5 or Matek BEC 12S‑PRO) | Raspberry Pi 5 | 25 |
| 1 | 5 V 3 A buck | sensors + LEDs | 8 |
| — | Silicone wire 12 / 16 / 20 AWG, XT30 pigtails, CAN twisted pair, JST‑GH | | 30 |

## Computing & sensors
| Qty | Part | Notes | ~USD |
|---|---|---|---|
| 1 | **Teensy 4.1** | real‑time controller, 3 × CAN | 32 |
| 3 | SN65HVD230 CAN transceiver breakout | one per bus | 3 × 3 |
| 1 | ICM‑42688‑P IMU breakout (SPI) | | 12 |
| 1 | **Raspberry Pi 5, 8 GB** + active cooler + 64 GB A2 microSD | brain | 95 |
| 2 | **Arducam IMX219 NoIR with M12 220° fisheye lens**, Pi 5 FPC cables (22‑pin) | front + rear | 2 × 35 |
| 1 | **Holybro M10 GPS** (u‑blox M10 + IST8310 compass) | the usual Pixhawk/Mission Planner puck | 40 |
| 4 | RCWL‑1601 ultrasonic (3.3 V HC‑SR04) | | 4 × 2 |
| 2 | Sharp GP2Y0A21YK0F IR distance (10–80 cm) | drop‑off sensors | 2 × 8 |
| 2 | WS2812B 12‑LED ring (37 mm) | eyes | 2 × 3 |
| 2 + 4 | 1 W white LED + 850 nm IR LEDs, 2 × AO3400 MOSFET | headlights / night vision | 8 |
| 1 | Piezo buzzer, resistors (100k/6.8k, 10k/20k, 330R, 100R, 10k) | | 3 |

## Mechanical hardware
| Qty | Part | Use |
|---|---|---|
| 12 | 608‑2RS bearing (8 × 22 × 7) | knees (4) and foot joints (2), spares |
| 6 | M8 × 30 shoulder bolt + nyloc | knee / foot pins |
| ~60 | M3 heat‑set inserts (M3 × 5.7) | frame, sled, shell bosses |
| ~80 | M3 SHCS (8–16 mm), M4 × 12 (wheel stators) | |
| 16 | M2.5 × 6 + standoffs | Pi 5, cameras |
| 14 | 6 × 3 mm N52 disc magnets | shell, lids |
| 4 | Rubber grommets (M2.5) | Raspberry Pi damping (NOT the IMU) |
| 2 | M3 thumb screws | clamp the electronics sled |
| 1 | 20 mm Velcro strap | battery |

## Filament (≈ 2.5 kg printed)
| Material | Parts | ≈ g |
|---|---|---|
| PA‑CF (or PETG‑CF) | side plates, floor, top frame, thighs, shins | 1 050 |
| PETG‑CF | belly pan, sled, wheel rims | 540 |
| PETG | shell halves, payload bay + lid, cap, tray, brackets | 520 |
| TPU 95A | tyres, bumpers | 410 |
| clear PETG | face visor (smoke‑tinted) | 15 |

**Total ≈ USD 1 250** (motors are ~60 %). A budget variant with hobby servos for the legs cannot jump and is not covered here.
