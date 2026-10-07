# Chapter 6 — How the electronics work

Wiring diagrams: [`electronics/power_distribution.png`](../../electronics/power_distribution.png) and [`electronics/signal_wiring.png`](../../electronics/signal_wiring.png). Pin numbers: [`firmware/src/config.h`](../../firmware/src/config.h).

## 6.1 Three ideas you need
* **Voltage (V)** is "pressure", **current (A)** is "flow", **power (W) = V × A**.
* Wires and fuses are sized for **current**. Electronics are damaged by the wrong **voltage**.
* Every circuit needs a return path. All grounds (0 V) must be connected, at **one point**.

## 6.2 The battery
**6S LiPo**: six lithium cells in series, 3.7 V each nominal → 22.2 V (25.2 V full, ~19.8 V empty). The hip motors are rated for 24 V systems, so 6S is the right match; 8S (33.6 V) would damage them. 5000 mAh × 22.2 V = **111 Wh** of energy.

**Power budget (estimates):**

| Situation | Power | Current at 22 V |
|---|---|---|
| Pi 5 + cameras + Teensy + sensors | ~12 W | 0.5 A |
| standing and balancing (hips holding ~1.8 Nm each) | ~30–50 W | 2 A |
| driving 1–2 m/s | ~60–100 W | 3–5 A |
| jump burst (4 hips near 7 Nm and 10 rad/s + wheels) | ~500–600 W for 0.2 s | ~25 A |

So expect roughly **1–1.5 hours** of mixed use. A "50C" 5 Ah pack can deliver 250 A, so the bursts are no problem for the battery.

## 6.3 The power path, step by step
1. **XT90‑S anti‑spark connector.** Plugging a battery into big capacitors causes a spark that pits the connector. The "S" version has a resistor that pre‑charges them first.
2. **Main fuse, 40 A.** Protects the wiring if something shorts. 12 AWG silicone wire handles 40 A comfortably. A slow‑blow blade fuse doesn't trip on 0.2 s jump bursts.
3. **Logic branch (before the e‑stop), 5 A fuse.** Feeds the computers so they stay alive when the e‑stop is pressed. You can still see what happened.
4. **E‑stop (mushroom switch, 2 poles).** Pole 1 cuts the motor bus. Pole 2 opens a sense wire to Teensy pin 32, so the software knows too and sets torque to zero.
5. **Power distribution board (PDB).** Splits the motor bus into six XT30 outputs with 10 A fuses (16 AWG wire). It also carries:
   * **1000 µF low‑ESR capacitor**: a local "reservoir" for sudden current demands;
   * **TVS diode (SMBJ28A)**: when the robot brakes hard or lands, the motors act as generators and push current back. The voltage spikes, and the TVS clamps it before the motor drivers are damaged.
6. **Buck converter #1, 5 V / 5 A → Raspberry Pi 5.** A buck converter steps the voltage down efficiently (~90 %), unlike a linear regulator, which would burn (22 − 5) V × 5 A = 85 W as heat. The Pi needs 5 V at up to 5 A. Because it is fed through its GPIO pins, set `usb_max_current_enable=1`.
7. **Buck converter #2, 5 V / 3 A → sensors and LEDs.**
8. The **Teensy** gets 5 V through its USB cable from the Pi.

**Grounding:** all grounds meet at one star point on the PDB. If the Pi's ground went through a different path than the motors', motor current would flow through the signal grounds and corrupt the CAN and sensor signals.

## 6.4 Talking to the motors: CAN bus
CAN is the bus used in cars. Two wires, **CAN‑H** and **CAN‑L**, carry the signal as a *difference* between them, so noise that hits both wires equally cancels out. That is why it works next to noisy motor cables.

* **Termination**: a 120 Ω resistor at **each end** of the bus absorbs reflections. Missing or extra terminators are the #1 CAN problem.
* **Transceiver**: the Teensy's CAN pins are logic signals (TX/RX). An **SN65HVD230** chip turns them into the differential bus signal. It runs on 3.3 V, which matches the Teensy.
* **Messages**: each frame has an **ID** (who it's for) and up to 8 data bytes. Hip motors listen on IDs 1–4 and answer on 0x11–0x14. Wheel motors use 0x141/0x142.
* **Bus load**: at 1 Mbit/s an 8‑byte frame takes ~0.13 ms. Each millisecond every motor needs one command and one reply. Four frames per bus = 0.52 ms = 52 % load. One bus for all six motors would need 1.56 ms, which doesn't fit in 1 ms. That is why BOLT uses **three buses** (left hips, right hips, wheels). The Teensy 4.1 has exactly three CAN controllers.

## 6.5 The IMU: SPI
The ICM‑42688‑P measures rotation rates (gyro) and acceleration. **SPI** is a fast synchronous bus with four wires: clock (SCK), data out (MOSI), data in (MISO) and chip select (CS). The Teensy pulls CS low, clocks out a register address with the read bit set (`0x1D | 0x80`), and clocks in 14 bytes: temperature plus 3 accelerations plus 3 rotation rates, 16 bits each. At 10 MHz that takes ~12 µs, so reading it every millisecond is easy.

Scaling: ±16 g range → 2048 counts per g; ±2000 °/s → 16.4 counts per °/s.

## 6.6 Pi ↔ Teensy: USB serial
The Teensy appears as `/dev/ttyACM0` on the Pi. The two exchange small binary frames (chapter 7.3): commands at 50 Hz, telemetry at 100 Hz. The same cable powers the Teensy and lets the Pi re‑flash it, so no extra wires are needed.

## 6.7 Sensors
* **Ultrasonic (RCWL‑1601).** A 10 µs pulse on TRIG makes it chirp at 40 kHz. ECHO stays high until the echo returns. Distance = time × 343 m/s ÷ 2 (there and back) = **0.1715 mm per µs**. The Teensy measures the echo with interrupts, firing one sensor every 30 ms so they don't hear each other. The RCWL‑1601 is used instead of the classic HC‑SR04 because **its echo output is 3.3 V**. The Teensy 4.1 is *not* 5 V tolerant.
* **Sharp GP2Y0A21 IR distance (10–80 cm).** It sends an infrared beam and measures the angle of the reflection. The output is an analog voltage (higher = closer, about `d[cm] ≈ 29.99 · V^-1.173`). It can reach 3.1 V, so a **voltage divider** (10 kΩ + 20 kΩ, ratio 2/3) keeps it below 2.1 V for the 3.3 V ADC. Two of them look down‑forward from the chin: if the floor "disappears" (reading jumps beyond the expected 0.27 m), there's a drop‑off ahead.
* **Battery voltage.** Divider 100 kΩ / 6.8 kΩ: 25.2 V × 6.8/106.8 = 1.60 V at the ADC pin, safely below 3.3 V. The firmware multiplies back by 15.7.
* **GPS (u‑blox M10).** Sends text lines over UART (serial, 115200 baud), for example `$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,…*47`. That is time, latitude, longitude, fix quality, satellites, accuracy and altitude. `gps.py` checks the checksum after `*` and decodes it.
* **Compass (IST8310).** On I²C, a 2‑wire bus (SDA data, SCL clock) where each chip has an address (0x0E). Optional here.
* **Cameras.** MIPI‑CSI ribbon cables go straight into the Pi 5's two camera ports. That is fast enough for HD video without loading the CPU.

## 6.8 Outputs
* **LED eyes (WS2812B).** Each LED has a tiny controller. A single data wire carries 24 bits per LED (green, red, blue) at 800 kHz, and each LED keeps its 24 bits and passes the rest on. The Teensy library uses a hardware serial port with DMA, so it costs no CPU time. A 330 Ω resistor in the data line protects against ringing. *Note:* these LEDs officially want a 5 V data signal (≥ 3.5 V); 3.3 V usually works. If the eyes flicker, add a 74AHCT125 level shifter.
* **Headlights and IR illuminators.** The Teensy can't supply 300 mA, so it switches an **AO3400 MOSFET** (a transistor that acts like a switch, fully on at 2.5 V gate). **PWM** turns it on and off thousands of times a second; the fraction of on‑time sets the brightness. The 100 Ω gate resistor limits switching current; the 10 kΩ pull‑down keeps the light off while the Teensy boots. A 1 W white LED from 5 V needs a series resistor of about (5 − 3.2) V / 0.3 A ≈ 6 Ω (1 W), or a small constant‑current driver.
* **Buzzer** on a PWM pin (beeps at startup and on errors).

## 6.9 Safety layers
| Layer | What it catches |
|---|---|
| hardware e‑stop | everything; physically cuts motor power |
| motor CAN timeout (50 ms) | Teensy crashed or cable unplugged: motors go limp on their own |
| Teensy watchdog (100 ms) | the control loop hangs: the Teensy resets |
| command timeout (300 ms) | Pi or Wi‑Fi died: the robot stops and keeps balancing |
| fall detection | tipped past 49° pitch / 43° roll: torque off, so it doesn't thrash on the floor |
| battery monitor | below 21.0 V: limited to 0.5 m/s; below 19.8 V: motors off (protects the LiPo) |
| calibration check | never enables motors with unknown leg zero positions |

## 6.10 Assembly order for the wiring (beginner tips)
1. Build and test the **power path without motors**: battery → fuse → e‑stop → PDB. Measure voltages with a multimeter at every stage.
2. Add the 5 V bucks, **adjust and measure 5.0–5.1 V before** connecting the Pi.
3. Connect the Teensy to a PC first. Flash the firmware and watch the serial output.
4. Add CAN bus 3 (wheels) with one motor, test, then add the second, then the hip buses.
5. Check termination with the battery **unplugged**: measure between CAN‑H and CAN‑L, expect **60 Ω** (two 120 Ω in parallel).
6. LiPo safety: charge in a fire‑safe bag with a balance charger, never below 3.3 V per cell, unplug for storage.
