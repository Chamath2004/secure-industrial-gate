# Secure Industrial Loading Gate — ESP32 IoT Embedded System

An ESP32-based controller for an automated industrial loading gate, built for
**COMP50069 – Hardware, Microcontrollers and Sensors** (Scenario 1). The system
opens on vehicle detection, closes automatically after a tunable delay when the
lane is clear, and runs a layered fail-safe recovery whenever an obstruction is
detected while the gate is closing — all on a fully non-blocking state machine.

---

## Overview

Automated rolling gates in warehouses and fulfilment centres must keep vehicle
lanes moving while guaranteeing they never close on a person, forklift or
stalled vehicle. This project models that trade-off: a servo-actuated gate with
motion detection, ultrasonic obstruction sensing, live supervisor tuning, and a
latched safety-recovery routine.

The controller is implemented as an eight-state finite-state machine. Every
transition is triggered by an explicit condition (a sensor reading, an elapsed
time, or a button event), and the main loop never uses `delay()` for its timing,
so the ultrasonic safety monitor stays responsive at all times.

---

## Features

- **Autonomous operation** – PIR opens the gate; it auto-closes after a
  supervisor-set delay once the lane is clear.
- **Manual override** – hold the button 3 s to take manual control; tap to
  toggle the gate; hold 3 s to return to automatic.
- **Layered safety recovery (design challenge)** – obstruction while closing →
  halt + buzzer → auto-retreat to open → first strike auto-retries after a wait,
  second strike latches into a lockout that needs a button reset **and** a
  verified clear lane.
- **Live tuning** – potentiometer sets the hold-open delay (2–15 s); serial
  commands set the safety distance and servo speed on the fly.
- **Non-blocking design** – `millis()` timing plus a hardware-timer interrupt
  for serial telemetry.
- **OLED status** – contextual messages for every mode on a 128×64 I2C display.

---

## Hardware

| Component | Purpose | ESP32 Pin |
|---|---|---|
| PIR motion sensor | Vehicle detection | GPIO13 |
| Ultrasonic HC-SR04 – TRIG | Obstruction trigger | GPIO5 |
| Ultrasonic HC-SR04 – ECHO | Obstruction echo | GPIO18 |
| Servo motor | Gate mechanism (PWM) | GPIO19 |
| Buzzer | Audible alarm (PWM) | GPIO23 |
| Potentiometer | Hold-open delay (ADC) | GPIO34 |
| Push button | Manual mode / fault reset | GPIO27 |
| OLED display – SDA | Status display (I2C) | GPIO21 |
| OLED display – SCL | Status display (I2C) | GPIO22 |

**Power:** PIR, ultrasonic, servo and buzzer on 5 V; potentiometer and OLED on
3V3; all grounds common.

> **Note (real hardware):** the HC-SR04 ECHO pin outputs 5 V, but ESP32 GPIOs
> are 3.3 V only. Add a voltage divider (e.g. 1 kΩ + 2 kΩ) on the ECHO line.
> Not required in the Wokwi simulation.

---

## Software Requirements

- [Arduino IDE](https://www.arduino.cc/en/software) (or PlatformIO)
- **ESP32 board package** (Arduino-ESP32 core 3.x)
- Libraries:
  - `ESP32Servo`
  - `Adafruit SSD1306`
  - `Adafruit GFX`

---

## Getting Started

### Run in simulation (Wokwi)

1. Open the Wokwi project: **[Wokwi simulation](PASTE_YOUR_WOKWI_LINK_HERE)**
2. Press **Play**.
3. Click the **PIR sensor** and press *Simulate motion* to open the gate.
4. Click the **HC-SR04** and drag its distance below 20 cm while the gate is
   closing to trigger the safety routine.

### Flash to hardware

1. Install the board package and libraries listed above.
2. Open `gate_controller.ino` in the Arduino IDE.
3. Select **ESP32 Dev Module** and the correct COM port.
4. Upload, then open the Serial Monitor at **115200 baud**.

---

## Operating the Gate

| Mode | How to use it |
|---|---|
| **Autonomous** | Default. Motion opens the gate; it closes after the delay once clear. |
| **Manual** | Hold button 3 s to enter → tap to toggle → hold 3 s to exit. |
| **Safety** | Automatic. Obstruction < 20 cm while closing → halt → retreat → retry/lockout. |

---

## Serial Commands (115200 baud)

| Command | Action |
|---|---|
| `HELP` | List all commands |
| `STATUS` | Print one live status line |
| `OPEN` / `CLOSE` | Drive the gate from the terminal |
| `AUTO` | Return to automatic mode |
| `SAFEDIST=<cm>` | Set the obstruction threshold (1–499) |
| `SPEED=<ms>` | Set servo speed in ms per degree (5–200) |

A status line is also printed automatically every 500 ms, e.g.:

```
STATE=CLOSED ANGLE=0 DIST=45.2 MOTION=0 HOLD_MS=5000 SAFEDIST=20 SPEED=50
```

---

## State Machine

| State | Behaviour |
|---|---|
| `CLOSED` | Secured, waiting for a vehicle (PIR) |
| `OPENING` | Servo sweeping 0° → 180° |
| `IDLE_OPEN` | Held open while a vehicle is present |
| `CLOSING` | Servo sweeping 180° → 0°, ultrasonic monitoring |
| `SAFETY_HALT` | Frozen, buzzer chirping after an obstruction |
| `RETREATING` | Returning to the open (safe) position |
| `RETRY_WAIT` | Waiting before an automatic retry (first strike) |
| `LOCKOUT` | Held open, awaiting a verified button reset (second strike) |
| `MANUAL_MODE` | Operator-controlled |

---

## Repository Structure

```
.
├── gate_controller.ino   # Main firmware (Doxygen-documented)
├── diagram.json          # Wokwi circuit definition
├── README.md             # This file
└── docs/                 # Report, diagrams, screenshots (optional)
```

---

## Author

- **[ Your Name ]** — [ Your Student ID ]
- APIIT Sri Lanka | University of Staffordshire
- Module: COMP50069 – Hardware, Microcontrollers and Sensors

## License

Created for academic assessment. Free to reference for educational purposes.
