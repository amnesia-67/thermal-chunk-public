# Hybrid Temperature Control System

**Precision Thermal Regulation with Dual Algorithms (PID + Constant-Power Model)**

This Arduino project implements an intelligent hybrid temperature controller for a Peltier (TEC) system or aluminum heating/cooling plate. It uses two distinct control algorithms that automatically hand off depending on the proximity to the temperature setpoint:

* **Algorithm 2 – Lag-Aware PID (Outer Control)**
  Used when temperature is more than 1 °C away from target (up to 5 °C).
  Provides fast, predictive, and stable approach using:

  * 2-DOF PI-D feedback
  * g–h observer for sensor lag estimation
  * feed-forward ambient compensation
  * predictive braking and polarity soft-start

* **Algorithm 1 – Constant-Power Model (Inner Control)**
  Used when temperature is within ±1 °C of target.
  Provides highly stable precision holding with minimal oscillation using:

  * exponential power model
  * ±2 % “nudge” for fine adjustments
  * polarity hysteresis to prevent mode flapping

Together, the hybrid system delivers rapid convergence and low steady-state ripple, ideal for thermal chambers, hotplates, or material testing rigs.

---

## Features

* Dual-mode hybrid control (PID + Constant-Power)
* Automatic algorithm handoff based on error magnitude
* Full-power “boost” beyond ±5 °C for fast startup
* Sensor-lag prediction (g–h filter + lead compensation)
* Predictive braking to prevent overshoot
* Soft-start ramp on polarity reversals
* Per-setpoint duty learning table (persistent tuning)
* Live serial command interface for runtime tuning
* Real-time CSV telemetry output

---

## Control Regions

| Error from Setpoint | Active Algorithm    | Behavior                                              |
| ------------------- | ------------------- | ----------------------------------------------------- |
| > 5 °C              | Max Power Mode      | Full heating/cooling to approach target quickly       |
| 1 – 5 °C            | Algorithm 2 (PID)   | Predictive PI-D control with feed-forward and braking |
| ≤ 1 °C              | Algorithm 1 (Model) | Constant-power fine control for tight stability       |

Thresholds can be changed at runtime:

```
mid X     → outer (PID) region width [default 5.0 °C]
tight X   → inner (model) region width [default 1.0 °C]
```

---

## Hardware Setup

| Component           | Description                                             |
| ------------------- | ------------------------------------------------------- |
| Controller          | ESP32 (Arduino core v3)                                 |
| Power Driver        | BTS7960 H-bridge                                        |
| Temperature Sensors | DS18B20 (up to 16 per bus)                              |
| Bus A / Bus B Pins  | GPIO 4 / GPIO 5                                         |
| PWM Outputs         | RPWM = GPIO 9, LPWM = GPIO 10                           |
| PWM Frequency       | 3 kHz, 10-bit resolution                                |
| Sensor Roles        | A2 = Room Temp, A3 = Disk Temp, others = surface points |

---

## Serial Commands

Type `m` in the serial monitor for menu. Common examples:

```
t24           → set target = 24 °C  
p1 / p0       → enable / disable control  
sel A0        → choose sensor A0 as control PV  
list          → list connected DS18B20 sensors  
s             → stop outputs immediately  

kp/ki/kd X    → tune PID gains  
ghg/ghh X     → tune g–h observer gains  
lead X        → set lead seconds  
brake X       → predictive brake horizon  

knum X        → adjust exponential model K_NUM  
tauc X        → model cooling time constant  
tauh X        → model heating time constant  
nudge X       → ±nudge % for fine control  

db/min/max X  → set deadband / duty caps  
mid X / tight X → adjust hybrid region widths  
res 10|11     → set DS18B20 resolution
```

---

## Serial CSV Output

Each control tick (≈10 Hz) logs:

```
time_s, PV, SP, SPcmd, A[0..], B[0..], mode, duty%
```

* **mode** = H (heating), C (cooling), S (stop)
* **SPcmd** = ramped internal setpoint
* **A[] / B[]** = sensor readings on each bus

---

## Customization

| Parameter             | Description                 | Command              |
| --------------------- | --------------------------- | -------------------- |
| HYBRID_MID_BAND_C     | Outer PID region width      | `mid X`              |
| HYBRID_TIGHT_BAND_C   | Constant-Power region width | `tight X`            |
| K_NUM, T_COOL, T_HEAT | Exponential model shape     | `knum / tauc / tauh` |
| NUDGE                 | Fine correction %           | `nudge X`            |
| PID Gains             | Kp, Ki, Kd                  | `kp/ki/kd`           |
| Feed-Forward          | % per °C slope              | `kff X`              |
| Lead Compensation     | Predictive seconds          | `lead X`             |

---

## Behavior Summary

* On boot: discovers DS18B20 sensors, prints menu and CSV header.
* Continuously reads sensors asynchronously.
* Selects control sensor (default A0).
* Computes predictive PV (`pvCtrl`) via g–h filter.
* Chooses algorithm based on error: full → PID → Model.
* Sends PWM commands to H-bridge.
* Streams real-time telemetry to serial.

---

## Example Operation

For a 24 °C setpoint:

1. System powers on and detects all sensors.
2. If current = 30 °C, error = 6 °C → full cooling (MAX 88 %).
3. As it reaches 25 °C, PID takes over with predictive braking.
4. Within ±1 °C, the constant-power model takes control for stability.
5. Holds 24.0 °C with ±0.05 °C ripple and no mode flapping.

---

## Requirements

* Arduino IDE 2.x (ESP32 board support v3.x)
* Libraries:

  * OneWire
  * DallasTemperature
* Power supply ≥ 12 V / 5 A (for Peltier module)
* Proper heat-sinking and thermal insulation

---

## Author

**Anish Subash**, **Caden Wate**
Hybrid Thermal Control for Smart Surface Systems
Arizona State University, 2025

---
