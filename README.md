# TorqMIT — AK40-10 Torque/Current Verification Tool

Desktop GUI application for torque and current verification of the **CubeMars AK40-10** brushless actuator motor (planetary gearbox 10:1, 14 pole pairs). Supports **UART Servo**, **CAN Servo**, and **CAN MIT** (impedance control) modes with real-time plotting, automated step sweep, and CSV data recording.

---

## Table of Contents

- [Features](#features)
- [Motor Specifications](#motor-specifications)
- [Communication Modes](#communication-modes)
- [Architecture](#architecture)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
  - [Main Application (GUI)](#main-application-gui)
  - [CAN Diagnostic Script](#can-diagnostic-script)
  - [MIT Continuous Test Script](#mit-continuous-test-script)
- [GUI Workflow](#gui-workflow)
  - [Connection](#1-connection)
  - [Input Mode Selection](#2-input-mode-selection)
  - [Parameter Configuration](#3-parameter-configuration)
  - [Sending Commands](#4-sending-commands)
  - [Step Sweep Automation](#5-step-sweep-automation)
  - [Data Recording](#6-data-recording)
  - [Capture & Annotation](#7-capture--annotation)
  - [Emergency Stop](#8-emergency-stop)
- [MIT Mode Protocol](#mit-mode-protocol)
- [Key Formulas](#key-formulas)
- [CSV Data Format](#csv-data-format)
- [Project Structure](#project-structure)
- [Troubleshooting](#troubleshooting)
- [References](#references)

---

## Features

- **3 communication interfaces**: UART Serial (921600 baud), CAN Servo (extended frames), CAN MIT (standard frames)
- **Dual input mode**: Current (A) or Torque (Nm) with automatic conversion via effective Kt
- **Real-time dual-plot**: Current (A) and Torque (Nm) — target vs actual, 30-second rolling window at 20 Hz
- **Automated step sweep**: Configurable start/end/step/dwell with auto-capture at 80% of dwell time
- **CSV data recording**: 14-column format with metadata header, auto-starts on connect
- **Capture & annotation**: Mark data points on graphs with labels (target, actual, error, torque, speed)
- **CAN auto-detect**: Automatically detects motor CAN ID from servo broadcast traffic
- **MIT mode auto-enter**: Aggressive 5-attempt MIT enter sequence with verification
- **Current ramping**: Configurable ramp rate (A/s or Nm/s) to prevent sudden jerks
- **E-STOP**: ESC key shortcut, immediately zeros all outputs
- **Simulation mode**: Full physics simulation with noise for testing without hardware
- **Dark theme UI**: PySide6/Qt6 with custom dark stylesheet
- **Serial monitor**: Optional UART monitoring alongside CAN communication
- **Live telemetry**: Current, torque, speed, position, temperature, voltage, error code

---

## Motor Specifications

| Parameter | Value | Description |
|---|---|---|
| **Kt** | 0.056 Nm/A | Torque constant (pre-gearbox, rotor side) |
| **Gear Ratio** | 10:1 | Planetary gearbox built-in |
| **Gear Efficiency** | 0.86 | Back-calculated from rated operating point |
| **Effective Kt** | 0.482 Nm/A | `0.056 x 10 x 0.86` — torque per ampere at output shaft |
| **Pole Pairs** | 14 | 28 magnets (14 N-S pairs) |
| **Rated Current** | 2.7 A | Continuous maximum |
| **Peak Current** | 7.3 A | Instantaneous maximum |
| **Rated Torque** | 1.3 Nm | Continuous at output shaft |
| **Peak Torque** | 4.1 Nm | Instantaneous at output shaft |
| **Rated Speed** | 435 rpm | Output shaft (= 60,900 ERPM) |
| **Encoder** | 14-bit magnetic | Absolute, single-turn, inner ring |
| **Backlash** | 18 arcmin (0.3 deg) | Undetectable by inner ring encoder |

---

## Communication Modes

| Mode | Interface | Speed | Frame Type | Protocol |
|---|---|---|---|---|
| **UART Servo** | Serial USB | 40 Hz | CRC16-framed packets | `COMM_SET_CURRENT (6)` — current loop |
| **CAN Servo** | socketCAN | 100 Hz | Extended CAN frames | `CAN_PACKET_SET_CURRENT (1)` — current loop |
| **CAN MIT** | socketCAN | 100 Hz | Standard CAN frames | Impedance control: `tau = kp*(p_des-p) + kd*(v_des-v) + t_ff` |

The application auto-detects the interface type from the dropdown selection:
- **COM ports** (Windows) / **`/dev/ttyXXX`** (Linux) → UART Servo
- **`can0`**, **`vcan0`** → CAN bus (tries MIT first, falls back to Servo)
- **Simulation** → Software simulation with physics model

---

## Architecture

```
+--------------------------------------------------------------------+
|                    Torsi_Encoder.py (PySide6 GUI)                   |
|  +------------+  +-------------------------------------+           |
|  |  App GUI   |  |       Real-Time Plots (pyqtgraph)   |           |
|  | - controls |  |  Plot 1: Current (A) Target vs Act  |           |
|  | - params   |  |  Plot 2: Torque (Nm) Target vs Act  |           |
|  | - sweep    |  +-------------------------------------+           |
|  | - record   |                                                    |
|  +-----+------+                                                    |
|        | QTimer 50ms                                               |
|  +-----v--------------------------------------------------+        |
|  |              Motor Controllers                          |        |
|  |  +-----------------+      +-------------------------+   |        |
|  |  | Motor (UART)    |      |  MotorCAN (CAN bus)     |   |        |
|  |  | 40 Hz thread    |      |  100 Hz thread          |   |        |
|  |  | Serial 921600   |      |  socketCAN 1 Mbps       |   |        |
|  |  | CRC16 framed    |      |  MIT + SERVO modes      |   |        |
|  |  +--------+--------+      +------------+------------+   |        |
|  +-----------|-----------------------------|----------------+        |
+--------------|-----------------------------|------------------------+
               |                             |
               v                             v
      +----------------+          +--------------------+
      |  USB-UART      |          |  MCP2515 SPI-CAN   |
      |  (COM port)    |          |  (socketCAN)        |
      +-------+--------+          +---------+----------+
              |                             |
              v                             v
      +---------------------------------------------+
      |         CubeMars AK40-10 Motor               |
      |   (14 PP, 10:1 gear, 4.1 Nm max)            |
      +---------------------------------------------+

      Data output:  data/AK40-10_YYYY-MM-DD_HH-MM-SS.csv
```

### Class Overview

| Class | Lines | Responsibility |
|---|---|---|
| `Motor` | 185–395 | UART Servo mode controller. Background thread at 40 Hz, CRC16 packet framing, current ramping, simulation mode. |
| `MotorCAN` | 402–1083 | CAN bus controller (Servo + MIT). Background thread at 100 Hz, auto-detect motor ID, MIT enter/verify sequence, MCP2515 echo filtering, optional serial monitor, physics simulation. |
| `App` | 1089–1960 | PySide6 GUI (QMainWindow). Builds UI, manages connection lifecycle, graph updates at 20 Hz, sweep automation, CSV recording, capture/annotation. |

---

## Prerequisites

- **Python** >= 3.10
- **OS**: Windows (UART) or Linux (UART + CAN)
- **Hardware** (for real motor testing):
  - CubeMars AK40-10 motor + driver board
  - USB-UART adapter (for UART mode) OR
  - MCP2515 SPI-CAN HAT on Raspberry Pi (for CAN mode)
  - 24V DC power supply

> CAN bus features (`can0`, MIT mode) require **Linux with socketCAN** support. On Windows, only UART and Simulation modes are available.

---

## Installation

```bash
# Clone or copy the project
cd TorqMIT

# Install dependencies
pip install -r requirements.txt
```

### Dependencies

| Package | Version | Purpose |
|---|---|---|
| `PySide6` | >= 6.8.0 | Qt6 GUI framework |
| `pyqtgraph` | >= 0.13.0 | Real-time plotting |
| `pyserial` | >= 3.5 | Serial/UART communication |
| `numpy` | >= 2.0.0 | Numerical computation |
| `python-can` | >= 4.4.0 | CAN bus communication (optional on Windows) |

### CAN Bus Setup (Linux/Raspberry Pi only)

```bash
# Load kernel modules
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mcp251x   # for MCP2515 HAT

# Bring up CAN interface at 1 Mbps
sudo ip link set can0 up type can bitrate 1000000

# Verify
ip -details link show can0

# (Optional) Virtual CAN for testing without hardware
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

---

## Usage

### Main Application (GUI)

```bash
python Torsi_Encoder.py
```

Launches the full GUI with connection panel, parameter controls, real-time plots, sweep automation, and CSV recording.

### CAN Diagnostic Script

```bash
# Linux only, requires can0 interface up
python can_diag.py
```

Step-by-step CAN bus diagnostic (202 lines):

| Step | Action |
|---|---|
| 1 | Passive listen (1s) — detect motor IDs from servo broadcast |
| 2 | Send Servo current command (0.5 A) via extended frame, then stop |
| 3 | MIT ENTER — up to 3 attempts (standard + extended frames) |
| 4 | MIT zero-torque — verify MIT mode is active |
| 5 | MIT 0.5 Nm — send torque for 2 seconds, print feedback at 10 Hz |
| 6 | MIT EXIT — clean shutdown |

### MIT Continuous Test Script

```bash
# Linux only, requires can0 interface up
python can_test_mit.py
```

10-second MIT mode test (148 lines):

| Phase | Duration | Action |
|---|---|---|
| 1 | 0–5 s | Zero torque — move motor by hand, observe position/speed feedback |
| 2 | 5–10 s | 0.5 Nm torque with kd=0.5 — motor spins at ~1 rad/s |

Runs at 50 Hz, prints every 5th feedback frame, reports TX/RX/no-reply counts.

---

## GUI Workflow

### 1. Connection

```
[Interface dropdown] -> [CAN ID] -> [Connect]
```

1. Click **R** (refresh) to scan available interfaces
2. Select interface from dropdown:
   - `COMx` / `/dev/ttyUSBx` — UART Servo mode
   - `can0` / `vcan0` — CAN bus (auto-enters MIT, fallback to Servo)
   - `Simulation` — software simulation
3. Set **CAN ID** (default 2, set 0 for auto-detect)
4. Click **Connect**

For CAN connections, the app runs a 4-step sequence:
1. Auto-detect motor ID from servo broadcast traffic
2. Send MIT EXIT to clean state
3. Aggressive MIT ENTER (5 attempts, filtering MCP2515 echoes)
4. Verify MIT with zero-torque command

If MIT fails, falls back to CAN Servo mode automatically.

### 2. Input Mode Selection

| Mode | Description |
|---|---|
| **Current (A)** | Send amperage directly to motor. Default 0.5 A. |
| **Torque (Nm)** | Convert to amperage via `I = tau / 0.482`, then send. |

Toggle via radio buttons in the "Input Mode" panel.

### 3. Parameter Configuration

**UART Servo Parameters:**

| Param | Default | Unit | Notes |
|---|---|---|---|
| des P | 0.00 | deg | Stored, not sent in current loop |
| des S | 5000 | ERPM | Stored, not sent in current loop |
| des A | 30000 | ERPM/s^2 | Stored, not sent in current loop |
| Ramp | 1.0 | A/s | Active — controls current ramp rate |

**MIT Control Parameters:**

| Param | Default | Unit | Range | Notes |
|---|---|---|---|---|
| Kp | 0.0 | - | 0–500 | Position gain |
| Kd | 0.5 | - | 0–5 | Velocity damping gain |
| p_des | 0.0 | rad | -12.5 to 12.5 | Desired position |
| v_des | 0.0 | rad/s | -45.5 to 45.5 | Desired velocity |
| Ramp | 0.5 | Nm/s | - | Torque ramp rate |

**MIT Torque Equation:**
```
tau = kp * (p_des - p) + kd * (v_des - v) + t_ff
```

For pure torque with speed damping, set `kp=0, v_des=0, kd>0`. Maximum speed = `t_ff / kd`.

**Recommended MIT Settings:**
| Kd | t_ff | Max Speed |
|---|---|---|
| 0.5 | 0.5 Nm | ~1 rad/s |
| 0.3 | 1.0 Nm | ~3.3 rad/s |

### 4. Sending Commands

1. Enter setpoint value in the **Setpoint** box
2. Click **Send** — motor ramps to target at configured ramp rate
3. Click **Stop** — motor ramps down to zero

The ramp prevents sudden current spikes:
```
step = ramp_rate * dt
if abs(target - ramped) < step:
    ramped = target
else:
    ramped += step * sign(target - ramped)
```

### 5. Step Sweep Automation

```
[Start] [End] [Step] [Dwell(s)] -> [Run Sweep]
```

1. Configure sweep parameters:
   - **Start**: first setpoint value
   - **End**: last setpoint value
   - **Step**: increment between steps
   - **Dwell**: hold time per step (seconds)
2. Click **Run Sweep**
3. App automatically:
   - Generates step sequence
   - Sends each step to motor
   - Holds for dwell time
   - **Auto-captures** data at 80% of dwell time (when motor is settled)
   - Auto-starts CSV recording if not already active
   - Advances to next step
4. Click **Abort** to stop mid-sweep

Example: sweeping 0.1 to 0.5 A with step=0.1 and dwell=5s:
```
Step 1: 0.100 A (hold 5s, capture at 4.0s)
Step 2: 0.200 A (hold 5s, capture at 4.0s)
Step 3: 0.300 A (hold 5s, capture at 4.0s)
Step 4: 0.400 A (hold 5s, capture at 4.0s)
Step 5: 0.500 A (hold 5s, capture at 4.0s)
-> Sweep COMPLETE!
```

### 6. Data Recording

- **Auto-start**: recording begins automatically on connect
- **Manual toggle**: Start/Stop recording buttons
- Files saved to `data/` folder as `AK40-10_YYYY-MM-DD_HH-MM-SS.csv`
- Row counter displayed during recording
- See [CSV Data Format](#csv-data-format) for details

### 7. Capture & Annotation

- Click **Capture Now** to mark current data point on graphs
- Wait ~2 seconds for motor to settle, then snapshot
- Annotations show: target value, actual value, error %, torque, speed
- Green dots + text labels appear on both plots

### 8. Emergency Stop

- Press **ESC** key or click **E-STOP [ESC]** button
- Immediately zeros all outputs (current/torque set to 0)
- Motor stops as fast as possible
- Works in any mode (UART, CAN Servo, CAN MIT)

---

## MIT Mode Protocol

MIT mode uses standard CAN frames (not extended) with 8-byte packed commands:

### Command Frame (TX)

```
CAN ID: motor_id (standard frame)
Data [8 bytes]:
  [0-1]  p_des    (16-bit, range -12.5 to 12.5 rad)
  [2-3]  v_des    (12-bit) | kp (upper 4-bit)
  [4]    kp       (lower 8-bit)
  [5]    kd       (upper 8-bit)
  [6]    kd (lower 4-bit) | t_ff (upper 4-bit)
  [7]    t_ff     (lower 8-bit)
```

### Reply Frame (RX)

```
Data [8 bytes]:
  [0]    motor_id
  [1-2]  position  (16-bit, -12.5 to 12.5 rad)
  [3-4]  velocity  (12-bit, -45.5 to 45.5 rad/s) | torque (upper 4-bit)
  [5]    torque    (lower 8-bit, -5.0 to 5.0 Nm)
  [6]    temperature (raw - 40 = Celsius)
  [7]    error code
```

### Special Commands

| Command | Data (8 bytes) | Purpose |
|---|---|---|
| **Enter MIT** | `FF FF FF FF FF FF FF FC` | Switch motor to MIT mode |
| **Exit MIT** | `FF FF FF FF FF FF FF FD` | Return to Servo mode |
| **Set Zero** | `FF FF FF FF FF FF FF FE` | Set current position as origin |

---

## Key Formulas

### Torque at Output Shaft

```
tau_output = Iq * Kt * Gear_Ratio * Gear_Efficiency
tau_output = Iq * 0.056 * 10 * 0.86
tau_output = Iq * 0.482
```

### Current from Desired Torque

```
Iq = tau_output / Effective_Kt
Iq = tau_output / 0.482

Example: 0.5 Nm -> 0.5 / 0.482 = 1.038 A
```

### ERPM Conversion

```
ERPM = RPM_output * Pole_Pairs * Gear_Ratio
ERPM = RPM_output * 14 * 10
ERPM = RPM_output * 140

Example: 435 rpm (rated) = 435 * 140 = 60,900 ERPM
```

---

## CSV Data Format

Files are saved in `data/` with metadata headers and 14 data columns.

### Header (prefixed with `#`)

```
# AK40-10 Torque/Current Verification Log
# Date: 2026-09-17 09:35:38
# Mode: MIT
# CAN ID: 2
# Kt_eff: 0.4816 Nm/A
# Gear Ratio: 10.0
# Input Mode: AMPS
# MIT Kp: 0.0
# MIT Kd: 0.5
# MIT v_des: 0.0
# MIT p_des: 0.0
```

### Data Columns

| Column | Unit | Description |
|---|---|---|
| `time_s` | s | Elapsed time since connect |
| `target_current_A` | A | Commanded current |
| `actual_current_A` | A | Measured Iq current (feedback) |
| `target_torque_Nm` | Nm | Computed target torque |
| `actual_torque_Nm` | Nm | Computed actual torque |
| `position_rad` | rad | Motor position (MIT) or degrees (Servo) |
| `position_deg` | deg | Position in degrees |
| `speed_rad_s` | rad/s | Angular velocity |
| `speed_rpm` | rpm | Rotational speed |
| `motor_temp_C` | C | Motor winding temperature |
| `mos_temp_C` | C | MOSFET/driver temperature |
| `voltage_V` | V | Input voltage |
| `error_code` | - | 0=OK, 1-7=fault (see manual) |
| `sweep_step` | - | Current sweep setpoint (or 0) |

---

## Project Structure

```
TorqMIT/
├── Torsi_Encoder.py       # Main GUI application (1998 lines)
│   ├── Motor class         # UART Servo mode controller (40 Hz)
│   ├── MotorCAN class      # CAN bus controller: Servo + MIT (100 Hz)
│   └── App class           # PySide6 GUI, plots, sweep, recording
│
├── can_diag.py             # CAN diagnostic CLI tool (202 lines)
├── can_test_mit.py         # MIT mode continuous test CLI (148 lines)
├── requirements.txt        # Python dependencies
├── README.md               # This file
│
└── data/                   # Auto-generated CSV data logs
    └── AK40-10_YYYY-MM-DD_HH-MM-SS.csv
```

---

## Troubleshooting

### UART

| Problem | Solution |
|---|---|
| Port not listed | Click **R** to refresh. Check USB cable connection. |
| No telemetry data | Verify baudrate is 921600. Check TX/RX wiring (TX->RX, RX->TX). |
| Motor not responding | Ensure motor is in **Servo mode** via CubeMarsTool. Check power supply (24V). |

### CAN Bus

| Problem | Solution |
|---|---|
| `can0` not listed | Run `sudo ip link set can0 up type can bitrate 1000000` |
| Motor ID not detected | Check CAN_H/CAN_L wiring. Ensure motor is powered. Try setting ID manually (default: 2). |
| MIT mode fails | Motor must be switched to MIT firmware via CubeMarsTool first. App will fall back to Servo mode. |
| Echo frames (MCP2515) | Normal behavior — the app filters self-transmitted frames automatically. |
| `python-can` not found | Install with `pip install python-can`. Only required for CAN mode. |

### General

| Problem | Solution |
|---|---|
| Motor spins fast in current loop | Normal — current loop has no speed limiter. Use MIT mode (`kd > 0`) for speed damping. |
| Torque values seem off | Torque is calculated (`Iq * 0.482`), not measured. Verify with torque sensor for accuracy. |
| Motor doesn't move < 0.5 A | Breakaway friction of gearbox is ~0.241 Nm (~0.5 A). Increase current. |

---

## References

- **AK Series Driver Manual V1.0.18** — CubeMars (protocol specification, sections 5.1–5.3)
- **AK40-10 Datasheet** — CubeMars (motor constants, electrical parameters)
- **AK40-2410-1A-A1 Drive Installation Instructions** — CubeMars
- [CubeMars Official Website](https://www.cubemars.com/)
