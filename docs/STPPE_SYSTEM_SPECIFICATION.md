# STPPE Reborn — Complete System Specification & Architecture

**English** · [Polski](STPPE_SYSTEM_SPECIFICATION.pl.md)

> **System Teleoperacyjny do Precyzyjnych Prac Elektronicznych**  
> *Teleoperation System for High-Precision Electronics Servicing & Micro-Assembly*

> **Implementation status — 2026-10-13:** This document combines design intent with existing features. The current sketch does **not** define `ENABLE_ENCODERS` or `MANUAL_HOMING`; bare `HOME` starts switch homing, and `HOME MANUAL` assumes a zero-angle reference rather than the proposed boot pose below. Protocol examples also require reconciliation with the implementation. Precision and cost figures are design targets. See the [setup guide](GETTING_STARTED.md#connecting-physical-hardware) for current caveats and the [roadmap](ROADMAP.md) for completion criteria.

[Project overview](../README.md) · [Setup guide](GETTING_STARTED.md) · [Roadmap](ROADMAP.md)

---

## 1. Project Overview & Mission

* **Full Polish Name**: *System Teleoperacyjny do Precyzyjnych Prac Elektronicznych* (STPPE).
* **Project Designation**: **STPPE Reborn**.
* **Primary Objective**: Provide an ergonomic, high-precision teleoperated robotic platform designed for delicate electronics rework, SMD component placement (down to 0603/0402), BGA alignment, and soldering operations.
* **Core Value Proposition**:
  * Eliminates physiological hand tremor from manual microscopic work.
  * Targets sub-millimeter positioning accuracy ($\le 0.5\text{ mm}$ at the tool tip; physical validation pending).
  * Accessible, modular hardware platform (< 2000 PLN budget target).
  * Real-time CAD digital twin 3D visualization and sequence planning.

---

## 2. Mechanical Architecture & Kinematics

### 2.1 Kinematic Configuration
* **Configuration**: 6-DOF articulated serial robot arm + 1-DOF gripper (7 controlled axes total).
* **Kinematic Model**: `stppe_zlozenie_nowe` (URDF standard).
* **Physical Structure**: 3D-printed load-bearing structure with precision ball bearings and reduction stages.

### 2.2 Link Geometry & Offsets
| Parameter | Value | Physical Reference |
| :--- | :--- | :--- |
| **$H_{\text{base}}$ (Base Height)** | $143.896\text{ mm}$ | Mounting floor to J2 shoulder rotation axis |
| **$L_1$ (Shoulder / Link 1)** | $185.845\text{ mm}$ | J2 shoulder axis to J3 elbow axis |
| **$L_2$ (Forearm / Link 2)** | $225.478\text{ mm}$ | J3 elbow axis to J5 spherical wrist center |
| **$L_3$ (Wrist / Link 3)** | $36.105\text{ mm}$ | J5 center to J6 tool mounting flange |
| **$L_{\text{tool}}$ (Tool Radial Offset)** | $31.530\text{ mm}$ | Flange to Tool Center Point (TCP) |
| **Tool Rest Angle** | $4.896^\circ$ ($0.08545\text{ rad}$) | Static angular cant of the end-effector |

### 2.3 Joint Travel Limits
| Joint | Motion Type | Min Angle | Max Angle | Gearing / Reduction |
| :--- | :--- | :--- | :--- | :--- |
| **J1** | Base Pan | $-90.0^\circ$ | $+90.0^\circ$ | **8.33:1** belt/pulley |
| **J2** | Shoulder Pitch | $-65.0^\circ$ | $+115.0^\circ$ | **20:1** high-reduction stage |
| **J3** | Elbow Pitch | $-35.0^\circ$ | $+170.0^\circ$ | **20:1** high-reduction stage |
| **J4** | Forearm Roll | $-90.0^\circ$ | $+90.0^\circ$ | Direct / reduction wrist drive |
| **J5** | Wrist Pitch | $-120.0^\circ$ | $+120.0^\circ$ | Direct / reduction wrist drive |
| **J6** | Tool Roll | $-180.0^\circ$ | $+180.0^\circ$ | Direct / reduction wrist drive |

### 2.4 Safety Envelopes
* **Floor Clearance**: Minimum $Z \ge 20.0\text{ mm}$ clearance maintained by workspace validator.
* **Base Cylinder Exclusion Zone**: $r^2 \ge 6400\text{ mm}^2$ ($r \ge 80.0\text{ mm}$) to prevent mechanical collision with the base tower.

---

## 3. Electronics & Actuation

### 3.1 Controller Board
* **MCU**: **Teensy 4.1** (NXP i.MX RT1062, ARM Cortex-M7 @ 600 MHz).
* **Firmware**: [`Firmware/Teensy_4.1/IK_Control/IK_Control.ino`](../Firmware/Teensy_4.1/IK_Control/IK_Control.ino) (v3.2.0).

### 3.2 Stepper Motors & Drivers
* **Drivers**: 6× **TMC2209** silent stepper motor drivers.
* **Control Lines**:
  * High-speed hardware `STEP` / `DIR` / `EN` lines driven via Teensy hardware timers.
  * Dynamic UART bus on `Serial4` ($R_{\text{sense}} = 0.11\,\Omega$) for programmatic run/hold current, StealthChop configuration, and microstepping (1/256 interpolation).
* **Current Pinout Mapping**:
  * **J1 (Base)**: `STEP = 3`, `DIR = 2`, `EN = 6`
  * **J2 (Shoulder)**: `STEP = 11`, `DIR = 10`, `EN = 27`
  * **J3 (Elbow)**: `STEP = 29`, `DIR = 28`, `EN = 32`
  * **J4 (Forearm Roll)**: `STEP = 34`, `DIR = 33`, `EN = 12`
  * **J5 (Wrist Pitch)**: `STEP = 31`, `DIR = 30`, `EN = 12`
  * **J6 (Tool Roll)**: `STEP = 23`, `DIR = 22`, `EN = 12`
  * **Gripper Servo**: `PWM = Pin 21`

### 3.3 Encoders & Feedback Strategy (Current vs Future)
* **Current Physical Baseline**:
  * **Open-loop stepper operation**: No physical encoders are currently installed.
  * Motion is commanded in deterministic microsteps with TMC2209 StealthChop.
* **Future Upgrade Path**:
  * Firmware maintains hooks for **AS5600** absolute magnetic encoders (or AS5048A via SPI) across primary joints.
  * Planned configuration: `#define ENABLE_ENCODERS false` to support operation without installed encoders. This flag is not yet implemented; any future closed-loop configuration will need calibration and validation.

### 3.4 Homing & Reference Zeroing
* **Intended Strategy**: **Manual Boot Homing** (implementation pending).
* **Proposed Procedure**:
  * The robot powers up parked in its defined mechanical Home reference pose (`Home: [0.0°, 25.0°, 140.0°, 0.0°, 0.0°, 0.0°]`).
  * If a mechanical collision or power interruption occurs, the operator simply returns the arm to its index marks by hand and resets/re-enables.
  * Planned configuration: `#define MANUAL_HOMING true` to bypass automated switch homing. This flag is not yet implemented; bare `HOME` currently starts switch homing.

---

## 4. End-Effector & Tooling

* **Active End-Effector**: Dedicated precision servo micro-gripper connected to PWM Pin 21.
* **Ancillary Electronics Tools**: Soldering irons, hot-air nozzles, inspection microscopes, and flux dispensers remain **separate standalone fixtures** rather than arm-mounted burdens to minimize end-effector mass and inertia.

---

## 5. Control Interfaces & Teleoperation

STPPE employs a dual-tier master control architecture:

```
  ┌────────────────────────────────────────────────────────┐
  │         Control Studio (WebControl ES6 SPA)            │
  │    3D CAD Twin, Sequence Studio, Kinematics, Telemetry  │
  └──────────────────────────┬─────────────────────────────┘
                             │ WebSerial (USB) / WebSocket
                             ▼
  ┌────────────────────────────────────────────────────────┐
  │                 Teensy 4.1 Master MCU                  │
  │     Analytical IK, Hermite Spline Engine, TMC2209      │
  └──────────────────────────▲─────────────────────────────┘
                             │ Wired USB / Wireless ESP
  ┌──────────────────────────┴─────────────────────────────┐
  │         Custom Teach Pendant (Raspberry Pi / ESP)       │
  │   Handheld Jogs, Deadman / Clutch, Micro-Stepping Knobs │
  └────────────────────────────────────────────────────────┘
```

### 5.1 Primary Desktop Interface: Control Studio (`WebControl`)
* **Technology**: Zero-build pure ES6 Single Page Application with Three.js CAD visualizer.
* **Safety System**:
  * **Two-State Execution**: Default safe *Preview / Target Mode* (Amber Ghost Arm) prevents un-reviewed moves. Live hardware requires explicit arming into *Live Direct Drive*.
  * Tactile hold-jog acceleration with 250ms threshold and S-curve ramping throttled to 20 Hz serial packets.
* **Kinematics & Modes**:
  * Mutually exclusive **Cartesian Target (IK)** vs **Joint Control (FK)** tabs.
  * 7 analytical wrist orientation modes (W0 Level, W1 Down 0°, W2 Down 90°, W3-W5 Pitch presets, W6 Manual).
  * Multi-waypoint lookahead FIFO trajectory playback engine supporting curved splines and straight linear paths (`MOVL`).
* **Test Verification**: 562 automated tests in [`tests/run_tests_node.js`](../WebControl/tests/run_tests_node.js) passing at 100%.

### 5.2 Handheld Teleoperation: Custom Teach Pendant
* **Platform**: Custom handheld enclosure powered by **Raspberry Pi** (or similar SBC/MCU).
* **Controls**: Dual tactile joysticks, rotary encoder knob for precision step-nudge, clutch / deadman safety switch, and emergency stop.
* **Connectivity**:
  * **Wired Mode**: Direct USB Host link into controller.
  * **Wireless Mode**: High-reliability low-latency Wi-Fi/ESP serial bridge.

---

## 6. Communication Protocols & Formats

The following is a design-level protocol outline, not a verified command reference. Consult the current firmware parser and browser packet builder before using these examples with hardware; command names, argument formats, and telemetry fields must be reconciled as part of the roadmap.

### 6.1 Downlink Control Packets (Host / Pendant → Teensy)
Framed with Dallas/Maxim CRC-8 (polynomial `0x31`):
* **Point-to-Point / Trajectory Motion**:
  ```text
  <j1, j2, j3, j4, j5, dx, dy, dur, crc>
  ```
* **Immediate System Commands**:
  * `STOP\n` — Immediate emergency deceleration & motor halt.
  * `HOME\n` — Currently starts switch homing; the proposed manual boot reference is not implemented by this command.
  * `GRIP:<0-100>\n` — Command servo gripper opening percentage.
  * `OVR:<0.0-1.0>\n` — Global feedrate speed override.

### 6.2 Uplink Telemetry Stream (Teensy → Host)
Streamed continuously at 10 Hz as `JTELEM` JSON payload:
```json
{
  "t": 128450,
  "joints": [0.0, 25.0, 140.0, 0.0, 0.0, 0.0],
  "target": [0.0, 25.0, 140.0, 0.0, 0.0, 0.0],
  "tcp": {"x": 220.4, "y": 0.0, "z": 150.2},
  "grip": 0,
  "queue": 0,
  "homed": true,
  "estop": false,
  "status": "IDLE"
}
```

---

## 7. Workspace & Codebase Organization

| Directory / File | Description | Status |
| :--- | :--- | :--- |
| [`WebControl/`](../WebControl/) | Production Control Studio web application, tests, assets, and launcher | **Canonical Active** |
| [`Firmware/Teensy_4.1/`](../Firmware/Teensy_4.1/) | Controller firmware (`IK_Control.ino`); hardware configuration alignment pending | **Active Source** |
| [`docs/`](../docs/) | Architecture, images, banners, and technical specifications | **Active** |
| `WebControl-v2`, `v3`, `v4` | Previous exploratory UI and layout test iterations | **Test Archives** |

---

## 8. Implementation Roadmap

1. **Phase 1: Firmware Safety & Mode Flagging**  
   Add `#define ENABLE_ENCODERS false` and `#define MANUAL_HOMING true` to [`IK_Control.ino`](../Firmware/Teensy_4.1/IK_Control/IK_Control.ino) so the Teensy compiles cleanly for the current open-loop physical hardware without I2C timeouts or limit switch stalls.
2. **Phase 2: Single-Axis Bench Bring-Up**  
   Connect Teensy over USB WebSerial to Control Studio, power the 12V/24V stepper bus, and verify individual J1–J6 jog response and direction calibration.
3. **Phase 3: Directory Cleanup**  
   Consolidate or safely archive `WebControl-v2`, `v3`, and `v4` to keep the workspace clean and maintain `WebControl/` as the single source of truth.
4. **Phase 4: Raspberry Pi Teach Pendant Implementation**  
   Develop the handheld pendant client and ESP wireless bridge for cable-free workbench teleoperation.
