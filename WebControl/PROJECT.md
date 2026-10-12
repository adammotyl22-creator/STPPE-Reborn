# Project: Teensy 4.1 Robot Arm Automation Studio Redesign

## Architecture
- **Architecture Overview**: Zero-build, modular ES6 Single-Page Application (SPA) communicating with Teensy 4.1 6-DOF robot arm firmware v3.1.0 (`IK_Control.ino`) via WebSerial API and Dallas/Maxim CRC-8 packet protocol (`<j1,j2,j3,j4,j5,dx,dy,dur,crc>`).
- **Core Subsystems**:
  - `AppState`: Reactive pub/sub store with scoped snapshot isolation and immutable state paths.
  - `WebSerialManager` / `MockSerialPort`: WebSerial communication and high-fidelity virtual hardware simulator (10Hz JSON telemetry, 50Hz quintic trajectory physics, CRC-8 validation).
  - `PacketBuilder` / `TelemetryParser`: CRC-8 Maxim packet builder and 10Hz `JTELEM` JSON / AR4 / Collision stream parser.
  - `RobotKinematics` / `WorkspaceValidator`: Forward kinematics (FK), analytical inverse kinematics (IK for W0..W6), workspace safety bounds, floor clearance, base cylinder exclusion.
  - `RobotVisualizer3D` / `ThreeScene` / `RobotModel` / `WorkspaceVisualizer`: Three.js WebGL digital twin rendering Live Arm (Cyan PBR) and Ghost Arm (Amber PBR) with reachability dome, collision planes, and HUD.
  - UI Controllers: `HeaderController`, `MotionController`, `TrajectoryController`, `TelemetryController`, `ConsoleController`.

## Feature Inventory
| # | Feature | Description | Milestone | Source |
|---|---------|-------------|-----------|--------|
| 1 | Typography & Google Fonts | Inter (headings/UI/structure) & IBM Plex Mono (telemetry/code/coordinates) | M1 | REVISION §R1 |
| 2 | Titanium Slate Color Tokens | #0b0e14 void, #12161f panel, #1a202c card, #242c3d elevated, non-neon signal accents | M1 | REVISION §R1 |
| 3 | Anti-Slop Trope Removal | Eliminate fuzzy glows, purple gradients, 12px rounded cards, glassmorphic blurs, em-dashes | M1 | REVISION §R1 |
| 4 | Tactile Inset & Press System | 1px metallic borders, inset highlights, active translateY(1px) tactile states | M1 | REVISION §R1 |
| 5 | Master Navigation Header Bar | High-density status bar, baud selector, connection pill, gantry jog, speed override | M2 | REVISION §R2 |
| 6 | Heavy Industrial ESTOP & Re-enable | High-visibility crimson ESTOP actuator with flashing state, tactile re-enable latch | M2 | REVISION §R2 |
| 7 | Joint Sliders & Nudge Matrix | High-density slider tracks, numeric feedback boxes, bracket limit indicators, nudges | M2 | REVISION §R2 |
| 8 | Cartesian IK Input Matrix & Wrist Modes | 3-axis precision readouts (X,Y,Z), W0-W6 wrist selector, reachability badge, MOV/MOVL triggers | M2 | REVISION §R2 |
| 9 | 3D Viewport HUD & Preset Bar | Monospace coordinate HUD overlay, camera presets (ISO/TOP/FRONT/SIDE), layer toggles | M3 | REVISION §R2 |
| 10 | Visual Trajectory Timeline & Studio | Compact timeline editor, step badges, playback transport, loop toggle, speed scale, teach panel | M3 | REVISION §R2 |
| 11 | Telemetry Strip Charts & HUD Gauges | High-contrast oscilloscope charts, 1px grid lines, status gauges, collision matrix | M4 | REVISION §R2 |
| 12 | Interactive Terminal & Console | High-density log stream with color-coded chips, quick commands keypad, CLI prompt | M4 | REVISION §R2 |
| 13 | 100% Automated Test Suite Passing | Maintain 275/275 passing tests across 11 suites in `node tests/run_tests_node.js` | M5 | REVISION §R3 |
| 14 | Adversarial Hardening & DOM Integrity | Zero DOM ID/attribute regressions, seamless WebSerial/MockSerial integration | M5 | REVISION §R3 |


## Milestones
| # | Name | Scope | Dependencies | Status |
|---|------|-------|-------------|--------|
| 1 | M1: Design System & Typography Foundation | Google Fonts import in `index.html`, Obsidian/Slate tokens in `css/theme/cyberpunk-dark.css`, base tactile utilities & scrollbars in `css/main.css` | none | DONE |
| 2 | M2: Header & Motion Control Workstation Overhaul | Overhaul `css/components/header.css` & `css/components/motion-panel.css` for tactile engineering workstation ergonomics | M1 | DONE |
| 3 | M3: 3D Viewport HUD & Trajectory Studio Overhaul | Overhaul `css/components/viewport.css` & `css/components/trajectory-studio.css` for precision CAD/studio ergonomics | M1 | DONE |
| 4 | M4: Telemetry Charts & Console Dock Overhaul | Overhaul `css/components/telemetry-charts.css` & `css/components/console.css` for oscilloscope & industrial terminal | M1 | DONE |
| 5 | M5: Full E2E Verification & Adversarial Hardening | Execute 250/250 automated tests in `tests/run_tests_node.js`, verify complete UI integration and adversarial resilience | M2, M3, M4 | DONE |

## Interface Contracts
### DOM & Data Attribute Stability Contract
- All DOM IDs (`#btn_serial_connect`, `#select_baud_rate`, `#btn_estop`, `#btn_reenable`, `#slider_j1`..`#slider_grip`, `#val_j1`..`#val_grip`, `#input_x`..`#input_z`, `#select_wrist_mode`, `#badge_reachability`, `#btn_mov_spline`, `#btn_mov_linear`, `#viewport-container`, `#hud-tcp-x`..`#hud-tcp-z`, `#hud-floor-alert`, `#timeline-cards-list`, `#scrubber-track`, `#bottom-dock`, `.dock-tabs`, etc.) MUST remain intact.
- All `data-*` attributes (`data-action`, `data-joint-slider`, `data-joint-val`, `data-cartesian-input`, `data-cartesian-nudge`, `data-grip-macro`, `data-home-target`, `data-gantry-jog`, `data-camera-preset`, `data-dock-tab`, `data-filter`, etc.) MUST remain unchanged.

### JavaScript Controller API Contract
- `AppState`, `WebSerialManager`, `MockSerialPort`, `PacketBuilder`, `TelemetryParser`, `RobotKinematics`, `WorkspaceValidator`, `RobotVisualizer3D`, and all 5 UI controllers retain their full public API, method signatures, event bindings, and lifecycle behavior.

## Code Layout
- `index.html`: Main HTML entry point (contains Google Fonts link and container DOM).
- `css/main.css`: Master grid layout, scrollbars, panel separators.
- `css/theme/cyberpunk-dark.css`: Design tokens, color palette, typography definitions, tactile button/input base classes.
- `css/components/header.css`: Header bar, baud selector, ESTOP, homing, gantry jog.
- `css/components/motion-panel.css`: Joint sliders, Cartesian matrix, wrist modes, MOV/MOVL.
- `css/components/viewport.css`: 3D viewport bezel, HUD overlay pills, camera presets, layer chips.
- `css/components/trajectory-studio.css`: Timeline cards, scrubber, playback deck, teach panel, JSON modal.
- `css/components/telemetry-charts.css`: Status HUD cards, 2D canvas chart containers, channel pills.
- `css/components/console.css`: Console header, filter chips, log stream, quick commands, CLI prompt.
- `tests/run_tests_node.js`: Headless CLI test runner (250 tests).
