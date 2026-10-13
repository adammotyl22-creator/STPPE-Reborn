# STPPE Reborn roadmap

[← Project overview](../README.md) · [Setup guide](GETTING_STARTED.md) · [System specification](STPPE_SYSTEM_SPECIFICATION.md)

This roadmap separates what can be explored today from the evidence and engineering work needed for a reproducible physical system. The milestones are an intended sequence, not release dates or claims of completed hardware validation.

## Available today

- Control Studio with Cartesian and joint controls, CAD visualization, target preview, and a waypoint sequencer.
- A virtual controller, telemetry display, and command console for exploring the interface without hardware.
- Browser transport code for USB Web Serial and a compatible external WebSocket bridge.
- Teensy 4.1 firmware source, robot meshes, and a URDF model.
- A main JavaScript suite with 562 passing assertions and a passing Python server smoke check, verified locally on 2026-10-13.

Software test results do not establish physical accuracy, build reproducibility, or reliability of a complete wireless installation.

## 1. Match firmware to the physical build

**Outcome:** a documented firmware configuration for the encoder-free hardware described in the specification.

- [ ] Implement and verify an explicit encoder-disabled configuration.
- [ ] Implement and verify the intended manual boot-homing mode and reference pose.
- [ ] Reconcile `HOME`, `HOME MANUAL`, telemetry, and motion commands between firmware, browser, and specification.
- [ ] Record the Teensy toolchain, library versions, and exact build/upload procedure.

**Complete when:** a clean checkout builds with the recorded environment, the intended configuration does not depend on absent sensors, and the documented reference pose agrees with firmware behavior.

## 2. Establish bench performance

**Outcome:** published measurements readers can use to judge the arm's capabilities.

- [ ] Verify J1–J6 direction, travel limits, reduction ratios, and gripper response.
- [ ] Record stop/re-enable behavior and recovery after loss of position reference.
- [ ] Measure tool-tip positioning error and repeatability at representative workspace locations.
- [ ] Publish the measurement method, load/tool configuration, results, and limitations.
- [ ] Record a short physical demonstration alongside the corresponding Control Studio view.

**Complete when:** the repository contains a reproducible bench report and actual results. The ≤ 0.5 mm positioning figure remains a target until supported by measurements.

## 3. Make the build reproducible

**Outcome:** a new contributor can understand, build, and validate the project from the repository.

- [ ] Publish a bill of materials with quantities, sourcing notes, and dated costs against the 2000 PLN target.
- [ ] Add wiring diagrams, assembly photos, and calibration instructions.
- [ ] Automate the software checks on pull requests and document their coverage.
- [ ] Consolidate historical interface experiments around the canonical `WebControl/` application.
- [ ] Document first-party licensing and third-party asset attribution before presenting the project as a reusable distribution.

**Complete when:** someone outside the original development setup can follow the instructions, reproduce the software checks, and identify every required hardware component.

## 4. Extend workbench teleoperation

**Outcome:** a dedicated handheld controller and a documented wireless option.

- [ ] Define the teach pendant hardware and interaction model: joysticks, precision knob, and clutch/deadman input.
- [ ] Implement the pendant client and compatible bridge firmware/configuration.
- [ ] Document control ownership when desktop and pendant are both present.
- [ ] Measure latency and verify behavior on connection loss and reconnection.

**Complete when:** wired and wireless demonstrations include the configuration, observed timing, and tested disconnect behavior.

## Contribute to a milestone

Open an issue describing the milestone, the concrete result you want to deliver, and how it can be checked. Documentation corrections, simulator bug reports, CAD improvements, and measured bench data all help move the project forward. See [Contributing](../CONTRIBUTING.md).
