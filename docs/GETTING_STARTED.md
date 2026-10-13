# Getting started with STPPE Reborn

[← Project overview](../README.md) · [System specification](STPPE_SYSTEM_SPECIFICATION.md) · [Roadmap](ROADMAP.md)

Start with the virtual controller to explore Control Studio. It uses the same interface as a hardware connection and does not require a Teensy, motors, or a serial port.

## Requirements

| Task | What you need |
| :--- | :--- |
| Explore the app and simulator | Python 3 and a browser with WebGL |
| Run the main automated checks | Node.js; no npm dependencies are required by the main suite |
| Connect through USB | A browser exposing `navigator.serial`, a USB data cable, and a configured controller |
| Connect through Wi-Fi | A separately configured, compatible WebSocket-to-controller bridge |

The frontend uses native ES modules and bundled Three.js assets. The Python launcher uses the standard library. Serve the app through the launcher rather than opening `index.html` as a local file.

## Launch Control Studio

Clone the repository, or download and extract it using GitHub's **Code → Download ZIP**:

```bash
git clone https://github.com/adammotyl22-creator/STPPE-Reborn.git
cd STPPE-Reborn
python WebControl/launch.py
```

If your Python command is `py` on Windows or `python3` on macOS/Linux, substitute it in the last command. Windows launchers are also included as [`start_webcontrol.bat`](../WebControl/start_webcontrol.bat) and [`start_webcontrol.ps1`](../WebControl/start_webcontrol.ps1).

The launcher searches for a port starting at 8000, prints a **Local URL**, and opens the default browser. If that browser does not open, copy the printed URL into one manually. Keep the terminal running while using the app; `Ctrl+C` stops the server.

## Your first simulated move

1. Click the **DISCONNECTED** pill in the header to open connection settings.
2. Check **Mock**, then click **Connect**. The console should report a mock connection and telemetry should begin updating.
3. Open **Motion → Target (IK)**. Start with a small change to the displayed target and inspect the amber ghost arm and reachability indicator.
4. With Mock still active, click **Send to Target (PTP)** to execute the target on the virtual controller. Compare the simulated live pose with the target and watch the telemetry.
5. Switch to **Joint Control (FK)** to explore joint targets, or **Sequencer** to inspect waypoint editing and playback.

The interface's **PREVIEW (GHOST)** and **LIVE FOLLOW / Direct Drive** settings change how target edits are sent. Preview lets you inspect edits before an explicit send; it does not make every action in the interface passive. Keep the connection in Mock mode for this walkthrough.

The [dashboard screenshot](assets/webcontrol-dashboard.png) shows the overall layout. English and Polish labels are available, so wording may differ with the selected language.

## Connecting physical hardware

The checked-in sketch is [`IK_Control.ino`](../Firmware/Teensy_4.1/IK_Control/IK_Control.ino), declaring firmware version `v3.2.0`. The intended controller is Teensy 4.1, with six TMC2209 drivers and a servo gripper. Pin assignments and mechanical dimensions are in the [system specification](STPPE_SYSTEM_SPECIFICATION.md).

There is a concrete gap between the hardware baseline described in the specification and this sketch:

| Topic | Checked-in behavior / remaining work |
| :--- | :--- |
| Encoder configuration | Encoder code is present; `ENABLE_ENCODERS` is not defined. The documented encoder-free build mode remains to be implemented and verified. |
| Homing configuration | `MANUAL_HOMING` is not defined. Bare `HOME` starts the switch-homing routine. |
| Manual reference command | `HOME MANUAL` exists, but its handler assumes a zero-angle reference. This differs from the specification's proposed `[0, 25, 140, 0, 0, 0]` boot pose. |
| Build reproducibility | The sketch includes external libraries, but this guide does not claim a verified toolchain or firmware build. Pin library versions and record a successful build as part of the roadmap. |

For a controller already configured and validated for your build, open Control Studio at the launcher's local URL, disconnect Mock, uncheck **Mock**, select **USB**, and click **Connect** to choose the controller's serial port. Observe the connection and telemetry before commanding motion. Resolve the homing and encoder configuration above before following the specification as a bring-up procedure.

## Open the interface on another device

From the repository root:

```bash
python WebControl/launch.py --lan
```

Open the printed **Phone URL** from another device on the same trusted network. If Windows Firewall prompts, allow Python on the private network. Without `--lan`, the server binds to the host computer only.

LAN serving exposes the interface files, not the host computer's USB connection. USB control depends on the browser and security context of the device opening the page. The app's Wi-Fi transport needs an actual compatible bridge; selecting Wi-Fi does not create one.

## Troubleshooting

| Symptom | What to check |
| :--- | :--- |
| `python` is not recognized | Try `py` on Windows or `python3` on macOS/Linux; confirm Python 3 is installed. |
| Browser did not open | Copy the exact **Local URL** printed by the launcher. |
| Blank page or module-loading errors | Use the HTTP URL, not `file://`; check the browser console and launcher output. |
| No USB selection / Web Serial unsupported | Use Mock to explore. For USB, check `navigator.serial` support in the browser and use the local launcher URL. |
| Serial port is unavailable | Check the USB data cable and close other software holding the port, such as a serial monitor. |
| No simulated telemetry | Enable **Mock** before connecting. Disconnect before changing the connection mode. |
| Phone cannot reach the page | Start with `--lan`, use the printed network URL, and check the private-network firewall and Wi-Fi connection. |
| Wi-Fi transport fails | Check that a separate bridge is running and its configured WebSocket address matches the app settings. |

For unresolved problems, [open an issue](https://github.com/adammotyl22-creator/STPPE-Reborn/issues/new/choose) with the reproduction steps, OS/browser, connection mode, and relevant console output.

## Verify the software locally

Run from the repository root:

```bash
node WebControl/tests/run_tests_node.js
python WebControl/tests/test_server.py
```

The main Node suite and Python server check require no package installation. Other files in `WebControl/tests/` may have their own dependencies; for example, the browser E2E script imports Playwright. See [Contributing](../CONTRIBUTING.md) for how to report verification results.
