# Contributing to STPPE Reborn

**English** · [Polski](CONTRIBUTING.pl.md)

[Project overview](README.md) · [Setup guide](docs/GETTING_STARTED.md) · [Roadmap](docs/ROADMAP.md)

Contributions can improve the control software, firmware, CAD, documentation, or the evidence behind the physical build. Start with the simulator if you do not have the hardware.

## Report a problem

Use the [bug report template](https://github.com/adammotyl22-creator/STPPE-Reborn/issues/new?template=bug_report.md). Include the steps to reproduce, expected and actual behavior, operating system/browser, and whether the issue uses Mock, USB, or Wi-Fi. For hardware issues, include the board, firmware revision, and relevant wiring or calibration details.

Console output and screenshots are useful. Remove credentials, Wi-Fi passwords, and personal data before sharing logs.

## Propose an improvement

Use the [feature request template](https://github.com/adammotyl22-creator/STPPE-Reborn/issues/new?template=feature_request.md). Explain the task that is difficult today, the behavior you propose, and what would demonstrate success. Link the relevant [roadmap milestone](docs/ROADMAP.md) when applicable.

## Work on a change

1. Fork and clone the repository, then create a branch for one focused improvement.
2. Launch `python WebControl/launch.py` from the repository root and reproduce the relevant behavior.
3. Make the change in the canonical `WebControl/`, `Firmware/`, or `docs/` area. Keep generated logs, build output, and local editor settings out of the change.
4. Run checks appropriate to the change and record what you verified.
5. Open a pull request describing the problem, resulting behavior, and verification. Include before/after screenshots for visible interface changes.

## Verification

The baseline checks, run from the repository root, are:

```bash
node WebControl/tests/run_tests_node.js
python WebControl/tests/test_server.py
```

The main JavaScript suite covers application logic and UI contracts. The Python smoke check starts a temporary local server and requests the application assets. Neither requires a robot or additional package installation.

For documentation-only changes, verify relative links, code examples, and factual claims. For interface changes, exercise the affected flow in Mock mode as well as running relevant tests. For firmware changes, record the toolchain, build result, and any bench validation separately; simulator success is not hardware validation.

Prefer regression tests that demonstrate the bug or behavior being changed. Additional browser tests may require Playwright and browser binaries; describe those prerequisites if your change relies on them.

## Project conventions

- Keep the frontend compatible with its native ES-module, no-build setup.
- Preserve controller APIs and DOM/data-attribute contracts unless a change explicitly updates their consumers and tests.
- Update English and Polish translations when changing localized interface labels.
- State whether a hardware claim is a design target, source-code behavior, or a measured result.
- Keep the system specification and setup instructions aligned with implementation changes.
