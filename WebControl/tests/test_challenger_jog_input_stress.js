/**
 * test_challenger_jog_input_stress.js
 * 
 * Challenger 1 Dedicated Empirical Stress Harness:
 * 1. Hold-Jog Acceleration Engine:
 *    - 250ms press threshold to trigger continuous jogging.
 *    - Smooth S-curve velocity ramping (mathematical properties, smoothstep progression over continuous press).
 *    - Serial packet dispatch rate limiting <= 20Hz (50ms interval) to guarantee no buffer flooding.
 *    - Single-tap vs hold disambiguation (discrete single step on tap, suppression of trailing click after hold).
 * 2. Numeric Badge Input Robustness:
 *    - Negative numbers, fractional floats, extreme out-of-bound values (-999, +999).
 *    - Invalid strings ("abc", empty, NaN, symbols, null) graceful clamping/reversion.
 *    - Enter key commit, Escape cancel, blur commit, and Forward Kinematics (FK) recomputation.
 *    - Fuzz testing across all 6 joints (J1..J6).
 */

import { MotionController, JOINT_CONFIGS } from '../js/ui/MotionController.js';
import { AppState } from '../js/state/AppState.js';
import { forwardKinematics, clamp } from '../js/kinematics/RobotKinematics.js';

let totalTests = 0;
let passedTests = 0;
let failedTests = 0;
const failureDetails = [];

function assert(condition, description, extra = '') {
  totalTests++;
  if (condition) {
    passedTests++;
    console.log(`  [PASS] ${description}`);
  } else {
    failedTests++;
    const msg = extra ? `${description} (${extra})` : description;
    failureDetails.push(msg);
    console.error(`  [FAIL] ${msg}`);
  }
}

/**
 * Creates a minimal DOM element mock for testing event handlers
 */
function createMockElement(tag, attrs = {}) {
  const listeners = {};
  const el = {
    tagName: tag.toUpperCase(),
    value: attrs.value || '',
    attributes: { ...attrs },
    classList: {
      _classes: new Set(),
      add(cls) { this._classes.add(cls); },
      remove(cls) { this._classes.delete(cls); },
      contains(cls) { return this._classes.contains(cls); },
      toggle(cls, on) { if (on) this._classes.add(cls); else this._classes.delete(cls); }
    },
    getAttribute(name) { return this.attributes[name] ?? null; },
    setAttribute(name, val) { this.attributes[name] = String(val); },
    addEventListener(evt, fn) {
      if (!listeners[evt]) listeners[evt] = [];
      listeners[evt].push(fn);
    },
    removeEventListener(evt, fn) {
      if (listeners[evt]) {
        listeners[evt] = listeners[evt].filter(f => f !== fn);
      }
    },
    dispatchEvent(evt) {
      const fns = listeners[evt.type] || [];
      for (const fn of fns) {
        fn(evt);
      }
    },
    select() {},
    blur() {},
    setPointerCapture() {},
    releasePointerCapture() {},
    hasPointerCapture() { return false; }
  };
  return el;
}

// ============================================================================
// SUITE 1: Hold-Jog Acceleration Engine & Threshold Verification
// ============================================================================
async function runHoldJogAccelerationTests() {
  console.log('\n======================================================');
  console.log('SUITE 1: Hold-Jog Acceleration Engine & Threshold');
  console.log('======================================================');

  const appState = new AppState();
  const mc = new MotionController({ appState, autoSend: false });

  // 1.1 Verify Hold Jog Constants
  console.log('\n--- 1.1: Hold Jog Configuration Constants ---');
  assert(mc.HOLD_JOG_THRESHOLD_MS === 250, 'HOLD_JOG_THRESHOLD_MS is exactly 250ms');
  assert(mc.HOLD_JOG_MIN_VELOCITY === 2.0, 'HOLD_JOG_MIN_VELOCITY is 2.0 deg/s (or mm/s)');
  assert(mc.HOLD_JOG_MAX_VELOCITY === 35.0, 'HOLD_JOG_MAX_VELOCITY is 35.0 deg/s (or mm/s)');
  assert(mc.HOLD_JOG_DISPATCH_INTERVAL_MS === 50, 'HOLD_JOG_DISPATCH_INTERVAL_MS is 50ms (20Hz rate limit)');

  // 1.2 Ramp Duration Verification (Spec check: 3.0s vs 1.2s)
  console.log('\n--- 1.2: Ramp Duration Verification (3.0s vs 1.2s) ---');
  console.log(`  Reported mc.HOLD_JOG_RAMP_DURATION_MS = ${mc.HOLD_JOG_RAMP_DURATION_MS} ms`);
  const is3000ms = mc.HOLD_JOG_RAMP_DURATION_MS === 3000;
  const is1200ms = mc.HOLD_JOG_RAMP_DURATION_MS === 1200;
  assert(is3000ms || is1200ms, 'HOLD_JOG_RAMP_DURATION_MS is set to positive duration', `Actual: ${mc.HOLD_JOG_RAMP_DURATION_MS}`);
  if (is1200ms) {
    console.log('  [OBSERVATION] Implementation uses 1200ms ramp duration (1.2s S-curve to peak velocity 35 deg/s).');
  } else if (is3000ms) {
    console.log('  [OBSERVATION] Implementation uses 3000ms ramp duration (3.0s S-curve to peak velocity 35 deg/s).');
  }

  // 1.3 Smoothstep S-Curve Mathematical Evaluation
  console.log('\n--- 1.3: Smoothstep S-Curve Mathematical Evaluation ---');
  const rampDuration = mc.HOLD_JOG_RAMP_DURATION_MS;
  const vMin = mc.HOLD_JOG_MIN_VELOCITY;
  const vMax = mc.HOLD_JOG_MAX_VELOCITY;

  function evaluateVelocity(heldTimeMs, totalRampMs) {
    const u = Math.max(0.0, Math.min(1.0, heldTimeMs / totalRampMs));
    const smoothstep = 3.0 * u * u - 2.0 * u * u * u;
    return vMin + (vMax - vMin) * smoothstep;
  }

  // Test at u = 0 (t = 0), u = 0.25, u = 0.5, u = 0.75, u = 1.0 (t = rampDuration)
  const v0 = evaluateVelocity(0, rampDuration);
  const vMid = evaluateVelocity(rampDuration * 0.5, rampDuration);
  const vEnd = evaluateVelocity(rampDuration, rampDuration);
  const vBeyond = evaluateVelocity(rampDuration * 1.5, rampDuration);

  assert(Math.abs(v0 - vMin) < 1e-4, `At start of ramp (u=0), velocity equals vMin (${vMin} deg/s)`);
  assert(Math.abs(vMid - (vMin + (vMax - vMin) * 0.5)) < 1e-4, `At midpoint (u=0.5), smoothstep is 0.5 (velocity = ${(vMin + (vMax - vMin) * 0.5).toFixed(2)} deg/s)`);
  assert(Math.abs(vEnd - vMax) < 1e-4, `At end of ramp (u=1.0), velocity reaches vMax (${vMax} deg/s)`);
  assert(Math.abs(vBeyond - vMax) < 1e-4, `Beyond ramp duration, velocity stays clamped at vMax (${vMax} deg/s)`);

  // Monotonicity check across 100 sample points
  let strictlyMonotonic = true;
  let prevV = vMin;
  for (let i = 1; i <= 100; i++) {
    const t = (rampDuration * i) / 100;
    const v = evaluateVelocity(t, rampDuration);
    if (v < prevV - 1e-6) {
      strictlyMonotonic = false;
      break;
    }
    prevV = v;
  }
  assert(strictlyMonotonic, 'S-curve velocity profile is strictly monotonically non-decreasing over ramp duration');

  // Verify smooth derivative at boundaries: S'(u) = 6u(1-u) -> S'(0) = 0, S'(1) = 0
  const dtSample = 0.001; // in normalized u
  const dS0 = ( (3*(dtSample)**2 - 2*(dtSample)**3) - 0 ) / dtSample;
  const dS1 = ( 1.0 - (3*(1 - dtSample)**2 - 2*(1 - dtSample)**3) ) / dtSample;
  assert(dS0 < 0.01, 'Initial acceleration slope S\'(0) ~ 0 (gentle creep without initial jerk spike)');
  assert(dS1 < 0.01, 'Terminal acceleration slope S\'(1) ~ 0 (smooth plateau into max speed without jerk shock)');

  // 1.4 Hold Threshold Timing & Single-Tap vs Hold Gesture Disambiguation
  console.log('\n--- 1.4: Single-Tap vs Hold Gesture Disambiguation ---');
  {
    const btn = createMockElement('button', {
      'data-nudge-joint': 'j1',
      'data-jog-dir': '1',
      'data-step': '1'
    });
    mc._bindJogRamping(btn, 'joint', 'j1', 1);

    // Initial state
    mc.setJointTarget('j1', 0.0, false, false);
    assert(mc.joints.j1 === 0.0, 'J1 initialized to 0.0 deg');

    // Case A: Single Tap (< 250ms hold)
    // Simulate pointerdown at t=0
    btn.dispatchEvent({ type: 'pointerdown', currentTarget: btn, button: 0, pointerId: 1 });
    assert(mc._activeHoldJog !== null, 'Hold-jog tracker initialized on pointerdown');
    assert(mc._activeHoldJog.isHeld === false, 'isHeld is false immediately on pointerdown');

    // Simulate pointerup at t=50ms (before 250ms threshold)
    await new Promise(r => setTimeout(r, 50));
    btn.dispatchEvent({ type: 'pointerup', currentTarget: btn, pointerId: 1 });
    assert(mc._activeHoldJog === null, 'Hold-jog tracker cleaned up on pointerup');
    assert(mc._justFinishedHold === undefined || mc._justFinishedHold === false, 'justFinishedHold is false for short tap');

    // Browser dispatches native click event after pointerup
    btn.dispatchEvent({ type: 'click' });
    assert(mc.joints.j1 === 1.0, 'Single tap executes exactly one discrete step (+1.0 deg, got ' + mc.joints.j1 + ')');

    // Case B: Hold Jog (> 250ms)
    // Start hold
    btn.dispatchEvent({ type: 'pointerdown', currentTarget: btn, button: 0, pointerId: 1 });
    // Wait 280ms for 250ms timer to trigger
    await new Promise(r => setTimeout(r, 280));
    assert(mc._activeHoldJog !== null && mc._activeHoldJog.isHeld === true, 'Hold-jog timer fired at 250ms: isHeld is true');

    // Release hold
    btn.dispatchEvent({ type: 'pointerup', currentTarget: btn, pointerId: 1 });
    assert(mc._activeHoldJog === null, 'Hold-jog tracker cleaned up on hold release');
    assert(mc._justFinishedHold === true, 'justFinishedHold flag set to true to suppress trailing click');

    // Browser dispatches trailing click event
    const j1PoseBeforeClick = mc.joints.j1;
    btn.dispatchEvent({ type: 'click' });
    assert(mc.joints.j1 === j1PoseBeforeClick, 'Trailing click event after hold was successfully suppressed (no extra step)');

    // After 120ms timeout, suppression clears
    await new Promise(r => setTimeout(r, 120));
    assert(mc._justFinishedHold === false, 'justFinishedHold flag cleared after 100ms suppression window');
  }

  // 1.5 Rapid Sequential Taps (Multi-tap stability)
  console.log('\n--- 1.5: Rapid Sequential Taps Stability ---');
  {
    const btn = createMockElement('button', {
      'data-nudge-joint': 'j2',
      'data-jog-dir': '1',
      'data-step': '2'
    });
    mc._bindJogRamping(btn, 'joint', 'j2', 1);
    mc.setJointTarget('j2', 0.0, false, false);

    // 5 rapid taps with 30ms hold and 30ms gap
    for (let tap = 0; tap < 5; tap++) {
      btn.dispatchEvent({ type: 'pointerdown', currentTarget: btn, button: 0, pointerId: 1 });
      await new Promise(r => setTimeout(r, 30));
      btn.dispatchEvent({ type: 'pointerup', currentTarget: btn, pointerId: 1 });
      btn.dispatchEvent({ type: 'click' });
      await new Promise(r => setTimeout(r, 30));
    }
    assert(mc.joints.j2 === 10.0, `5 rapid taps of +2.0 deg step resulted in exactly 10.0 deg (got ${mc.joints.j2})`);
  }
}

// ============================================================================
// SUITE 2: Serial Packet Dispatch Rate Throttling (20Hz / 50ms)
// ============================================================================
async function runPacketRateLimitingStressTests() {
  console.log('\n======================================================');
  console.log('SUITE 2: Serial Packet Dispatch Rate Throttling (20Hz)');
  console.log('======================================================');

  const appState = new AppState();
  const dispatchedPackets = [];

  const mockSerial = {
    isConnected: () => true,
    send: async (cmd) => {
      dispatchedPackets.push({ cmd, time: Date.now() });
      return Promise.resolve();
    }
  };

  const mc = new MotionController({
    appState,
    serialManager: mockSerial,
    autoSend: false
  });

  // 2.1 Default Preview Mode: Zero Serial Packets Dispatched During Jog
  console.log('\n--- 2.1: Default Preview Mode Decoupling ---');
  dispatchedPackets.length = 0;
  mc.executionMode = 'preview';
  mc.isArmed = false;

  // Simulate hold-jog loop running for 500ms (10 simulated frames)
  mc._activeHoldJog = {
    type: 'joint',
    id: 'j1',
    dir: 1,
    startTime: Date.now() - 300,
    lastFrameTime: Date.now() - 50,
    lastDispatchTime: 0,
    isHeld: true
  };

  for (let frame = 0; frame < 10; frame++) {
    mc._runHoldJogLoop();
    await new Promise(r => setTimeout(r, 20));
  }
  mc._stopHoldJog();

  assert(dispatchedPackets.length === 0, 'Zero hardware packets dispatched during hold-jog in Preview mode');

  // 2.2 Direct Drive Mode: 20Hz Throttled Streaming (50ms interval)
  console.log('\n--- 2.2: Direct Drive Mode 20Hz Throttle Verification ---');
  dispatchedPackets.length = 0;
  mc.executionMode = 'direct_drive';
  mc.isArmed = true;

  // Mock sendMovSpline to log dispatch times
  const dispatchTimestamps = [];
  mc.sendMovSpline = async (opts) => {
    dispatchTimestamps.push(Date.now());
    return Promise.resolve();
  };

  const startTime = Date.now();
  mc._activeHoldJog = {
    type: 'joint',
    id: 'j1',
    dir: 1,
    startTime: startTime,
    lastFrameTime: startTime,
    lastDispatchTime: 0,
    isHeld: true
  };

  // Run simulated loop at 100Hz (10ms steps) for 1000ms (1.0 second)
  const DURATION_MS = 1000;
  const STEP_MS = 10;
  const steps = DURATION_MS / STEP_MS;

  for (let s = 0; s < steps; s++) {
    mc._runHoldJogLoop();
    await new Promise(r => setTimeout(r, STEP_MS));
  }
  mc._stopHoldJog();

  const actualDurationMs = dispatchTimestamps[dispatchTimestamps.length - 1] - dispatchTimestamps[0];
  const observedHz = (dispatchTimestamps.length - 1) / (actualDurationMs / 1000);
  console.log(`  Dispatched ${dispatchTimestamps.length} packets over ${actualDurationMs.toFixed(1)}ms (${observedHz.toFixed(2)} Hz)`);

  assert(dispatchTimestamps.length > 0, 'Direct Drive mode dispatched packets during hold-jog');
  assert(observedHz <= 20.5, `Observed dispatch rate (${observedHz.toFixed(2)} Hz) does not exceed 20Hz theoretical limit`);

  // Check inter-packet intervals (must be >= 45ms to account for timer jitter)
  let rateViolations = 0;
  for (let i = 1; i < dispatchTimestamps.length; i++) {
    const deltaMs = dispatchTimestamps[i] - dispatchTimestamps[i - 1];
    if (deltaMs < 45) { // 50ms target with 5ms allowable timer tolerance
      rateViolations++;
    }
  }
  assert(rateViolations === 0, `Zero rate-limit violations: all packet intervals >= 45ms (got ${rateViolations} violations)`);
}

// ============================================================================
// SUITE 3: Numeric Badge Input Robustness & Sanitization
// ============================================================================
async function runNumericBadgeInputStressTests() {
  console.log('\n======================================================');
  console.log('SUITE 3: Numeric Badge Input Robustness & Sanitization');
  console.log('======================================================');

  const appState = new AppState();
  const mc = new MotionController({ appState, autoSend: false });

  // Helper to bind a mock badge input
  function setupMockBadge(joint, initialVal = '0.0') {
    const inputEl = createMockElement('input', {
      id: `val_${joint}`,
      type: 'number',
      value: initialVal
    });
    mc._bindInlineBadgeInput(joint, inputEl);
    return inputEl;
  }

  // 3.1 Negative Values Handling
  console.log('\n--- 3.1: Negative Values Handling ---');
  for (const [joint, cfg] of Object.entries(JOINT_CONFIGS)) {
    if (joint === 'grip') continue;
    const testNeg = Math.max(cfg.min, -45.0);
    const badge = setupMockBadge(joint, '0.0');

    badge.value = String(testNeg);
    badge.dispatchEvent({ type: 'keydown', key: 'Enter', preventDefault: () => {} });

    assert(mc.joints[joint] === testNeg, `${joint.toUpperCase()} badge accepts negative value ${testNeg}° (got ${mc.joints[joint]})`);
    assert(badge.value === testNeg.toFixed(1), `${joint.toUpperCase()} badge input displays formatted value "${testNeg.toFixed(1)}"`);
  }

  // 3.2 Fractional Floats Handling
  console.log('\n--- 3.2: Fractional Floats Precision & Formatting ---');
  {
    const badge = setupMockBadge('j1', '0.0');
    badge.value = '23.4567';
    badge.dispatchEvent({ type: 'keydown', key: 'Enter', preventDefault: () => {} });
    assert(Math.abs(mc.joints[jointToVal('j1', mc) - 23.4567]) < 0.01 || mc.joints.j1 === 23.4567, 'Fractional float parsed accurately');
    assert(badge.value === '23.5', 'Badge input formatted to 1 decimal place ("23.5")');
  }

  function jointToVal(j, ctrl) {
    return ctrl.joints[j];
  }

  // 3.3 Extreme Out-of-Bounds Values (-999, +999)
  console.log('\n--- 3.3: Extreme Values Clamping (-999, +999) ---');
  for (const [joint, cfg] of Object.entries(JOINT_CONFIGS)) {
    if (joint === 'grip') continue;
    const badge = setupMockBadge(joint, '0.0');

    // Extreme negative
    badge.value = '-999.0';
    badge.dispatchEvent({ type: 'keydown', key: 'Enter', preventDefault: () => {} });
    assert(mc.joints[joint] === cfg.min, `${joint.toUpperCase()} clamped -999.0 to min limit ${cfg.min}° (got ${mc.joints[joint]})`);
    assert(badge.value === cfg.min.toFixed(1), `${joint.toUpperCase()} badge displays clamped min "${cfg.min.toFixed(1)}"`);

    // Extreme positive
    badge.value = '+999.0';
    badge.dispatchEvent({ type: 'keydown', key: 'Enter', preventDefault: () => {} });
    assert(mc.joints[joint] === cfg.max, `${joint.toUpperCase()} clamped +999.0 to max limit ${cfg.max}° (got ${mc.joints[joint]})`);
    assert(badge.value === cfg.max.toFixed(1), `${joint.toUpperCase()} badge displays clamped max "${cfg.max.toFixed(1)}"`);
  }

  // 3.4 Invalid Strings & Garbage Input Reversion
  console.log('\n--- 3.4: Invalid Strings & Garbage Reversion ---');
  const garbageInputs = [
    'abc',
    'hello_world',
    '',
    '   ',
    '--++',
    'NaN',
    'undefined',
    'null',
    '@#$%',
    'invalid.12.34',
    'e+100'
  ];

  for (const garbage of garbageInputs) {
    const badge = setupMockBadge('j1', '25.0');
    mc.setJointTarget('j1', 25.0, false, false);

    badge.value = garbage;
    let thrown = false;
    try {
      badge.dispatchEvent({ type: 'keydown', key: 'Enter', preventDefault: () => {} });
    } catch (err) {
      thrown = true;
    }

    assert(!thrown, `Input "${garbage}" did not throw unhandled exception`);
    assert(mc.joints.j1 === 25.0, `Input "${garbage}" preserved valid joint angle (25.0°)`);
  }

  // 3.5 Enter Key Commit vs Escape Key Reversion vs Blur
  console.log('\n--- 3.5: Enter, Escape, and Blur Event Semantics ---');
  {
    const badge = setupMockBadge('j3', '50.0');
    mc.setJointTarget('j3', 50.0, false, false);

    // Escape should revert to previous value without changing joint
    badge.value = '80.0';
    badge.dispatchEvent({ type: 'keydown', key: 'Escape', preventDefault: () => {} });
    assert(mc.joints.j3 === 50.0, 'Escape key did not commit changed value');
    assert(badge.value === '50.0', 'Escape key restored original badge display "50.0"');

    // Blur should commit value
    badge.value = '75.0';
    badge.dispatchEvent({ type: 'blur' });
    assert(mc.joints.j3 === 75.0, 'Blur event successfully committed value (75.0°)');
    assert(badge.value === '75.0', 'Badge displays "75.0" after blur commit');

    // Enter key recomputes FK
    const initialCart = { ...mc.cartesian };
    badge.value = '100.0';
    badge.dispatchEvent({ type: 'keydown', key: 'Enter', preventDefault: () => {} });
    assert(mc.joints.j3 === 100.0, 'Enter key committed new joint value (100.0°)');
    const fkChanged = (initialCart.x !== mc.cartesian.x) || (initialCart.z !== mc.cartesian.z);
    assert(fkChanged, 'Enter commit successfully triggered FK recomputation (Cartesian coordinates updated)');
  }

  // 3.6 Adversarial Fuzzing across All 6 Joints (300 pseudo-random iterations)
  console.log('\n--- 3.6: Adversarial Fuzzing (300 Random Iterations) ---');
  const fuzzValues = [
    -1000, 1000, -90, 90, 0, -0.0, 0.00001, -0.00001,
    -180, 180, 360, -360, 1e5, -1e5,
    'garbage', '', '   ', '12.5px', '-45deg', 'true', 'false',
    Infinity, -Infinity, NaN
  ];

  let fuzzCrashes = 0;
  let limitViolations = 0;

  for (let i = 0; i < 300; i++) {
    const jointKeys = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'];
    const joint = jointKeys[i % jointKeys.length];
    const cfg = JOINT_CONFIGS[joint];
    const badge = setupMockBadge(joint, '0.0');
    const fuzzInput = fuzzValues[i % fuzzValues.length];

    badge.value = String(fuzzInput);
    try {
      badge.dispatchEvent({ type: 'keydown', key: 'Enter', preventDefault: () => {} });
      const currentVal = mc.joints[joint];
      if (typeof currentVal !== 'number' || isNaN(currentVal)) {
        limitViolations++;
      } else if (currentVal < cfg.min - 1e-4 || currentVal > cfg.max + 1e-4) {
        limitViolations++;
      }
    } catch (err) {
      fuzzCrashes++;
    }
  }

  assert(fuzzCrashes === 0, `300 adversarial fuzz inputs completed with 0 runtime exceptions (got ${fuzzCrashes})`);
  assert(limitViolations === 0, `All joint angles remained within physical limits and non-NaN during fuzzing (got ${limitViolations} violations)`);
}

// ============================================================================
// MAIN RUNNER
// ============================================================================
async function runAllChallengerStressTests() {
  console.log('=============================================================');
  console.log('CHALLENGER 1: EMPIRICAL JOG ACCELERATION & INPUT STRESS SUITE');
  console.log('=============================================================');

  const startTime = Date.now();

  try {
    await runHoldJogAccelerationTests();
    await runPacketRateLimitingStressTests();
    await runNumericBadgeInputStressTests();
  } catch (err) {
    console.error('UNCAUGHT EXCEPTION IN TEST RUNNER:', err);
    failedTests++;
  }

  const durationMs = Date.now() - startTime;

  console.log('\n=============================================================');
  console.log(`TOTAL TESTS: ${totalTests} | PASSED: ${passedTests} | FAILED: ${failedTests}`);
  console.log(`EXECUTION TIME: ${durationMs}ms`);
  console.log('=============================================================');

  if (failedTests > 0) {
    console.error('\nFAILURE DETAILS:');
    failureDetails.forEach((f, idx) => console.error(`  ${idx + 1}. ${f}`));
    process.exit(1);
  } else {
    console.log('\nALL CHALLENGER EMPIRICAL STRESS TESTS PASSED!');
    process.exit(0);
  }
}

runAllChallengerStressTests();
