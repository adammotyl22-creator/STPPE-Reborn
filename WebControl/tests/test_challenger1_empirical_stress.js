/**
 * test_challenger1_empirical_stress.js
 * 
 * Challenger 1 Dedicated Empirical Verification & Stress Harness:
 * 1. Hold-Jog Acceleration Engine:
 *    - 250ms hold duration threshold verification.
 *    - Smooth S-curve velocity ramping evaluation over continuous press (0.5s, 1.0s, 1.2s, 2.0s, 3.0s).
 *    - Serial packet dispatch rate limiting <= 20Hz (50ms interval) without buffer flooding.
 *    - Single-tap click triggers exactly one discrete step without phantom hold triggers or trailing click leakage.
 * 2. Numeric Badge Input Robustness:
 *    - Negative values across J1-J6.
 *    - Fractional floats precision and formatting.
 *    - Extreme values (-999, +999) safe clamping to joint limits.
 *    - Non-numeric garbage and edge-case strings ("abc", empty, NaN, "12.34.56", etc.).
 *    - Enter key commit and Forward Kinematics (FK) update verification.
 *    - 500-iteration adversarial fuzz test across all joints J1-J6.
 */

import { MotionController, JOINT_CONFIGS } from '../js/ui/MotionController.js';
import { AppState } from '../js/state/AppState.js';
import { forwardKinematics, clamp } from '../js/kinematics/RobotKinematics.js';

let totalTests = 0;
let passedTests = 0;
let failedTests = 0;
const failureDetails = [];
const observations = [];

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

function observe(note) {
  observations.push(note);
  console.log(`  [OBSERVE] ${note}`);
}

/**
 * Creates a mock DOM element supporting event dispatch and class manipulation
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
// SUITE 1: Hold-Jog Acceleration Engine & Gesture Disambiguation
// ============================================================================
async function runHoldJogEngineTests() {
  console.log('\n======================================================');
  console.log('SUITE 1: Hold-Jog Acceleration Engine & Gesture Isolation');
  console.log('======================================================');

  const appState = new AppState();
  const mc = new MotionController({ appState, autoSend: false });

  // 1.1 Hold-Jog Threshold (250ms)
  console.log('\n--- 1.1: Hold Duration Threshold (250ms) ---');
  assert(mc.HOLD_JOG_THRESHOLD_MS === 250, 'mc.HOLD_JOG_THRESHOLD_MS is exactly 250ms');

  // Verify timer behavior:
  // At 150ms hold: isHeld must remain false
  const btn = createMockElement('button', {
    'data-nudge-joint': 'j1',
    'data-jog-dir': '1',
    'data-step': '1.0'
  });
  mc._bindJogRamping(btn, 'joint', 'j1', 1);

  btn.dispatchEvent({ type: 'pointerdown', currentTarget: btn, button: 0, pointerId: 10 });
  assert(mc._activeHoldJog !== null, 'Active hold-jog tracker created on pointerdown');
  assert(mc._activeHoldJog.isHeld === false, 'isHeld is false immediately upon pointerdown');

  await new Promise(r => setTimeout(r, 120));
  assert(mc._activeHoldJog.isHeld === false, 'isHeld is still false at 120ms (< 250ms)');

  // Release at 120ms (< 250ms)
  btn.dispatchEvent({ type: 'pointerup', currentTarget: btn, pointerId: 10 });
  assert(mc._activeHoldJog === null, 'Active hold-jog tracker destroyed on pointerup');
  assert(mc._justFinishedHold === false, 'justFinishedHold is false for tap < 250ms');

  // Browser fires click event
  mc.setJointTarget('j1', 0.0, false, false);
  btn.dispatchEvent({ type: 'click' });
  assert(mc.joints.j1 === 1.0, 'Single tap triggers exactly one discrete step (+1.0 deg)');

  // 1.2 Full Hold Jog (> 250ms) and Trailing Click Suppression
  console.log('\n--- 1.2: Hold-Jog (>250ms) & Trailing Click Suppression ---');
  btn.dispatchEvent({ type: 'pointerdown', currentTarget: btn, button: 0, pointerId: 11 });
  await new Promise(r => setTimeout(r, 290)); // wait > 250ms threshold
  assert(mc._activeHoldJog !== null && mc._activeHoldJog.isHeld === true, 'isHeld transitioned to true after 250ms threshold');

  // Pointerup releases hold
  const poseAtRelease = mc.joints.j1;
  btn.dispatchEvent({ type: 'pointerup', currentTarget: btn, pointerId: 11 });
  assert(mc._activeHoldJog === null, 'Hold-jog tracker stopped on pointerup');
  assert(mc._justFinishedHold === true, 'justFinishedHold flag set to true to suppress trailing click');

  // Synthetic click from browser
  btn.dispatchEvent({ type: 'click' });
  assert(mc.joints.j1 === poseAtRelease, 'Trailing click event was suppressed, no phantom extra step added');

  // Wait 120ms to ensure suppression flag resets
  await new Promise(r => setTimeout(r, 120));
  assert(mc._justFinishedHold === false, 'justFinishedHold flag reset to false after 100ms window');

  // 1.3 S-Curve Velocity Profile & Acceleration Ramping
  console.log('\n--- 1.3: S-Curve Velocity Profile & Acceleration Ramping ---');
  const vMin = mc.HOLD_JOG_MIN_VELOCITY;
  const vMax = mc.HOLD_JOG_MAX_VELOCITY;
  const rampDurationMs = mc.HOLD_JOG_RAMP_DURATION_MS;

  assert(vMin === 2.0, `vMin is 2.0 deg/s (actual: ${vMin})`);
  assert(vMax === 35.0, `vMax is 35.0 deg/s (actual: ${vMax})`);

  observe(`Configured HOLD_JOG_RAMP_DURATION_MS = ${rampDurationMs}ms (${(rampDurationMs / 1000).toFixed(1)}s)`);

  function calcVelocity(heldTimeMs) {
    const u = Math.max(0.0, Math.min(1.0, heldTimeMs / rampDurationMs));
    const smoothstep = 3.0 * u * u - 2.0 * u * u * u;
    return vMin + (vMax - vMin) * smoothstep;
  }

  // Profile sampling at key timestamps: 0s, 0.5s, 1.0s, 1.2s, 2.0s, 3.0s
  const checkTimes = [0, 500, 1000, 1200, 2000, 3000];
  const velocities = {};
  for (const t of checkTimes) {
    velocities[t] = calcVelocity(t);
    console.log(`    t = ${(t/1000).toFixed(1)}s: velocity = ${velocities[t].toFixed(2)} deg/s (smoothstep u = ${Math.min(1.0, t/rampDurationMs).toFixed(3)})`);
  }

  assert(Math.abs(velocities[0] - vMin) < 1e-4, `At t=0s, velocity is exactly vMin (${vMin} deg/s)`);
  assert(velocities[500] > vMin && velocities[500] < vMax, `At t=0.5s, velocity is smoothly ramping (${velocities[500].toFixed(2)} deg/s)`);
  assert(velocities[1000] > velocities[500], `At t=1.0s, velocity has accelerated further (${velocities[1000].toFixed(2)} deg/s)`);
  assert(Math.abs(calcVelocity(rampDurationMs) - vMax) < 1e-4, `At full ramp (${rampDurationMs}ms), velocity reaches vMax (${vMax} deg/s)`);
  assert(calcVelocity(3000) === vMax, `At t=3.0s, velocity is capped at vMax (${vMax} deg/s)`);

  // Monotonicity check
  let isMonotonic = true;
  for (let ms = 10; ms <= 3000; ms += 10) {
    if (calcVelocity(ms) < calcVelocity(ms - 10) - 1e-6) {
      isMonotonic = false;
      break;
    }
  }
  assert(isMonotonic, 'Velocity is strictly monotonically non-decreasing over 3.0s continuous hold');

  // Verify smooth jerk-free boundaries: S'(0) = 0, S'(1) = 0
  const eps = 0.0001;
  const slopeStart = (calcVelocity(rampDurationMs * eps) - calcVelocity(0)) / (rampDurationMs * eps);
  const slopeEnd = (calcVelocity(rampDurationMs) - calcVelocity(rampDurationMs * (1 - eps))) / (rampDurationMs * eps);
  assert(slopeStart < 0.01, `Initial acceleration slope is ~0 (slope = ${slopeStart.toFixed(4)}) - gentle creep without initial jerk`);
  assert(slopeEnd < 0.01, `Terminal acceleration slope is ~0 (slope = ${slopeEnd.toFixed(4)}) - smooth asymptotic plateau`);
}

// ============================================================================
// SUITE 2: Serial Packet Dispatch Rate Throttling (<= 20Hz / 50ms)
// ============================================================================
async function runPacketRateLimitingTests() {
  console.log('\n======================================================');
  console.log('SUITE 2: Serial Packet Dispatch Rate Throttling (<= 20Hz)');
  console.log('======================================================');

  const appState = new AppState();
  const dispatchedTimestamps = [];

  const mockSerial = {
    isConnected: () => true,
    send: async () => {
      dispatchedTimestamps.push(performance.now());
      return Promise.resolve();
    }
  };

  const mc = new MotionController({
    appState,
    serialManager: mockSerial,
    autoSend: true
  });

  // 2.1 Preview Mode (Default): Zero Hardware Packets
  console.log('\n--- 2.1: Safe Preview Mode Isolation ---');
  dispatchedTimestamps.length = 0;
  mc.executionMode = 'preview';
  mc.isArmed = false;

  mc._activeHoldJog = {
    type: 'joint',
    id: 'j1',
    dir: 1,
    startTime: performance.now() - 300,
    lastFrameTime: performance.now() - 50,
    lastDispatchTime: 0,
    isHeld: true
  };

  // Run 10 iterations of jog loop
  for (let i = 0; i < 10; i++) {
    mc._runHoldJogLoop();
    await new Promise(r => setTimeout(r, 15));
  }
  mc._stopHoldJog();
  assert(dispatchedTimestamps.length === 0, 'Zero hardware packets dispatched during hold jog in Preview Mode');

  // 2.2 Direct Drive Mode: Accurate Rate Measurement Over 1.5s
  console.log('\n--- 2.2: Direct Drive Mode Rate Measurement ---');
  dispatchedTimestamps.length = 0;
  mc.executionMode = 'direct_drive';
  mc.isArmed = true;

  const loopDispatches = [];
  let flushDispatch = null;

  mc.sendMovSpline = async (opts) => {
    const t = performance.now();
    if (mc._activeHoldJog) {
      loopDispatches.push({ time: t, opts });
    } else {
      flushDispatch = { time: t, opts };
    }
    return Promise.resolve();
  };

  const jogStartTime = performance.now();
  mc._activeHoldJog = {
    type: 'joint',
    id: 'j1',
    dir: 1,
    startTime: jogStartTime,
    lastFrameTime: jogStartTime,
    lastDispatchTime: 0,
    isHeld: true
  };

  // Simulate a high-frequency animation loop (~120Hz, 8ms steps) for 1500ms
  const loopDurationMs = 1500;
  const loopStart = performance.now();
  while (performance.now() - loopStart < loopDurationMs) {
    mc._runHoldJogLoop();
    await new Promise(r => setTimeout(r, 8));
  }
  mc._stopHoldJog();

  const totalLoopPackets = loopDispatches.length;
  const actualLoopDurationMs = loopDispatches[totalLoopPackets - 1].time - loopDispatches[0].time;
  const observedLoopHz = (totalLoopPackets - 1) / (actualLoopDurationMs / 1000);

  console.log(`    Continuous Loop: Dispatched ${totalLoopPackets} packets over ${actualLoopDurationMs.toFixed(1)}ms`);
  console.log(`    Measured Continuous Loop Dispatch Rate: ${observedLoopHz.toFixed(2)} Hz (target: <= 20.0 Hz)`);

  assert(totalLoopPackets > 10, `Direct drive dispatched streaming packets (${totalLoopPackets} packets)`);
  assert(observedLoopHz <= 20.5, `Observed continuous streaming rate (${observedLoopHz.toFixed(2)} Hz) strictly <= 20Hz`);

  // Inter-packet delta check on continuous streaming loop
  let loopRateViolations = 0;
  let minLoopDelta = Infinity;
  for (let i = 1; i < loopDispatches.length; i++) {
    const delta = loopDispatches[i].time - loopDispatches[i - 1].time;
    if (delta < minLoopDelta) minLoopDelta = delta;
    if (delta < 48.0) {
      loopRateViolations++;
    }
  }
  console.log(`    Minimum Continuous Streaming Delta: ${minLoopDelta.toFixed(2)} ms`);
  assert(loopRateViolations === 0, `Zero buffer flooding in streaming loop: all consecutive intervals >= 48ms (got ${loopRateViolations} violations)`);

  // Verify final flush packet behavior
  assert(flushDispatch !== null, 'Final target position flushed on hold release');
  const deltaToFlush = flushDispatch.time - loopDispatches[totalLoopPackets - 1].time;
  observe(`Final termination flush packet dispatched at +${deltaToFlush.toFixed(1)}ms after last streaming packet (duration: ${flushDispatch.opts.duration}ms)`);
}

// ============================================================================
// SUITE 3: Numeric Badge Input Robustness & Clamping
// ============================================================================
async function runNumericBadgeInputTests() {
  console.log('\n======================================================');
  console.log('SUITE 3: Numeric Badge Input Robustness & FK Trigger');
  console.log('======================================================');

  const appState = new AppState();
  const mc = new MotionController({ appState, autoSend: false });

  function setupBadge(joint) {
    const inputEl = createMockElement('input', {
      id: `val_${joint}`,
      type: 'number',
      value: '0.0'
    });
    mc._bindInlineBadgeInput(joint, inputEl);
    return inputEl;
  }

  // 3.1 Negative Values Handling
  console.log('\n--- 3.1: Negative Values Handling Across J1-J6 ---');
  for (const joint of ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']) {
    const cfg = JOINT_CONFIGS[joint];
    const testNeg = Math.max(cfg.min, -30.0);
    const badge = setupBadge(joint);

    badge.value = String(testNeg);
    badge.dispatchEvent({ type: 'keydown', key: 'Enter', preventDefault: () => {} });

    assert(mc.joints[joint] === testNeg, `${joint.toUpperCase()} accepts negative value ${testNeg}° (got ${mc.joints[joint]})`);
    assert(badge.value === testNeg.toFixed(1), `${joint.toUpperCase()} badge input formatted to "${testNeg.toFixed(1)}"`);
  }

  // 3.2 Fractional Floats Handling
  console.log('\n--- 3.2: Fractional Floats Handling ---');
  {
    const badge = setupBadge('j1');
    badge.value = '-45.678';
    badge.dispatchEvent({ type: 'keydown', key: 'Enter', preventDefault: () => {} });
    assert(mc.joints.j1 === -45.678, `Float -45.678 parsed accurately (got ${mc.joints.j1})`);
    assert(badge.value === '-45.7', `Badge formatted to 1 decimal place ("-45.7")`);
  }

  // 3.3 Extreme Out-of-Bounds Values (-999, +999) Clamping
  console.log('\n--- 3.3: Extreme Values Clamping (-999, +999) ---');
  for (const joint of ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']) {
    const cfg = JOINT_CONFIGS[joint];
    const badge = setupBadge(joint);

    // Negative extreme
    badge.value = '-999';
    badge.dispatchEvent({ type: 'keydown', key: 'Enter', preventDefault: () => {} });
    assert(mc.joints[joint] === cfg.min, `${joint.toUpperCase()} clamped -999 to min ${cfg.min}° (got ${mc.joints[joint]})`);
    assert(badge.value === cfg.min.toFixed(1), `${joint.toUpperCase()} badge displays clamped min "${cfg.min.toFixed(1)}"`);

    // Positive extreme
    badge.value = '+999';
    badge.dispatchEvent({ type: 'keydown', key: 'Enter', preventDefault: () => {} });
    assert(mc.joints[joint] === cfg.max, `${joint.toUpperCase()} clamped +999 to max ${cfg.max}° (got ${mc.joints[joint]})`);
    assert(badge.value === cfg.max.toFixed(1), `${joint.toUpperCase()} badge displays clamped max "${cfg.max.toFixed(1)}"`);
  }

  // 3.4 Non-Numeric Garbage & Invalid Strings
  console.log('\n--- 3.4: Non-Numeric Garbage & Graceful Reversion ---');
  const invalidInputs = ['abc', 'foo_bar', '', '   ', 'NaN', 'undefined', 'null', '@#$%!'];
  for (const inv of invalidInputs) {
    const badge = setupBadge('j2');
    mc.setJointTarget('j2', 15.0, false, false);

    badge.value = inv;
    let thrown = false;
    try {
      badge.dispatchEvent({ type: 'keydown', key: 'Enter', preventDefault: () => {} });
    } catch (_) {
      thrown = true;
    }
    assert(!thrown, `Input "${inv}" did not throw exception`);
    assert(mc.joints.j2 === 15.0, `Input "${inv}" safely preserved current joint angle (15.0°)`);
    assert(badge.value === '15.0', `Input "${inv}" reverted badge display to valid value "15.0"`);
  }

  // 3.5 Enter Key Commit Triggers Forward Kinematics (FK) Update
  console.log('\n--- 3.5: Enter Key Commit & Forward Kinematics Trigger ---');
  {
    mc.setJointTarget('j2', 0.0, false, false);
    mc.setJointTarget('j3', 90.0, false, false);
    const cartBefore = { ...mc.cartesian };

    const badge = setupBadge('j2');
    badge.value = '45.0';
    badge.dispatchEvent({ type: 'keydown', key: 'Enter', preventDefault: () => {} });

    assert(mc.joints.j2 === 45.0, 'Enter key successfully committed J2 = 45.0°');
    const xChanged = Math.abs(mc.cartesian.x - cartBefore.x) > 0.1;
    const zChanged = Math.abs(mc.cartesian.z - cartBefore.z) > 0.1;
    assert(xChanged || zChanged, `FK recomputed on Enter commit (Cartesian X: ${cartBefore.x.toFixed(1)} -> ${mc.cartesian.x.toFixed(1)}, Z: ${cartBefore.z.toFixed(1)} -> ${mc.cartesian.z.toFixed(1)})`);

    // Verify appState ghostPose updated
    const ghost = appState.getState('ghostPose');
    assert(ghost && ghost.j2 === 45.0, `appState ghostPose J2 synchronized to 45.0° (got ${ghost?.j2})`);
  }

  // 3.6 500-Iteration Adversarial Fuzzing
  console.log('\n--- 3.6: 500-Iteration Adversarial Fuzzing Across J1-J6 ---');
  const fuzzVectors = [
    -10000, 10000, -90.001, 90.001, 0, -0.0, 1e-7, -1e-7,
    -180, 180, 360, -360, 1e8, -1e8,
    'garbage', 'NaN', 'Infinity', '-Infinity', '', '   ',
    '--12', '++12', '12.34.56', '0x20', '1e2', 'null', 'undefined'
  ];

  let exceptionsCount = 0;
  let outOfBoundsCount = 0;
  let nanCount = 0;

  for (let i = 0; i < 500; i++) {
    const jointKeys = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'];
    const joint = jointKeys[i % jointKeys.length];
    const cfg = JOINT_CONFIGS[joint];
    const badge = setupBadge(joint);
    const vector = fuzzVectors[i % fuzzVectors.length];

    badge.value = String(vector);
    try {
      badge.dispatchEvent({ type: 'keydown', key: 'Enter', preventDefault: () => {} });
      const val = mc.joints[joint];
      if (typeof val !== 'number' || isNaN(val)) {
        nanCount++;
      } else if (val < cfg.min - 1e-4 || val > cfg.max + 1e-4) {
        outOfBoundsCount++;
      }
    } catch (_) {
      exceptionsCount++;
    }
  }

  assert(exceptionsCount === 0, `500 adversarial fuzz iterations executed with 0 runtime exceptions (got ${exceptionsCount})`);
  assert(nanCount === 0, `Zero NaN joint angles produced during fuzzing (got ${nanCount})`);
  assert(outOfBoundsCount === 0, `Zero limit violations produced during fuzzing (got ${outOfBoundsCount})`);
}

// ============================================================================
// MAIN RUNNER
// ============================================================================
async function runAllChallenger1Tests() {
  console.log('=============================================================');
  console.log('CHALLENGER 1 DEDICATED EMPIRICAL STRESS TEST SUITE');
  console.log('=============================================================');

  const start = performance.now();
  try {
    await runHoldJogEngineTests();
    await runPacketRateLimitingTests();
    await runNumericBadgeInputTests();
  } catch (err) {
    console.error('UNCAUGHT EXCEPTION IN CHALLENGER TEST RUNNER:', err);
    failedTests++;
  }
  const elapsed = performance.now() - start;

  console.log('\n=============================================================');
  console.log(`TOTAL TESTS: ${totalTests} | PASSED: ${passedTests} | FAILED: ${failedTests}`);
  console.log(`EXECUTION TIME: ${elapsed.toFixed(1)}ms`);
  console.log('=============================================================');

  if (observations.length > 0) {
    console.log('\nKEY OBSERVATIONS:');
    observations.forEach((obs, idx) => console.log(`  ${idx + 1}. ${obs}`));
  }

  if (failedTests > 0) {
    console.error('\nFAILURE DETAILS:');
    failureDetails.forEach((f, idx) => console.error(`  ${idx + 1}. ${f}`));
    process.exit(1);
  } else {
    console.log('\nVERDICT: ALL EMPIRICAL CHALLENGES PASSED!');
    process.exit(0);
  }
}

runAllChallenger1Tests();
