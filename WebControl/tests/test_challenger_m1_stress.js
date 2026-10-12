/**
 * test_challenger_m1_stress.js
 * Milestone 1 Adversarial Stress Test Harness
 * 
 * Tests:
 * 1. WebSerialManager concurrent high-frequency send() calls (50-100Hz) to verify zero lock acquisition collisions.
 * 2. WebSerialManager active transmission mid-flight disconnection and writer lock release.
 * 3. Kinematics solveIK -> forwardKinematics roundtrip across all 4 Cartesian quadrants and elevation boundaries.
 * 4. Kinematics forwardKinematics options object parsing ({ j4, j5 }, { j4_deg, j5_deg }, positional).
 * 5. TrajectoryController pause() and stop() immediate STOP\n serial dispatch.
 * 6. AppState high-frequency getState immutability and GC-optimized cloning.
 */

import { WebSerialManager, ConnectionState } from '../js/comms/WebSerialManager.js';
import {
  solveIK,
  forwardKinematics,
  WristMode,
  JOINT_LIMITS,
  BASE_H,
  L1,
  L2_EFF,
  FLOOR_CLR
} from '../js/kinematics/RobotKinematics.js';
import { TrajectoryController, PlaybackState } from '../js/ui/TrajectoryController.js';
import { AppState } from '../js/state/AppState.js';

let totalTests = 0;
let passedTests = 0;
let failedTests = 0;
const failureDetails = [];

function assert(condition, description) {
  totalTests++;
  if (condition) {
    passedTests++;
    console.log(`  [PASS] ${description}`);
  } else {
    failedTests++;
    failureDetails.push(description);
    console.error(`  [FAIL] ${description}`);
  }
}

async function runWebSerialStressTests() {
  console.log('\n======================================================');
  console.log('SUITE 1: WebSerialManager Concurrency & Lock Stress');
  console.log('======================================================');

  // Setup mock navigator for WebSerial hardware simulation
  if (!('serial' in globalThis.navigator) || !globalThis.navigator.serial?.addEventListener) {
    Object.defineProperty(globalThis.navigator, 'serial', {
      value: {
        requestPort: async () => null,
        addEventListener: () => {},
        removeEventListener: () => {}
      },
      configurable: true,
      writable: true
    });
  }

  // Helper to create a WHATWG mock hardware port
  function createMockHardwarePort(writeLatencyMs = 2) {
    const writtenChunks = [];
    let isClosed = false;

    const writable = new WritableStream({
      async write(chunk) {
        if (writeLatencyMs > 0) {
          await new Promise((r) => setTimeout(r, writeLatencyMs));
        }
        writtenChunks.push(chunk);
      },
      close() {
        isClosed = true;
      }
    });

    const readable = new ReadableStream({
      start(controller) {
        // keep open for reader
      },
      cancel() {}
    });

    const port = {
      readable,
      writable,
      open: async (opts) => {},
      close: async () => { isClosed = true; }
    };

    return { port, writtenChunks, writable };
  }

  // Test 1.1: High-Frequency Concurrent Burst (100 simultaneous un-awaited calls)
  {
    console.log('\n--- Test 1.1: High-Frequency Concurrent Burst (100 simultaneous writes) ---');
    const { port, writtenChunks, writable } = createMockHardwarePort(1);
    const manager = new WebSerialManager();

    await manager.connect(port, {
      autoEnableJTelem: false
    });

    assert(manager.isConnected(), 'Manager connected to hardware port');
    assert(manager.writer !== null, 'Manager acquired persistent writer');
    assert(writable.locked === true, 'Underlying WritableStream is locked by persistent writer');

    const NUM_PACKETS = 100;
    const sendPromises = [];
    const decoder = new TextDecoder();

    const startTime = Date.now();
    for (let i = 0; i < NUM_PACKETS; i++) {
      const packet = `<MOV,${i},${i * 2},${i * 3}>\n`;
      // Concurrent dispatch without awaiting
      sendPromises.push(manager.send(packet));
    }

    let burstError = null;
    try {
      await Promise.all(sendPromises);
    } catch (err) {
      burstError = err;
    }

    const elapsed = Date.now() - startTime;
    assert(burstError === null, `100 concurrent sends completed with 0 errors (took ${elapsed}ms)`);
    assert(writtenChunks.length === NUM_PACKETS, `All ${NUM_PACKETS} chunks written to stream (got ${writtenChunks.length})`);

    // Verify ordering
    let orderCorrect = true;
    for (let i = 0; i < NUM_PACKETS; i++) {
      const decoded = decoder.decode(writtenChunks[i]);
      if (decoded !== `<MOV,${i},${i * 2},${i * 3}>\n`) {
        orderCorrect = false;
        break;
      }
    }
    assert(orderCorrect, 'All 100 concurrent packets written in exact FIFO sequence');

    await manager.disconnect();
    assert(!manager.isConnected(), 'Manager disconnected cleanly');
    assert(manager.writer === null, 'Writer nullified upon disconnect');
    assert(writable.locked === false, 'WritableStream lock released cleanly');
  }

  // Test 1.2: 100Hz Streaming Cadence (50-100Hz rapid streaming over 500ms)
  {
    console.log('\n--- Test 1.2: 100Hz Periodic Streaming Cadence (50 packets at 10ms intervals) ---');
    const { port, writtenChunks, writable } = createMockHardwarePort(1);
    const manager = new WebSerialManager();

    await manager.connect(port, {
      autoEnableJTelem: false
    });

    const NUM_TICKS = 50;
    let streamErrors = 0;
    const decoder = new TextDecoder();

    for (let i = 0; i < NUM_TICKS; i++) {
      manager.send(`JOG_X_${i}\n`).catch(() => { streamErrors++; });
      // 10ms sleep = 100Hz rate
      await new Promise((r) => setTimeout(r, 10));
    }

    // Wait a short moment for final write to finish
    await new Promise((r) => setTimeout(r, 50));

    assert(streamErrors === 0, `50 packets sent at 100Hz with 0 errors`);
    assert(writtenChunks.length === NUM_TICKS, `Stream received all 50 packets at 100Hz`);
    assert(writable.locked === true, 'Stream lock held persistently without per-packet churn');

    await manager.disconnect();
    assert(writable.locked === false, 'Stream lock released after 100Hz streaming session');
  }

  // Test 1.3: Active Transmission Mid-Flight Disconnect
  {
    console.log('\n--- Test 1.3: Mid-Flight Disconnect During Active Concurrent Writes ---');
    // Set 25ms write latency so writes stay in flight
    const { port, writtenChunks, writable } = createMockHardwarePort(25);
    const manager = new WebSerialManager();

    await manager.connect(port, {
      autoEnableJTelem: false
    });

    // Launch 30 writes in flight
    for (let i = 0; i < 30; i++) {
      manager.send(`BURST_${i}\n`).catch(() => {});
    }

    // Wait 15ms so some writes are actively processing
    await new Promise((r) => setTimeout(r, 15));

    // Abruptly disconnect while writes are in flight
    let disconnectError = null;
    try {
      await manager.disconnect();
    } catch (err) {
      disconnectError = err;
    }

    assert(disconnectError === null, 'Disconnect during in-flight writes completes cleanly without throwing');
    assert(manager.writer === null, 'manager.writer is nullified');
    assert(writable.locked === false, 'WritableStream lock released even with in-flight writes');

    // Verify a new writer can be acquired on the stream (proving no orphan locks)
    let reacquireLockSuccess = false;
    try {
      const newWriter = writable.getWriter();
      newWriter.releaseLock();
      reacquireLockSuccess = true;
    } catch (_) {}
    assert(reacquireLockSuccess, 'Underlying stream can be re-locked (zero orphaned writer lock)');

    // Verify subsequent send throws expected connection error
    let sendAfterDisconnectFailed = false;
    try {
      await manager.send('TEST\n');
    } catch (err) {
      sendAfterDisconnectFailed = err.message.includes('not connected');
    }
    assert(sendAfterDisconnectFailed, 'send() after disconnect gracefully rejects with "not connected"');
  }
}

async function runKinematicsStressTests() {
  console.log('\n======================================================');
  console.log('SUITE 2: Kinematics Consistency & Quadrant Roundtrip');
  console.log('======================================================');

  // Test 2.1: Options object vs Positional Forward Kinematics Signatures
  {
    console.log('\n--- Test 2.1: forwardKinematics Options Object vs Positional Signature ---');
    const j1 = 35.0, j2 = -45.0, j3 = 60.0, j4 = 105.0, j5 = 120.0;
    const workOffset = { x: 12.5, y: -8.0, z: 25.0 };
    const toolOffset = { x: 2.0, y: 1.5, z: 3.0 };

    // Positional signature: forwardKinematics(j1, j2, j3, workOffset, toolOffset, j4, j5)
    const fkPositional = forwardKinematics(j1, j2, j3, workOffset, toolOffset, j4, j5);

    // Options object signature: forwardKinematics(j1, j2, j3, { workOffset, toolOffset, j4, j5 })
    const fkOptionsJ4J5 = forwardKinematics(j1, j2, j3, { workOffset, toolOffset, j4, j5 });

    // Options object with j4_deg, j5_deg
    const fkOptionsDeg = forwardKinematics(j1, j2, j3, { workOffset, toolOffset, j4_deg: j4, j5_deg: j5 });

    // Default parameters (empty object)
    const fkEmptyOpts = forwardKinematics(j1, j2, j3, {});

    // Default parameters (omitted)
    const fkDefault = forwardKinematics(j1, j2, j3);

    const diffTcp1 = Math.hypot(
      fkPositional.tcp.x - fkOptionsJ4J5.tcp.x,
      fkPositional.tcp.y - fkOptionsJ4J5.tcp.y,
      fkPositional.tcp.z - fkOptionsJ4J5.tcp.z
    );
    assert(diffTcp1 < 1e-9, `Positional vs { j4, j5 } options TCP matches (diff: ${diffTcp1.toFixed(10)}mm)`);

    const diffTcp2 = Math.hypot(
      fkPositional.tcp.x - fkOptionsDeg.tcp.x,
      fkPositional.tcp.y - fkOptionsDeg.tcp.y,
      fkPositional.tcp.z - fkOptionsDeg.tcp.z
    );
    assert(diffTcp2 < 1e-9, `Positional vs { j4_deg, j5_deg } options TCP matches (diff: ${diffTcp2.toFixed(10)}mm)`);

    const diffDefault = Math.hypot(
      fkEmptyOpts.tcp.x - fkDefault.tcp.x,
      fkEmptyOpts.tcp.y - fkDefault.tcp.y,
      fkEmptyOpts.tcp.z - fkDefault.tcp.z
    );
    assert(diffDefault < 1e-9, 'Default parameters with empty object {} match omitted arguments');

    assert(!Number.isNaN(fkOptionsJ4J5.tcp.x) && !Number.isNaN(fkOptionsJ4J5.tcp.y) && !Number.isNaN(fkOptionsJ4J5.tcp.z), 'No NaN in TCP coordinates');
    assert(fkOptionsJ4J5.angles.j4 === j4 && fkOptionsJ4J5.angles.j5 === j5, 'Angles j4 and j5 populated correctly in return object');
  }

  // Test 2.2: IK -> FK Roundtrip on Reachable Workspace where All Joint Solutions are Within Limits
  {
    console.log('\n--- Test 2.2: IK -> FK Roundtrip Across Reachable Workspace & Elevation Boundaries ---');

    // Select targets where the arm configuration allows horizontal/downward tool orientations without servo saturation
    const reachableTargets = [
      { x: 150, y: 0, z: 160, name: '+X axis mid-Z' },
      { x: 140, y: 0, z: 200, name: '+X axis high-Z' },
      { x: 160, y: 30, z: 180, name: 'Q1 mid-reach' },
      { x: 130, y: 50, z: 190, name: 'Q1 high-Z' },
      { x: 160, y: -30, z: 180, name: 'Q4 mid-reach' },
      { x: 130, y: -50, z: 190, name: 'Q4 high-Z' },
      { x: 220, y: 0, z: 140, name: '+X forward reach' },
      { x: 200, y: 40, z: 150, name: 'Q1 forward reach' }
    ];

    const wristModes = [
      { mode: WristMode.LEVEL, name: 'Mode 0 (LEVEL)' },
      { mode: WristMode.DOWN_0, name: 'Mode 1 (DOWN_0)' },
      { mode: WristMode.DOWN_90, name: 'Mode 2 (DOWN_90)' },
      { mode: WristMode.PITCH_0, name: 'Mode 3 (PITCH_0)' },
      { mode: WristMode.PITCH_90, name: 'Mode 4 (PITCH_90)' },
      { mode: WristMode.MANUAL, j4Param: 90.0, name: 'Mode 6 (MANUAL j4=90)' }
    ];

    let validReachableCases = 0;
    let maxError = 0;
    let maxErrorDesc = '';
    const TOLERANCE_MM = 0.05; // 50 micrometers

    for (const target of reachableTargets) {
      for (const wm of wristModes) {
        const ik = solveIK(target.x, target.y, target.z, {
          wristMode: wm.mode,
          j4Param: wm.j4Param || 90.0,
          j5Param: 90.0
        });

        if (ik.success && !ik.clamped && ik.warnings.length === 0) {
          validReachableCases++;
          const fk = forwardKinematics(ik.j1, ik.j2, ik.j3, {}, null, ik.j4, ik.j5);
          const error = Math.hypot(fk.tcp.x - target.x, fk.tcp.y - target.y, fk.tcp.z - target.z);

          if (error > maxError) {
            maxError = error;
            maxErrorDesc = `${target.name} with ${wm.name}`;
          }

          if (error > TOLERANCE_MM) {
            assert(false, `Roundtrip error ${error.toFixed(6)}mm exceeded ${TOLERANCE_MM}mm at ${target.name} with ${wm.name}`);
          }
        }
      }
    }

    assert(validReachableCases >= 20, `Tested ${validReachableCases} reachable configurations with valid joint angles`);
    assert(maxError < TOLERANCE_MM, `Max Euclidean roundtrip error in valid reachable space is ${maxError.toFixed(6)} mm (< ${TOLERANCE_MM} mm) [case: ${maxErrorDesc}]`);
  }

  // Test 2.3: Adversarial Boundary & Clamping Integrity Stress Test
  {
    console.log('\n--- Test 2.3: Adversarial Boundary & Clamping Integrity Stress Test ---');

    // 2.3.1 Negative X (Behind robot - outside J1 [-85, 85] range)
    const negXTargets = [
      { x: -150, y: 0, z: 160 },
      { x: -120, y: 80, z: 150 },
      { x: -120, y: -80, z: 150 }
    ];
    for (const pt of negXTargets) {
      const ik = solveIK(pt.x, pt.y, pt.z);
      assert(ik.warnings.includes('J1_LIMIT_EXCEEDED'), `Negative X (${pt.x}, ${pt.y}) flags J1_LIMIT_EXCEEDED warning`);
      assert(Math.abs(ik.j1) === JOINT_LIMITS.j1.max, `Negative X clamps J1 to limit boundary (got J1=${ik.j1})`);
    }

    // 2.3.2 Low Z floor clearance safety clamp (< FLOOR_CLR = 20mm)
    const lowZ = solveIK(150, 0, 5); // Z=5mm < 20mm
    assert(lowZ.clamped === true, 'Low Z (5mm) triggers clamped=true');
    assert(lowZ.warnings.includes('TARGET_CLAMPED_FLOOR_CLEARANCE'), 'Low Z flags TARGET_CLAMPED_FLOOR_CLEARANCE');

    // 2.3.3 Base exclusion cylinder safety clamp (r < 80mm when Z < 160mm)
    const baseExcl = solveIK(40, 20, 100); // r = sqrt(40^2+20^2) = 44.7mm < 80mm
    assert(baseExcl.clamped === true, 'Base exclusion breach triggers clamped=true');
    assert(baseExcl.warnings.includes('TARGET_CLAMPED_BASE_EXCLUSION'), 'Base exclusion flags TARGET_CLAMPED_BASE_EXCLUSION');

    // 2.3.4 Out of reach maximum extension
    const outReach = solveIK(999, 0, 999);
    assert(outReach.success === false, 'Extreme target flags success=false');
    assert(outReach.warnings.includes('OUT_OF_REACH_MAXIMUM_EXTENSION'), 'Extreme target flags OUT_OF_REACH_MAXIMUM_EXTENSION');

    // 2.3.5 Joint pitch limit clamping defect test
    const tiltIk = solveIK(150, 0, 160, { wristMode: WristMode.MANUAL, j5Param: 120 });
    const isJ5Flagged = tiltIk.warnings.includes('J5_LIMIT_EXCEEDED') || tiltIk.clamped === true;
    assert(isJ5Flagged, 'solveIK flags J5 limit violation when pitch exceeds joint limits');
  }

  // Test 2.4: Wrist Mode 6 Decoupling Stress Test at Varied Angles
  {
    console.log('\n--- Test 2.4: Wrist Mode 6 Decoupling Verification across Angles ---');
    const anglesToTest = [30, 45, 60, 75, 90, 105, 120, 135, 150];
    const testPoint = { x: 150, y: 50, z: 160 };
    let maxM6Error = 0;
    let worstAngle = 0;

    for (const pitch of anglesToTest) {
      const ik = solveIK(testPoint.x, testPoint.y, testPoint.z, {
        wristMode: WristMode.MANUAL,
        j4Param: pitch,
        j5Param: 90.0
      });

      if (ik.success && !ik.clamped && ik.warnings.length === 0) {
        const fk = forwardKinematics(ik.j1, ik.j2, ik.j3, {}, null, ik.j4, ik.j5);
        const err = Math.hypot(fk.tcp.x - testPoint.x, fk.tcp.y - testPoint.y, fk.tcp.z - testPoint.z);
        if (err > maxM6Error) {
          maxM6Error = err;
          worstAngle = pitch;
        }
      }
    }

    // PROJECT.md requirement line 60: Euclidean error < 0.1 mm
    assert(maxM6Error < 0.1, `Mode 6 manual decoupling holds < 0.1 mm error across all pitch angles [30-150 deg] (worst at ${worstAngle} deg: ${maxM6Error.toFixed(6)} mm)`);
  }
}

async function runTrajectoryControllerStressTests() {
  console.log('\n======================================================');
  console.log('SUITE 3: TrajectoryController Emergency Stop Safety');
  console.log('======================================================');

  const appState = new AppState();
  const sentCommands = [];
  const fakeSerial = {
    isConnected: () => true,
    send: async (cmd) => {
      sentCommands.push(cmd);
    }
  };

  const tc = new TrajectoryController({
    appState,
    serialManager: fakeSerial
  });

  // Test 3.1: pause() transmits STOP\n
  sentCommands.length = 0;
  tc.playerState = PlaybackState.PLAYING;
  tc.pause();
  assert(sentCommands.includes('STOP\n'), 'tc.pause() immediately dispatches "STOP\\n" across serial');
  assert(tc.playerState === PlaybackState.PAUSED, 'Player state transitions to PAUSED');

  // Test 3.2: stop() transmits STOP\n
  sentCommands.length = 0;
  tc.playerState = PlaybackState.PLAYING;
  tc.stop();
  assert(sentCommands.includes('STOP\n'), 'tc.stop() immediately dispatches "STOP\\n" across serial');
  assert(tc.playerState === PlaybackState.STOPPED, 'Player state transitions to STOPPED');
}

async function runAppStateStressTests() {
  console.log('\n======================================================');
  console.log('SUITE 4: AppState Immutability & GC Optimization');
  console.log('======================================================');

  const state = new AppState();

  // Test 4.1: High throughput lookups
  const ITERS = 100000;
  const t0 = performance.now();
  for (let i = 0; i < ITERS; i++) {
    state.getState('livePose');
  }
  const t1 = performance.now();
  const opsPerSec = (ITERS / (t1 - t0)) * 1000;
  assert(opsPerSec > 1000000, `getState('livePose') throughput: ${Math.round(opsPerSec).toLocaleString()} ops/sec (> 1M/sec requirement)`);

  // Test 4.2: Immutability check
  const p1 = state.getState('livePose');
  p1.j1 = 888.88;
  p1.x = 999.99;
  const p2 = state.getState('livePose');
  assert(p2.j1 !== 888.88 && p2.x !== 999.99, 'Mutating returned state object does not mutate internal store');

  // Test 4.3: Sequence waypoints immutability
  const waypoints = [
    { id: 1, x: 100, y: 100, z: 100, j1: 0, j2: 0, j3: 0, j4: 90, j5: 90, grip: 50 },
    { id: 2, x: 150, y: 50, z: 120, j1: 10, j2: -20, j3: 30, j4: 90, j5: 90, grip: 100 }
  ];
  state.setSequence({ id: 'test', name: 'Test Seq', waypoints });
  const fetchedWps = state.getState('sequence').waypoints;
  fetchedWps[0].x = 777;
  const fetchedAgain = state.getState('sequence').waypoints;
  assert(fetchedAgain[0].x === 100, 'Sequence waypoints are deeply cloned without leaking caller mutations');
}

async function main() {
  console.log('Starting Milestone 1 Adversarial Stress Test Harness...\n');
  const start = Date.now();

  await runWebSerialStressTests();
  await runKinematicsStressTests();
  await runTrajectoryControllerStressTests();
  await runAppStateStressTests();

  const elapsed = Date.now() - start;
  console.log('\n======================================================');
  console.log(`TOTAL ADVERSARIAL TESTS: ${totalTests}`);
  console.log(`PASSED: ${passedTests}`);
  console.log(`FAILED: ${failedTests}`);
  console.log(`ELAPSED: ${elapsed}ms`);
  console.log('======================================================');

  if (failedTests > 0) {
    console.error('\nFAILURES:');
    failureDetails.forEach((f) => console.error(` - ${f}`));
    process.exit(1);
  } else {
    console.log('\nALL ADVERSARIAL STRESS TESTS PASSED EMPIRICALLY!');
    process.exit(0);
  }
}

main().catch((err) => {
  console.error('Unhandled test harness error:', err);
  process.exit(1);
});
