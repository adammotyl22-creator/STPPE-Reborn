/**
 * test_6dof_upgrades.js
 * Comprehensive automated verification for:
 * 1. 6-DOF Analytical Inverse & Forward Kinematics (FK/IK roundtrip)
 * 2. Singularity Avoidance & DLS (Wrist singularity, Boundary extension, Shoulder azimuth)
 * 3. Lookahead Corner Blending & Centripetal Velocity Scaling
 * 4. Software Backlash Direction Reversal Logic
 * 5. 6-DOF & Backlash PacketBuilder Serial Command Generation
 */

import {
  solveIK6DOF,
  forwardKinematics6DOF,
  rpyToRotMat,
  rotMatToRpy,
  BASE_H,
  L1,
  L2,
  TOOL_TOTAL_LEN
} from '../js/kinematics/RobotKinematics.js';

import {
  buildMovOrientationCommand,
  buildBacklashCommand
} from '../js/comms/PacketBuilder.js';
import { AppState } from '../js/state/AppState.js';
import { MotionController } from '../js/ui/MotionController.js';

let passed = 0;
let failed = 0;

function assert(condition, message) {
  if (condition) {
    console.log(`  [PASS] ${message}`);
    passed++;
  } else {
    console.error(`  [FAIL] ${message}`);
    failed++;
  }
}

console.log('======================================================');
console.log('SUITE 1: 6-DOF Kinematics Roundtrip Accuracy');
console.log('======================================================');

// Test poses generated across diverse reachable joint configurations within physical limits
const testConfigs = [
  { j1: 0,   j2: 15, j3: -10, j4: 0,   j5: 20,  j6: 0 },
  { j1: 20,  j2: 25, j3: 10,  j4: -20, j5: 35,  j6: 15 },
  { j1: -30, j2: 40, j3: 30,  j4: 25,  j5: -25, j6: -45 },
  { j1: 45,  j2: 0,  j3: 60,  j4: -40, j5: 45,  j6: 50 },
  { j1: -15, j2: 30, j3: 0,   j4: 15,  j5: -40, j6: 90 }
];

testConfigs.forEach((cfg, i) => {
  const fk = forwardKinematics6DOF(cfg.j1, cfg.j2, cfg.j3, cfg.j4, cfg.j5, cfg.j6);
  const ik = solveIK6DOF(fk.tcp.x, fk.tcp.y, fk.tcp.z, fk.roll, fk.pitch, fk.yaw, { currentAngles: cfg });

  assert(ik.success, `Config ${i + 1} IK solved successfully`);
  assert(!isNaN(ik.j1) && !isNaN(ik.j2) && !isNaN(ik.j3) && !isNaN(ik.j4) && !isNaN(ik.j5) && !isNaN(ik.j6),
    `Config ${i + 1} all joint angles are finite numbers`);

  const fkRound = forwardKinematics6DOF(ik.j1, ik.j2, ik.j3, ik.j4, ik.j5, ik.j6);
  const errPos = Math.hypot(fkRound.tcp.x - fk.tcp.x, fkRound.tcp.y - fk.tcp.y, fkRound.tcp.z - fk.tcp.z);
  assert(errPos < 0.05, `Config ${i + 1} position roundtrip error = ${errPos.toFixed(8)} mm (< 0.05 mm)`);
});

console.log('\n======================================================');
console.log('SUITE 2: Singularity Avoidance & Dampening');
console.log('======================================================');

// Test 2.1: Wrist Singularity (|J5| < 2.5 deg)
{
  const prevJ = { j1: 15, j2: 20, j3: 10, j4: 35, j5: 0.5, j6: 20 };
  const fkSingular = forwardKinematics6DOF(prevJ.j1, prevJ.j2, prevJ.j3, prevJ.j4, prevJ.j5, prevJ.j6);
  const ikSingular = solveIK6DOF(fkSingular.tcp.x, fkSingular.tcp.y, fkSingular.tcp.z,
                                fkSingular.roll, fkSingular.pitch, fkSingular.yaw,
                                { currentAngles: prevJ });

  assert(ikSingular.wristSingular, 'Detected wrist singularity condition (|J5| < 2.5 deg)');
  assert(Math.abs(ikSingular.j4 - prevJ.j4) < 1e-4, 'Wrist singularity locked J4 to previous angle without flip');
  const fkRoundtrip = forwardKinematics6DOF(ikSingular.j1, ikSingular.j2, ikSingular.j3, ikSingular.j4, ikSingular.j5, ikSingular.j6);
  const errSingular = Math.hypot(fkRoundtrip.tcp.x - fkSingular.tcp.x, fkRoundtrip.tcp.y - fkSingular.tcp.y, fkRoundtrip.tcp.z - fkSingular.tcp.z);
  assert(errSingular < 0.05, `Wrist singularity roundtrip position error = ${errSingular.toFixed(6)} mm`);
}

// Test 2.2: Reach Boundary Singularity (D approaching L1 + L2)
{
  const maxReach = L1 + L2; // ~411.3 mm
  // Target wrist position at 0.99 * maxReach
  const targetX = (maxReach * 0.99) + TOOL_TOTAL_LEN;
  const ikBoundary = solveIK6DOF(targetX, 0, BASE_H, 0, 0, 0);
  assert(ikBoundary.isNearBoundary, 'Near-boundary condition flagged when D > 0.98 * maxReach');
  assert(!isNaN(ikBoundary.j2) && !isNaN(ikBoundary.j3), 'Joint angles remain stable and non-NaN at boundary');

  const ikBeyond = solveIK6DOF(maxReach + 100.0, 0, BASE_H, 0, 0, 0);
  assert(ikBeyond.clamped, 'Target beyond maximum reach is smoothly clamped');
  assert(ikBeyond.warnings.includes('OUT_OF_REACH_MAXIMUM_EXTENSION'), 'OUT_OF_REACH_MAXIMUM_EXTENSION warning emitted');
}

// Test 2.3: Shoulder / Centerline Singularity (r -> 0)
{
  const prevJ = { j1: 42.5, j2: 10, j3: 10, j4: 0, j5: 0, j6: 0 };
  const ikCenter = solveIK6DOF(0.01, 0.01, 200, 0, 90, 0, { currentAngles: prevJ });
  assert(Math.abs(ikCenter.j1 - prevJ.j1) < 1e-4, 'Shoulder singularity (r ~ 0) preserves previous azimuth J1');
}

console.log('\n======================================================');
console.log('SUITE 3: Lookahead Trajectory Corner Blending Math');
console.log('======================================================');

function computeCornerScale(v1, v2) {
  let dot = 0, n1 = 0, n2 = 0;
  for (let k = 0; k < v1.length; ++k) {
    dot += v1[k] * v2[k];
    n1 += v1[k] * v1[k];
    n2 += v2[k] * v2[k];
  }
  n1 = Math.sqrt(n1);
  n2 = Math.sqrt(n2);
  if (n1 < 0.001 || n2 < 0.001) return 1.0;
  const cosTh = Math.max(-1.0, Math.min(1.0, dot / (n1 * n2)));
  if (cosTh <= 0.0) return 0.0;
  const th = Math.acos(cosTh);
  return Math.cos(th * 0.5);
}

// Test 3.1: Collinear segments (0 degree turn)
{
  const v1 = [10, 0, 0, 0, 0, 0];
  const v2 = [10, 0, 0, 0, 0, 0];
  const scale = computeCornerScale(v1, v2);
  assert(Math.abs(scale - 1.0) < 1e-5, `Collinear path yields cornerScale = 1.0 (got ${scale.toFixed(4)})`);
}

// Test 3.2: 60 degree turn
{
  const v1 = [10, 0, 0, 0, 0, 0];
  const v2 = [5, 8.66025, 0, 0, 0, 0]; // 60 deg turn
  const scale = computeCornerScale(v1, v2);
  const expected = Math.cos((60 * Math.PI / 180) * 0.5); // cos(30 deg) = 0.866
  assert(Math.abs(scale - expected) < 1e-3, `60 deg turn yields cornerScale = ${expected.toFixed(4)} (got ${scale.toFixed(4)})`);
}

// Test 3.3: 90 degree turn (sharp right angle)
{
  const v1 = [10, 0, 0, 0, 0, 0];
  const v2 = [0, 10, 0, 0, 0, 0];
  const scale = computeCornerScale(v1, v2);
  assert(scale === 0.0, `90 deg turn yields cornerScale = 0.0 (apex stop, got ${scale})`);
}

// Test 3.4: 180 degree reversal
{
  const v1 = [10, 0, 0, 0, 0, 0];
  const v2 = [-10, 0, 0, 0, 0, 0];
  const scale = computeCornerScale(v1, v2);
  assert(scale === 0.0, `180 deg reversal yields cornerScale = 0.0 (smooth stop at apex, got ${scale})`);
}

console.log('\n======================================================');
console.log('SUITE 4: Software Backlash Reversal Logic');
console.log('======================================================');

class BacklashCompSimulator {
  constructor(backlashDeg, stepsPerDeg) {
    this.backlashDeg = backlashDeg;
    this.stepsPerDeg = stepsPerDeg;
    this.lastDir = 0;
    this.backlashOffset = 0;
  }

  step(deltaDeg) {
    if (Math.abs(deltaDeg) < 0.001) return this.backlashOffset;
    const dir = deltaDeg > 0 ? 1 : -1;
    if (this.lastDir !== 0 && dir !== this.lastDir) {
      this.backlashOffset += (dir > 0) ?
        Math.round(this.backlashDeg * this.stepsPerDeg) :
        -Math.round(this.backlashDeg * this.stepsPerDeg);
    }
    this.lastDir = dir;
    return this.backlashOffset;
  }
}

{
  const sim = new BacklashCompSimulator(0.40, 20.0); // 0.40 deg backlash, 20 steps/deg = 8 steps
  assert(sim.step(10) === 0, 'First positive motion establishes direction without offset');
  assert(sim.step(5) === 0, 'Continuing in same direction injects zero backlash offset');
  assert(sim.step(-5) === -8, 'Reversal to negative direction injects -8 backlash steps');
  assert(sim.step(-10) === -8, 'Continuing in negative direction maintains current offset');
  assert(sim.step(5) === 0, 'Reversal back to positive direction injects +8 steps (offset returns to 0)');
}

console.log('\n======================================================');
console.log('SUITE 5: 6-DOF & Backlash PacketBuilder Commands');
console.log('======================================================');

{
  const cmdMov = buildMovOrientationCommand(200.5, -50.2, 140.0, 10.0, 45.0, -90.0, 1500, false);
  assert(cmdMov === 'MOV X200.5 Y-50.2 Z140.0 R10.0 P45.0 YW-90.0 D1500\n',
    `buildMovOrientationCommand (MOV): ${JSON.stringify(cmdMov)}`);

  const cmdMovL = buildMovOrientationCommand(200.5, -50.2, 140.0, 10.0, 45.0, -90.0, 2000, true);
  assert(cmdMovL === 'MOVL X200.5 Y-50.2 Z140.0 R10.0 P45.0 YW-90.0 D2000\n',
    `buildMovOrientationCommand (MOVL): ${JSON.stringify(cmdMovL)}`);

  const cmdQuery = buildBacklashCommand();
  assert(cmdQuery === 'BACKLASH\n', `buildBacklashCommand (query): ${JSON.stringify(cmdQuery)}`);

  const cmdSet = buildBacklashCommand(3, 0.45);
  assert(cmdSet === 'BACKLASH J3 0.450\n', `buildBacklashCommand (set J3): ${JSON.stringify(cmdSet)}`);
}

console.log('\n======================================================');
console.log('SUITE 6: MotionController & AppState 6-DOF Integration');
console.log('======================================================');

{
  const appState = new AppState();
  const live = appState.getState('livePose');
  const ghost = appState.getState('ghostPose');

  assert(live.roll !== undefined && live.pitch !== undefined && live.yaw !== undefined,
    'AppState livePose tracks roll, pitch, yaw');
  assert(ghost.roll !== undefined && ghost.pitch !== undefined && ghost.yaw !== undefined,
    'AppState ghostPose tracks roll, pitch, yaw');

  // Ingest telemetry with 6-DOF angles
  appState.updateLiveTelemetry({ j1: 0, j2: 30, j3: 45, j4: 10, j5: 20, j6: -15 });
  const updatedLive = appState.getState('livePose');
  assert(typeof updatedLive.roll === 'number' && typeof updatedLive.pitch === 'number' && typeof updatedLive.yaw === 'number',
    'AppState compute 6-DOF roll, pitch, yaw from joint angles');

  // Test MotionController 6-DOF methods
  const mc = new MotionController({ appState, autoSend: false });
  assert(mc.cartesian.orientationMode === '6dof', 'MotionController default orientationMode is "6dof"');

  // Test setOrientationTarget
  mc.setOrientationTarget('roll', 25.0, false);
  assert(mc.cartesian.roll === 25.0, `mc.setOrientationTarget('roll', 25.0) set roll to 25.0 (got ${mc.cartesian.roll})`);

  // Test nudgeOrientation
  const pitchBefore = mc.cartesian.pitch;
  mc.nudgeOrientation('pitch', 15.0, false);
  assert(Math.abs(mc.cartesian.pitch - (pitchBefore + 15.0)) < 1e-4, `mc.nudgeOrientation('pitch', 15.0) updated pitch from ${pitchBefore} to ${mc.cartesian.pitch}`);

  // Test setOrientationMode
  mc.setOrientationMode('preset');
  assert(mc.cartesian.orientationMode === 'preset', 'mc.setOrientationMode("preset") switches mode');
  mc.setOrientationMode('6dof');
  assert(mc.cartesian.orientationMode === '6dof', 'mc.setOrientationMode("6dof") switches mode');

  // Test backlash methods
  mc.setBacklash('j2', 0.55);
  assert(mc.backlash.j2 === 0.55, `mc.setBacklash('j2', 0.55) updated backlash (got ${mc.backlash.j2})`);

  const qRes = await mc.queryBacklash();
  assert(qRes.command === 'BACKLASH\n', `mc.queryBacklash() produced command: ${JSON.stringify(qRes.command)}`);

  const sRes = await mc.saveBacklash();
  assert(sRes.commands.length === 6, `mc.saveBacklash() produced 6 axis commands (got ${sRes.commands.length})`);
  assert(sRes.commands[1] === 'BACKLASH J2 0.550\n', `mc.saveBacklash() command for J2: ${JSON.stringify(sRes.commands[1])}`);

  // Test sendMovLinear with 6-DOF orientation
  const linearRes = await mc.sendMovLinear({ x: 250, y: 0, z: 150, roll: 0, pitch: 0, yaw: 0, duration: 2000 });
  assert(linearRes.command.startsWith('MOVL X250.0 Y0.0 Z150.0 R0.0 P0.0 YW0.0 D2000'),
    `mc.sendMovLinear() with 6-DOF dispatched: ${JSON.stringify(linearRes.command)}`);
}

console.log('\n======================================================');
console.log(`TOTAL 6-DOF UPGRADE TESTS: ${passed + failed} | PASSED: ${passed} | FAILED: ${failed}`);
console.log('======================================================');

if (failed > 0) {
  process.exit(1);
}
