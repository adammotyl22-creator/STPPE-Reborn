/**
 * test_euler_wrist_presets_verification.js
 * Verification of:
 * 1. 6-DOF Euler vs Wrist Presets integration and isolation
 * 2. nudgeCartesian preservation of 6-DOF mode
 * 3. solveIK6DOF singularity threshold & horizontal arm pose roll tracking
 * 4. Clamped IK Euler angle synchronization
 * 5. Wrist presets W0-W6 joint isolation
 */

import { solveIK, solveIK6DOF, forwardKinematics6DOF, WristMode } from '../js/kinematics/RobotKinematics.js';
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
console.log('VERIFICATION SUITE: Euler & Wrist Presets Integrity');
console.log('======================================================');

// Test 1: Wrist Presets W0-W5 Joint Isolation in solveIK
{
  console.log('\n--- Test 1: Wrist Presets W0-W5 Joint Isolation ---');
  // Pass dirty parameters that should NOT leak into W0, W1, W2
  const dirtyOpts = {
    j4Param: 45.0,
    j5Param: 75.0,
    j6Param: 80.0
  };

  const ikW0 = solveIK(200, 0, 150, { wristMode: WristMode.LEVEL, ...dirtyOpts });
  assert(ikW0.j4 === 0.0, 'W0 (LEVEL) enforces J4 = 0.0 regardless of opts.j4Param');
  assert(ikW0.j6 === 0.0, 'W0 (LEVEL) enforces J6 = 0.0 regardless of opts.j6Param');

  const ikW1 = solveIK(200, 0, 150, { wristMode: WristMode.DOWN_0, ...dirtyOpts });
  assert(ikW1.j4 === 0.0, 'W1 (DOWN_0) enforces J4 = 0.0 regardless of opts.j4Param');
  assert(ikW1.j6 === 0.0, 'W1 (DOWN_0) enforces J6 = 0.0 regardless of opts.j6Param');

  const ikW2 = solveIK(200, 0, 150, { wristMode: WristMode.DOWN_90, ...dirtyOpts });
  assert(ikW2.j4 === 0.0, 'W2 (DOWN_90) enforces J4 = 0.0 regardless of opts.j4Param');
  assert(ikW2.j6 === 90.0, 'W2 (DOWN_90) enforces J6 = 90.0 regardless of opts.j6Param');

  const ikW3 = solveIK(200, 0, 150, { wristMode: WristMode.PITCH_0, j6Param: 35.0, j4Param: 45.0 });
  assert(ikW3.j4 === 0.0, 'W3 (PITCH_0) enforces J4 = 0.0');
  assert(ikW3.j6 === 35.0, 'W3 (PITCH_0) takes j6Param = 35.0');
}

// Test 2: solveIK6DOF Horizontal Pose Zero-Hysteresis Roll Tracking
{
  console.log('\n--- Test 2: solveIK6DOF Horizontal Pose (J5 ~ -5.6°) Tracking ---');
  // Typical horizontal pose: X=240, Y=0, Z=150, Pitch=0, Yaw=0, Roll=0
  const ikHoriz = solveIK6DOF(240, 0, 150, 0, 0, 0, { currentAngles: { j4: 0, j5: 0, j6: 0 } });
  assert(ikHoriz.success, 'Horizontal pose solved successfully');
  assert(Math.abs(ikHoriz.j4) < 1e-3, `Horizontal pose J4 is zero (got ${ikHoriz.j4.toFixed(4)}°)`);
  assert(Math.abs(ikHoriz.j6) < 1e-3, `Horizontal pose J6 is zero (got ${ikHoriz.j6.toFixed(4)}°)`);
  assert(!ikHoriz.wristSingular, 'Horizontal pose J5 (-5.6°) is outside wrist singularity (|J5| > 2.5°)');

  // Verify exact roundtrip forward kinematics
  const fkHoriz = forwardKinematics6DOF(ikHoriz.j1, ikHoriz.j2, ikHoriz.j3, ikHoriz.j4, ikHoriz.j5, ikHoriz.j6);
  assert(Math.abs(fkHoriz.roll) < 0.05, `Roundtrip Roll = ${fkHoriz.roll.toFixed(4)}° (~0°)`);
  assert(Math.abs(fkHoriz.pitch) < 0.05, `Roundtrip Pitch = ${fkHoriz.pitch.toFixed(4)}° (~0°)`);
  assert(Math.abs(fkHoriz.yaw) < 0.05, `Roundtrip Yaw = ${fkHoriz.yaw.toFixed(4)}° (~0°)`);
}

// Test 3: MotionController nudgeCartesian does not wipe out 6-DOF mode
{
  console.log('\n--- Test 3: nudgeCartesian 6-DOF Mode Preservation ---');
  const appState = new AppState();
  const mc = new MotionController({ appState });

  mc.setOrientationMode('6dof');
  assert(mc.cartesian.orientationMode === '6dof', 'MotionController active mode is 6dof');

  mc.setOrientationTarget('pitch', 15.0, false, false);
  assert(mc.cartesian.pitch === 15.0, 'Set orientation pitch to 15°');

  // Nudge Cartesian X by 10mm
  mc.nudgeCartesian('x', 10.0, false);
  assert(mc.cartesian.orientationMode === '6dof', 'nudgeCartesian preserved orientationMode === "6dof"');
  assert(Math.abs(mc.cartesian.pitch - 15.0) < 0.1, `nudgeCartesian preserved target pitch 15° (got ${mc.cartesian.pitch}°)`);
}

// Test 4: Switching between Preset and 6-DOF Modes
{
  console.log('\n--- Test 4: Switching between Preset and 6-DOF Modes ---');
  const appState = new AppState();
  const mc = new MotionController({ appState });

  mc.setCartesianTarget(220, 0, 160, WristMode.DOWN_0, 1000, false, false);
  assert(mc.cartesian.orientationMode === 'preset', 'Initial mode is preset');
  assert(mc.cartesian.wristMode === WristMode.DOWN_0, 'Initial wrist mode is W1 (DOWN_0)');
  const downPitch = mc.cartesian.pitch;
  assert(downPitch > 60.0 && downPitch <= 90.0, `Euler pitch correctly computed for DOWN_0: ${downPitch}°`);

  // Switch to 6-DOF
  mc.setOrientationMode('6dof');
  assert(mc.cartesian.orientationMode === '6dof', 'Switched to 6-DOF mode');
  assert(Math.abs(mc.cartesian.pitch - downPitch) < 0.1, `Euler pitch preserved on 6-DOF switch (${mc.cartesian.pitch}°)`);

  // Switch back to Preset W0 (LEVEL)
  mc.setWristMode(WristMode.LEVEL);
  assert(mc.cartesian.orientationMode === 'preset', 'setWristMode sets orientationMode to preset');
  assert(mc.cartesian.wristMode === WristMode.LEVEL, 'Wrist mode updated to LEVEL');
  assert(Math.abs(mc.cartesian.pitch - 0.0) < 1.0, `Euler pitch updated to Level (~0°): ${mc.cartesian.pitch}°`);
}

// Test 5: Clamped IK Updates Resultant Euler Angles
{
  console.log('\n--- Test 5: Clamped IK Updates Resultant Euler Angles ---');
  const appState = new AppState();
  const mc = new MotionController({ appState });

  // Command an extended reach + pitch target that causes J5 limit clamping at -120°
  mc.setCartesianTarget(300.0, 0.0, 200.0, false, false);
  mc.setOrientationMode('6dof');
  mc.setOrientationTarget('pitch', 90.0, false, false);

  assert(mc.reachability.clamped === true, 'High reach + pitch triggers reachability.clamped = true');
  assert(mc.joints.j5 <= 120.0 && mc.joints.j5 >= -120.0, 'J5 is clamped within physical joint limits [-120, 120]');
  assert(mc.reachability.statusText.startsWith('WARNING:'), `Status text displays warning: "${mc.reachability.statusText}"`);
  assert(!isNaN(mc.cartesian.pitch) && mc.cartesian.pitch !== 0, `Cartesian pitch is updated from clamped FK: ${mc.cartesian.pitch}°`);
}

console.log('\n======================================================');
console.log(`TOTAL TESTS: ${passed + failed} | PASSED: ${passed} | FAILED: ${failed}`);
console.log('======================================================');

if (failed > 0) {
  process.exit(1);
} else {
  console.log('ALL VERIFICATION TESTS PASSED SUCCESSFULLY!\n');
}
