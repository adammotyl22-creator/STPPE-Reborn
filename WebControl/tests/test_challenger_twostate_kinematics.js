/**
 * test_challenger_twostate_kinematics.js
 * Empirical Challenger Verification Harness:
 * 1. Two-State Safety Architecture (Safe Preview vs Live Direct Drive, modal interlocking, packet suppression)
 * 2. Tool-Frame Cartesian Jogging Math ($R_{06}$ rotation matrix projection, orthogonal zero-drift)
 * 3. Quick Pose Presets (Exact angles, soft limits, workspace safety, collision-free envelope)
 */

import { AppState } from '../js/state/AppState.js';
import { MotionController, JOINT_CONFIGS } from '../js/ui/MotionController.js';
import {
  forwardKinematics6DOF,
  solveIK6DOF,
  FLOOR_CLR
} from '../js/kinematics/RobotKinematics.js';
import { checkWorkspace } from '../js/kinematics/WorkspaceValidator.js';

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

// Mock Serial Manager that records dispatched packets
class MockRecordingSerialManager {
  constructor() {
    this.connected = true;
    this.sentPackets = [];
  }
  isConnected() {
    return this.connected;
  }
  async send(packet) {
    if (!this.connected) throw new Error('Not connected');
    this.sentPackets.push(packet);
    return true;
  }
  clear() {
    this.sentPackets = [];
  }
}

class MockClassList {
  constructor() {
    this.classes = new Set();
  }
  add(...names) {
    for (const n of names) this.classes.add(n);
  }
  remove(...names) {
    for (const n of names) this.classes.delete(n);
  }
  toggle(name, force) {
    if (force !== undefined) {
      if (force) this.classes.add(name);
      else this.classes.delete(name);
      return force;
    }
    if (this.classes.has(name)) {
      this.classes.delete(name);
      return false;
    } else {
      this.classes.add(name);
      return true;
    }
  }
  contains(name) {
    return this.classes.has(name);
  }
  has(name) {
    return this.classes.has(name);
  }
}

// Lightweight Mock DOM Element
class MockElement {
  constructor(id = '', tagName = 'div') {
    this.id = id;
    this.tagName = tagName.toUpperCase();
    this.attributes = {};
    this.classList = new MockClassList();
    this.eventListeners = {};
    this.value = '';
    this.textContent = '';
    this.checked = false;
    this.disabled = false;
    this.children = [];
  }
  getAttribute(name) {
    return this.attributes[name] ?? null;
  }
  setAttribute(name, val) {
    this.attributes[name] = String(val);
  }
  removeAttribute(name) {
    delete this.attributes[name];
  }
  hasAttribute(name) {
    return name in this.attributes;
  }
  addEventListener(event, handler) {
    if (!this.eventListeners[event]) this.eventListeners[event] = [];
    this.eventListeners[event].push(handler);
  }
  removeEventListener(event, handler) {
    if (!this.eventListeners[event]) return;
    this.eventListeners[event] = this.eventListeners[event].filter(h => h !== handler);
  }
  dispatchEvent(event) {
    const list = this.eventListeners[event.type] || [];
    for (const h of list) {
      h(event);
    }
  }
  querySelector(selector) {
    return this.querySelectorAll(selector)[0] || null;
  }
  querySelectorAll(selector) {
    const results = [];
    const check = (el) => {
      let match = false;
      if (selector.startsWith('#') && el.id === selector.slice(1)) match = true;
      else if (selector.startsWith('.') && el.classList.has(selector.slice(1))) match = true;
      else if (selector.startsWith('[') && selector.endsWith(']')) {
        const inner = selector.slice(1, -1);
        if (inner.includes('=')) {
          const [k, v] = inner.split('=').map(s => s.replace(/["']/g, '').trim());
          if (el.getAttribute(k) === v) match = true;
        } else {
          if (el.hasAttribute(inner)) match = true;
        }
      }
      if (match) results.push(el);
      for (const child of el.children) check(child);
    };
    check(this);
    return results;
  }
}

// Helper to construct a mock DOM tree representing index.html motion panel
function createMockMotionDom() {
  const container = new MockElement('left-panel');

  // Sliders and inputs for J1..J6 and grip
  const joints = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'grip'];
  for (const j of joints) {
    const slider = new MockElement(`slider_${j}`, 'input');
    slider.setAttribute('data-joint-slider', j);
    slider.value = '0';
    container.children.push(slider);

    const valInput = new MockElement(`val_${j}`, 'input');
    valInput.setAttribute('data-joint-val', j);
    valInput.value = '0.0';
    container.children.push(valInput);

    // Jog minus & plus buttons
    const btnMinus = new MockElement(`btn_jog_${j}_minus`, 'button');
    btnMinus.setAttribute('data-nudge-joint', j);
    btnMinus.setAttribute('data-jog-dir', '-1');
    btnMinus.classList.add('btn-jog');
    btnMinus.classList.add('btn-jog-minus');
    container.children.push(btnMinus);

    const btnPlus = new MockElement(`btn_jog_${j}_plus`, 'button');
    btnPlus.setAttribute('data-nudge-joint', j);
    btnPlus.setAttribute('data-jog-dir', '1');
    btnPlus.classList.add('btn-jog');
    btnPlus.classList.add('btn-jog-plus');
    container.children.push(btnPlus);
  }

  // Safety Controls
  const toggleDD = new MockElement('toggle_direct_drive', 'input');
  toggleDD.checked = false;
  container.children.push(toggleDD);

  const bannerDD = new MockElement('banner_direct_drive_active', 'div');
  bannerDD.classList.add('hidden');
  container.children.push(bannerDD);

  const badgeExec = new MockElement('badge_execution_mode', 'span');
  container.children.push(badgeExec);

  const modalDD = new MockElement('modal_direct_drive_confirm', 'div');
  modalDD.classList.add('hidden');
  container.children.push(modalDD);

  const btnConfirmDD = new MockElement('btn_confirm_direct_drive', 'button');
  container.children.push(btnConfirmDD);

  const btnCancelDD = new MockElement('btn_cancel_direct_drive', 'button');
  container.children.push(btnCancelDD);

  const btnSendGhost = new MockElement('btn_send_ghost', 'button');
  container.children.push(btnSendGhost);

  const btnPlanTraj = new MockElement('btn_plan_trajectory', 'button');
  container.children.push(btnPlanTraj);

  // Quick Preset buttons
  for (const p of ['home', 'zero', 'ready', 'stow']) {
    const btnPreset = new MockElement(`btn_preset_${p}`, 'button');
    btnPreset.setAttribute('data-pose-preset', p);
    container.children.push(btnPreset);
  }

  // Cartesian stepping buttons
  for (const axis of ['x', 'y', 'z']) {
    for (const dir of [1, -1]) {
      const btn = new MockElement(`btn_step_${axis}_${dir > 0 ? 'pos' : 'neg'}`, 'button');
      btn.setAttribute('data-cart-axis', axis);
      btn.setAttribute('data-cart-dir', String(dir));
      container.children.push(btn);
    }
  }

  return {
    container,
    toggleDD,
    bannerDD,
    badgeExec,
    modalDD,
    btnConfirmDD,
    btnCancelDD,
    btnSendGhost,
    btnPlanTraj
  };
}

async function runTwoStateSafetyTests() {
  console.log('\n======================================================');
  console.log('CHALLENGER SUITE 1: Two-State Execution Safety Architecture');
  console.log('======================================================');

  const appState = new AppState();
  const serialMgr = new MockRecordingSerialManager();
  const dom = createMockMotionDom();

  // Set up global document stub for MotionController.bindDom
  globalThis.document = {
    querySelector: (sel) => dom.container.querySelector(sel),
    querySelectorAll: (sel) => dom.container.querySelectorAll(sel),
    getElementById: (id) => dom.container.querySelector('#' + id)
  };

  const mc = new MotionController({
    appState,
    serialManager: serialMgr,
    container: dom.container
  });

  // 1. Initial State Verification
  assert(mc.executionMode === 'preview', 'Default executionMode is "preview" (Safe Preview)');
  assert(mc.isArmed === false, 'Default isArmed is false');
  assert(dom.toggleDD.checked === false, '#toggle_direct_drive is unchecked by default');
  assert(dom.bannerDD.classList.has('hidden'), '#banner_direct_drive_active has "hidden" class');
  assert(!dom.bannerDD.classList.has('active'), '#banner_direct_drive_active does NOT have "active" class');

  // 2. Safe Preview: Slider Adjustments update Ghost Target only, ZERO packets dispatched
  serialMgr.clear();
  mc.setJointTarget('j1', 35.0, false, true); // Continuous drag
  assert(Math.abs(mc.joints.j1 - 35.0) < 0.01, 'setJointTarget sets local joint J1');
  const ghostPose1 = appState.getState('ghostPose');
  assert(Math.abs(ghostPose1.j1 - 35.0) < 0.01, 'appState ghostPose.j1 updated to 35.0');
  assert(serialMgr.sentPackets.length === 0, 'ZERO serial packets dispatched during slider drag in preview mode');

  mc.flushFinalTarget(); // Slider release / change
  assert(serialMgr.sentPackets.length === 0, 'ZERO serial packets dispatched on slider release/flush in preview mode');

  // Multi-joint slider adjustments
  for (const j of ['j2', 'j3', 'j4', 'j5', 'j6']) {
    mc.setJointTarget(j, 15.0, false, true);
    mc.flushFinalTarget();
  }
  assert(serialMgr.sentPackets.length === 0, 'All joint sliders (J2..J6) in preview mode produce 0 serial packets');

  // 3. Safe Preview: Hold-Jog acceleration updates Ghost Target only, ZERO packets dispatched
  mc._activeHoldJog = {
    type: 'joint',
    id: 'j1',
    dir: 1,
    startTime: Date.now() - 500, // Past 250ms threshold
    lastFrameTime: Date.now() - 50,
    lastDispatchTime: 0,
    isHeld: true
  };
  mc._runHoldJogLoop();
  assert(serialMgr.sentPackets.length === 0, 'ZERO serial packets dispatched during hold-jog loop in preview mode');
  mc._stopHoldJog();
  assert(serialMgr.sentPackets.length === 0, 'ZERO serial packets dispatched on stopHoldJog in preview mode');

  // 4. Modal Interlocking: Clicking toggle does NOT immediately arm Direct Drive
  const mockPreventDefault = {
    defaultPrevented: false,
    preventDefault() { this.defaultPrevented = true; }
  };
  dom.toggleDD.checked = true; // User clicked the checkbox attempting to toggle it on
  dom.toggleDD.dispatchEvent({ type: 'click', ...mockPreventDefault });

  assert(dom.toggleDD.checked === false, 'Toggle click reverted checked state (blocked immediate arming)');
  assert(!dom.modalDD.classList.has('hidden'), 'Confirmation modal removed "hidden" class');
  assert(dom.modalDD.classList.has('active'), 'Confirmation modal added "active" class');
  assert(mc.executionMode === 'preview', 'executionMode remains "preview" pending confirmation');
  assert(mc.isArmed === false, 'isArmed remains false pending confirmation');
  assert(dom.bannerDD.classList.has('hidden'), 'Pulsing banner remains hidden');

  // 5. Modal Cancellation: Clicking Cancel dismisses modal without arming
  dom.btnCancelDD.dispatchEvent({ type: 'click' });
  assert(dom.modalDD.classList.has('hidden'), 'Modal closed on cancel');
  assert(!dom.modalDD.classList.has('active'), 'Modal "active" removed on cancel');
  assert(dom.toggleDD.checked === false, 'Toggle remains unchecked');
  assert(mc.executionMode === 'preview', 'executionMode remains "preview"');
  assert(mc.isArmed === false, 'isArmed remains false');

  // 6. Modal Confirmation: Clicking Arm Direct Drive arms the system
  // Re-trigger toggle click to show modal
  dom.toggleDD.checked = true;
  dom.toggleDD.dispatchEvent({ type: 'click', ...mockPreventDefault });
  assert(dom.modalDD.classList.has('active'), 'Modal reopened');

  dom.btnConfirmDD.dispatchEvent({ type: 'click' });
  assert(dom.modalDD.classList.has('hidden'), 'Modal closed on confirm');
  assert(dom.toggleDD.checked === true, 'Toggle is now checked');
  assert(mc.executionMode === 'direct_drive', 'executionMode is now "direct_drive"');
  assert(mc.isArmed === true, 'isArmed is now true');
  assert(!dom.bannerDD.classList.has('hidden'), 'Pulsing banner is now visible');
  assert(dom.bannerDD.classList.has('active'), 'Pulsing banner has "active" class');

  // 7. Live Direct Drive Streaming: Packets stream during hold-jogging
  serialMgr.clear();
  mc._activeHoldJog = {
    type: 'joint',
    id: 'j1',
    dir: 1,
    startTime: Date.now() - 500,
    lastFrameTime: Date.now() - 50,
    lastDispatchTime: 0,
    isHeld: true
  };
  mc._runHoldJogLoop();
  assert(serialMgr.sentPackets.length > 0, `Live packet dispatched during hold-jog when armed (got ${serialMgr.sentPackets.length} packets)`);
  assert(serialMgr.sentPackets[0].startsWith('<') || serialMgr.sentPackets[0].startsWith('MOV') || serialMgr.sentPackets[0].startsWith('#'), 'Dispatched packet is a valid motion packet');
  serialMgr.clear();

  // 8. Immediate Disarming on Toggle: Disarming requires 0 confirmation and restores safe preview
  dom.toggleDD.checked = false; // User unchecked the toggle
  dom.toggleDD.dispatchEvent({ type: 'click', ...mockPreventDefault });
  assert(mc.executionMode === 'preview', 'Disarming immediately resets executionMode to "preview"');
  assert(mc.isArmed === false, 'isArmed immediately reset to false');
  assert(dom.bannerDD.classList.has('hidden'), 'Pulsing banner immediately hidden');

  // Verify that after disarm, hold jog again produces 0 packets
  serialMgr.clear();
  mc._activeHoldJog = {
    type: 'joint',
    id: 'j1',
    dir: 1,
    startTime: Date.now() - 500,
    lastFrameTime: Date.now() - 50,
    lastDispatchTime: 0,
    isHeld: true
  };
  mc._runHoldJogLoop();
  assert(serialMgr.sentPackets.length === 0, 'ZERO packets dispatched after disarming');
  mc._stopHoldJog();

  // 9. Point-to-Point (PTP) Send Button: Dispatches move even in Preview Mode
  serialMgr.clear();
  await mc.sendTargetPTP(1500);
  assert(serialMgr.sentPackets.length === 1, 'Send to Target (PTP) button dispatches 1 coordinated hardware packet');
}

async function runToolFrameKinematicsTests() {
  console.log('\n======================================================');
  console.log('CHALLENGER SUITE 2: Tool-Frame Cartesian Jogging Math');
  console.log('======================================================');

  const appState = new AppState();
  const serialMgr = new MockRecordingSerialManager();
  const mc = new MotionController({
    appState,
    serialManager: serialMgr,
    autoSend: false
  });

  // Test across multiple orientations:
  // 1. Zero pose
  // 2. Pitch down 90 deg
  // 3. Roll 45 deg, Pitch 30 deg, Yaw -60 deg
  // 4. Complex 6-DOF pose
  const testPoses = [
    { name: 'Home Pose', joints: { j1: 0, j2: 25, j3: 140, j4: 0, j5: 0, j6: 0 } },
    { name: 'Wrist Pitch -90°', joints: { j1: 0, j2: 0, j3: 90, j4: 0, j5: -90, j6: 0 } },
    { name: 'Multi-Axis Roll/Pitch/Yaw', joints: { j1: 30, j2: -20, j3: 60, j4: 45, j5: -30, j6: 60 } },
    { name: 'Asymmetric 6-DOF Pose', joints: { j1: -45, j2: 40, j3: 100, j4: -60, j5: 50, j6: -45 } }
  ];

  mc.cartesianFrame = 'tool';
  const stepSize = 10.0; // mm

  for (const pose of testPoses) {
    console.log(`\n--- Testing Tool Frame Jogging at: ${pose.name} ---`);
    for (const [k, v] of Object.entries(pose.joints)) {
      mc.joints[k] = v;
    }
    mc._recomputeFK();

    // Verify initial FK
    const initialFk = forwardKinematics6DOF(
      mc.joints.j1, mc.joints.j2, mc.joints.j3,
      mc.joints.j4, mc.joints.j5, mc.joints.j6
    );
    const R06 = initialFk.R06;
    assert(R06 && R06.length === 3, `R06 rotation matrix available at ${pose.name}`);

    // Verify R06 is orthogonal: R * R^T = I, det(R) = +1
    let isOrthogonal = true;
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) {
        let dot = 0;
        for (let k = 0; k < 3; k++) dot += R06[r][k] * R06[c][k];
        const expected = (r === c) ? 1.0 : 0.0;
        if (Math.abs(dot - expected) > 1e-4) isOrthogonal = false;
      }
    }
    assert(isOrthogonal, `R06 is orthogonal SO(3) matrix at ${pose.name}`);

    // Test Tool +X, Tool +Y, Tool +Z steps
    const axes = [
      { axis: 'x', col: 0, unitVector: [1, 0, 0] },
      { axis: 'y', col: 1, unitVector: [0, 1, 0] },
      { axis: 'z', col: 2, unitVector: [0, 0, 1] }
    ];

    for (const { axis, col, unitVector } of axes) {
      // Re-establish base pose
      for (const [k, v] of Object.entries(pose.joints)) {
        mc.joints[k] = v;
      }
      mc._recomputeFK();
      const p0 = { x: mc.cartesian.x, y: mc.cartesian.y, z: mc.cartesian.z };

      // Expected delta in base frame: col_vector * stepSize
      const expectedDeltaBase = [
        R06[0][col] * stepSize,
        R06[1][col] * stepSize,
        R06[2][col] * stepSize
      ];

      // Perform stepCartesian along tool axis
      mc.stepCartesian(axis, 1, stepSize);

      const p1 = { x: mc.cartesian.x, y: mc.cartesian.y, z: mc.cartesian.z };
      const actualDeltaBase = [
        p1.x - p0.x,
        p1.y - p0.y,
        p1.z - p0.z
      ];

      // Check base frame delta components
      const diffX = Math.abs(actualDeltaBase[0] - expectedDeltaBase[0]);
      const diffY = Math.abs(actualDeltaBase[1] - expectedDeltaBase[1]);
      const diffZ = Math.abs(actualDeltaBase[2] - expectedDeltaBase[2]);
      const baseMatch = (diffX < 0.2 && diffY < 0.2 && diffZ < 0.2);

      assert(baseMatch, `Tool +${axis.toUpperCase()} base delta matches R06 col ${col} projection (diffs: [${diffX.toFixed(3)}, ${diffY.toFixed(3)}, ${diffZ.toFixed(3)}] mm)`);

      // Transform delta back to tool frame: delta_tool = R06^T * delta_base
      const deltaTool = [
        R06[0][0] * actualDeltaBase[0] + R06[1][0] * actualDeltaBase[1] + R06[2][0] * actualDeltaBase[2],
        R06[0][1] * actualDeltaBase[0] + R06[1][1] * actualDeltaBase[1] + R06[2][1] * actualDeltaBase[2],
        R06[0][2] * actualDeltaBase[0] + R06[1][2] * actualDeltaBase[1] + R06[2][2] * actualDeltaBase[2]
      ];

      // Verify ZERO drift along orthogonal tool axes
      const targetToolDelta = deltaTool[col];
      const other1 = deltaTool[(col + 1) % 3];
      const other2 = deltaTool[(col + 2) % 3];

      assert(Math.abs(targetToolDelta - stepSize) < 0.2, `Tool +${axis.toUpperCase()} traveled ${targetToolDelta.toFixed(2)} mm along tool axis (expected ${stepSize} mm)`);
      assert(Math.abs(other1) < 0.2, `Tool +${axis.toUpperCase()} orthogonal drift 1 is negligible (${other1.toFixed(4)} mm)`);
      assert(Math.abs(other2) < 0.2, `Tool +${axis.toUpperCase()} orthogonal drift 2 is negligible (${other2.toFixed(4)} mm)`);

      // Check Euclidean distance
      const distance = Math.sqrt(actualDeltaBase[0]**2 + actualDeltaBase[1]**2 + actualDeltaBase[2]**2);
      assert(Math.abs(distance - stepSize) < 0.2, `Tool +${axis.toUpperCase()} 3D step distance is exactly ${distance.toFixed(2)} mm`);
    }
  }
}

async function runQuickPresetsTests() {
  console.log('\n======================================================');
  console.log('CHALLENGER SUITE 3: Quick Pose Presets & Safety Envelope');
  console.log('======================================================');

  const appState = new AppState();
  const serialMgr = new MockRecordingSerialManager();
  const mc = new MotionController({
    appState,
    serialManager: serialMgr,
    autoSend: false
  });

  const expectedPresets = {
    home: {
      name: 'Home Pose',
      joints: { j1: 0.0, j2: 25.0, j3: 140.0, j4: 0.0, j5: 0.0, j6: 0.0, grip: 90.0 }
    },
    zero: {
      name: 'Zero All',
      joints: { j1: 0.0, j2: 0.0, j3: 0.0, j4: 0.0, j5: 0.0, j6: 0.0, grip: 0.0 }
    },
    ready: {
      name: 'Ready',
      joints: { j1: 0.0, j2: -30.0, j3: 90.0, j4: 0.0, j5: 30.0, j6: 0.0, grip: 90.0 }
    },
    stow: {
      name: 'Stow',
      joints: { j1: 0.0, j2: -60.0, j3: 160.0, j4: 0.0, j5: -85.0, j6: 0.0, grip: 0.0 }
    }
  };

  for (const [key, exp] of Object.entries(expectedPresets)) {
    console.log(`\n--- Verifying Preset: "${exp.name}" (${key}) ---`);

    // 1. Apply Preset in Preview Mode
    serialMgr.clear();
    const res = mc.applyPosePreset(key);

    assert(res.preset === exp.name, `applyPosePreset("${key}") returned preset name "${exp.name}"`);
    assert(serialMgr.sentPackets.length === 0, `Preset "${key}" in preview mode dispatches ZERO serial packets`);

    // 2. Exact Joint Angles
    let anglesMatch = true;
    for (const [j, expectedAngle] of Object.entries(exp.joints)) {
      const actualAngle = mc.joints[j];
      if (Math.abs(actualAngle - expectedAngle) > 0.01) {
        anglesMatch = false;
        console.error(`  Mismatch on ${j}: expected ${expectedAngle}, got ${actualAngle}`);
      }
    }
    assert(anglesMatch, `Preset "${key}" applied exact target joint angles`);

    // 3. AppState ghostPose synchronization
    const ghost = appState.getState('ghostPose');
    let ghostMatch = true;
    for (const [j, expectedAngle] of Object.entries(exp.joints)) {
      if (Math.abs(ghost[j] - expectedAngle) > 0.01) ghostMatch = false;
    }
    assert(ghostMatch, `Preset "${key}" synchronized exact angles to appState ghostPose`);

    // 4. Soft Limits Check
    let withinLimits = true;
    for (const [j, angle] of Object.entries(mc.joints)) {
      const cfg = JOINT_CONFIGS[j];
      if (angle < cfg.min || angle > cfg.max) {
        withinLimits = false;
        console.error(`  Joint ${j} out of range: ${angle} not in [${cfg.min}, ${cfg.max}]`);
      }
    }
    assert(withinLimits, `Preset "${key}" all joints strictly within soft limits`);

    // 5. Workspace and Collision Safety Check
    const ws = checkWorkspace(
      mc.joints.j1,
      mc.joints.j2,
      mc.joints.j3,
      mc.joints.j4,
      mc.joints.j5,
      mc.joints.j6
    );
    assert(ws.safe === true, `Preset "${key}" checkWorkspace reports safe: true`);
    assert(ws.violations.length === 0, `Preset "${key}" zero safety violations (got: ${ws.violations.join(', ') || 'none'})`);

    // 6. TCP Floor Clearance Check
    assert(mc.cartesian.z >= FLOOR_CLR, `Preset "${key}" TCP elevation Z (${mc.cartesian.z} mm) >= FLOOR_CLR (${FLOOR_CLR} mm)`);

    // 7. Direct Drive Mode preset dispatch
    mc.setExecutionMode('direct_drive', true);
    serialMgr.clear();
    mc.applyPosePreset(key);
    assert(serialMgr.sentPackets.length === 1, `Preset "${key}" in Direct Drive mode dispatches 1 coordinated motion packet (sent: ${serialMgr.sentPackets.length})`);
    if (serialMgr.sentPackets.length > 0) {
      assert(serialMgr.sentPackets[0].startsWith('<') || serialMgr.sentPackets[0].startsWith('MOV') || serialMgr.sentPackets[0].startsWith('#'), `Preset packet is valid motion packet: ${serialMgr.sentPackets[0].trim()}`);
    } else {
      assert(false, `Preset "${key}" failed to dispatch packet in Direct Drive mode`);
    }

    // Revert back to preview mode
    mc.setExecutionMode('preview', false);
  }
}

async function runAllChallengerTests() {
  console.log('=============================================================');
  console.log('CHALLENGER 2: EMPIRICAL TWO-STATE SAFETY & KINEMATICS TESTER');
  console.log('=============================================================');

  await runTwoStateSafetyTests();
  await runToolFrameKinematicsTests();
  await runQuickPresetsTests();

  console.log('\n=============================================================');
  console.log(`TOTAL ADVERSARIAL TESTS: ${totalTests} | PASSED: ${passedTests} | FAILED: ${failedTests}`);
  console.log('=============================================================');

  if (failedTests > 0) {
    console.error('\nFAILURE DETAILS:');
    failureDetails.forEach((f, idx) => console.error(`  ${idx + 1}. ${f}`));
    process.exit(1);
  } else {
    console.log('\nALL EMPIRICAL CHALLENGER TESTS PASSED SUCCESSFULLY! (100% Pass Rate)');
    process.exit(0);
  }
}

runAllChallengerTests().catch((err) => {
  console.error('Unhandled error in challenger test runner:', err);
  process.exit(1);
});
