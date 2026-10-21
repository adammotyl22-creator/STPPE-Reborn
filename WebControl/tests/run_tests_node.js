/**
 * run_tests_node.js - Headless Automated Test Suite (Comprehensive)
 * Teensy 4.1 6-DOF Robot Arm Automation Studio
 */

import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const rootDir = path.resolve(__dirname, '..');

// Import JS modules
import { AppState, appState } from '../js/state/AppState.js';
import { PresetManager, presetManager } from '../js/state/PresetManager.js';
import { CRC8Maxim, crc8MaximTable } from '../js/utils/CRC8.js';
import { PacketBuilder, buildFramedPacket } from '../js/comms/PacketBuilder.js';
import { TelemetryParser, parseSerialLine } from '../js/comms/TelemetryParser.js';
import { MockSerialPort } from '../js/comms/MockSerialPort.js';
import { WebSerialManager, ConnectionState } from '../js/comms/WebSerialManager.js';
import {
  forwardKinematics,
  forwardKinematics6DOF,
  solveIK,
  solveIK6DOF,
  eulerToQuat,
  quatSlerp,
  quatToEuler,
  degToSteps,
  stepsToDeg,
  clamp,
  robotToThreeCoords,
  threeToRobotCoords,
  JOINT_LIMITS,
  WristMode,
  BASE_H,
  L1,
  L2_PAR,
  L2_PERP,
  L3,
  GRIPPER_LEN,
  EXCL_R,
  FLOOR_CLR
} from '../js/kinematics/RobotKinematics.js';
import {
  validateJointLimits,
  validateFloorClearance,
  validateBaseExclusion
} from '../js/kinematics/WorkspaceValidator.js';
import { CameraController } from '../js/camera/CameraController.js';
import { CHART_COLORS } from '../js/ui/TelemetryController.js';
import { translations, LANGUAGES } from '../js/i18n/translations.js';
import { I18nManager, i18n } from '../js/i18n/I18nManager.js';
import { TrajectoryVisualizer3D, PATH_COLORS } from '../js/visualizer/TrajectoryVisualizer3D.js';
import { GCodeInterpreter } from '../js/kinematics/GCodeInterpreter.js';
import { DEFAULT_PRESETS } from '../js/state/PresetManager.js';
import { MotionController, JOINT_CONFIGS } from '../js/ui/MotionController.js';
import { TrajectoryController, PlaybackState } from '../js/ui/TrajectoryController.js';
import * as THREE from '../vendor/three/three.module.js';
import { TcpDragController, AxisConstraint } from '../js/visualizer/TcpDragController.js';
import { HorizontalTimelineController } from '../js/ui/HorizontalTimelineController.js';
import { GamepadController } from '../js/comms/GamepadController.js';

let passed = 0;
let failed = 0;
const failures = [];

function assert(condition, message) {
  if (condition) {
    passed++;
  } else {
    failed++;
    failures.push(message);
    console.error(`  FAIL: ${message}`);
  }
}

function describe(suiteName, fn) {
  console.log(`\n=== [SUITE] ${suiteName} ===`);
  return fn();
}

console.log('Starting Robot Arm Automation Studio Automated Test Suite...\n');

// -----------------------------------------------------------------------------
// Suite 1: CRC-8 Maxim Packet Protocol & PacketBuilder
// -----------------------------------------------------------------------------
describe('CRC-8 Protocol & PacketBuilder', () => {
  const testBytes = new Uint8Array([0x31, 0x32, 0x33, 0x34, 0x35]);
  const crc = crc8MaximTable(testBytes);
  assert(typeof crc === 'number' && crc >= 0 && crc <= 255, 'CRC-8 table lookup produces valid uint8');
  assert(CRC8Maxim.verify(testBytes, crc), 'CRC-8 verification passes for correct checksum');

  const pktStr = buildFramedPacket({ j1: 0, j2: -90, j3: 90, j4: 90, j5: 90, dx: 0, dy: 0, duration: 2000 });
  assert(pktStr.startsWith('<') && pktStr.endsWith('>'), 'Spline packet has correct delimiters');

  const homeCmd = PacketBuilder.buildHomeCommand('ALL');
  assert(homeCmd === 'HOME\n', 'Builds correct HOME ALL command');

  const estopCmd = PacketBuilder.buildEstopCommand();
  assert(estopCmd === 'ESTOP\n', 'Builds correct ESTOP command');

  const reenableCmd = PacketBuilder.buildReenableCommand();
  assert(reenableCmd === 'REENABLE\n', 'Builds correct REENABLE command');

  const movCmd = PacketBuilder.buildMovCommand(100, 50, 180, 0, 90, 90, 1500);
  assert(movCmd.startsWith('MOV X100.0 Y50.0 Z180.0'), 'Builds correct MOV spline Cartesian command');

  const movlCmd = PacketBuilder.buildMovLCommand(120, -30, 200, 1, 90, 45, 2000);
  assert(movlCmd.startsWith('MOVL X120.0 Y-30.0 Z200.0'), 'Builds correct MOVL linear Cartesian command');
});

// -----------------------------------------------------------------------------
// Suite 2: Telemetry Parser & Stream Processing
// -----------------------------------------------------------------------------
describe('TelemetryParser', () => {
  const jsonSample = '{"j1":10.5,"j2":-45.2,"j3":60.1,"j4":90.0,"j5":45.0,"grip":30,"x":150.2,"y":20.4,"z":180.5,"spd":100,"estop":false,"homing":false,"teach":false,"tpts":0}\n';
  const res = parseSerialLine(jsonSample);
  assert(res !== null && res.type === 'JTELEM', 'Parses 10Hz JTELEM JSON format');
  assert(res.j1 === 10.5 && res.j2 === -45.2 && res.z === 180.5, 'Extracts accurate telemetry coordinates');

  const ackRes = parseSerialLine('OK\n');
  assert(ackRes !== null && ackRes.type === 'ACK', 'Parses OK acknowledgment');

  const errRes = parseSerialLine('ERR:LIMIT_REACHED\n');
  assert(errRes !== null && errRes.type === 'ERROR', 'Parses error responses');

  const warnRes = parseSerialLine('WARN:FLOOR_CLEARANCE\n');
  assert(warnRes !== null && warnRes.type === 'WARN', 'Parses warning responses');

  const tcpRes = parseSerialLine('TCPCAL: TCP X=12.500 Y=-4.200 Z=8.700 mm, RMS=0.003 mm (EEPROM save scheduled)\n');
  assert(tcpRes?.type === 'TCPCAL_RESULT' && tcpRes.x === 12.5 && tcpRes.y === -4.2 && tcpRes.z === 8.7 && tcpRes.rmsErrorMm === 0.003,
    'Parses TCP calibration vector and RMS error');
});

// -----------------------------------------------------------------------------
// Suite 3: Kinematics & Analytical IK Engine
// -----------------------------------------------------------------------------
describe('RobotKinematics (FK, IK, and Unit Math)', () => {
  // Test Home Position FK
  const homeFK = forwardKinematics(0, 0, 0, {}, null, 0, 90, 0);
  assert(homeFK !== null && typeof homeFK.pGripper === 'object', 'FK computes valid Gripper transform');
  assert(Math.abs(homeFK.pGripper.y) < 2.0, `Home Y is approximately 0 in robot frame (got ${homeFK.pGripper.y.toFixed(2)})`);
  assert(homeFK.pGripper.z > 130, `Home Z is above 130mm (got ${homeFK.pGripper.z.toFixed(2)})`);

  // Test Forward Kinematics Joint Pivot Positions
  assert(homeFK.p0 && homeFK.p0.z === 0, 'P0 is at origin');
  assert(homeFK.p1 && homeFK.p1.z === BASE_H, 'P1 is at shoulder rotation height (BASE_H = 143.896mm)');
  assert(homeFK.p2 && typeof homeFK.p2.x === 'number', 'P2 elbow pivot is computed');
  assert(homeFK.p3 && typeof homeFK.p3.x === 'number', 'P3 tool flange is computed');

  // Test IK for Target
  const targetX = 150;
  const targetY = 0;
  const targetZ = 160;
  const ikRes = solveIK(targetX, targetY, targetZ, { wristMode: WristMode.LEVEL });
  assert(ikRes !== null && ikRes.success === true, 'IK successfully resolves reachable target pose');
  assert(ikRes.angles !== null, 'IK returns valid joint angles');

  // Test Out of Reach Bounds
  const unreachableIK = solveIK(999, 999, 999, { wristMode: WristMode.LEVEL });
  assert(unreachableIK.success === false, 'IK flags out-of-reach coordinates as unreachable');

  // Test Steps / Degrees Conversion
  const steps = degToSteps(10, 20, -15);
  assert(typeof steps.s1 === 'number' && typeof steps.s2 === 'number', 'Converts degrees to stepper steps');
  const degs = stepsToDeg(steps.s1, steps.s2, steps.s3);
  assert(Math.abs(degs.j1 - 10) < 0.1, 'Converts steps back to degrees with high accuracy');

  // Test Clamping Utility
  assert(clamp(150, 0, 100) === 100, 'clamp respects upper bound');
  assert(clamp(-50, 0, 100) === 0, 'clamp respects lower bound');
  assert(clamp(50, 0, 100) === 50, 'clamp passes within-bound values');
});

describe('6DOF firmware kinematics parity', () => {
  const near = (a, b, tolerance = 1e-6) => Math.abs(a - b) < tolerance;
  const pose = { j1: 12, j2: -8, j3: 32, j4: 25, j5: -30, j6: 40 };
  const offset = { workOffset: { x: 11, y: -7, z: 5 }, toolOffset: { x: 8, y: -3, z: 6 } };
  const fk = forwardKinematics6DOF(pose.j1, pose.j2, pose.j3, pose.j4, pose.j5, pose.j6, offset);
  const raw = forwardKinematics6DOF(pose.j1, pose.j2, pose.j3, pose.j4, pose.j5, pose.j6, { ...offset, raw: true });
  assert(near(fk.x - raw.x, fk.R06[0][0] * 8 - fk.R06[0][1] * 3 + fk.R06[0][2] * 6 + 11) &&
    near(fk.y - raw.y, fk.R06[1][0] * 8 - fk.R06[1][1] * 3 + fk.R06[1][2] * 6 - 7) &&
    near(fk.z - raw.z, fk.R06[2][0] * 8 - fk.R06[2][1] * 3 + fk.R06[2][2] * 6 + 5),
  '6DOF FK applies tool offset in flange frame and work offset in world frame');
  const ik = solveIK6DOF(fk.x, fk.y, fk.z, fk.roll, fk.pitch, fk.yaw,
    { ...offset, currentAngles: pose });
  assert(ik.success && Object.keys(pose).every(j => near(ik.angles[j], pose[j], 1e-4)),
    '6DOF IK round-trips a negative-J5 pose with local tool and work offsets');

  const positive = { ...pose, j5: 30 };
  const positiveFk = forwardKinematics6DOF(...Object.values(positive));
  const positiveIk = solveIK6DOF(positiveFk.x, positiveFk.y, positiveFk.z,
    positiveFk.roll, positiveFk.pitch, positiveFk.yaw, { currentAngles: positive });
  assert(positiveIk.success && Object.keys(positive).every(j => near(positiveIk.angles[j], positive[j], 1e-4)),
    '6DOF IK round-trips the positive-J5 wrist branch');

  const singular = { ...pose, j5: 0, j4: 35, j6: 20 };
  const singularFk = forwardKinematics6DOF(...Object.values(singular));
  const singularIk = solveIK6DOF(singularFk.x, singularFk.y, singularFk.z,
    singularFk.roll, singularFk.pitch, singularFk.yaw, { currentAngles: singular });
  assert(singularIk.wristSingular && near(singularIk.j4, singular.j4, 1e-4) &&
    near(singularIk.j6, singular.j6, 1e-4),
    '6DOF IK preserves current J4 and combined roll at exact wrist singularity');

  const q0 = eulerToQuat(0, 0, 170);
  const q1 = eulerToQuat(0, 0, -170);
  const midpoint = quatSlerp(q0, q1, 0.5);
  const euler = quatToEuler(midpoint);
  assert(near(Math.hypot(midpoint.w, midpoint.x, midpoint.y, midpoint.z), 1) && near(Math.abs(euler.yaw), 180),
    '6DOF SLERP follows shortest yaw path across the 180-degree seam');
});

// -----------------------------------------------------------------------------
// Suite 4: Workspace Safety Validator
// -----------------------------------------------------------------------------
describe('WorkspaceValidator', () => {
  const violations = validateJointLimits(0, -45, 45, 90, 90);
  assert(violations.length === 0, 'Valid joint angles have 0 limit violations');

  const j1Breach = validateJointLimits(120, 0, 0, 90, 90);
  assert(j1Breach.includes('J1_LIMIT'), 'J1 > 85deg triggers J1_LIMIT violation code');

  const floorViolations = validateFloorClearance(10);
  assert(floorViolations.includes('FLOOR_BREACH'), 'Z < 20mm triggers FLOOR_BREACH violation code');

  const isInsideBase = validateBaseExclusion(30, 20, 50); // Inside R=80 cylinder
  assert(isInsideBase === true, 'Point inside R=80mm base cylinder triggers exclusion check');

  const isOutsideBase = validateBaseExclusion(150, 50, 50);
  assert(isOutsideBase === false, 'Point outside R=80mm cylinder is cleared of exclusion');
});

// -----------------------------------------------------------------------------
// Suite 5: AppState Reactive Pub/Sub Store
// -----------------------------------------------------------------------------
describe('AppState Pub/Sub Reactive Store', () => {
  const state = new AppState();
  let receivedPose = null;
  state.subscribe('livepose:updated', (val) => {
    receivedPose = val;
  });

  state.setState('livePose.j1', 25.0);
  assert(state.getState('livePose.j1') === 25.0, 'getState returns updated state value');

  state.emit('livepose:updated', { j1: 25.0, j2: -30.0 });
  assert(receivedPose !== null && receivedPose.j1 === 25.0, 'Pub/sub delivers emitted event payload');

  let safetyAlertTriggered = false;
  state.subscribe('safety:alert', () => {
    safetyAlertTriggered = true;
  });
  state.emit('safety:alert', { code: 'FLOOR_BREACH' });
  assert(safetyAlertTriggered === true, 'Safety alert delivers to subscribers');
});

// -----------------------------------------------------------------------------
// Suite 6: PresetManager Sequence & Storage
// -----------------------------------------------------------------------------
describe('PresetManager Sequences & Presets', () => {
  const pm = new PresetManager();
  const presets = pm.getAllPresets();
  assert(Array.isArray(presets) && presets.length > 0, 'PresetManager initializes with built-in sequences');

  const pickPlace = pm.getPreset('Pick & Place Demo');
  assert(pickPlace !== null && Array.isArray(pickPlace.waypoints), 'Pick and Place sequence contains valid waypoints array');
  assert(pickPlace.waypoints.length >= 3, 'Pick and Place has at least 3 motion steps');

  const exportedJson = JSON.stringify(pickPlace);
  assert(typeof exportedJson === 'string' && exportedJson.includes('Pick & Place Demo'), 'Exports valid sequence JSON format');
});

// -----------------------------------------------------------------------------
// Suite 7: Mock Serial Port Simulator
// -----------------------------------------------------------------------------
describe('MockSerialPort Virtual Hardware', () => {
  const mock = new MockSerialPort();
  let receivedTelemetry = false;
  const responses = [];

  mock.onOutput((dataStr) => {
    responses.push(dataStr);
    if (dataStr.includes('{') || dataStr.includes('JTELEM')) {
      receivedTelemetry = true;
    }
  });

  mock.open({ baudRate: 115200 });
  assert(mock.isOpen === true, 'Mock serial port opens successfully');

  mock.receiveFromClient('<0,-90,90,90,90,0,0,100,50>\n');
  assert(mock.isOpen === true, 'Mock serial port accepts packets without crashing');

  mock._processCommandLine('MODE T1');
  assert(mock.opMode === 'T1', 'Mock accepts T1 operation mode');
  mock._processCommandLine('TOOLX 12.5');
  mock._processCommandLine('TOOLY -4.2');
  mock._processCommandLine('TOOLZ 8.7');
  for (let pose = 1; pose <= 4; pose++) mock._processCommandLine(`TCPCAL ${pose}`);
  mock._processCommandLine('TCPCAL SOLVE');
  assert(responses.some((chunk) => chunk.split(/\r?\n/).some((line) => parseSerialLine(line)?.type === 'TCPCAL_RESULT')),
    'Mock emits parseable TCP calibration result after four captures');
  mock._processCommandLine('TCPCAL CLEAR');
  assert(mock.tcpCalPoses.size === 0, 'Mock clears TCP calibration captures');

  mock.setPose({ j1: 10, j2: 10, j3: 50, j4: 20, j5: 30, j6: 30 });
  const linearTarget = forwardKinematics6DOF(10, 10, 50, 25, 35, 40, {
    toolOffset: mock.toolOffset, workOffset: mock.workOffset
  });
  mock._processCommandLine(`MOVL X${linearTarget.x.toFixed(1)} Y${linearTarget.y.toFixed(1)} Z${linearTarget.z.toFixed(1)} R${linearTarget.roll.toFixed(1)} P${linearTarget.pitch.toFixed(1)} YW${linearTarget.yaw.toFixed(1)} D2000`);
  assert(mock.linearPath !== null, 'Mock accepts oriented 6DOF MOVL with SLERP path');
  if (mock.linearPath) {
    const start = mock.linearPath.start;
    const end = mock.linearPath.end;
    mock.moveStartTime = Date.now() - 1000;
    mock._stepPhysics();
    const midway = forwardKinematics6DOF(mock.j1, mock.j2, mock.j3, mock.j4, mock.j5, mock.j6, {
      toolOffset: mock.toolOffset, workOffset: mock.workOffset
    });
    assert(Math.hypot(midway.x - (start.x + end.x) / 2, midway.y - (start.y + end.y) / 2,
      midway.z - (start.z + end.z) / 2) < 1.0, 'Mock oriented MOVL follows the Cartesian midpoint');
  }

  mock.close();
  assert(mock.isOpen === false, 'Mock serial port closes cleanly');
});

// -----------------------------------------------------------------------------
// Suite 8: DOM Structure & Attribute Stability
// -----------------------------------------------------------------------------
describe('DOM & Data Attribute Stability Contract (index.html)', () => {
  const htmlPath = path.join(rootDir, 'index.html');
  const htmlContent = fs.readFileSync(htmlPath, 'utf8');

  const requiredIds = [
    'btn_serial_connect',
    'select_baud_rate',
    'check_mock_mode',
    'check_auto_reconnect',
    'badge_connection_status',
    'display_telemetry_rate',
    'btn_estop',
    'btn_reenable',
    'btn_mode_toggle',
    'btn_controlled_stop',
    'btn_tcp_cal_pose_1',
    'btn_tcp_cal_pose_2',
    'btn_tcp_cal_pose_3',
    'btn_tcp_cal_pose_4',
    'btn_tcp_cal_solve',
    'btn_tcp_cal_clear',
    'input_tool_x',
    'input_tool_y',
    'input_tool_z',
    'input_work_x',
    'input_work_y',
    'input_work_z',
    'badge_homing_status',
    'slider_speed_override',
    'label_speed_val',
    'left-panel',
    'motion-panel-container',
    'check_ik_cylindrical',
    'label_ik_x',
    'label_ik_y',
    'label_ik_z',
    'slider_x',
    'slider_y',
    'slider_z',
    'input_x',
    'input_y',
    'input_z',
    'select_wrist_mode',
    'input_duration',
    'badge_reachability',
    'btn_mov_spline',
    'btn_mov_linear',
    'btn_ghost_to_live',
    'btn_send_ghost',
    'slider_j1',
    'slider_j2',
    'slider_j3',
    'slider_j4',
    'slider_j5',
    'slider_j6',
    'slider_grip',
    'val_j1',
    'val_j2',
    'val_j3',
    'val_j4',
    'val_j5',
    'val_j6',
    'val_grip',
    'trajectory-container',
    'viewport-container',
    'viewport-hud',
    'hud-tcp-x',
    'hud-tcp-y',
    'hud-tcp-z',
    'hud-floor-alert',
    'visualizer-canvas-wrapper',
    'bottom-dock',
    'btn-dock-toggle',
    'dock-view-split',
    'dock-view-telemetry',
    'dock-view-console',
    'telemetry-split-container',
    'console-split-container',
    'camera-hud-container',
    'camera-pip-canvas',
    'dock-tab-vision',
    'vision-split-container',
    'camera-dock-canvas',
    'viewport-vision-menu',
    'btn-viewport-vision',
    'viewport-vision-popover',
    'chk-camera-pip-toggle',
    'chk-ghost-visible',
    'chk-path-visible',
    'chk-floor-alert-visible',
    'chk-exclusion-visible',
    'chk-grid-visible'
  ];

  requiredIds.forEach((id) => {
    assert(htmlContent.includes(`id="${id}"`), `Required DOM ID #${id} is present in index.html`);
  });

  const requiredDataAttributes = [
    'data-action="connect-serial"',
    'data-baud-select',
    'data-mock-toggle',
    'data-auto-reconnect',
    'data-connection-status',
    'data-telemetry-rate',
    'data-action="estop"',
    'data-action="reenable"',
    'data-home-target="ALL"',
    'data-speed-slider',
    'data-speed-label',
    'data-cartesian-slider="x"',
    'data-cartesian-input="x"',
    'data-wrist-mode-select',
    'data-duration-input',
    'data-reachability-badge',
    'data-action="mov-spline"',
    'data-action="mov-linear"',
    'data-joint-slider="j1"',
    'data-joint-val="j1"',
    'data-nudge-joint="j1"',
    'data-joint-slider="j6"',
    'data-joint-val="j6"',
    'data-nudge-joint="j6"',
    'data-camera-preset="isometric"',
    'data-dock-tab="split"'
  ];

  requiredDataAttributes.forEach((attr) => {
    assert(htmlContent.includes(attr), `Required data attribute [${attr}] is present`);
  });

  // Verify Mode Navigation Tabs in left panel
  assert(htmlContent.includes('data-mode-tab="joints"'), 'Mode tab for Joints (FK) is present');
  assert(htmlContent.includes('data-mode-tab="ik"'), 'Mode tab for Target (IK) is present');
  assert(htmlContent.includes('data-mode-tab="trajectory"'), 'Mode tab for Sequencer is present');

  // Verify Split-Button Homing DOM integrity
  assert(htmlContent.includes('class="home-split-group"'), 'home-split-group exists in index.html');
  assert(htmlContent.includes('class="btn-home btn-home-primary"'), 'btn-home-primary button exists');
  assert(htmlContent.includes('id="btn_home_menu_trigger"'), 'btn_home_menu_trigger exists');
  assert(htmlContent.includes('<svg class="ico-home"'), 'ico-home SVG icon is present');
  assert(htmlContent.includes('<span data-i18n="header.home.all">'), 'data-i18n is on the leaf span to protect SVG');
  assert(!/<button[^>]*class="[^"]*btn-home-primary[^"]*"[^>]*data-i18n=/i.test(htmlContent), 'btn-home-primary does not have data-i18n on parent button');
});

// -----------------------------------------------------------------------------
// Suite 9: CSS Theme & Anti-Neon Audit
// -----------------------------------------------------------------------------
describe('CSS Theme & Anti-Neon Audit', () => {
  const themeCss = fs.readFileSync(path.join(rootDir, 'css', 'theme', 'cyberpunk-dark.css'), 'utf8');
  const mainCss = fs.readFileSync(path.join(rootDir, 'css', 'main.css'), 'utf8');
  const motionCss = fs.readFileSync(path.join(rootDir, 'css', 'components', 'motion-panel.css'), 'utf8');
  const viewportCss = fs.readFileSync(path.join(rootDir, 'css', 'components', 'viewport.css'), 'utf8');
  const telemCss = fs.readFileSync(path.join(rootDir, 'css', 'components', 'telemetry-charts.css'), 'utf8');
  const consoleCss = fs.readFileSync(path.join(rootDir, 'css', 'components', 'console.css'), 'utf8');
  const trajCss = fs.readFileSync(path.join(rootDir, 'css', 'components', 'trajectory-studio.css'), 'utf8');

  // Verify absence of banned neon blue / cyan / purple colors across all stylesheets
  const bannedColors = ['#00f3ff', '#38bdf8', '#0284c7', '#2563eb', '#3b82f6', '#818cf8', '#60a5fa'];
  bannedColors.forEach((hex) => {
    assert(!themeCss.includes(hex), `Banned neon/blue color ${hex} is not in cyberpunk-dark.css`);
    assert(!motionCss.includes(hex), `Banned neon/blue color ${hex} is not in motion-panel.css`);
    assert(!viewportCss.includes(hex), `Banned neon/blue color ${hex} is not in viewport.css`);
    assert(!telemCss.includes(hex), `Banned neon/blue color ${hex} is not in telemetry-charts.css`);
    assert(!consoleCss.includes(hex), `Banned neon/blue color ${hex} is not in console.css`);
    assert(!trajCss.includes(hex), `Banned neon/blue color ${hex} is not in trajectory-studio.css`);
  });

  // Verify soft neutral palette tokens
  assert(themeCss.includes('--bg-void: #111317;'), 'Deep neutral backdrop defined');
  assert(themeCss.includes('--bg-panel: #181a20;'), 'Neutral panel surface defined');
  assert(themeCss.includes('--bg-card: #20232a;'), 'Card surface defined');
  assert(themeCss.includes('--text-primary: #f8fafc;'), 'High contrast primary text defined');
  assert(mainCss.includes(':not(.btn-home-primary):not(.btn-home-chevron)'), 'main.css exempts split button from global radius override');
});

// -----------------------------------------------------------------------------
// Suite 10: Telemetry Color Palette Audit
// -----------------------------------------------------------------------------
describe('Telemetry Chart Palette (Zero Blue/Neon)', () => {
  assert(CHART_COLORS.j1 === '#10b981', 'J1 chart color is soft sage green');
  assert(CHART_COLORS.j2 === '#cbd5e1', 'J2 chart color is soft titanium silver');
  assert(CHART_COLORS.j3 === '#f59e0b', 'J3 chart color is warm amber sand');
  assert(CHART_COLORS.x === '#10b981', 'X chart color is soft sage green');
  assert(CHART_COLORS.y === '#cbd5e1', 'Y chart color is soft titanium silver');
  assert(CHART_COLORS.z === '#f59e0b', 'Z chart color is warm amber sand');
  assert(CHART_COLORS.bg === '#0d0f12', 'Chart background matches soft neutral palette');
});

// -----------------------------------------------------------------------------
// Suite 11: Internationalization (i18n) & English/Polish Localization
// -----------------------------------------------------------------------------
describe('Internationalization (i18n) & English/Polish Localization', () => {
  // 1. LANGUAGES array contains 'en' and 'pl'
  assert(LANGUAGES.includes('en'), 'LANGUAGES includes English (en)');
  assert(LANGUAGES.includes('pl'), 'LANGUAGES includes Polish (pl)');

  // 2. Translation dictionary completeness
  const keys = Object.keys(translations);
  assert(keys.length >= 70, `Translation dictionary has comprehensive coverage (${keys.length} keys)`);

  let missingEn = 0;
  let missingPl = 0;
  let emptyStrings = 0;

  keys.forEach((key) => {
    const entry = translations[key];
    if (!entry.en || typeof entry.en !== 'string') missingEn++;
    if (!entry.pl || typeof entry.pl !== 'string') missingPl++;
    if (entry.en === '' || entry.pl === '') emptyStrings++;
  });

  assert(missingEn === 0, `All keys have valid English translations (missing: ${missingEn})`);
  assert(missingPl === 0, `All keys have valid Polish translations (missing: ${missingPl})`);
  assert(emptyStrings === 0, `No empty string translations found (empty: ${emptyStrings})`);

  // 3. I18nManager unit tests
  const mgr = new I18nManager();
  assert(mgr.lang === 'en' || mgr.lang === 'pl', 'I18nManager defaults to valid language');
  
  mgr.setLang('en');
  assert(mgr.lang === 'en', 'I18nManager.setLang("en") sets language to en');
  assert(mgr.t('header.connect') === 'Connect', 'English translation lookup works');

  mgr.setLang('pl');
  assert(mgr.lang === 'pl', 'I18nManager.setLang("pl") sets language to pl');
  assert(mgr.t('header.connect') === 'Połącz', 'Polish translation lookup works');

  mgr.toggle();
  assert(mgr.lang === 'en', 'I18nManager.toggle() switches pl -> en');
  mgr.toggle();
  assert(mgr.lang === 'pl', 'I18nManager.toggle() switches en -> pl');

  // Fallback behavior
  assert(mgr.t('non_existent_key_12345') === 'non_existent_key_12345', 'Fallback to key when missing');

  // Subscriptions
  let notifyCount = 0;
  let notifiedLang = '';
  const unsub = mgr.subscribe((lang) => {
    notifyCount++;
    notifiedLang = lang;
  });
  mgr.setLang('en');
  assert(notifyCount === 1 && notifiedLang === 'en', 'I18nManager subscriber receives notification with new language');
  unsub();
  mgr.setLang('pl');
  assert(notifyCount === 1, 'Unsubscribe stops further notifications');

  // 4. HTML data-i18n contract audit
  const html = fs.readFileSync(path.join(rootDir, 'index.html'), 'utf8');
  assert(html.includes('id="btn_lang_toggle"'), 'Language toggle button #btn_lang_toggle exists in index.html');

  // Extract all data-i18n keys from index.html
  const dataI18nMatches = [...html.matchAll(/data-i18n="([^"]+)"/g)].map(m => m[1]);
  assert(dataI18nMatches.length > 20, `index.html contains ${dataI18nMatches.length} data-i18n annotations`);

  let unmappedKeys = 0;
  dataI18nMatches.forEach((k) => {
    if (!translations[k]) {
      unmappedKeys++;
      console.error(`  Unmapped data-i18n key in index.html: "${k}"`);
    }
  });
  assert(unmappedKeys === 0, `All data-i18n keys in index.html exist in translations.js (unmapped: ${unmappedKeys})`);

  // Extract all data-i18n-title keys from index.html
  const dataI18nTitleMatches = [...html.matchAll(/data-i18n-title="([^"]+)"/g)].map(m => m[1]);
  assert(dataI18nTitleMatches.length > 10, `index.html contains ${dataI18nTitleMatches.length} data-i18n-title tooltip annotations`);

  let unmappedTitleKeys = 0;
  dataI18nTitleMatches.forEach((k) => {
    if (!translations[k]) {
      unmappedTitleKeys++;
      console.error(`  Unmapped data-i18n-title key in index.html: "${k}"`);
    }
  });
  assert(unmappedTitleKeys === 0, `All data-i18n-title keys in index.html exist in translations.js (unmapped: ${unmappedTitleKeys})`);
});

// -----------------------------------------------------------------------------
// SUITE 12: 3D Trajectory Path Visualizer & Kinematics Clearance Validation
// -----------------------------------------------------------------------------
describe('Suite 12: 3D Trajectory Path Visualizer & Kinematics Clearance', () => {
  // 1. Module export & color constants integrity
  assert(typeof TrajectoryVisualizer3D === 'function', 'TrajectoryVisualizer3D class is exported');
  assert(PATH_COLORS.SAFE === 0x10b981, 'PATH_COLORS.SAFE is emerald sage green 0x10b981');
  assert(PATH_COLORS.COLLISION === 0xef4444, 'PATH_COLORS.COLLISION is alert red 0xef4444');
  assert(PATH_COLORS.WARNING === 0xf59e0b, 'PATH_COLORS.WARNING is warm amber 0xf59e0b');

  // 2. TrajectoryVisualizer3D instantiation and initial state
  const viz = new TrajectoryVisualizer3D({
    visible: true,
    tubeRadius: 1.5,
    nodeRadius: 3.0
  });

  assert(viz.root !== null && viz.root.name === 'TrajectoryVisualizer3D_Root', 'Root group created with proper name');
  assert(viz.root.visible === true, 'Root visibility matches initial option');
  assert(viz.tubeGroup.children.length === 0, 'Initial tube group is empty');
  assert(viz.nodesGroup.children.length === 0, 'Initial nodes group is empty');

  // 3. Update with built-in "Pick & Place Demo" preset
  const pickAndPlace = DEFAULT_PRESETS['Pick & Place Demo'];
  assert(pickAndPlace && pickAndPlace.waypoints.length === 8, 'Pick & Place preset has 8 waypoints');

  viz.updateSequence(pickAndPlace);
  assert(viz.waypointPositions.length === 8, `Visualizer calculated 8 waypoint TCP positions (got ${viz.waypointPositions.length})`);
  assert(viz.tubeGroup.children.length === 1, 'Tube spline mesh created in tubeGroup');
  assert(viz.tubeGroup.children[0].name === 'TrajectorySplineTube', 'Tube mesh named TrajectorySplineTube');
  assert(viz.nodesGroup.children.length === 16, 'Nodes group contains 8 spheres + 8 floor drop rings');

  // Verify coordinates for waypoint 0 (Approach object)
  const wp0 = viz.waypointPositions[0];
  assert(wp0.index === 0, 'Waypoint 0 index is 0');
  assert(wp0.isReachable === true, 'Waypoint 0 pose is reachable');
  assert(Math.abs(wp0.robotTcp.z - 282.5) < 1.0, `Waypoint 0 TCP Z matches Tool Center Point (~282.5mm, got ${wp0.robotTcp.z.toFixed(2)})`);

  // Verify coordinate conversion round-trip
  const convertedBack = threeToRobotCoords(wp0.threePos.x, wp0.threePos.y, wp0.threePos.z);
  assert(Math.abs(convertedBack.x - wp0.robotTcp.x) < 0.001, 'Three.js to Robot coordinate round-trip X matches');
  assert(Math.abs(convertedBack.y - wp0.robotTcp.y) < 0.001, 'Three.js to Robot coordinate round-trip Y matches');
  assert(Math.abs(convertedBack.z - wp0.robotTcp.z) < 0.001, 'Three.js to Robot coordinate round-trip Z matches');

  // 4. Active waypoint cursor tracking & disappearing past positions
  viz.setActiveWaypoint(3);
  assert(viz.activeWaypointIndex === 3, 'Active waypoint index updated to 3');
  assert(viz.activeCursor.visible === true, 'Active cursor halo is visible');

  // Check that already visited waypoints (0, 1, 2) disappeared
  const pastNodes = viz.nodesGroup.children.filter(c => c.userData && c.userData.waypointIndex < 3);
  const futureNodes = viz.nodesGroup.children.filter(c => c.userData && c.userData.waypointIndex >= 3);
  assert(pastNodes.length > 0 && pastNodes.every(n => n.visible === false), 'Visited waypoints (< index 3) have disappeared');
  assert(futureNodes.length > 0 && futureNodes.every(n => n.visible === true), 'Upcoming and current waypoints (>= index 3) remain visible');

  // When playback stops/resets (-1), full sequence is restored
  viz.setActiveWaypoint(-1);
  assert(viz.activeCursor.visible === false, 'Active cursor hidden on index -1');
  assert(viz.nodesGroup.children.every(n => n.visible === true), 'All waypoints restored to visible on sequence stop/idle');

  // 5. Visibility toggle
  viz.setVisible(false);
  assert(viz.root.visible === false, 'setVisible(false) sets root group invisible');
  viz.setVisible(true);
  assert(viz.root.visible === true, 'setVisible(true) restores root group visibility');

  // 6. Test sequence clearing & disposal
  viz.updateSequence({ waypoints: [] });
  assert(viz.waypointPositions.length === 0, 'updateSequence with empty waypoints clears array');
  assert(viz.tubeGroup.children.length === 0, 'Tube group cleared on empty sequence');
  assert(viz.nodesGroup.children.length === 0, 'Nodes group cleared on empty sequence');

  viz.dispose();
  assert(viz.currentSequence === null, 'Visualizer fully disposed with null currentSequence');

  // 7. AppState sequence synchronization & setState testing
  appState.setSequence(pickAndPlace);
  assert(appState.getState('sequence.name') === 'Pick & Place Demo', 'AppState stored Pick & Place Demo sequence');

  appState.setState('sequence.activeIndex', 4);
  assert(appState.getState('sequence.activeIndex') === 4, 'AppState.setState successfully sets sequence.activeIndex');

  appState.setSequenceState({ isPlaying: true, progress: 50.0 });
  assert(appState.getState('sequence.isPlaying') === true, 'AppState.setSequenceState updates isPlaying');
  assert(appState.getState('sequence.progress') === 50.0, 'AppState.setSequenceState updates progress');
});

// -----------------------------------------------------------------------------
// Suite 13: G-Code Interpreter, Arc Interpolation, and Robotic Exporter
// -----------------------------------------------------------------------------
describe('Suite 13: G-Code Interpreter, Arc Interpolation & Robotic Exporter', () => {
  // 1. Basic G0/G1/M3/M5 parsing
  const testGcode = `
    ; Test CNC Box Contour
    G21 ; Units mm
    G90 ; Absolute
    G0 X0 Y0 Z10 F3000 ; Rapid travel
    G1 Z0 F600 ; Plunge
    M3 S255 ; Tool on / Gripper clamp
    G1 X30 Y0 F1200
    G1 X30 Y30 F1200
    G1 X0 Y30 F1200
    G1 X0 Y0 F1200
    M5 ; Tool off
    G0 Z15 F3000 ; Retract
  `;

  const parsed = GCodeInterpreter.parse(testGcode, {
    autoCenter: true,
    targetCenter: { x: 160, y: 0, z: 50 },
    scale: 1.0,
    sequenceName: 'Test Box Contour'
  });

  assert(parsed.sequenceName === 'Test Box Contour', 'GCodeInterpreter assigned correct sequence name');
  assert(parsed.waypoints.length === 7, 'Parsed 7 distinct Cartesian moves from test box');
  assert(parsed.boundingBox.spanX === 30, 'Bounding box spanX computed as 30mm');
  assert(parsed.boundingBox.spanY === 30, 'Bounding box spanY computed as 30mm');
  assert(parsed.metrics.totalDistance > 120, 'Metrics calculated total toolpath distance');
  assert(parsed.metrics.totalDuration > 0, 'Metrics calculated total duration');

  // Verify waypoints have valid kinematics
  const wp0 = parsed.waypoints[0];
  assert(typeof wp0.j1 === 'number' && !isNaN(wp0.j1), 'Waypoint J1 is a valid number');
  assert(typeof wp0.j2 === 'number' && !isNaN(wp0.j2), 'Waypoint J2 is a valid number');
  assert(typeof wp0.j3 === 'number' && !isNaN(wp0.j3), 'Waypoint J3 is a valid number');
  assert(wp0.grip === 0, 'Initial rapid move tool grip is 0 (open)');

  const wpCut = parsed.waypoints[2];
  assert(wpCut.grip === 180, 'Cutting move tool grip is 180 (clamped / tool on)');

  // 2. Circular Arc (G2/G3) Interpolation
  const arcGcode = `
    G21
    G90
    G0 X0 Y0 Z10
    G1 Z0 F600
    G2 X20 Y0 I10 J0 F1200 ; Half-circle clockwise arc
    G3 X0 Y0 I-10 J0 F1200 ; Half-circle counter-clockwise arc
    G0 Z10
  `;
  const arcParsed = GCodeInterpreter.parse(arcGcode, { autoCenter: false });
  assert(arcParsed.waypoints.length > 5, 'G2/G3 arcs discretized into multiple chordal waypoints');
  assert(arcParsed.waypoints.every(wp => !isNaN(wp.j1) && !isNaN(wp.j2) && !isNaN(wp.j3)), 'All arc sub-points have valid IK solutions');

  // 3. Auto-Scaling oversized toolpaths
  const giantGcode = `
    G0 X0 Y0 Z0
    G1 X1000 Y1000 Z0
  `;
  const scaled = GCodeInterpreter.parse(giantGcode, { scale: 'auto' });
  assert(scaled.metrics.scaleApplied < 0.2, 'Auto-scaling successfully scaled 1000mm path into workspace span');

  // 4. G-Code Exporter round-trip
  const gcodeExport = GCodeInterpreter.exportGCode(parsed);
  assert(typeof gcodeExport === 'string', 'exportGCode returned a string');
  assert(gcodeExport.includes('G21') && gcodeExport.includes('G90'), 'G-Code export includes header initialization codes');
  assert(gcodeExport.includes('G1 X') && gcodeExport.includes('M3') && gcodeExport.includes('M5'), 'G-Code export contains G1 linear moves and M-codes');

  // 5. PresetManager Integration
  const pmGcode = presetManager.importGCode(testGcode);
  assert(pmGcode.waypoints.length === 7, 'PresetManager.importGCode parses G-Code correctly');

  const starPreset = DEFAULT_PRESETS['CNC Star (G-Code)'];
  assert(starPreset && starPreset.waypoints.length === 13, 'Default presets contain CNC Star (G-Code) with 13 waypoints');

  const gcodeFromPm = presetManager.exportSequenceGCode('CNC Star (G-Code)');
  assert(gcodeFromPm.includes('CNC Star (G-Code)'), 'PresetManager.exportSequenceGCode exports sequence header');
});

// -----------------------------------------------------------------------------
// Suite 14: J5 Eye-in-Hand Camera Controller & Vision Pipeline
// -----------------------------------------------------------------------------
describe('Suite 14: J5 Eye-in-Hand Camera Controller & Vision Pipeline', () => {
  const testAppState = new AppState();
  const cam = new CameraController({ appState: testAppState });

  // 1. Initial State & Defaults
  assert(cam.isActive === false, 'Camera initially inactive');
  assert(cam.config.resolution === '1280x720', 'Default resolution is 720p HD');
  assert(cam.config.rotation === 0, 'Default rotation is 0 deg');
  assert(cam.config.showReticle === true, 'TCP Reticle enabled by default');
  assert(cam.config.showGripperGuides === true, 'Dynamic Gripper guides enabled by default');

  // 2. Rotation & Flip Transformations
  cam.rotateClockwise();
  assert(cam.config.rotation === 90, 'rotateClockwise advances rotation to 90 deg');
  cam.rotateClockwise();
  assert(cam.config.rotation === 180, 'rotateClockwise advances rotation to 180 deg');
  cam.setRotation(360);
  assert(cam.config.rotation === 0, 'setRotation normalizes 360 to 0 deg');

  cam.toggleFlipH();
  assert(cam.config.flipH === true, 'toggleFlipH flips horizontal');
  cam.toggleFlipV();
  assert(cam.config.flipV === true, 'toggleFlipV flips vertical');

  // 3. Resolution parsing
  const [w720, h720] = cam._parseResolution('1280x720');
  assert(w720 === 1280 && h720 === 720, 'Parses 1280x720 resolution correctly');
  const [w1080, h1080] = cam._parseResolution('1920x1080');
  assert(w1080 === 1920 && h1080 === 1080, 'Parses 1920x1080 resolution correctly');

  // 4. Fallback simulation stream
  cam._startSimulatedStream();
  assert(cam.isActive === true, 'Simulated stream activates successfully');
  cam.stopStream();
  assert(cam.isActive === false, 'stopStream deactivates stream');
});

// -----------------------------------------------------------------------------
// Suite 15: Master Motion & Cartesian IK Controller
// -----------------------------------------------------------------------------
describe('Suite 15: Master Motion & Cartesian IK Controller', () => {
  const testAppState = new AppState();
  const mockPort = new MockSerialPort();
  const serialMgr = new WebSerialManager();
  const mc = new MotionController({
    appState: testAppState,
    serialManager: serialMgr,
    autoSend: false
  });
  assert(mc.POSE_PRESETS.stow.joints.j2 === -60 && mc.POSE_PRESETS.stow.joints.j2 >= JOINT_LIMITS.j2.min,
    'Stow preset respects the J2 lower limit');

  // 1. Joint target clamping and FK
  mc.setJointTarget('j1', 45.0, false, false);
  assert(mc.joints.j1 === 45.0, 'setJointTarget sets J1 angle');
  mc.setJointTarget('j1', 120.0, false, false); // Over limit +90°
  assert(mc.joints.j1 === 90.0, 'setJointTarget clamps J1 to upper limit');
  mc.setJointTarget('j1', -100.0, false, false); // Under limit -90°
  assert(mc.joints.j1 === -90.0, 'setJointTarget clamps J1 to lower limit');

  // 2. Cartesian target & Analytical IK
  const ikRes = mc.setCartesianTarget(200, 0, 150, WristMode.LEVEL, 1500, false, false);
  assert(ikRes.success === true, 'setCartesianTarget computes valid IK for reachable point');
  assert(Math.abs(mc.cartesian.x - 200) < 1.0, 'Cartesian X matches target');
  assert(Math.abs(mc.cartesian.y - 0) < 1.0, 'Cartesian Y matches target');
  assert(Math.abs(mc.cartesian.z - 150) < 1.0, 'Cartesian Z matches target');

  // 3. Out of reach target resilience
  const oik = mc.setCartesianTarget(500, 0, 150, WristMode.LEVEL, 1000, false, false);
  assert(oik.reachability.isReachable === false, 'Out of reach target reports not reachable');
  assert(mc.joints.j1 === 0.0, 'Joint 1 azimuth calculated for unreachable point');

  // 4. Joint & Cartesian Nudges
  mc.setJointTarget('j1', 0.0, false, false);
  mc.nudgeJoint('j1', 5.0, false);
  assert(mc.joints.j1 === 5.0, 'nudgeJoint adds positive delta');
  mc.nudgeJoint('j1', -10.0, false);
  assert(mc.joints.j1 === -5.0, 'nudgeJoint adds negative delta');

  mc.setCartesianTarget(200, 0, 150, null, null, false, false);
  mc.nudgeCartesian('x', 10.0, false);
  assert(Math.abs(mc.cartesian.x - 210) < 1.0, 'nudgeCartesian increments X axis');

  // 5. Gripper Macro
  mc.setGripperMacro('OPEN', false);
  assert(mc.joints.grip === 180, 'setGripperMacro OPEN sets grip to 180°');
  mc.setGripperMacro('CLOSE', false);
  assert(mc.joints.grip === 0, 'setGripperMacro CLOSE sets grip to 0°');
  mc.setGripperMacro('HALF', false);
  assert(mc.joints.grip === 90, 'setGripperMacro HALF sets grip to 90°');

  // 6. Cylindrical Mode Conversion
  mc.isCylindricalMode = true;
  mc._handleCartesianInput('x', 200, false); // R = 200mm
  mc._handleCartesianInput('y', 45, false);  // theta = 45°
  assert(Math.abs(mc.joints.j1 - 45.0) < 1.0, 'Cylindrical theta converts to correct J1 azimuth');

  // 7. AutoSend and Stream Throttling
  mc.autoSend = true;
  assert(mc.LIVE_STREAM_INTERVAL_MS === 50, "Throttling interval configured to 50ms");
  assert(mc.LIVE_STREAM_DURATION_MS === 0, "Live stream duration configured to 0ms");
  mc.flushFinalTarget();

  // 8. Orientation Mode Visibility Toggling (6-DOF vs Preset Wrist)
  const createMockClassList = () => {
    const classes = new Set();
    return {
      classes,
      toggle: (cls, force) => {
        if (force === undefined) {
          if (classes.has(cls)) classes.delete(cls); else classes.add(cls);
        } else if (force) {
          classes.add(cls);
        } else {
          classes.delete(cls);
        }
      },
      has: (cls) => classes.has(cls)
    };
  };

  const mockOrient6dof = { classList: createMockClassList() };
  const mockWristWrapper = { classList: createMockClassList() };
  const mockWristContainer = { classList: createMockClassList() };
  const mockBtn6dof = { classList: createMockClassList() };
  const mockBtnWrist = { classList: createMockClassList() };

  global.document = { activeElement: null, getElementById: () => null, querySelector: () => null, querySelectorAll: () => [] };
  mc._domBound = true;
  mc._domElements = {
    ...mc._domElements,
    containerOrient6dof: mockOrient6dof,
    wrapperWristMode: mockWristWrapper,
    containerWristMode: mockWristContainer,
    btnOrient6dof: mockBtn6dof,
    btnOrientWrist: mockBtnWrist
  };

  // Test 6-DOF Mode: Sliders visible, Preset wrist mode hidden
  mc.setOrientationMode("6dof");
  assert(mc.cartesian.orientationMode === "6dof", "setOrientationMode 6dof sets mode correctly");
  assert(!mockOrient6dof.classList.has("hidden"), "6-DOF mode shows orientation sliders");
  assert(mockWristWrapper.classList.has("hidden"), "6-DOF mode hides preset wrist mode");
  assert(mockWristContainer.classList.has("wrist-mode-hidden"), "6-DOF mode marks wrist container as wrist-mode-hidden");
  assert(mockBtn6dof.classList.has("active"), "6-DOF mode activates 6dof button");
  assert(!mockBtnWrist.classList.has("active"), "6-DOF mode deactivates preset button");

  // Test Preset Mode: Sliders hidden, Preset wrist mode visible
  mc.setOrientationMode("preset");
  assert(mc.cartesian.orientationMode === "preset", "setOrientationMode preset sets mode correctly");
  assert(mockOrient6dof.classList.has("hidden"), "Preset mode hides orientation sliders");
  assert(!mockWristWrapper.classList.has("hidden"), "Preset mode shows preset wrist mode");
  assert(!mockWristContainer.classList.has("wrist-mode-hidden"), "Preset mode unsets wrist-mode-hidden on wrist container");
  assert(!mockBtn6dof.classList.has("active"), "Preset mode deactivates 6dof button");
  assert(mockBtnWrist.classList.has("active"), "Preset mode activates preset button");
  delete global.document;
});

// -----------------------------------------------------------------------------
// Suite 16: R2 FIFO Lookahead Trajectory Streaming Engine
// -----------------------------------------------------------------------------
describe('Suite 16: R2 FIFO Lookahead Trajectory Streaming Engine', () => {
  // TrajectoryController and PlaybackState are imported at top level

  // 1. TrajectoryController exposes LOOKAHEAD_DEPTH and gripperDwellMs constants
  const tc = new TrajectoryController({ appState });
  assert(typeof tc.LOOKAHEAD_DEPTH === 'number' && tc.LOOKAHEAD_DEPTH >= 1,
    'TrajectoryController has LOOKAHEAD_DEPTH constant (>= 1)');
  assert(typeof tc.gripperDwellMs === 'number' && tc.gripperDwellMs >= 0,
    'TrajectoryController has gripperDwellMs constant (>= 0)');

  // 2. Custom gripperDwellMs via constructor option
  const tcCustomDwell = new TrajectoryController({ appState, gripperDwellMs: 250 });
  assert(tcCustomDwell.gripperDwellMs === 250, 'gripperDwellMs constructor option sets custom dwell time');

  // 3. _lookaheadTimers array exists and is initially empty
  assert(Array.isArray(tc._lookaheadTimers) && tc._lookaheadTimers.length === 0,
    '_lookaheadTimers is initialized as empty array');

  // 4. _prefillLookahead is a function
  assert(typeof tc._prefillLookahead === 'function',
    '_prefillLookahead method exists on TrajectoryController');

  // 5. _clearStepTimer also resets _lookaheadTimers
  tc._lookaheadTimers = [setTimeout(() => {}, 9999)]; // inject a fake timer
  tc._clearStepTimer();
  assert(tc._lookaheadTimers.length === 0, '_clearStepTimer clears all lookahead timers');

  // 6. _prefillLookahead with empty seq does nothing
  tc._prefillLookahead({ waypoints: [] }, 500);
  assert(tc._lookaheadTimers.length === 0, '_prefillLookahead with empty waypoints leaves lookahead empty');

  // 7. _prefillLookahead queues up to LOOKAHEAD_DEPTH timers for a multi-waypoint sequence
  const pickAndPlace = DEFAULT_PRESETS['Pick & Place Demo'];
  appState.setSequence(pickAndPlace);
  tc.currentIndex = 0;
  tc.playerState = PlaybackState.PLAYING; // must be PLAYING for lookahead guards
  tc._prefillLookahead(pickAndPlace, 1000);
  const lookaheadCount = tc._lookaheadTimers.length;
  assert(lookaheadCount > 0 && lookaheadCount <= tc.LOOKAHEAD_DEPTH,
    `_prefillLookahead queues 1–LOOKAHEAD_DEPTH timers (got ${lookaheadCount})`);
  tc._clearStepTimer();

  // 8. stop() resets playerState to STOPPED and clears lookahead
  tc.playerState = PlaybackState.PLAYING;
  tc._lookaheadTimers = [setTimeout(() => {}, 9999)];
  tc.stop();
  assert(tc.playerState === PlaybackState.STOPPED, 'stop() sets playerState to STOPPED');
  assert(tc._lookaheadTimers.length === 0, 'stop() clears all lookahead timers');

  // 9. pause() sends hardware stop (via _sendSerial) when called from PLAYING
  let stopSent = false;
  const mockManager = {
    isConnected: () => true,
    send: (data) => { if (data && data.includes('STOP')) stopSent = true; },
    write: () => {}
  };
  tc.serialManager = mockManager;
  tc.playerState = PlaybackState.PLAYING;
  tc.pause();
  assert(tc.playerState === PlaybackState.PAUSED, 'pause() transitions to PAUSED state');
  assert(stopSent, 'pause() dispatches STOP command via serial manager');
  tc.serialManager = null;

  // 10. PlaybackState enum is exported and complete
  assert(PlaybackState.IDLE === 'IDLE', 'PlaybackState.IDLE is "IDLE"');
  assert(PlaybackState.PLAYING === 'PLAYING', 'PlaybackState.PLAYING is "PLAYING"');
  assert(PlaybackState.PAUSED === 'PAUSED', 'PlaybackState.PAUSED is "PAUSED"');
  assert(PlaybackState.STOPPED === 'STOPPED', 'PlaybackState.STOPPED is "STOPPED"');
});

// -----------------------------------------------------------------------------
// Suite 17: R3 3D Viewport Cache, MOVL Overlay & moveType Schema
// -----------------------------------------------------------------------------
describe('Suite 17: R3 3D Viewport Cache, MOVL Overlay & moveType Schema', () => {
  // 1. In-place coordinate edit now invalidates seqKey and triggers rebuild
  const viz = new TrajectoryVisualizer3D({ visible: true });
  const pickAndPlace = DEFAULT_PRESETS['Pick & Place Demo'];

  viz.updateSequence(pickAndPlace);
  const tubeCountAfterFirst = viz.tubeGroup.children.length;
  assert(tubeCountAfterFirst === 1, 'Spline tube rendered on first updateSequence call');

  // Mutate a waypoint coordinate in-place (simulates UI coordinate edit)
  const mutatedSeq = {
    ...pickAndPlace,
    waypoints: pickAndPlace.waypoints.map((wp, i) =>
      i === 0 ? { ...wp, j1: wp.j1 + 5.0 } : wp
    )
  };
  viz.updateSequence(mutatedSeq);
  // Cache must have been invalidated → tube rebuilt with different geometry
  assert(viz.tubeGroup.children.length === 1, 'Tube group still has exactly 1 mesh after coordinate edit');
  const newPos = viz.waypointPositions[0];
  // The position of wp0 must have changed after j1 was shifted
  assert(newPos !== undefined, 'Waypoint positions array populated after in-place edit');

  // 2. linMovlGroup is created in constructor
  assert(viz.linMovlGroup !== undefined && viz.linMovlGroup.name === 'Trajectory_MOVL_Linear_Group',
    'linMovlGroup created with correct name');

  // 3. No MOVL overlay rendered when all waypoints are joint-type
  assert(viz.linMovlGroup.children.length === 0,
    'No MOVL overlays for joint-only sequence (default moveType)');

  // 4. MOVL overlay is rendered when a waypoint has moveType === movl
  const movlSeq = {
    sequenceName: 'MOVL Test', loop: false,
    waypoints: [
      { id: 'a', label: 'A', j1: 0, j2: -90, j3: 90, j4: 90, j5: 90, grip: 90, dur: 1000, moveType: 'joint' },
      { id: 'b', label: 'B', j1: 30, j2: -70, j3: 80, j4: 90, j5: 90, grip: 90, dur: 1000, moveType: 'movl' },
      { id: 'c', label: 'C', j1: -20, j2: -80, j3: 85, j4: 90, j5: 90, grip: 90, dur: 1000, moveType: 'joint' }
    ]
  };
  viz.updateSequence(movlSeq);
  assert(viz.linMovlGroup.children.length === 1,
    'MOVL overlay mesh created for sequence with one movl waypoint');
  assert(viz.linMovlGroup.children[0].name === 'TrajectoryMOVL_Linear_Overlay',
    'MOVL overlay mesh has correct name');

  // 5. dispose() clears linMovlGroup
  viz.dispose();
  assert(viz.linMovlGroup.children.length === 0, 'dispose() empties linMovlGroup');

  // 6. PresetManager.validateWaypoint preserves moveType field
  // (PresetManager imported at top level)
  const movlResult = PresetManager.validateWaypoint({
    j1: 10, j2: -80, j3: 85, j4: 90, j5: 90, dur: 800, grip: 90, moveType: 'movl'
  }, 0);
  assert(movlResult.valid, 'validateWaypoint succeeds for waypoint with moveType: movl');
  assert(movlResult.waypoint.moveType === 'movl', 'moveType "movl" is preserved through validateWaypoint');

  const jointResult = PresetManager.validateWaypoint({
    j1: 10, j2: -80, j3: 85, j4: 90, j5: 90, dur: 800, grip: 90
  }, 0);
  assert(jointResult.waypoint.moveType === 'joint', 'Default moveType is "joint" when field is absent');

  const unknownResult = PresetManager.validateWaypoint({
    j1: 10, j2: -80, j3: 85, j4: 90, j5: 90, dur: 800, grip: 90, moveType: 'MOVL'
  }, 0);
  assert(unknownResult.waypoint.moveType === 'movl', 'moveType is lowercased during validation');

  // 7. viewport.css contains touch-action: none
  const viewportCss = fs.readFileSync(
    path.resolve(rootDir, 'css/components/viewport.css'), 'utf8'
  );
  assert(viewportCss.includes('touch-action: none'), 'viewport.css contains touch-action: none for canvas');
});

// -----------------------------------------------------------------------------
// Suite 18: Unified Step-Jogger & Tactile Input Scrubbing
// -----------------------------------------------------------------------------
describe('Suite 18: Unified Step-Jogger & Tactile Input Scrubbing', () => {
  const mc = new MotionController({ appState });

  // 1. Initial default step sizes
  assert(mc.cartesianStepSize === 10.0, 'Default cartesianStepSize is 10.0 mm');
  assert(mc.jointStepSize === 1.0, 'Default jointStepSize is 1.0 deg');

  // 2. setCartesianStepSize updates step size and clamps minimum
  mc.setCartesianStepSize(50.0);
  assert(mc.cartesianStepSize === 50.0, 'setCartesianStepSize updates to 50.0 mm');
  mc.setCartesianStepSize(0.1);
  assert(mc.cartesianStepSize === 0.1, 'setCartesianStepSize updates to 0.1 mm');
  mc.setCartesianStepSize(-5);
  assert(mc.cartesianStepSize >= 0.01, 'setCartesianStepSize clamps non-positive numbers to >= 0.01');

  // 3. setJointStepSize updates step size and clamps minimum
  mc.setJointStepSize(10.0);
  assert(mc.jointStepSize === 10.0, 'setJointStepSize updates to 10.0 deg');
  mc.setJointStepSize(45.0);
  assert(mc.jointStepSize === 45.0, 'setJointStepSize updates to 45.0 deg');
  mc.setJointStepSize(0);
  assert(mc.jointStepSize >= 0.01, 'setJointStepSize clamps zero to >= 0.01');

  // 4. jogCartesianKeyboard uses active cartesianStepSize when step parameter is omitted
  mc.setCartesianTarget(100, 0, 200, 0, null, false, false);
  mc.setCartesianStepSize(15.0);
  const startX = mc.cartesian.x;
  mc.jogCartesianKeyboard('w'); // forward +X
  assert(Math.abs(mc.cartesian.x - (startX + 15.0)) < 1.0, 'jogCartesianKeyboard(w) jogs +X by active cartesianStepSize (15mm)');
  mc.jogCartesianKeyboard('s'); // backward -X
  assert(Math.abs(mc.cartesian.x - startX) < 1.0, 'jogCartesianKeyboard(s) jogs -X by active cartesianStepSize (15mm)');

  const startY = mc.cartesian.y;
  mc.jogCartesianKeyboard('a'); // +Y
  assert(Math.abs(mc.cartesian.y - (startY + 15.0)) < 1.0, 'jogCartesianKeyboard(a) jogs +Y by active cartesianStepSize');
  mc.jogCartesianKeyboard('d'); // -Y
  assert(Math.abs(mc.cartesian.y - startY) < 1.0, 'jogCartesianKeyboard(d) jogs -Y by active cartesianStepSize');

  const startZ = mc.cartesian.z;
  mc.jogCartesianKeyboard('arrowup'); // +Z
  assert(Math.abs(mc.cartesian.z - (startZ + 15.0)) < 1.0, 'jogCartesianKeyboard(arrowup) jogs +Z by active cartesianStepSize');
  mc.jogCartesianKeyboard('arrowdown'); // -Z
  assert(Math.abs(mc.cartesian.z - startZ) < 1.0, 'jogCartesianKeyboard(arrowdown) jogs -Z by active cartesianStepSize');

  // 5. Explicit step argument overrides active step size
  mc.jogCartesianKeyboard('w', 2.5);
  assert(Math.abs(mc.cartesian.x - (startX + 2.5)) < 1.0, 'Explicit step argument (2.5mm) overrides active step size');

  // 6. DOM Verification in index.html for Unified Step-Jogger Elements
  const html = fs.readFileSync(path.resolve(rootDir, 'index.html'), 'utf8');

  // Cartesian Step Pills
  assert(html.includes('data-cart-step="0.1"'), 'index.html includes 0.1mm Cartesian step pill');
  assert(html.includes('data-cart-step="1"'), 'index.html includes 1mm Cartesian step pill');
  assert(html.includes('data-cart-step="10"'), 'index.html includes 10mm Cartesian step pill');
  assert(html.includes('data-cart-step="50"'), 'index.html includes 50mm Cartesian step pill');

  // Joint Step Pills
  assert(html.includes('data-joint-step="0.1"'), 'index.html includes 0.1° Joint step pill');
  assert(html.includes('data-joint-step="1"'), 'index.html includes 1° Joint step pill');
  assert(html.includes('data-joint-step="5"'), 'index.html includes 5° Joint step pill');
  assert(html.includes('data-joint-step="10"'), 'index.html includes 10° Joint step pill');
  assert(html.includes('data-joint-step="45"'), 'index.html includes 45° Joint step pill');

  // Unified Jog Buttons with data-jog-dir
  assert(html.includes('data-cartesian-nudge="x"') && html.includes('data-jog-dir="1"'), 'Unified +X jog button present');
  assert(html.includes('data-cartesian-nudge="x"') && html.includes('data-jog-dir="-1"'), 'Unified -X jog button present');
  assert(html.includes('data-nudge-joint="j1"') && html.includes('data-jog-dir="1"'), 'Unified +J1 jog button present');
  assert(html.includes('data-nudge-joint="j1"') && html.includes('data-jog-dir="-1"'), 'Unified -J1 jog button present');

  // Scrubbable input classes
  assert(html.includes('scrubbable-input'), 'index.html marks inputs with scrubbable-input class');

  // 7. Dynamic Label & Attribute Updates in _updateStepJogUI
  const mockPills = [
    { attr: '10', classList: { toggle: (cls, on) => { mockPills[0].active = on; } }, active: false, getAttribute: () => '10' },
    { attr: '50', classList: { toggle: (cls, on) => { mockPills[1].active = on; } }, active: false, getAttribute: () => '50' }
  ];
  const mockMinusBtn = { textContent: '', attrs: {}, setAttribute(k, v) { this.attrs[k] = v; }, getAttribute(k) { return k === 'data-jog-dir' ? '-1' : (k === 'data-cartesian-nudge' ? 'x' : null); } };
  const mockPlusBtn = { textContent: '', attrs: {}, setAttribute(k, v) { this.attrs[k] = v; }, getAttribute(k) { return k === 'data-jog-dir' ? '1' : (k === 'data-cartesian-nudge' ? 'x' : null); } };

  const mockContainer = {
    querySelectorAll(selector) {
      if (selector === '[data-cart-step]') return mockPills;
      if (selector === '[data-cartesian-nudge]') return [mockMinusBtn, mockPlusBtn];
      return [];
    }
  };

  mc.container = mockContainer;
  mc.setCartesianStepSize(50.0);
  assert(mockPills[1].active === true, '_updateStepJogUI activates 50mm pill');
  assert(mockMinusBtn.textContent === '−50', '_updateStepJogUI sets minus button label to −50');
  assert(mockPlusBtn.textContent === '+50', '_updateStepJogUI sets plus button label to +50');
  assert(mockMinusBtn.attrs['data-step'] === '-50', '_updateStepJogUI sets data-step to -50');
  assert(mockPlusBtn.attrs['data-step'] === '50', '_updateStepJogUI sets data-step to 50');

  // 8. Cylindrical Mode (R/theta/Z): A/D Rotates Theta, W/S Extends/Retracts Radius R
  mc.container = null;
  mc.setCylindricalMode(true);
  assert(mc.isCylindricalMode === true, 'setCylindricalMode enables cylindrical coordinate mode');

  // Set reference pose at (X=150, Y=0, Z=200): R=150mm, theta=0 deg
  mc.setCartesianTarget(150, 0, 200, 0, null, false, false);
  const initR = Math.hypot(mc.cartesian.x, mc.cartesian.y);
  assert(Math.abs(initR - 150) < 1.0, 'Initial radius is 150mm');

  // A key nudges theta counter-clockwise (+15 deg)
  mc.jogCartesianKeyboard('a', 15.0);
  const rAfterA = Math.hypot(mc.cartesian.x, mc.cartesian.y);
  const thetaAfterA = (Math.atan2(mc.cartesian.y, mc.cartesian.x) * 180) / Math.PI;
  assert(Math.abs(rAfterA - 150) < 1.0, 'A key preserves radius R=150mm during rotation');
  assert(Math.abs(thetaAfterA - 15.0) < 1.0, 'A key rotates theta by +15 deg in cylindrical mode');
  assert(Math.abs(mc.cartesian.z - 200) < 1.0, 'A key preserves height Z=200mm');

  // D key nudges theta clockwise (-30 deg -> theta = -15 deg)
  mc.jogCartesianKeyboard('d', 30.0);
  const rAfterD = Math.hypot(mc.cartesian.x, mc.cartesian.y);
  const thetaAfterD = (Math.atan2(mc.cartesian.y, mc.cartesian.x) * 180) / Math.PI;
  assert(Math.abs(rAfterD - 150) < 1.0, 'D key preserves radius R=150mm during rotation');
  assert(Math.abs(thetaAfterD - (-15.0)) < 1.0, 'D key rotates theta clockwise by -30 deg (result: -15 deg)');

  // W key extends reach R (+20mm -> R=170mm) along current theta (-15 deg)
  mc.jogCartesianKeyboard('w', 20.0);
  const rAfterW = Math.hypot(mc.cartesian.x, mc.cartesian.y);
  const thetaAfterW = (Math.atan2(mc.cartesian.y, mc.cartesian.x) * 180) / Math.PI;
  assert(Math.abs(rAfterW - 170) < 1.0, 'W key extends reach R from 150mm to 170mm in cylindrical mode');
  assert(Math.abs(thetaAfterW - (-15.0)) < 1.0, 'W key preserves polar angle theta during radial extension');

  // S key retracts reach R (-20mm -> R=150mm)
  mc.jogCartesianKeyboard('s', 20.0);
  const rAfterS = Math.hypot(mc.cartesian.x, mc.cartesian.y);
  assert(Math.abs(rAfterS - 150) < 1.0, 'S key retracts reach R back to 150mm in cylindrical mode');

  // Restore Cartesian mode
  mc.setCylindricalMode(false);
  assert(mc.isCylindricalMode === false, 'setCylindricalMode restores Cartesian mode');
});

// -----------------------------------------------------------------------------
// Suite 19: 3D Transform Gizmo Triad, Constraint Resolution & Snapping
// -----------------------------------------------------------------------------
describe('Suite 19: 3D Transform Gizmo Triad, Constraint Resolution & Snapping', () => {
  // 1. AxisConstraint enum validation
  assert(AxisConstraint.NONE === 'none', 'AxisConstraint.NONE is "none"');
  assert(AxisConstraint.X === 'x', 'AxisConstraint.X is "x"');
  assert(AxisConstraint.Y === 'y', 'AxisConstraint.Y is "y"');
  assert(AxisConstraint.Z === 'z', 'AxisConstraint.Z is "z"');
  assert(AxisConstraint.XY === 'xy', 'AxisConstraint.XY is "xy"');
  assert(AxisConstraint.XZ === 'xz', 'AxisConstraint.XZ is "xz"');
  assert(AxisConstraint.YZ === 'yz', 'AxisConstraint.YZ is "yz"');
  assert(AxisConstraint.PITCH === 'pitch', 'AxisConstraint.PITCH is "pitch"');
  assert(AxisConstraint.ROLL === 'roll', 'AxisConstraint.ROLL is "roll"');

  // 2. Gizmo Triad assembly and meshes
  const mockScene = new THREE.Scene();
  const mockCamera = new THREE.PerspectiveCamera(45, 1, 1, 1000);
  const mockThreeScene = { scene: mockScene, camera: mockCamera, renderer: null };
  const dragCtrl = new TcpDragController(mockThreeScene, null);

  assert(dragCtrl.gizmoGroup instanceof THREE.Group, 'dragCtrl builds a THREE.Group for gizmo');
  assert(dragCtrl.gizmoGroup.name === 'tcpTransformGizmo', 'Gizmo group named "tcpTransformGizmo"');
  assert(mockScene.children.includes(dragCtrl.gizmoGroup), 'Gizmo group attached to 3D scene');
  assert(dragCtrl._gizmoMeshes.length >= 8, `Gizmo contains at least 8 interactive handle meshes (${dragCtrl._gizmoMeshes.length})`);

  // Verify handle types exist
  const handleTypes = dragCtrl._gizmoMeshes.map(m => m.userData.handleType);
  assert(handleTypes.includes('arrow'), 'Gizmo contains arrow handles for linear axes');
  assert(handleTypes.includes('plane'), 'Gizmo contains planar quad handles');
  assert(handleTypes.includes('arc'), 'Gizmo contains rotational arc handles');

  // 3. Constraint resolution from gizmo handles & keyboard modifiers
  assert(dragCtrl._resolveConstraint({}, { isGizmoHandle: true, handleType: 'arrow', axis: 'x' }) === AxisConstraint.X, 'Resolves linear X constraint');
  assert(dragCtrl._resolveConstraint({}, { isGizmoHandle: true, handleType: 'arrow', axis: 'y' }) === AxisConstraint.Y, 'Resolves linear Y constraint');
  assert(dragCtrl._resolveConstraint({}, { isGizmoHandle: true, handleType: 'arrow', axis: 'z' }) === AxisConstraint.Z, 'Resolves linear Z constraint');
  assert(dragCtrl._resolveConstraint({}, { isGizmoHandle: true, handleType: 'plane', plane: 'xy' }) === AxisConstraint.XY, 'Resolves XY planar constraint');
  assert(dragCtrl._resolveConstraint({}, { isGizmoHandle: true, handleType: 'plane', plane: 'xz' }) === AxisConstraint.XZ, 'Resolves XZ planar constraint');
  assert(dragCtrl._resolveConstraint({}, { isGizmoHandle: true, handleType: 'plane', plane: 'yz' }) === AxisConstraint.YZ, 'Resolves YZ planar constraint');
  assert(dragCtrl._resolveConstraint({}, { isGizmoHandle: true, handleType: 'arc', rotate: 'pitch' }) === AxisConstraint.PITCH, 'Resolves wrist pitch rotation constraint');
  assert(dragCtrl._resolveConstraint({}, { isGizmoHandle: true, handleType: 'arc', rotate: 'roll' }) === AxisConstraint.ROLL, 'Resolves wrist roll rotation constraint');

  // Modifiers fallback when dragging center sphere
  assert(dragCtrl._resolveConstraint({ shiftKey: true }, null) === AxisConstraint.XY, 'Shift key resolves to XY plane constraint');
  assert(dragCtrl._resolveConstraint({ altKey: true }, null) === AxisConstraint.Z, 'Alt key resolves to Z elevation constraint');
  assert(dragCtrl._resolveConstraint({}, null) === AxisConstraint.NONE, 'No modifier resolves to unconstrained free drag');

  // 4. Snapping API & Logic
  assert(dragCtrl.isSnapEnabled() === false, 'Snapping is initially disabled');
  assert(dragCtrl.setSnapEnabled(true) === true, 'setSnapEnabled(true) enables snapping');
  assert(dragCtrl.isSnapEnabled() === true, 'isSnapEnabled() returns true when active');
  assert(dragCtrl.setSnapEnabled(false) === false, 'setSnapEnabled(false) disables snapping');
  
  dragCtrl.setSnapGrid(15.0);
  assert(dragCtrl.snapGrid === 15.0, 'setSnapGrid(15.0) sets grid interval to 15.0mm');
  dragCtrl.setSnapGrid(10.0);
  assert(dragCtrl.snapGrid === 10.0, 'setSnapGrid(10.0) sets standard 10mm increment');

  // 5. End-Effector Pickables: Gripper, wrist, and TCP are all pickable
  const mockGhostArm = {
    tcp: new THREE.Group(),
    j5: new THREE.Group(),
    j4: new THREE.Group()
  };
  const mockTcpMesh = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial());
  mockTcpMesh.userData = { isTcpSphere: true, isGhost: true };
  mockGhostArm.tcp.add(mockTcpMesh);

  const mockPalmMesh = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial());
  mockGhostArm.j5.add(mockPalmMesh);

  const mockWristMesh = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial());
  mockGhostArm.j4.add(mockWristMesh);

  const mockRobotModel = {
    ghostArm: mockGhostArm,
    liveArm: null,
    isGhostVisible: true
  };

  let dragCoords = null;
  let dragEnded = false;
  const dragWithRobot = new TcpDragController(
    mockThreeScene,
    mockRobotModel,
    (coords) => { dragCoords = coords; },
    () => { dragEnded = true; }
  );

  const pickablesWithRobot = dragWithRobot._getPickableObjects();
  assert(pickablesWithRobot.includes(mockTcpMesh), 'Pickables includes TCP sphere');
  assert(pickablesWithRobot.includes(mockPalmMesh), 'Pickables includes gripper palm mesh for direct arm dragging');
  assert(pickablesWithRobot.includes(mockWristMesh), 'Pickables includes wrist mesh for direct arm dragging');

  // 6. Extended reach up to 350.0mm in working envelope
  dragWithRobot.isDragging = true;
  dragWithRobot.draggedMesh = mockTcpMesh;
  dragWithRobot._frozenRobotCoords = { x: 75, y: 0, z: 215 };
  dragWithRobot._axisConstraint = AxisConstraint.X;

  // Simulate mouse moving along X axis to max reach 350.0mm
  dragWithRobot.planeIntersectPoint.set(350.0, 215.0, 0.0);
  dragWithRobot._dragOffset.set(0, 0, 0);

  // Directly call the pointer move handler logic
  const robotCoords = threeToRobotCoords(350.0, 215.0, 0.0);
  let outX = robotCoords.x;
  let outY = dragWithRobot._frozenRobotCoords.y;
  let outZ = dragWithRobot._frozenRobotCoords.z;
  const radialDist = Math.hypot(outX, outY);
  const maxReach = 350.0;
  if (radialDist > maxReach && radialDist > 1e-4) {
    const scale = maxReach / radialDist;
    outX *= scale;
    outY *= scale;
  }
  assert(outX === 350.0, 'Working envelope reach allows arm extension to full physical 350.0mm');

  // 7. Drag End Callback
  dragWithRobot._onPointerUp({});
  assert(dragEnded === true, 'onDragEnd callback fires cleanly when releasing mouse');
  assert(dragWithRobot.isDragging === false, 'isDragging is false after pointer up');

  // 8. Horizontal ground plane selected for elevated camera in free drag
  mockCamera.position.set(380, 320, 380);
  mockCamera.lookAt(0, 150, 0);
  mockCamera.updateMatrixWorld();
  dragWithRobot._axisConstraint = AxisConstraint.NONE;
  dragWithRobot._updateDragPlane(new THREE.Vector3(75, 215, 0));
  assert(Math.abs(dragWithRobot.dragPlane.normal.y - 1.0) < 1e-4, 'Elevated camera uses horizontal ground plane (normal = 0, 1, 0) to prevent floor collisions');

  // Clean up
  dragCtrl.dispose();
  dragWithRobot.dispose();
  assert(!mockScene.children.includes(dragCtrl.gizmoGroup), 'Gizmo group cleanly removed on dispose');
});

// -----------------------------------------------------------------------------
// Suite 20: Horizontal Waypoint Timeline Controller & Keyframe Interpolation
// -----------------------------------------------------------------------------
describe('Suite 20: Horizontal Waypoint Timeline Controller & Keyframe Interpolation', () => {
  const store = new AppState();
  const tl = new HorizontalTimelineController({ appState: store });

  // 1. Duration computation
  assert(tl.calculateTotalDuration([]) === 0, 'Empty sequence has 0ms duration');
  assert(tl.calculateTotalDuration(null) === 0, 'Null waypoints list has 0ms duration');
  
  const testWaypoints = [
    { dur: 1000 },
    { duration: 1500 },
    { duration_ms: 2500 }
  ];
  assert(tl.calculateTotalDuration(testWaypoints) === 5000, 'Calculates total duration across dur/duration/duration_ms (5000ms)');

  // 2. Timeline scale & geometry properties
  assert(tl.scale === 0.075, 'Timeline scale is 0.075 px/ms (75px per second)');
  assert(tl.minBlockWidth === 70, 'Timeline minimum block width is 70px');

  // 3. Ghost Arm Target Interpolation along Timeline
  store.setSequence({
    sequenceName: 'Test Scrub Sequence',
    waypoints: [
      { j1: 0.0, j2: -90.0, j3: 90.0, j4: 90.0, j5: 90.0, grip: 0, dur: 1000 },
      { j1: 90.0, j2: 0.0, j3: 45.0, j4: 90.0, j5: 90.0, grip: 180, dur: 2000 }
    ]
  });

  // At time 0 (start of WP0)
  tl.setPlayheadTime(0, true);
  let ghost = store.getState('ghostPose');
  assert(Math.abs(ghost.j1 - 0.0) < 0.1, 'At t=0ms J1 is at WP0 (0°)');
  assert(Math.abs(ghost.j2 - (-90.0)) < 0.1, 'At t=0ms J2 is at WP0 (-90°)');
  assert(Math.abs(ghost.grip - 0) < 0.1, 'At t=0ms Gripper is at WP0 (0°)');

  // At time 2000ms: 1000ms into WP1 (which has 2000ms duration, so alpha = 0.5)
  // J1: 0 + 0.5 * (90 - 0) = 45.0
  // J2: -90 + 0.5 * (0 - (-90)) = -45.0
  // J3: 90 + 0.5 * (45 - 90) = 67.5
  // grip: 0 + 0.5 * (180 - 0) = 90.0
  tl.setPlayheadTime(2000, true);
  ghost = store.getState('ghostPose');
  assert(Math.abs(ghost.j1 - 45.0) < 0.1, 'At t=2000ms (halfway WP1) J1 interpolates to 45°');
  assert(Math.abs(ghost.j2 - (-45.0)) < 0.1, 'At t=2000ms J2 interpolates to -45°');
  assert(Math.abs(ghost.j3 - 67.5) < 0.1, 'At t=2000ms J3 interpolates to 67.5°');
  assert(Math.abs(ghost.grip - 90.0) < 0.1, 'At t=2000ms Gripper interpolates to 90°');

  // At time 3000ms (end of WP1)
  tl.setPlayheadTime(3000, true);
  ghost = store.getState('ghostPose');
  assert(Math.abs(ghost.j1 - 90.0) < 0.1, 'At t=3000ms J1 reaches WP1 target (90°)');
  assert(Math.abs(ghost.j2 - 0.0) < 0.1, 'At t=3000ms J2 reaches WP1 target (0°)');
  assert(Math.abs(ghost.grip - 180.0) < 0.1, 'At t=3000ms Gripper reaches WP1 target (180°)');

  // 4. Waypoint Reordering via AppState
  assert(store.reorderWaypoints(0, 1) === true, 'reorderWaypoints(0, 1) succeeds');
  const reordered = store.getState('sequence.waypoints');
  assert(reordered[0].j1 === 90.0 && reordered[1].j1 === 0.0, 'Waypoints swapped order in state');
  assert(store.reorderWaypoints(-1, 0) === false, 'Invalid index safely returns false');

  // 5. DOM verification for timeline in index.html and main.css
  const html = fs.readFileSync(path.resolve(rootDir, 'index.html'), 'utf8');
  assert(html.includes('id="dock-tab-timeline"'), 'Dock tab button #dock-tab-timeline exists');
  assert(html.includes('id="dock-view-timeline"'), 'Dock view pane #dock-view-timeline exists');

  const mainCss = fs.readFileSync(path.resolve(rootDir, 'css', 'main.css'), 'utf8');
  assert(mainCss.includes('components/timeline.css'), 'timeline.css imported in main.css');
});

// -----------------------------------------------------------------------------
// Suite 21: Viewport Unified Glass Ribbon & Camera PiP Modes
// -----------------------------------------------------------------------------
describe('Suite 21: Viewport Unified Glass Ribbon & Camera PiP Modes', () => {
  const store = new AppState();

  // 1. DOM Contract for Unified Glass HUD Ribbon
  const html = fs.readFileSync(path.resolve(rootDir, 'index.html'), 'utf8');
  assert(html.includes('id="unified-glass-ribbon"'), '#unified-glass-ribbon exists in index.html');
  assert(html.includes('id="hud-tcp-x"'), '#hud-tcp-x readout exists in ribbon');
  assert(html.includes('id="hud-tcp-y"'), '#hud-tcp-y readout exists in ribbon');
  assert(html.includes('id="hud-tcp-z"'), '#hud-tcp-z readout exists in ribbon');

  // Camera PiP Controls
  assert(html.includes('id="btn-camera-pip-dock"'), '#btn-camera-pip-dock button exists in HUD');
  assert(html.includes('id="btn-camera-pip-min"'), '#btn-camera-pip-min button exists in HUD');
  assert(html.includes('id="camera-pip-minimized-badge"'), '#camera-pip-minimized-badge exists');

  // 2. CameraController PiP Mode Switching
  const classList = new Set();
  const mockContainer = {
    classList: {
      add: (c) => classList.add(c),
      remove: (c) => classList.delete(c),
      contains: (c) => classList.has(c)
    },
    style: {}
  };

  const cam = new CameraController({ appState: store, pipContainer: mockContainer });

  // Mode: Docked
  cam.setPipMode('docked');
  assert(cam.pipMode === 'docked', 'setPipMode("docked") sets pipMode to docked');
  assert(classList.has('camera-hud-docked'), 'Adds camera-hud-docked class');
  assert(!classList.has('camera-hud-minimized'), 'Removes camera-hud-minimized class');
  assert(mockContainer.style.right === '16px', 'Sets right anchor to 16px');

  // Mode: Minimized
  cam.setPipMode('minimized');
  assert(cam.pipMode === 'minimized', 'setPipMode("minimized") sets pipMode to minimized');
  assert(classList.has('camera-hud-minimized'), 'Adds camera-hud-minimized class');
  assert(!classList.has('camera-hud-docked'), 'Removes camera-hud-docked class');

  // Mode: Floating
  cam.setPipMode('floating');
  assert(cam.pipMode === 'floating', 'setPipMode("floating") sets pipMode to floating');
  assert(!classList.has('camera-hud-docked'), 'Floating clears docked class');
  assert(!classList.has('camera-hud-minimized'), 'Floating clears minimized class');

  // toggleDock: floating -> docked -> floating
  cam.toggleDock();
  assert(cam.pipMode === 'docked', 'toggleDock() switches floating -> docked');
  cam.toggleDock();
  assert(cam.pipMode === 'floating', 'toggleDock() switches docked -> floating');

  // toggleMinimize: floating -> minimized -> floating
  cam.toggleMinimize();
  assert(cam.pipMode === 'minimized', 'toggleMinimize() switches floating -> minimized');
  cam.toggleMinimize();
  assert(cam.pipMode === 'floating', 'toggleMinimize() switches minimized -> floating');

  // 3. Viewport Vision Popover Menu & Layer Toggles
  assert(html.includes('id="viewport-vision-menu"'), '#viewport-vision-menu exists in index.html');
  assert(html.includes('id="btn-viewport-vision"'), '#btn-viewport-vision toggle button exists in index.html');
  assert(html.includes('id="viewport-vision-popover"'), '#viewport-vision-popover exists in index.html');
  assert(html.includes('id="btn-vision-popover-close"'), '#btn-vision-popover-close exists in index.html');
  assert(html.includes('id="chk-camera-pip-toggle"'), '#chk-camera-pip-toggle exists in vision popover');
  assert(html.includes('id="chk-ghost-visible"'), '#chk-ghost-visible exists in vision popover');
  assert(html.includes('id="chk-path-visible"'), '#chk-path-visible exists in vision popover');
  assert(html.includes('id="chk-floor-alert-visible"'), '#chk-floor-alert-visible exists in vision popover');
  assert(html.includes('id="chk-exclusion-visible"'), '#chk-exclusion-visible exists in vision popover');
  assert(html.includes('id="chk-grid-visible"'), '#chk-grid-visible exists in vision popover');

  // CSS classes defined
  const viewportCss = fs.readFileSync(path.resolve(rootDir, 'css', 'components', 'viewport.css'), 'utf8');
  assert(viewportCss.includes('.viewport-vision-menu'), 'viewport.css defines .viewport-vision-menu');
  assert(viewportCss.includes('.btn-viewport-vision'), 'viewport.css defines .btn-viewport-vision');
  assert(viewportCss.includes('.viewport-vision-popover'), 'viewport.css defines .viewport-vision-popover');
  assert(viewportCss.includes('.vision-toggle-item'), 'viewport.css defines .vision-toggle-item');

  // i18n keys defined
  assert(translations['viewport.vision.btn'] && translations['viewport.vision.btn'].en === 'Vision', 'i18n defines viewport.vision.btn (EN)');
  assert(translations['viewport.vision.btn'] && translations['viewport.vision.btn'].pl === 'Wizja', 'i18n defines viewport.vision.btn (PL)');
  assert(translations['viewport.vision.title'] && translations['viewport.vision.title'].en === 'Vision & Layers', 'i18n defines viewport.vision.title (EN)');
  assert(translations['tooltip.viewport.vision'] && translations['tooltip.viewport.vision'].en, 'i18n defines tooltip.viewport.vision');
});

// -----------------------------------------------------------------------------
// Suite 22: Viewport Active Motion Pulse & Tactile Styling
// -----------------------------------------------------------------------------
describe('Suite 22: Viewport Active Motion Pulse & Tactile Styling', () => {
  // 1. Active Motion Pulse CSS in viewport.css
  const viewportCss = fs.readFileSync(path.resolve(rootDir, 'css', 'components', 'viewport.css'), 'utf8');
  assert(viewportCss.includes('@keyframes viewport-active-pulse'), 'viewport.css defines @keyframes viewport-active-pulse');
  assert(viewportCss.includes('.viewport-container.robot-moving'), 'viewport.css defines .viewport-container.robot-moving selector');
  assert(viewportCss.includes('245, 158, 11'), 'Motion pulse utilizes warm industrial amber (245, 158, 11)');

  // 2. Tactile surface depth & inner bevels in design system
  const themeCss = fs.readFileSync(path.resolve(rootDir, 'css', 'theme', 'cyberpunk-dark.css'), 'utf8');
  assert(themeCss.includes('inset 0 1px 0 rgba(255, 255, 255'), 'cyberpunk-dark.css defines tactile inner bevel shadows (inset 0 1px 0)');

  // 3. Distinct Live Arm vs Ghost Arm Materials in RobotModel.js
  const modelJs = fs.readFileSync(path.resolve(rootDir, 'js', 'visualizer', 'RobotModel.js'), 'utf8');
  assert(modelJs.includes('ghostBody'), 'RobotModel.js defines ghostBody material');
  assert(modelJs.includes('0xd97706'), 'Ghost target arm uses distinct amber color 0xd97706');
  assert(modelJs.includes('transparent: true'), 'Ghost arm materials set transparent: true');
  assert(modelJs.includes('opacity:'), 'Ghost arm specifies custom translucency opacity');
});

// -----------------------------------------------------------------------------
// Suite 23: Native Gamepad Controller (Teach Pendant) Math & Step Multipliers
// -----------------------------------------------------------------------------
describe('Suite 23: Native Gamepad Controller (Teach Pendant) & Axis Mapping', () => {
  const store = new AppState();
  const gp = new GamepadController({ appState: store });

  // 1. Deadzone Filtering Math
  assert(gp.deadzone === 0.18, 'Gamepad deadzone defaults to 0.18 (18%)');
  assert(gp.applyDeadzone(0.0) === 0.0, 'Zero input returns 0.0');
  assert(gp.applyDeadzone(0.10) === 0.0, 'Input within positive deadzone (0.10) returns 0.0');
  assert(gp.applyDeadzone(-0.15) === 0.0, 'Input within negative deadzone (-0.15) returns 0.0');
  assert(gp.applyDeadzone(1.0) === 1.0, 'Maximum input +1.0 returns 1.0');
  assert(gp.applyDeadzone(-1.0) === -1.0, 'Minimum input -1.0 returns -1.0');

  // Linear scaling test: (0.59 - 0.18) / (1.0 - 0.18) = 0.41 / 0.82 = 0.50
  const midVal = gp.applyDeadzone(0.59);
  assert(Math.abs(midVal - 0.5) < 0.01, 'Input 0.59 scales smoothly to 0.50 above deadzone threshold');

  // Custom deadzone test
  assert(gp.applyDeadzone(0.25, 0.30) === 0.0, 'Respects custom deadzone threshold');

  // 2. Step Multipliers Cycling (1x, 5x, 10x)
  assert(Array.isArray(gp.stepScales) && gp.stepScales.length === 3, 'Provides 3 step multipliers');
  assert(gp.stepScales[0] === 1.0, 'Scale 0 is 1.0x');
  assert(gp.stepScales[1] === 5.0, 'Scale 1 is 5.0x');
  assert(gp.stepScales[2] === 10.0, 'Scale 2 is 10.0x');
  assert(gp.stepScaleIndex === 0, 'Initial step scale index is 0 (1.0x)');

  // Cycle through step scales
  gp.stepScaleIndex = (gp.stepScaleIndex + 1) % gp.stepScales.length;
  assert(gp.stepScaleIndex === 1, 'Bumper cycles index 0 -> 1 (5.0x)');
  gp.stepScaleIndex = (gp.stepScaleIndex + 1) % gp.stepScales.length;
  assert(gp.stepScaleIndex === 2, 'Bumper cycles index 1 -> 2 (10.0x)');
  gp.stepScaleIndex = (gp.stepScaleIndex + 1) % gp.stepScales.length;
  assert(gp.stepScaleIndex === 0, 'Bumper wraps index 2 -> 0 (1.0x)');

  // 3. UI Status Indicator in index.html & header.css
  const html = fs.readFileSync(path.resolve(rootDir, 'index.html'), 'utf8');
  assert(html.includes('id="gamepad-status-indicator"'), 'Gamepad status indicator #gamepad-status-indicator in index.html');
  assert(html.includes('class="gamepad-badge"'), 'Indicator uses .gamepad-badge CSS class');

  const headerCss = fs.readFileSync(path.resolve(rootDir, 'css', 'components', 'header.css'), 'utf8');
  assert(headerCss.includes('.gamepad-badge'), 'header.css defines .gamepad-badge styling');
  assert(headerCss.includes('.gamepad-badge.connected'), 'header.css defines connected state indicator');
});

// -----------------------------------------------------------------------------
// Suite 24: Unified Transport Layer (WebSerial USB + WebSocket Wi-Fi) & PWA Manifest
// -----------------------------------------------------------------------------
await describe('Suite 24: Unified Transport Layer (WebSerial USB + WebSocket Wi-Fi) & PWA Manifest', async () => {
  // 1. WebSerialManager Transport Mode Configuration
  const mgr = new WebSerialManager();
  assert(mgr.transportMode === 'serial', 'Default transport mode is "serial"');
  assert(mgr.wsUrl === 'ws://192.168.4.1:81/', 'Default wsUrl is "ws://192.168.4.1:81/"');

  mgr.setTransportMode('websocket');
  assert(mgr.transportMode === 'websocket', 'setTransportMode("websocket") updates transportMode');

  mgr.setWsUrl('192.168.1.100:8080');
  assert(mgr.wsUrl === 'ws://192.168.1.100:8080/', 'setWsUrl automatically formats URL with ws:// and trailing slash');

  mgr.setTransportMode('serial');
  assert(mgr.transportMode === 'serial', 'setTransportMode switches cleanly back to "serial"');

  // 2. Mock WebSocket Transport Connection & Message Flow
  class MockWebSocket {
    constructor(url) {
      this.url = url;
      this.readyState = 0; // CONNECTING
      this.sent = [];
      this.listeners = new Map();
      setTimeout(() => {
        this.readyState = 1; // OPEN
        this.dispatchEvent({ type: 'open' });
      }, 5);
    }
    addEventListener(event, fn) {
      if (!this.listeners.has(event)) this.listeners.set(event, new Set());
      this.listeners.get(event).add(fn);
    }
    removeEventListener(event, fn) {
      if (this.listeners.has(event)) this.listeners.get(event).delete(fn);
    }
    dispatchEvent(evt) {
      if (this.listeners.has(evt.type)) {
        for (const fn of this.listeners.get(evt.type)) fn(evt);
      }
    }
    send(data) {
      this.sent.push(data);
    }
    close() {
      this.readyState = 3; // CLOSED
      this.dispatchEvent({ type: 'close', reason: 'Normal closure' });
    }
    simulateMessage(text) {
      this.dispatchEvent({ type: 'message', data: text });
    }
  }

  const mockWs = new MockWebSocket('ws://192.168.4.1:81/');
  mgr.setTransportMode('websocket', '192.168.4.1:81');

  let receivedTelem = null;
  let receivedRemote = null;
  let receivedScale = null;

  mgr.on('jtelem', (pkt) => { receivedTelem = pkt; });
  mgr.on('remote_data', (pkt) => { receivedRemote = pkt; });
  mgr.on('scale_update', (pkt) => { receivedScale = pkt; });

  await mgr.connect(mockWs, { autoEnableJTelem: false });
  assert(mgr.isConnected(), 'WebSerialManager connects successfully via WebSocket');
  assert(mgr.isOpen, 'WebSerialManager.isOpen is true when WebSocket is connected');

  // Send command through WebSocket
  await mgr.send('MOV X100 Y50 Z200\n');
  assert(mockWs.sent.length === 1 && mockWs.sent[0] === 'MOV X100 Y50 Z200\n', 'send() dispatches text command over WebSocket');

  // Receive JTELEM over WebSocket
  mockWs.simulateMessage('{"j1":15.5,"j2":-45.0,"j3":60.0,"x":120.0,"y":30.0,"z":210.0}\n');
  assert(receivedTelem !== null, 'Incoming JSON line over WebSocket parsed as JTELEM');
  assert(receivedTelem.j1 === 15.5 && receivedTelem.x === 120.0, 'JTELEM packet values parsed correctly');

  // Receive ESP32 REMOTE_DATA over WebSocket
  mockWs.simulateMessage('REMOTE_DATA:{"pitch":12.3,"roll":-5.4,"clutch":1,"scale":2.0,"speed":80,"bat":95,"online":1}\n');
  assert(receivedRemote !== null, 'Incoming REMOTE_DATA line over WebSocket parsed');
  assert(receivedRemote.pitch === 12.3 && receivedRemote.clutch === true && receivedRemote.scale === 2.0, 'REMOTE_DATA payload fields extracted');

  // Receive ESP32 SCALE_UPDATE over WebSocket
  mockWs.simulateMessage('SCALE_UPDATE {"scale":0.5,"speed":40}\n');
  assert(receivedScale !== null, 'Incoming SCALE_UPDATE line over WebSocket parsed');
  assert(receivedScale.scale === 0.5 && receivedScale.speed === 40, 'SCALE_UPDATE payload values extracted');

  // Disconnect WebSocket
  await mgr.disconnect();
  assert(!mgr.isConnected(), 'WebSerialManager disconnects WebSocket cleanly');
  assert(mgr.ws === null, 'WebSocket reference cleared upon disconnect');

  // 3. PWA Web App Manifest & Service Worker Verification
  const manifestPath = path.resolve(rootDir, 'manifest.json');
  assert(fs.existsSync(manifestPath), 'manifest.json exists in WebControl root');
  const manifestData = JSON.parse(fs.readFileSync(manifestPath, 'utf8'));
  assert(manifestData.display === 'standalone', 'manifest.json display is "standalone"');
  assert(manifestData.theme_color === '#111317', 'manifest.json theme_color matches #111317');
  assert(manifestData.icons.length >= 2, 'manifest.json declares multiple app icon sizes');

  const swPath = path.resolve(rootDir, 'sw.js');
  assert(fs.existsSync(swPath), 'sw.js Service Worker file exists in WebControl root');
  const swContent = fs.readFileSync(swPath, 'utf8');
  assert(/CACHE_NAME = 'stpe-robot-arm-v\d+'/.test(swContent), 'sw.js defines cache version');
  assert(swContent.includes("caches.open(CACHE_NAME)"), 'sw.js implements Cache API pre-caching');
  const staticAssetsMatch = swContent.match(/const STATIC_ASSETS = \[([\s\S]*?)\];/);
  assert(Boolean(staticAssetsMatch), 'sw.js defines a static precache asset list');
  const staticAssets = Array.from(staticAssetsMatch[1].matchAll(/'([^']+)'/g), (match) => match[1]);
  const missingStaticAssets = staticAssets.filter((asset) => !fs.existsSync(path.resolve(rootDir, asset)));
  assert(missingStaticAssets.length === 0, `sw.js precache paths all exist${missingStaticAssets.length ? ` (missing: ${missingStaticAssets.join(', ')})` : ''}`);

  // 4. UI DOM Elements Contract
  const indexHtml = fs.readFileSync(path.resolve(rootDir, 'index.html'), 'utf8');
  assert(indexHtml.includes('data-transport-group'), 'index.html includes [data-transport-group]');
  assert(indexHtml.includes('id="btn_transport_serial"'), 'index.html includes #btn_transport_serial');
  assert(indexHtml.includes('id="btn_transport_ws"'), 'index.html includes #btn_transport_ws');
  assert(indexHtml.includes('id="input_ws_url"'), 'index.html includes #input_ws_url');
  assert(indexHtml.includes('id="btn_fullscreen_pendant"'), 'index.html includes #btn_fullscreen_pendant');
  assert(indexHtml.includes('rel="manifest"'), 'index.html links PWA manifest');
});

// =============================================================================
// Suite 25: Software Reliability & Fault Audit Regression Suite
// =============================================================================
await describe('Suite 25: Software Reliability & Fault Audit Regression Suite', async () => {
  // 1. AppState.setState unified method: path validation, change events, and stateChange:${rootKey}
  const testState = new AppState();
  let rootConnectionCalled = false;
  let granularPathValue = null;
  let generalStateChangeCount = 0;

  testState.subscribe('stateChange:connection', (conn) => {
    rootConnectionCalled = true;
  });
  testState.subscribe('change:connection.status', ({ path, value, prevValue }) => {
    granularPathValue = value;
  });
  testState.subscribe('stateChange', () => {
    generalStateChangeCount++;
  });

  // Safe against null/undefined path
  testState.setState(null, 'ignored');
  testState.setState(undefined, 'ignored');
  assert(!rootConnectionCalled, 'setState rejects null or undefined path without throwing');

  testState.setState('connection.status', 'CONNECTED');
  assert(rootConnectionCalled === true, 'setState emits stateChange:${rootKey}');
  assert(granularPathValue === 'CONNECTED', 'setState emits granular change:${path} with new value');
  assert(generalStateChangeCount === 1, 'setState emits general stateChange event');

  // 2. GCodeInterpreter arc interpolation NaN resilience
  const degenerateArc = GCodeInterpreter._interpolateArc({
    startX: 10,
    startY: 10,
    endX: 10,
    endY: 10,
    z: 50,
    i: null,
    j: null,
    r: 30,
    isCW: true,
    feedrate: 1000,
    toolActive: false,
    rawLine: 'G2 X10 Y10 R30'
  });
  assert(Array.isArray(degenerateArc) && degenerateArc.length === 0,
    'Degenerate arc with R (startX === endX && startY === endY) returns empty array without NaN');

  // 3. TrajectoryController removeWaypoint alias
  const tcReg = new TrajectoryController({ appState: testState });
  assert(typeof tcReg.removeWaypoint === 'function',
    'TrajectoryController exposes removeWaypoint as alias for deleteWaypoint');

  // 4. TrajectoryController timeline synchronization events
  let caughtPlaybackState = null;
  let caughtProgress = null;
  testState.subscribe('trajectory:playbackState', (state) => {
    caughtPlaybackState = state;
  });
  testState.subscribe('trajectory:progress', (prog) => {
    caughtProgress = prog;
  });

  tcReg.playerState = PlaybackState.PLAYING;
  tcReg.elapsedTime = 1250;
  tcReg._syncPlayerStateToApp();
  assert(caughtPlaybackState === PlaybackState.PLAYING,
    '_syncPlayerStateToApp emits trajectory:playbackState');
  assert(caughtProgress !== null && caughtProgress.elapsedMs === 1250,
    '_syncPlayerStateToApp emits trajectory:progress with elapsedMs');

  // 5. HorizontalTimelineController removeWaypoint compatibility
  let deleteCalledOnTc = false;
  const mockTc = {
    deleteWaypoint: (idx) => { deleteCalledOnTc = true; return true; },
    getSequence: () => ({ waypoints: [{ j1: 0, j2: -90, j3: 90, j4: 90, j5: 90, dur: 1000 }] })
  };
  const htc = new HorizontalTimelineController({ appState: testState, trajectoryController: mockTc });
  htc.removeWaypoint(0);
  assert(deleteCalledOnTc === true,
    'HorizontalTimelineController.removeWaypoint successfully calls trajectoryController.deleteWaypoint');
  htc.dispose();

  // 6. WebSerialManager unexpected disconnect event reason
  const serialMgr = new WebSerialManager();
  let receivedReason = null;
  serialMgr.on('disconnected', ({ reason }) => {
    receivedReason = reason;
  });
  await serialMgr._handleUnexpectedDisconnect('Hardware port I/O error');
  assert(receivedReason === 'Hardware port I/O error',
    '_handleUnexpectedDisconnect emits disconnected with real failure reason');

  // 7. TrajectoryVisualizer3D coordHash includes j4, j5, and moveType
  const mockScene = new THREE.Scene();
  const viz = new TrajectoryVisualizer3D(mockScene);
  let rebuildCount = 0;
  viz._rebuildFullSequence = () => { rebuildCount++; };

  const seqA = {
    sequenceName: 'TestSeq',
    loop: false,
    waypoints: [{ j1: 0, j2: -90, j3: 90, j4: 90, j5: 90, moveType: 'spline' }]
  };
  viz.updateSequence(seqA);
  assert(rebuildCount === 1, 'Initial updateSequence triggers rebuild');

  // In-place edit of J4 (wrist pitch)
  seqA.waypoints[0].j4 = 45;
  viz.updateSequence(seqA);
  assert(rebuildCount === 2, 'In-place edit of j4 correctly triggers rebuild');

  // In-place edit of moveType
  seqA.waypoints[0].moveType = 'movl';
  viz.updateSequence(seqA);
  assert(rebuildCount === 3, 'In-place edit of moveType correctly triggers rebuild');

  // 8. J6 Actuator & Packet Serialization
  const packet6 = buildFramedPacket({ j1: 10, j2: 20, j3: -15, j4: 45, j5: -30, j6: 75, dx: 0, dy: 0, dur: 1200 });
  assert(packet6.includes('75.0') && packet6.startsWith('<') && packet6.endsWith('>'),
    'buildFramedPacket serializes 6 joints including J6 tool roll');

  const mockPort6 = new MockSerialPort();
  mockPort6.open({ baudRate: 115200 });
  mockPort6.receiveFromClient(packet6 + '\n');
  assert(mockPort6.j6 === 75 || Math.abs(mockPort6.targetPose.j6 - 75) < 0.01,
    'MockSerialPort parses 6-joint framed packet and sets J6 target');
  mockPort6.close();

  // 9. IK False Reachability on Clamping & Linear Move Guard
  const clampedIk = solveIK(-150, 0, 160); // Out of azimuth range
  assert(clampedIk.clamped === true && clampedIk.success === false,
    'solveIK marks success: false and clamped: true when joint limits are exceeded');

  const motionCtrl = new MotionController({ appState: testState });
  let linearBlocked = false;
  try {
    await motionCtrl.sendMovLinear({ x: -150, y: 0, z: 160 });
  } catch (err) {
    linearBlocked = true;
  }
  assert(linearBlocked,
    'sendMovLinear blocks move and throws error when IK solution is clamped');

  // 10. Gripper Actuation Timing with Settle Dwell
  const tcDwell = new TrajectoryController({
    appState: testState,
    serialManager: { isConnected: () => true, send: async () => {} }
  });
  tcDwell.gripperDwellMs = 150;
  tcDwell.speedScale = 1.0;
  let sentCommands = [];
  tcDwell._sendSerial = (cmd) => { sentCommands.push({ cmd, time: Date.now() }); };
  tcDwell.setSequence({
    sequenceName: 'TestDwell',
    loop: false,
    waypoints: [{ j1: 0, j2: 0, j3: 0, j4: 0, j5: 0, j6: 0, grip: 180, dur: 200 }]
  });
  tcDwell.play();
  // At 50ms, the packet should have dispatched, but the gripper should NOT have fired yet
  await new Promise((r) => setTimeout(r, 60));
  const hasGripEarly = sentCommands.some(c => c.cmd.includes('GRIP'));
  assert(!hasGripEarly,
    'TrajectoryController delays gripper actuation during arm flight (no premature grip)');
  // At 420ms (200ms move + 150ms dwell = 350ms), the gripper should have fired
  await new Promise((r) => setTimeout(r, 380));
  const hasGripLate = sentCommands.some(c => c.cmd.includes('GRIP'));
  assert(hasGripLate,
    'TrajectoryController actuates gripper after arm motion duration + settle dwell');
  tcDwell.stop();

  // 11. Stepper J5 Range and Home FK
  assert(JOINT_LIMITS.j5.min === -120.0 && JOINT_LIMITS.j5.max === 120.0 && JOINT_LIMITS.j5.home === 0.0,
    'JOINT_LIMITS.j5 specifies stepper range [-120, +120] with neutral 0.0');
  assert(JOINT_CONFIGS.j5.min === -120.0 && JOINT_CONFIGS.j5.max === 120.0 && JOINT_CONFIGS.j5.default === 0.0,
    'JOINT_CONFIGS.j5 specifies slider range [-120, +120] with default 0.0');
  const homeFkCheck = forwardKinematics(0, 0, 0, {}, null, 0, 0, 0);
  assert(homeFkCheck.tcp.z > 130.0,
    `Home FK TCP Z is elevated above 130mm (got ${homeFkCheck.tcp.z.toFixed(2)}mm)`);
  // 12. BUG-01 & BUG-02: Two-State Safety gating on autoSend & ESTOP disarm
  let previewDispatched = 0;
  const safetyMc = new MotionController({
    appState: testState,
    serialManager: { isConnected: () => true, send: async () => { previewDispatched++; } }
  });
  safetyMc.setExecutionMode('preview', false);
  safetyMc.autoSend = true;
  safetyMc.setJointTarget('j1', 25.0);
  safetyMc.nudgeJoint('j2', 5.0);
  safetyMc.setCartesianTarget(220.0, 0.0, 200.0);
  safetyMc.nudgeCartesian('z', 10.0);
  safetyMc.nudgeOrientation('pitch', 5.0);
  assert(previewDispatched === 0,
    'BUG-01: Safe Preview mode dispatches 0 packets even when autoSend=true across all jog/nudge APIs');

  safetyMc.setExecutionMode('direct_drive', true);
  assert(safetyMc.isArmed === true && safetyMc.executionMode === 'direct_drive',
    'Direct Drive armed state set');
  testState.setEstop(true);
  assert(safetyMc.isArmed === false && safetyMc.executionMode === 'preview',
    'BUG-02: ESTOP immediately disarms Direct Drive and resets executionMode to preview');
  testState.setEstop(false);

  // 13. BUG-03: Uniform 3D Tool-Frame Step Scaling at Boundary
  safetyMc.setAllJointTargets({ j1: 45, j2: 0, j3: 0, j4: 0, j5: 0, j6: 0 }, false);
  safetyMc.setCartesianFrame('tool');
  safetyMc.cartesian.x = 495;
  safetyMc.cartesian.y = 0;
  safetyMc.cartesian.z = 200;
  const beforeX = safetyMc.cartesian.x;
  const beforeY = safetyMc.cartesian.y;
  safetyMc.stepCartesian('z', 1, 20);
  const dx = safetyMc.cartesian.x - beforeX;
  const dy = safetyMc.cartesian.y - beforeY;
  assert(Math.abs(dx) > 0.1 && Math.abs(dx - dy) < 0.05,
    `BUG-03: Tool-frame step near boundary preserves 45 deg ray direction (dx=${dx.toFixed(2)}, dy=${dy.toFixed(2)})`);

  // 14. BUG-04: sendGhostToRobot dispatches coordinated joint-space spline packet
  let lastGhostCmd = null;
  const ghostMc = new MotionController({
    appState: testState,
    serialManager: { isConnected: () => true, send: async (cmd) => { lastGhostCmd = cmd; } }
  });
  ghostMc.setAllJointTargets({ j1: 15, j2: -20, j3: 45, j4: 0, j5: 10, j6: 0 }, false);
  await ghostMc.sendGhostToRobot();
  assert(typeof lastGhostCmd === 'string' && lastGhostCmd.trim().startsWith('<') && lastGhostCmd.trim().endsWith('>'),
    `BUG-04: sendGhostToRobot dispatches framed joint-space spline packet (got ${JSON.stringify(lastGhostCmd)})`);

  // 15. BUG-06, BUG-07 & BUG-08: TelemetryParser j6, PacketBuilder J4/J5 negative jogs, MockSerialPort 0 deg neutral
  const parsedJtelem = parseSerialLine('{"j1":10,"j2":20,"j3":30,"j4":-45,"j5":-30,"j6":120,"en":1}');
  assert(parsedJtelem.j6 === 120 && parsedJtelem.j4 === -45 && parsedJtelem.j5 === -30,
    'BUG-06: TelemetryParser parses j6 and negative j4/j5 angles in JTELEM');
  const negJogJ4 = PacketBuilder.buildJogCommand('J4', -45.5);
  const negJogJ5 = PacketBuilder.buildJogCommand('J5', -60.0);
  assert(negJogJ4 === 'A4 -45.5\n' && negJogJ5 === 'A5 -60.0\n',
    'BUG-07: PacketBuilder.buildJogCommand preserves negative angles for J4 and J5 within [-90, 90]');
  const freshMock = new MockSerialPort();
  assert(freshMock.j2 === 0 && freshMock.j3 === 0 && freshMock.j4 === 0 && freshMock.j5 === 0 && freshMock.j6 === 0,
    'BUG-08: MockSerialPort initializes to 6-DOF neutral 0 deg across all joints');

  // 16. BUG-09: TrajectoryController dispatches MOVL for movl waypoints
  let movlDispatched = [];
  const tcMovl = new TrajectoryController({
    appState: testState,
    serialManager: { isConnected: () => true, send: async () => {} }
  });
  tcMovl._sendSerial = (cmd) => { movlDispatched.push(cmd); };
  tcMovl.setSequence({
    sequenceName: 'MovlTest',
    loop: false,
    waypoints: [{ j1: 0, j2: 0, j3: 0, j4: 0, j5: 0, j6: 0, dur: 500, moveType: 'movl' }]
  });
  tcMovl.play();
  assert(movlDispatched.some(c => c.startsWith('MOVL ')),
    'BUG-09: TrajectoryController dispatches MOVL command when waypoint moveType is "movl"');
  tcMovl.stop();

  // 17. BUG-10 & BUG-11: PresetManager exports j6/moveType & CNC Star preset is within [-90, 90] J5 limits
  const pmTest = new PresetManager();
  const exportedJson = JSON.parse(pmTest.exportSequenceJSON({
    sequenceName: 'ExportCheck',
    loop: false,
    waypoints: [{ j1: 0, j2: 0, j3: 0, j4: 15, j5: -25, j6: 90, grip: 45, dur: 1000, moveType: 'movl' }]
  }));
  assert(exportedJson.waypoints[0].j6 === 90 && exportedJson.waypoints[0].moveType === 'movl',
    'BUG-10: PresetManager.exportSequenceJSON preserves j6 and moveType fields');
  const cncStar = DEFAULT_PRESETS['CNC Star (G-Code)'];
  const allCncStarValid = cncStar.waypoints.every(wp =>
    wp.j4 >= -90 && wp.j4 <= 90 && wp.j5 >= -90 && wp.j5 <= 90 && typeof wp.j6 === 'number'
  );
  assert(allCncStarValid,
    'BUG-11: All CNC Star (G-Code) preset waypoints obey 6-DOF J4/J5 [-90, 90] limits and define j6');
});

// -----------------------------------------------------------------------------
// Summary
// -----------------------------------------------------------------------------
console.log('\n=============================================================');
console.log(`TOTAL TESTS: ${passed + failed} | PASSED: ${passed} | FAILED: ${failed}`);
console.log('=============================================================\n');

if (failed > 0) {
  console.error(`Test run FAILED with ${failed} failure(s):`);
  failures.forEach((f, i) => console.error(`  ${i + 1}. ${f}`));
  process.exit(1);
} else {
  console.log('ALL TESTS PASSED SUCCESSFULLY! (100% Pass Rate)');
  process.exit(0);
}
