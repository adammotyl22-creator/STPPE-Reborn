/**
 * PacketBuilder.js - Framed CRC-8 Packet & Text Command Generator
 * 
 * Generates framed CRC-8 Maxim packets `<j1,j2,j3,j4,j5,dx,dy,dur,crc>`
 * and ASCII text commands matching Teensy 4.1 IK_Control.ino v3.1.0.
 */

import { CRC8Maxim } from '../utils/CRC8.js';

/**
 * Normalizes input parameters for framed packets whether passed as object or positional args
 * @private
 */
function normalizeFramedPacketParams(arg1, j2, j3, j4, j5, j6OrDx, dxOrDy, dyOrDur, dur) {
  if (typeof arg1 === 'object' && arg1 !== null) {
    const hasJ6 = (arg1.j6 !== undefined);
    return {
      j1: Number(arg1.j1 ?? 0),
      j2: Number(arg1.j2 ?? 0),
      j3: Number(arg1.j3 ?? 0),
      j4: Number(arg1.j4 ?? 0),
      j5: Number(arg1.j5 ?? 0),
      j6: hasJ6 ? Number(arg1.j6) : undefined,
      dx: Number(arg1.dx ?? 0),
      dy: Number(arg1.dy ?? 0),
      dur: Number(arg1.dur ?? arg1.duration ?? 1000)
    };
  }

  // Positional signature:
  // If 9 arguments were passed, j6OrDx is j6:
  if (dur !== undefined) {
    return {
      j1: Number(arg1 ?? 0),
      j2: Number(j2 ?? 0),
      j3: Number(j3 ?? 0),
      j4: Number(j4 ?? 0),
      j5: Number(j5 ?? 0),
      j6: Number(j6OrDx ?? 0),
      dx: Number(dxOrDy ?? 0),
      dy: Number(dyOrDur ?? 0),
      dur: Number(dur ?? 1000)
    };
  }

  // If 8 arguments: legacy (j1, j2, j3, j4, j5, dx, dy, dur)
  return {
    j1: Number(arg1 ?? 0),
    j2: Number(j2 ?? 0),
    j3: Number(j3 ?? 0),
    j4: Number(j4 ?? 0),
    j5: Number(j5 ?? 0),
    j6: undefined,
    dx: Number(j6OrDx ?? 0),
    dy: Number(dxOrDy ?? 0),
    dur: Number(dyOrDur ?? 1000)
  };
}

/**
 * Builds a structured CRC-8 Maxim framed packet `<j1,j2,j3,j4,j5[,j6],dx,dy,dur,crc>`
 * 
 * Supports both 5-joint and 6-joint payloads.
 * 
 * @param {Object|number} p - Packet config object or j1 angle
 * @returns {string} Framed serial string ready for transmission
 */
export function buildFramedPacket(p, j2, j3, j4, j5, j6OrDx, dxOrDy, dyOrDur, dur) {
  const norm = normalizeFramedPacketParams(p, j2, j3, j4, j5, j6OrDx, dxOrDy, dyOrDur, dur);

  const f_j1 = norm.j1.toFixed(2);
  const f_j2 = norm.j2.toFixed(2);
  const f_j3 = norm.j3.toFixed(2);
  const f_j4 = norm.j4.toFixed(1);
  const f_j5 = norm.j5.toFixed(1);
  const i_dx = Math.round(norm.dx);
  const i_dy = Math.round(norm.dy);
  const i_dur = Math.max(1, Math.round(norm.dur));

  let payload;
  if (norm.j6 !== undefined) {
    const f_j6 = norm.j6.toFixed(1);
    payload = `${f_j1},${f_j2},${f_j3},${f_j4},${f_j5},${f_j6},${i_dx},${i_dy},${i_dur}`;
  } else {
    payload = `${f_j1},${f_j2},${f_j3},${f_j4},${f_j5},${i_dx},${i_dy},${i_dur}`;
  }
  const crc = CRC8Maxim.compute(payload);

  return `<${payload},${crc}>`;
}

/**
 * Alias for buildFramedPacket
 */
export const buildCrcPacket = buildFramedPacket;

/**
 * Builds Cartesian move text command (MOV or MOVL)
 * Format: `MOV X# Y# Z# W# J4# J5# [J6#] D#\n`
 */
export function buildCartesianCommand(opt, y, z, wristMode, j4, j5, j6OrDuration, durationArg, isLinearArg) {
  let xVal, yVal, zVal, wVal, j4Val, j5Val, j6Val, durVal, linearVal;

  if (typeof opt === 'object' && opt !== null) {
    xVal = Number(opt.x ?? 0);
    yVal = Number(opt.y ?? 0);
    zVal = Number(opt.z ?? 141);
    wVal = Number(opt.wristMode ?? opt.w ?? 0);
    j4Val = Number(opt.j4 ?? 0);
    j5Val = Number(opt.j5 ?? 0);
    j6Val = opt.j6 !== undefined ? Number(opt.j6) : undefined;
    durVal = Number(opt.duration ?? opt.dur ?? opt.d ?? 1000);
    linearVal = Boolean(opt.isLinear ?? opt.linear ?? false);
  } else {
    xVal = Number(opt ?? 0);
    yVal = Number(y ?? 0);
    zVal = Number(z ?? 141);
    wVal = Number(wristMode ?? 0);
    j4Val = Number(j4 ?? 0);
    j5Val = Number(j5 ?? 0);
    if (isLinearArg !== undefined) {
      j6Val = Number(j6OrDuration ?? 0);
      durVal = Number(durationArg ?? 1000);
      linearVal = Boolean(isLinearArg);
    } else {
      j6Val = undefined;
      durVal = Number(j6OrDuration ?? 1000);
      linearVal = Boolean(durationArg);
    }
  }

  const cmd = linearVal ? 'MOVL' : 'MOV';
  const j6Str = j6Val !== undefined ? ` J6${j6Val.toFixed(1)}` : '';
  return `${cmd} X${xVal.toFixed(1)} Y${yVal.toFixed(1)} Z${zVal.toFixed(1)} W${Math.round(wVal)} J4${j4Val.toFixed(1)} J5${j5Val.toFixed(1)}${j6Str} D${Math.max(1, Math.round(durVal))}\n`;
}

export function buildMovCommand(x, y, z, wristMode = 0, j4 = 0, j5 = 0, j6OrDuration = 1000, duration = 1000) {
  if (typeof j6OrDuration === 'number' && duration !== undefined && arguments.length >= 8) {
    return buildCartesianCommand(x, y, z, wristMode, j4, j5, j6OrDuration, duration, false);
  }
  return buildCartesianCommand(x, y, z, wristMode, j4, j5, j6OrDuration, false);
}

export function buildMovLCommand(x, y, z, wristMode = 0, j4 = 0, j5 = 0, j6OrDuration = 2000, duration = 2000) {
  if (typeof j6OrDuration === 'number' && duration !== undefined && arguments.length >= 8) {
    return buildCartesianCommand(x, y, z, wristMode, j4, j5, j6OrDuration, duration, true);
  }
  return buildCartesianCommand(x, y, z, wristMode, j4, j5, j6OrDuration, true);
}

/**
 * Builds full 6-DOF Cartesian move command with tool orientation
 * Format: `MOV X# Y# Z# R# P# YW# D#\n` (or `MOVL`)
 */
export function buildMovOrientationCommand(x, y, z, roll = 0, pitch = 0, yaw = 0, duration = 1000, isLinear = false) {
  const cmd = isLinear ? 'MOVL' : 'MOV';
  const xVal = Number(x || 0).toFixed(1);
  const yVal = Number(y || 0).toFixed(1);
  const zVal = Number(z || 0).toFixed(1);
  const rVal = Number(roll || 0).toFixed(1);
  const pVal = Number(pitch || 0).toFixed(1);
  const ywVal = Number(yaw || 0).toFixed(1);
  const dVal = Math.max(1, Math.round(Number(duration) || 1000));
  return `${cmd} X${xVal} Y${yVal} Z${zVal} R${rVal} P${pVal} YW${ywVal} D${dVal}\n`;
}

/**
 * Builds Backlash configuration or query command
 * @param {number} [axis] - Axis index 1-6. If omitted, queries backlash table.
 * @param {number} [deg] - Backlash compensation in degrees
 */
export function buildBacklashCommand(axis, deg) {
  if (axis !== undefined && deg !== undefined) {
    const a = Math.round(Number(axis));
    const d = Number(deg);
    return `BACKLASH J${a} ${d.toFixed(3)}\n`;
  }
  return 'BACKLASH\n';
}

/**
 * Builds Homing command
 * @param {string} [target='ALL'] - 'ALL', 'SWITCH', 'J1', 'J2', 'J3', 'GANTRY', 'MICRO', 'MANUAL', 'ENCODER'
 */
export function buildHomeCommand(target = 'ALL') {
  const norm = String(target).toUpperCase().trim();
  switch (norm) {
    case 'J1':
    case 'HOMEJ1':
      return 'HOMEJ1\n';
    case 'J2':
    case 'HOMEJ2':
      return 'HOMEJ2\n';
    case 'J3':
    case 'HOMEJ3':
      return 'HOMEJ3\n';
    case 'GANTRY':
    case 'MICRO':
    case 'HOME GANTRY':
    case 'HOME MICRO':
      return 'HOME GANTRY\n';
    case 'MANUAL':
    case 'ENCODER':
    case 'HOME MANUAL':
    case 'HOME ENCODER':
      return 'HOME MANUAL\n';
    case 'SWITCH':
    case 'HOME SWITCH':
      return 'HOME SWITCH\n';
    case 'ALL':
    case 'HOME':
    default:
      return 'HOME\n';
  }
}

/**
 * Builds Emergency Stop command
 */
export function buildEstopCommand() {
  return 'ESTOP\n';
}

/**
 * Builds Re-enable Drivers command
 */
export function buildReenableCommand() {
  return 'REENABLE\n';
}

/**
 * Builds Motion Stop command
 */
export function buildStopCommand() {
  return 'STOP\n';
}

/**
 * Builds Speed Override command (5% - 100%)
 */
export function buildSpeedCommand(percent) {
  const clamped = Math.min(100, Math.max(5, Math.round(Number(percent))));
  return `SPEED ${clamped}\n`;
}

/**
 * Builds Gripper command
 * @param {number|string} angleOrMacro - Angle 0-180 or 'OPEN' / 'CLOSE'
 */
export function buildGripCommand(angleOrMacro) {
  if (typeof angleOrMacro === 'string') {
    const norm = angleOrMacro.toUpperCase().trim();
    if (norm === 'OPEN') return 'GRIP OPEN\n';
    if (norm === 'CLOSE') return 'GRIP CLOSE\n';
  }
  const angle = Math.min(180, Math.max(0, Math.round(Number(angleOrMacro))));
  return `GRIP ${angle}\n`;
}

/**
 * Builds Jog / Delta interactive command
 * Axis 1-3 sends relative delta (D1, D2, D3); Axis 4-5 sends absolute angle (A4, A5)
 */
export function buildJogCommand(axis, val) {
  const strAxis = String(axis).toUpperCase().trim();
  const numVal = Number(val);

  if (strAxis === '1' || strAxis === 'J1' || strAxis === 'D1') {
    return `D1 ${numVal.toFixed(2)}\n`;
  }
  if (strAxis === '2' || strAxis === 'J2' || strAxis === 'D2') {
    return `D2 ${numVal.toFixed(2)}\n`;
  }
  if (strAxis === '3' || strAxis === 'J3' || strAxis === 'D3') {
    return `D3 ${numVal.toFixed(2)}\n`;
  }
  if (strAxis === '4' || strAxis === 'J4' || strAxis === 'A4' || strAxis === 'D4') {
    const clamped = Math.min(90, Math.max(-90, numVal));
    return `A4 ${clamped.toFixed(1)}\n`;
  }
  if (strAxis === '5' || strAxis === 'J5' || strAxis === 'A5' || strAxis === 'D5') {
    const clamped = Math.min(90, Math.max(-90, numVal));
    return `A5 ${clamped.toFixed(1)}\n`;
  }
  if (strAxis === '6' || strAxis === 'J6' || strAxis === 'A6' || strAxis === 'D6') {
    const clamped = Math.min(180, Math.max(-180, numVal));
    return `A6 ${clamped.toFixed(1)}\n`;
  }
  if (strAxis === 'GRIP' || strAxis === 'G') {
    return buildGripCommand(numVal);
  }
  throw new Error(`Invalid jog axis: ${axis}`);
}

/**
 * Builds Micro-gantry jog command (DX or DY relative steps)
 */
export function buildGantryJogCommand(axis, steps) {
  const normAxis = String(axis).toUpperCase().trim();
  const numSteps = Math.round(Number(steps));
  if (normAxis === 'X' || normAxis === 'DX') {
    return `DX ${numSteps}\n`;
  }
  if (normAxis === 'Y' || normAxis === 'DY') {
    return `DY ${numSteps}\n`;
  }
  throw new Error(`Invalid gantry axis: ${axis}`);
}

/**
 * Builds Gantry speed command (MS <steps/s>)
 */
export function buildGantrySpeedCommand(speed) {
  return `MS ${Math.max(1, Math.round(Number(speed)))}\n`;
}

/**
 * Builds Closed-Loop PID enable/disable command
 */
export function buildClosedLoopCommand(enable = true) {
  return enable ? 'CL_ON\n' : 'CL_OFF\n';
}

/**
 * Builds Calibration / Sync encoder offset command
 */
export function buildCalibrateCommand() {
  return 'CAL\n';
}

/**
 * Builds Driver Diagnostics dump command
 */
export function buildDiagCommand() {
  return 'DIAG\n';
}

/**
 * Builds Manual Collision Check command
 */
export function buildEncCheckCommand() {
  return 'ENCCHECK\n';
}

/**
 * Builds Status / Instant Telemetry query command
 */
export function buildStatusQueryCommand() {
  return 'STATUS\n';
}

/**
 * Builds 10Hz JSON Telemetry stream toggle command
 */
export function buildJTelemCommand(enable = true) {
  return `JTELEM ${enable ? 1 : 0}\n`;
}

/**
 * Builds AR4 Telemetry stream toggle command
 */
export function buildTelemCommand(enable = true) {
  return `TELEM ${enable ? 1 : 0}\n`;
}

/**
 * Builds Robot Position (AR4 format) query command
 */
export function buildRpQueryCommand() {
  return 'RP\n';
}

/**
 * Builds Save Configuration to EEPROM command
 */
export function buildSaveCfgCommand() {
  return 'SAVECFG\n';
}

/**
 * Builds Teach mode commands
 * @param {string|number} action - 'START'|0, 'RECORD'|1, 'STOP'|2, 'PLAY'|3, 'CLEAR'|4
 */
export function buildTeachCommand(action) {
  const norm = String(action).toUpperCase().trim();
  switch (norm) {
    case '0':
    case 'START':
      return 'TEACH START\n';
    case '1':
    case 'RECORD':
      return 'TEACH RECORD\n';
    case '2':
    case 'STOP':
      return 'TEACH STOP\n';
    case '3':
    case 'PLAY':
      return 'TEACH PLAY\n';
    case '4':
    case 'CLEAR':
      return 'TEACH CLEAR\n';
    default:
      throw new Error(`Invalid teach action: ${action}`);
  }
}

/**
 * Builds Tool Offset setting command (TOOLX, TOOLY, TOOLZ)
 */
export function buildToolOffsetCommand(axis, val) {
  const normAxis = String(axis).toUpperCase().replace('TOOL', '').trim();
  if (!['X', 'Y', 'Z'].includes(normAxis)) {
    throw new Error(`Invalid tool offset axis: ${axis}`);
  }
  return `TOOL${normAxis} ${Number(val).toFixed(2)}\n`;
}

/**
 * Builds Work Offset setting command (WORKX, WORKY, WORKZ)
 */
export function buildWorkOffsetCommand(axis, val) {
  const normAxis = String(axis).toUpperCase().replace('WORK', '').trim();
  if (!['X', 'Y', 'Z'].includes(normAxis)) {
    throw new Error(`Invalid work offset axis: ${axis}`);
  }
  return `WORK${normAxis} ${Number(val).toFixed(2)}\n`;
}

/**
 * Builds Axis Maximum Velocity configuration command
 */
export function buildMaxSpeedCommand(axis, speed) {
  const numAxis = Math.min(3, Math.max(1, Math.round(Number(axis))));
  return `MAXSPD ${numAxis} ${Math.max(1, Math.round(Number(speed)))}\n`;
}

/**
 * Builds Axis Maximum Acceleration configuration command
 */
export function buildMaxAccelCommand(axis, accel) {
  const numAxis = Math.min(3, Math.max(1, Math.round(Number(axis))));
  return `MAXACC ${numAxis} ${Math.max(1, Math.round(Number(accel)))}\n`;
}

/**
 * Builds Spline Safety Margin command
 */
export function buildSafetyMarginCommand(margin) {
  const val = Math.max(1.0, Number(margin));
  return `SAFETY ${val.toFixed(2)}\n`;
}

/**
 * Builds Calibration parameters dump command
 */
export function buildCalDumpCommand() {
  return 'CALDUMP\n';
}

/**
 * Builds Calibration automated test routine command
 */
export function buildCalTestCommand() {
  return 'CALTEST\n';
}

/**
 * Static container class grouping all packet and command builders
 */
export class PacketBuilder {
  static buildFramedPacket = buildFramedPacket;
  static buildCrcPacket = buildCrcPacket;
  static buildCartesianCommand = buildCartesianCommand;
  static buildMovCommand = buildMovCommand;
  static buildMovLCommand = buildMovLCommand;
  static buildHomeCommand = buildHomeCommand;
  static buildEstopCommand = buildEstopCommand;
  static buildReenableCommand = buildReenableCommand;
  static buildStopCommand = buildStopCommand;
  static buildSpeedCommand = buildSpeedCommand;
  static buildGripCommand = buildGripCommand;
  static buildJogCommand = buildJogCommand;
  static buildGantryJogCommand = buildGantryJogCommand;
  static buildGantrySpeedCommand = buildGantrySpeedCommand;
  static buildClosedLoopCommand = buildClosedLoopCommand;
  static buildCalibrateCommand = buildCalibrateCommand;
  static buildDiagCommand = buildDiagCommand;
  static buildEncCheckCommand = buildEncCheckCommand;
  static buildStatusQueryCommand = buildStatusQueryCommand;
  static buildJTelemCommand = buildJTelemCommand;
  static buildTelemCommand = buildTelemCommand;
  static buildRpQueryCommand = buildRpQueryCommand;
  static buildSaveCfgCommand = buildSaveCfgCommand;
  static buildTeachCommand = buildTeachCommand;
  static buildToolOffsetCommand = buildToolOffsetCommand;
  static buildWorkOffsetCommand = buildWorkOffsetCommand;
  static buildMaxSpeedCommand = buildMaxSpeedCommand;
  static buildMaxAccelCommand = buildMaxAccelCommand;
  static buildSafetyMarginCommand = buildSafetyMarginCommand;
  static buildCalDumpCommand = buildCalDumpCommand;
  static buildCalTestCommand = buildCalTestCommand;
}

export default PacketBuilder;
