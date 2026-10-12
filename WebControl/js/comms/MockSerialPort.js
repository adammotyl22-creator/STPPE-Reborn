/**
 * MockSerialPort.js - In-Browser Virtual Teensy 4.1 Hardware Simulator
 * 
 * Simulates Teensy 4.1 firmware v3.1.0 (IK_Control.ino):
 * - Streams 10Hz JSON telemetry (`JTELEM`) matching exact firmware schema
 * - Executes quintic spline physics for smooth trajectory simulation
 * - Evaluates CRC-8 Maxim packets and responds to all ASCII text commands
 * - Full WebSerial API duck-typing (readable/writable streams, open/close lifecycle)
 */

import { CRC8Maxim } from '../utils/CRC8.js';
import {
  BASE_H,
  L1,
  L2,
  L3,
  JOINT_LIMITS,
  forwardKinematics
} from '../kinematics/RobotKinematics.js';

/**
 * Quintic trajectory polynomial s(t) = 10t^3 - 15t^4 + 6t^5 (C2 continuous)
 */
function quinticInterpolate(start, target, t) {
  const tau = Math.max(0, Math.min(1, t));
  const s = tau * tau * tau * (10 + tau * (-15 + tau * 6));
  return start + (target - start) * s;
}

export class MockSerialPort {
  constructor(options = {}) {
    this.options = {
      telemIntervalMs: 100, // 10Hz
      initialPose: { j1: 0.0, j2: -90.0, j3: 90.0, j4: 90.0, j5: 0.0, j6: 0.0, grip: 0.0 },
      ...options
    };

    // Live state
    this.isOpen = false;
    this.j1 = this.options.initialPose.j1;
    this.j2 = this.options.initialPose.j2;
    this.j3 = this.options.initialPose.j3;
    this.j4 = this.options.initialPose.j4;
    this.j5 = this.options.initialPose.j5;
    this.j6 = this.options.initialPose.j6 ?? 0.0;
    this.grip = this.options.initialPose.grip;
    this.dx = 0;
    this.dy = 0;
    this.spd = 100;
    this.estopActive = false;
    this.homingActive = false;
    this.teachActive = false;
    this.teachBuffer = [];
    this.closedLoop = false;
    this.jtelemEnabled = true;
    this.telemEnabled = false;

    this.workOffset = { x: 0, y: 0, z: 0 };
    this.toolOffset = { x: 0, y: 0, z: 0 };

    // Motion interpolation state
    this.isMoving = false;
    this.moveStartTime = 0;
    this.moveDuration = 1000;
    this.startPose = { j1: this.j1, j2: this.j2, j3: this.j3, j4: this.j4, j5: this.j5, j6: this.j6 };
    this.targetPose = { j1: this.j1, j2: this.j2, j3: this.j3, j4: this.j4, j5: this.j5, j6: this.j6 };

    // Stream & callback listeners
    this.rxListeners = new Set();
    this.telemetryTimer = null;
    this.motionTimer = null;

    // WebSerial duck-typing streams
    this.readable = null;
    this.writable = null;
    this._streamController = null;
    this._incomingBuffer = '';
  }

  /**
   * Directly updates virtual hardware pose (for interactive slider/TCP drag sync)
   * @param {Object} pose - { j1, j2, j3, j4, j5, grip }
   */
  setPose(pose = {}) {
    if (pose.j1 !== undefined && !isNaN(pose.j1)) this.j1 = Number(pose.j1);
    if (pose.j2 !== undefined && !isNaN(pose.j2)) this.j2 = Number(pose.j2);
    if (pose.j3 !== undefined && !isNaN(pose.j3)) this.j3 = Number(pose.j3);
    if (pose.j4 !== undefined && !isNaN(pose.j4)) this.j4 = Number(pose.j4);
    if (pose.j5 !== undefined && !isNaN(pose.j5)) this.j5 = Number(pose.j5);
    if (pose.j6 !== undefined && !isNaN(pose.j6)) this.j6 = Number(pose.j6);
    if (pose.grip !== undefined && !isNaN(pose.grip)) this.grip = Number(pose.grip);
    this.isMoving = false;
  }

  /**
   * ESTOP status getter
   * @returns {boolean}
   */
  get isEstopped() {
    return this.estopActive;
  }

  /**
   * Opens the mock serial port
   */
  async open(options = { baudRate: 115200 }) {
    if (this.isOpen) {
      throw new Error('Mock port is already open');
    }
    this.isOpen = true;
    this.baudRate = options.baudRate || 115200;

    // Create browser/Node standard ReadableStream & WritableStream
    if (typeof ReadableStream !== 'undefined') {
      this.readable = new ReadableStream({
        start: (controller) => {
          this._streamController = controller;
        },
        cancel: () => {
          this._streamController = null;
        }
      });
    }

    if (typeof WritableStream !== 'undefined') {
      const decoder = new TextDecoder();
      this.writable = new WritableStream({
        write: (chunk) => {
          const text = typeof chunk === 'string' ? chunk : decoder.decode(chunk, { stream: true });
          this.receiveFromClient(text);
        }
      });
    }

    // Start 10Hz Telemetry & 50Hz Physics timers
    this._startTimers();
    return Promise.resolve();
  }

  /**
   * Closes the mock serial port
   */
  async close() {
    this.isOpen = false;
    this._stopTimers();
    if (this._streamController) {
      try {
        this._streamController.close();
      } catch (_) {}
      this._streamController = null;
    }
    return Promise.resolve();
  }

  /**
   * Adds listener for raw serial output from simulated Teensy
   */
  onOutput(callback) {
    this.rxListeners.add(callback);
    return () => this.rxListeners.delete(callback);
  }

  /**
   * Transmits data from simulated Teensy to client/listeners
   * @private
   */
  emitOutput(text) {
    if (!this.isOpen) return;

    // Push to WebSerial ReadableStream if available
    if (this._streamController) {
      try {
        const encoded = typeof TextEncoder !== 'undefined' ? new TextEncoder().encode(text) : Buffer.from(text);
        this._streamController.enqueue(encoded);
      } catch (_) {}
    }

    // Call direct listeners
    for (const fn of this.rxListeners) {
      try {
        fn(text);
      } catch (e) {
        console.error('Error in MockSerialPort rx listener:', e);
      }
    }
  }

  /**
   * Receives incoming command or packet from client
   * @param {string|Uint8Array} data
   */
  receiveFromClient(data) {
    if (!this.isOpen) return;
    const text = typeof data === 'string' ? data : new TextDecoder().decode(data);
    this._incomingBuffer += text;

    const lines = this._incomingBuffer.split(/\r?\n/);
    this._incomingBuffer = lines.pop() ?? '';

    for (const line of lines) {
      const trimmed = line.trim();
      if (trimmed) {
        this._processCommandLine(trimmed);
      }
    }
  }

  /**
   * Processes a single command line or framed packet
   * @private
   */
  _processCommandLine(rawLine) {
    // 1. Framed CRC-8 Packet: <j1,j2,j3,j4,j5,dx,dy,dur,crc>
    if (rawLine.startsWith('<') && rawLine.endsWith('>')) {
      this._handleCrcPacket(rawLine);
      return;
    }

    // 2. ASCII Text Commands
    const upper = rawLine.toUpperCase().trim();

    if (upper === 'ESTOP') {
      this.estopActive = true;
      this.isMoving = false;
      this.homingActive = false;
      this.emitOutput('!!! EMERGENCY STOP: MOTION HALTED, DRIVERS DISABLED !!!\n');
      return;
    }

    if (upper === 'REENABLE') {
      this.estopActive = false;
      this.emitOutput('Drivers re-enabled\n');
      return;
    }

    if (this.estopActive) {
      this.emitOutput('ERR: ESTOP ACTIVE - Send REENABLE to restore motion\n');
      return;
    }

    if (upper === 'VER') {
      this.emitOutput('VER v3.1.0 2026-09-01 15:44\n');
      return;
    }

    if (upper.startsWith('MOV ') || upper.startsWith('MOVL ')) {
      this._handleCartesianMove(upper);
      return;
    }

    if (upper.startsWith('HOME')) {
      this._handleHoming(upper);
      return;
    }

    if (upper === 'STOP') {
      this.isMoving = false;
      this.emitOutput('Motion stopped (queue cleared)\nOK\n');
      return;
    }

    if (upper.startsWith('SPEED')) {
      const match = upper.match(/SPEED\s+(\d+)/);
      if (match) {
        this.spd = Math.min(100, Math.max(5, parseInt(match[1], 10)));
        this.emitOutput(`Speed override: ${this.spd}%\n`);
      }
      return;
    }

    if (upper.startsWith('GRIP')) {
      if (upper === 'GRIP OPEN') {
        this.grip = 0;
        this.emitOutput('Gripper: 0\n');
      } else if (upper === 'GRIP CLOSE') {
        this.grip = 180;
        this.emitOutput('Gripper: 180\n');
      } else {
        const match = upper.match(/GRIP\s+(\d+)/);
        if (match) {
          this.grip = Math.min(180, Math.max(0, parseInt(match[1], 10)));
          this.emitOutput(`Gripper: ${this.grip}\n`);
        }
      }
      return;
    }

    // Interactive jogging (D1, D2, D3, A4, A5, A6, DX, DY)
    if (upper.startsWith('D1 ')) {
      const val = parseFloat(upper.substring(3));
      this.j1 = Math.max(JOINT_LIMITS.j1.min, Math.min(JOINT_LIMITS.j1.max, this.j1 + val));
      return;
    }
    if (upper.startsWith('D2 ')) {
      const val = parseFloat(upper.substring(3));
      this.j2 = Math.max(JOINT_LIMITS.j2.min, Math.min(JOINT_LIMITS.j2.max, this.j2 + val));
      return;
    }
    if (upper.startsWith('D3 ')) {
      const val = parseFloat(upper.substring(3));
      this.j3 = Math.max(JOINT_LIMITS.j3.min, Math.min(JOINT_LIMITS.j3.max, this.j3 + val));
      return;
    }
    if (upper.startsWith('A4 ')) {
      const val = parseFloat(upper.substring(3));
      this.j4 = Math.max(JOINT_LIMITS.j4.min, Math.min(JOINT_LIMITS.j4.max, val));
      return;
    }
    if (upper.startsWith('A5 ')) {
      const val = parseFloat(upper.substring(3));
      this.j5 = Math.max(JOINT_LIMITS.j5.min, Math.min(JOINT_LIMITS.j5.max, val));
      return;
    }
    if (upper.startsWith('A6 ')) {
      const val = parseFloat(upper.substring(3));
      this.j6 = Math.max(JOINT_LIMITS.j6.min, Math.min(JOINT_LIMITS.j6.max, val));
      return;
    }
    if (upper.startsWith('DX ')) {
      const val = parseInt(upper.substring(3), 10);
      this.dx += val;
      return;
    }
    if (upper.startsWith('DY ')) {
      const val = parseInt(upper.substring(3), 10);
      this.dy += val;
      return;
    }

    if (upper === 'CL_ON') {
      this.closedLoop = true;
      this.emitOutput('Closed Loop: ENABLED (manual)\n');
      return;
    }
    if (upper === 'CL_OFF') {
      this.closedLoop = false;
      this.emitOutput('Closed Loop: DISABLED\n');
      return;
    }
    if (upper === 'CAL') {
      this.emitOutput('CALIBRATED: Encoder Offsets synced to current commanded angles\n');
      return;
    }
    if (upper === 'DIAG') {
      this.emitOutput('=== DIAGNOSTICS ===\nJ1 (TMC2209): OK, Temp: NORMAL, CS: 16\nJ2 (TMC2209): OK, Temp: NORMAL, CS: 24\nJ3 (TMC2209): OK, Temp: NORMAL, CS: 20\nSTATUS: ALL DRIVERS NORMAL\n');
      return;
    }
    if (upper === 'ENCCHECK') {
      this.emitOutput('EC000\n');
      return;
    }
    if (upper === 'STATUS') {
      this._emitSingleJsonTelemetry();
      return;
    }
    if (upper === 'RP') {
      this._emitSingleAr4Telemetry();
      return;
    }
    if (upper.startsWith('JTELEM')) {
      const enable = upper.includes('1') || upper.includes('ON');
      this.jtelemEnabled = enable;
      this.emitOutput(`JSON Telemetry: ${enable ? 'ENABLED' : 'DISABLED'}\n`);
      return;
    }
    if (upper.startsWith('TELEM')) {
      const enable = upper.includes('1') || upper.includes('ON');
      this.telemEnabled = enable;
      this.emitOutput(`Telemetry: ${enable ? 'ENABLED' : 'DISABLED'}\n`);
      return;
    }
    if (upper === 'SAVECFG') {
      this.emitOutput('Config saved to EEPROM\n');
      return;
    }
    if (upper.startsWith('TEACH')) {
      this._handleTeach(upper);
      return;
    }

    // Default ACK for other valid commands
    this.emitOutput(`CMD OK: ${rawLine}\nOK\n`);
  }

  /**
   * Handles CRC-8 Framed Packet `<j1,j2,j3,j4,j5,dx,dy,dur,crc>`
   * @private
   */
  _handleCrcPacket(pktStr) {
    if (this.estopActive) {
      this.emitOutput('ERR: ESTOP ACTIVE\n');
      return;
    }

    const content = pktStr.slice(1, -1);
    const lastCommaIdx = content.lastIndexOf(',');
    if (lastCommaIdx === -1) {
      this.emitOutput('ERR: Malformed Packet (No comma)\n');
      return;
    }

    const payload = content.substring(0, lastCommaIdx);
    const crcStr = content.substring(lastCommaIdx + 1).trim();
    const recvCrc = parseInt(crcStr, 10);
    const calcCrc = CRC8Maxim.compute(payload);

    if (isNaN(recvCrc) || recvCrc !== calcCrc) {
      this.emitOutput(`ERR: CRC Mismatch (Calc:${calcCrc} Recv:${recvCrc})\n`);
      return;
    }

    const fields = payload.split(',');
    if (fields.length < 8) {
      this.emitOutput('ERR: Incomplete packet fields\n');
      return;
    }

    let targetJ1, targetJ2, targetJ3, targetJ4, targetJ5, targetJ6, targetDx, targetDy, duration;

    if (fields.length >= 9) {
      // 6-joint packet: <j1,j2,j3,j4,j5,j6,dx,dy,dur,crc>
      targetJ1 = parseFloat(fields[0]);
      targetJ2 = parseFloat(fields[1]);
      targetJ3 = parseFloat(fields[2]);
      targetJ4 = parseFloat(fields[3]);
      targetJ5 = parseFloat(fields[4]);
      targetJ6 = parseFloat(fields[5]);
      targetDx = parseInt(fields[6], 10);
      targetDy = parseInt(fields[7], 10);
      duration = Math.max(50, parseInt(fields[8], 10));
    } else {
      // 5-joint legacy packet: <j1,j2,j3,j4,j5,dx,dy,dur,crc>
      targetJ1 = parseFloat(fields[0]);
      targetJ2 = parseFloat(fields[1]);
      targetJ3 = parseFloat(fields[2]);
      targetJ4 = parseFloat(fields[3]);
      targetJ5 = parseFloat(fields[4]);
      targetJ6 = this.j6;
      targetDx = parseInt(fields[5], 10);
      targetDy = parseInt(fields[6], 10);
      duration = Math.max(50, parseInt(fields[7], 10));
    }

    // Acknowledge packet
    this.emitOutput(`PKT (CRC OK): ${payload},${calcCrc}\n`);

    // Start quintic movement
    this._startMove({
      j1: targetJ1,
      j2: targetJ2,
      j3: targetJ3,
      j4: targetJ4,
      j5: targetJ5,
      j6: targetJ6
    }, duration);

    this.dx += targetDx;
    this.dy += targetDy;
  }

  /**
   * Handles Cartesian move command (MOV / MOVL)
   * @private
   */
  _handleCartesianMove(cmd) {
    const isLinear = cmd.startsWith('MOVL');
    const xMatch = cmd.match(/X([-\d.]+)/);
    const yMatch = cmd.match(/Y([-\d.]+)/);
    const zMatch = cmd.match(/Z([-\d.]+)/);
    const j4Match = cmd.match(/J4([-\d.]+)/);
    const j5Match = cmd.match(/J5([-\d.]+)/);
    const j6Match = cmd.match(/J6([-\d.]+)/);
    const durMatch = cmd.match(/D(\d+)/);

    const x = xMatch ? parseFloat(xMatch[1]) : 0;
    const y = yMatch ? parseFloat(yMatch[1]) : 0;
    const z = zMatch ? parseFloat(zMatch[1]) : 215;
    
    let targetJ4 = this.j4;
    let targetJ5 = this.j5;
    let targetJ6 = this.j6;
    if (j4Match) {
      targetJ4 = parseFloat(j4Match[1]);
    }
    if (j5Match) {
      targetJ5 = parseFloat(j5Match[1]);
    }
    if (j6Match) {
      targetJ6 = parseFloat(j6Match[1]);
    }
    const dur = durMatch ? parseInt(durMatch[1], 10) : (isLinear ? 2000 : 1000);

    this.emitOutput(`${isLinear ? 'MOVL' : 'MOV'}: Target X=${x} Y=${y} Z=${z} D=${dur}\n`);

    // Approximate target joint angle motion for simulation
    const targetJ1 = Math.atan2(y, x) * (180 / Math.PI);
    this._startMove({
      j1: Math.max(-85, Math.min(85, targetJ1)),
      j2: this.j2,
      j3: this.j3,
      j4: Math.max(-90, Math.min(90, targetJ4)),
      j5: Math.max(-90, Math.min(90, targetJ5)),
      j6: Math.max(-180, Math.min(180, targetJ6))
    }, dur);

    this.emitOutput('OK\n');
  }

  /**
   * Handles Homing routine
   * @private
   */
  _handleHoming(cmd) {
    this.homingActive = true;
    this.emitOutput('HOMING: Starting (non-blocking)...\n');

    this._startMove({
      j1: 0.0,
      j2: -90.0,
      j3: 90.0,
      j4: 90.0,
      j5: 90.0
    }, 1500);

    setTimeout(() => {
      this.homingActive = false;
      this.j1 = 0.0;
      this.j2 = -90.0;
      this.j3 = 90.0;
      this.emitOutput('HOMING: Complete\nOK\n');
    }, 1500);
  }

  /**
   * Handles Teach mode commands
   * @private
   */
  _handleTeach(cmd) {
    if (cmd.includes('START') || cmd === 'TEACH 0') {
      this.teachActive = true;
      this.emitOutput('TEACH: Recording started...\n');
    } else if (cmd.includes('RECORD') || cmd === 'TEACH 1') {
      if (this.teachBuffer.length < 64) {
        this.teachBuffer.push({ j1: this.j1, j2: this.j2, j3: this.j3, j4: this.j4, j5: this.j5, grip: this.grip });
        this.emitOutput(`TEACH: Point ${this.teachBuffer.length} recorded\n`);
      } else {
        this.emitOutput('ERR: TEACH BUFFER FULL\n');
      }
    } else if (cmd.includes('STOP') || cmd === 'TEACH 2') {
      this.teachActive = false;
      this.emitOutput(`TEACH: ${this.teachBuffer.length} points stored\n`);
    } else if (cmd.includes('PLAY') || cmd === 'TEACH 3') {
      this.emitOutput('TEACH: Playback started\n');
    } else if (cmd.includes('CLEAR') || cmd === 'TEACH 4') {
      this.teachBuffer = [];
      this.emitOutput('TEACH: Buffer cleared\n');
    }
  }

  /**
   * Starts a smooth quintic movement toward target angles
   * @private
   */
  _startMove(target, durationMs) {
    this.startPose = { j1: this.j1, j2: this.j2, j3: this.j3, j4: this.j4, j5: this.j5, j6: this.j6 };
    this.targetPose = { ...this.startPose, ...target };
    this.moveStartTime = Date.now();
    this.moveDuration = Math.max(50, durationMs);
    this.isMoving = true;
  }

  /**
   * Advances trajectory physics step
   * @private
   */
  _stepPhysics() {
    if (!this.isMoving) return;

    const elapsed = Date.now() - this.moveStartTime;
    const t = elapsed / this.moveDuration;

    this.j1 = quinticInterpolate(this.startPose.j1, this.targetPose.j1, t);
    this.j2 = quinticInterpolate(this.startPose.j2, this.targetPose.j2, t);
    this.j3 = quinticInterpolate(this.startPose.j3, this.targetPose.j3, t);
    this.j4 = quinticInterpolate(this.startPose.j4, this.targetPose.j4, t);
    this.j5 = quinticInterpolate(this.startPose.j5, this.targetPose.j5, t);
    this.j6 = quinticInterpolate(this.startPose.j6 ?? 0, this.targetPose.j6 ?? 0, t);

    if (t >= 1.0) {
      this.isMoving = false;
      this.j1 = this.targetPose.j1;
      this.j2 = this.targetPose.j2;
      this.j3 = this.targetPose.j3;
      this.j4 = this.targetPose.j4;
      this.j5 = this.targetPose.j5;
      this.j6 = this.targetPose.j6;
    }
  }

  /**
   * Emits single JSON telemetry line
   * @private
   */
  _emitSingleJsonTelemetry() {
    const fk = forwardKinematics(this.j1, this.j2, this.j3, this.workOffset, this.toolOffset, this.j4, this.j5, this.j6);
    const tcpX = fk.tcp ? fk.tcp.x : (fk.x ?? 0);
    const tcpY = fk.tcp ? fk.tcp.y : (fk.y ?? 0);
    const tcpZ = fk.tcp ? fk.tcp.z : (fk.z ?? 0);
    const jsonStr = JSON.stringify({
      j1: parseFloat(this.j1.toFixed(2)),
      j2: parseFloat(this.j2.toFixed(2)),
      j3: parseFloat(this.j3.toFixed(2)),
      j4: Math.round(this.j4),
      j5: Math.round(this.j5),
      j6: Math.round(this.j6 ?? 0),
      grip: Math.round(this.grip),
      x: parseFloat(tcpX.toFixed(1)),
      y: parseFloat(tcpY.toFixed(1)),
      z: parseFloat(tcpZ.toFixed(1)),
      spd: this.spd,
      estop: this.estopActive,
      homing: this.homingActive,
      teach: this.teachActive,
      tpts: this.teachBuffer.length
    });
    this.emitOutput(`${jsonStr}\n`);
  }

  /**
   * Emits single AR4 telemetry line
   * @private
   */
  _emitSingleAr4Telemetry() {
    const fk = forwardKinematics(this.j1, this.j2, this.j3, this.workOffset, this.toolOffset, this.j4, this.j5, this.j6);
    const tcpX = fk.tcp ? fk.tcp.x : (fk.x ?? 0);
    const tcpY = fk.tcp ? fk.tcp.y : (fk.y ?? 0);
    const tcpZ = fk.tcp ? fk.tcp.z : (fk.z ?? 0);
    const ar4Str = `A${this.j1.toFixed(3)}B${this.j2.toFixed(3)}C${this.j3.toFixed(3)}D${Math.round(this.j4)}E${Math.round(this.j5)}F${Math.round(this.j6 ?? 0)}G${tcpX.toFixed(2)}H${tcpY.toFixed(2)}I${tcpZ.toFixed(2)}`;
    this.emitOutput(`${ar4Str}\n`);
  }

  /**
   * Starts telemetry and physics intervals
   * @private
   */
  _startTimers() {
    // 50Hz physics loop for smooth quintic stepping
    this.motionTimer = setInterval(() => {
      this._stepPhysics();
    }, 20);

    // 10Hz telemetry emission loop
    this.telemetryTimer = setInterval(() => {
      if (this.jtelemEnabled) {
        this._emitSingleJsonTelemetry();
      }
      if (this.telemEnabled) {
        this._emitSingleAr4Telemetry();
      }
    }, this.options.telemIntervalMs);
  }

  /**
   * Stops simulation intervals
   * @private
   */
  _stopTimers() {
    if (this.motionTimer) {
      clearInterval(this.motionTimer);
      this.motionTimer = null;
    }
    if (this.telemetryTimer) {
      clearInterval(this.telemetryTimer);
      this.telemetryTimer = null;
    }
  }
}

export default MockSerialPort;
