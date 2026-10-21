/**
 * MotionController.js - Master Motion & Cartesian IK UI Controller
 * 
 * Controls robot joint positions, Cartesian coordinates, wrist orientation modes,
 * reachability/safety validation, and spline/linear motion execution.
 * 
 * Features:
 * - Joint sliders with fine nudges ([-5°], [-1°], [-0.1°], [+0.1°], [+1°], [+5°])
 * - Gripper control with quick macro buttons ([Open 180°], [Close 0°], [Half 90°])
 * - Cartesian IK target panel (X, Y, Z in mm) with step buttons
 * - Wrist mode selection (W0 Level, W1 Down_0, W2 Down_90, W3 Pitch_0, W4 Pitch_90, W5 Pitch_180, W6 Manual)
 * - Movement duration input (ms, default 2000ms)
 * - Real-time reachability check and safety warning badge (calling solveIK & WorkspaceValidator)
 * - [ MOV (Spline) ] framed CRC packet generator & transmitter
 * - [ MOVL (Linear) ] ASCII linear command generator & transmitter
 * - Live vs Ghost sync controls ([Set Ghost to Live], [Send Ghost to Robot])
 */

import { appState as defaultAppState } from '../state/AppState.js';
import {
  JOINT_LIMITS,
  WristMode,
  forwardKinematics,
  forwardKinematics6DOF,
  solveIK,
  solveIK6DOF,
  eulerToQuat,
  quatSlerp,
  quatToEuler,
  clamp
} from '../kinematics/RobotKinematics.js';
import { checkWorkspace } from '../kinematics/WorkspaceValidator.js';
import {
  buildFramedPacket,
  buildCartesianCommand,
  buildMovLCommand,
  buildMovOrientationCommand,
  buildBacklashCommand,
  buildGripCommand,
  buildJogCommand
} from '../comms/PacketBuilder.js';
import { i18n } from '../i18n/I18nManager.js';

export const JOINT_CONFIGS = Object.freeze({
  j1: { min: -90.0, max: 90.0, step: 0.1, default: 0.0, label: 'Joint 1 (Base)' },
  j2: { min: -65.0, max: 115.0, step: 0.1, default: 0.0, label: 'Joint 2 (Shoulder)' },
  j3: { min: -35.0, max: 170.0, step: 0.1, default: 0.0, label: 'Joint 3 (Elbow)' },
  j4: { min: -90.0, max: 90.0, step: 0.1, default: 0.0, label: 'Joint 4 (Forearm Roll)' },
  j5: { min: -120.0, max: 120.0, step: 0.1, default: 0.0, label: 'Joint 5 (Wrist Pitch)' },
  j6: { min: -180.0, max: 180.0, step: 0.1, default: 0.0, label: 'Joint 6 (Tool Roll)' },
  grip: { min: 0.0, max: 180.0, step: 1.0, default: 0.0, label: 'Gripper' }
});

export const CARTESIAN_CONFIGS = Object.freeze({
  x: { min: -500.0, max: 500.0, step: 1.0, default: 253.5 },
  y: { min: -500.0, max: 500.0, step: 1.0, default: 0.0 },
  z: { min: -50.0, max: 590.0, step: 1.0, default: 141.0 },
  r: { min: 0.0, max: 443.0, step: 1.0, default: 253.5 },
  theta: { min: -90.0, max: 90.0, step: 0.1, default: 0.0 }
});

export const NUDGE_STEPS = Object.freeze([-5.0, -1.0, -0.1, 0.1, 1.0, 5.0]);
export const CARTESIAN_STEPS = Object.freeze([-50.0, -10.0, -1.0, 1.0, 10.0, 50.0]);

export class MotionController {
  /**
   * @param {Object} [options]
   * @param {HTMLElement|string} [options.container] - DOM element or selector
   * @param {AppState} [options.appState] - Reactive state store
   * @param {WebSerialManager} [options.serialManager] - WebSerial communication manager
   * @param {boolean} [options.autoSend=false] - If true, slider inputs send moves immediately
   * @param {number} [options.defaultDuration=2000] - Default trajectory duration in ms
   */
  constructor(options = {}) {
    this.appState = options.appState || defaultAppState;
    this.serialManager = options.serialManager || null;
    this._serialDataUnsub = null;
    this.toolOffset = { x: 0, y: 0, z: 0 };
    this.workOffset = { x: 0, y: 0, z: 0 };
    this.linearPreview = [];
    this.autoSend = options.autoSend !== undefined ? Boolean(options.autoSend) : true;
    this.duration = Math.max(1, Number(options.defaultDuration || options.duration || 2000));

    // Internal Ghost Target Joint Angles
    this.joints = {
      j1: JOINT_CONFIGS.j1.default,
      j2: JOINT_CONFIGS.j2.default,
      j3: JOINT_CONFIGS.j3.default,
      j4: JOINT_CONFIGS.j4.default,
      j5: JOINT_CONFIGS.j5.default,
      j6: JOINT_CONFIGS.j6.default,
      grip: JOINT_CONFIGS.grip.default
    };

    // Internal Ghost Cartesian Coordinates (Work Frame)
    this.cartesian = {
      x: 253.5,
      y: 0.0,
      z: 141.0,
      roll: 0.0,
      pitch: 0.0,
      yaw: 0.0,
      orientationMode: '6dof',
      wristMode: WristMode.LEVEL,
      j4Param: 0.0,
      j5Param: 0.0,
      j6Param: 0.0
    };

    // Software Backlash Settings (deg)
    this.backlash = {
      j1: 0.25,
      j2: 0.40,
      j3: 0.40,
      j4: 0.20,
      j5: 0.20,
      j6: 0.20
    };

    // Gantry offsets
    this.dx = 0;
    this.dy = 0;

    // Reachability and safety validation cache
    this.reachability = {
      isReachable: true,
      safe: true,
      clamped: false,
      warnings: [],
      violations: [],
      ikResult: null,
      workspaceResult: null,
      statusText: 'VALID (Reachable)'
    };

    // DOM Binding tracking
    this.container = null;
    this._domBound = false;
    this._domElements = {};
    this._eventCleanups = [];
    this._unsubState = [];

    // Live Stream Throttling State
    this._lastLiveStreamTime = 0;
    this._liveStreamTimer = null;
    this.LIVE_STREAM_INTERVAL_MS = 50; // 20Hz safe serial transmission rate
    this.LIVE_STREAM_DURATION_MS = 0; // Use critically damped firmware tracker (D0) for zero latency/stutter

    // Cylindrical (R/theta/Z) coordinate mode flag
    this.isCylindricalMode = false;

    // Unified Step-Jogger State
    this.cartesianStepSize = 10.0; // mm (options: 0.1, 1.0, 10.0, 50.0)
    this.jointStepSize = 1.0;      // deg (options: 0.1, 1.0, 5.0, 10.0, 45.0)

    // Milestone 1: Two-State Execution Safety Architecture
    this.executionMode = 'preview'; // 'preview' (safe ghost) | 'direct_drive' (live arm follow)
    this.isArmed = false;            // Requires confirmation dialog

    // Milestone 1: Cartesian Coordinate Frame Selector
    this.cartesianFrame = 'base';    // 'base' | 'world' | 'tool'

    // Milestone 1: Continuous Hold-Jog Acceleration Engine
    this.HOLD_JOG_THRESHOLD_MS = 250;        // 250ms press threshold to trigger continuous jog
    this.HOLD_JOG_RAMP_DURATION_MS = 1200;   // 1200ms duration for S-curve velocity ramp
    this.HOLD_JOG_MIN_VELOCITY = 2.0;        // Minimum creep velocity: 2.0 deg/s or 2.0 mm/s
    this.HOLD_JOG_MAX_VELOCITY = 35.0;       // Maximum jog velocity: 35.0 deg/s or 35.0 mm/s
    this.HOLD_JOG_DISPATCH_INTERVAL_MS = 50; // 20Hz rate limiting for serial transmission

    this._activeHoldJog = null;              // Tracks ongoing hold-jog interaction state

    // Milestone 1: Quick Pose Presets Library
    this.POSE_PRESETS = Object.freeze({
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
    });

    // Milestone 1: Persistent Gripper State & Hold-Jog Gesture State
    this.gripperPercent = 0; // 0% to 100%
    this._justFinishedHold = false;

    // Initialize initial Forward Kinematics and Ghost Pose
    this._recomputeFK();
    this._evaluateReachability();
    this._syncToAppState();

    // Subscribe to AppState updates
    this._initAppStateListeners();
    this.setSerialManager(this.serialManager);

    // Bind DOM if container is provided and environment supports DOM
    if (options.container && typeof document !== 'undefined') {
      const el = typeof options.container === 'string'
        ? document.querySelector(options.container)
        : options.container;
      if (el) {
        this.bindDom(el);
      }
    }
  }

  /**
   * Subscribes to central AppState changes
   * @private
   */
  _initAppStateListeners() {
    if (!this.appState) return;

    // When live telemetry updates and auto-sync is desired, update local state
    const unsubTelem = this.appState.subscribe('telemetry:updated', (live) => {
      if (this._autoSyncWithLive) {
        this.setGhostToLive(false);
      } else {
        this._updateJointDualMarkers();
      }
    });
    this._unsubState.push(unsubTelem);

    // Watch for safety changes (e.g. ESTOP)
    const unsubSafety = this.appState.subscribe('safety:estop', (estop) => {
      if (estop) {
        this._stopHoldJog();
        if (this._liveStreamTimer) {
          clearTimeout(this._liveStreamTimer);
          this._liveStreamTimer = null;
        }
        this._pendingStreamSend = false;
        this.setExecutionMode('preview', false);
      }
      this._updateMoveButtonsState(estop);
    });
    this._unsubState.push(unsubSafety);

    // Watch for language changes
    const unsubI18n = i18n.subscribe(() => {
      this._updateDomUI();
    });
    this._unsubState.push(unsubI18n);
  }

  /**
   * Sets serial communication manager
   * @param {WebSerialManager} manager
   */
  setSerialManager(manager) {
    if (this._serialDataUnsub) this._serialDataUnsub();
    this.serialManager = manager;
    this._serialDataUnsub = manager?.on?.('data', (packet) => {
      if (packet?.type !== 'TCPCAL_RESULT') return;
      this.toolOffset = { x: packet.x, y: packet.y, z: packet.z };
      for (const axis of ['x', 'y', 'z']) {
        const input = this.container?.querySelector(`#input_tool_${axis}`);
        if (input) input.value = String(packet[axis]);
      }
      this._setTcpStatus(`TCP X=${packet.x} Y=${packet.y} Z=${packet.z} mm; RMS=${packet.rmsErrorMm} mm`);
      this._recomputeFK();
      this._evaluateReachability();
      this._syncToAppState();
      this._updateDomUI();
    }) || null;
  }

  _setTcpStatus(message) {
    const status = this.container?.querySelector('#tcp_cal_status');
    if (status) status.textContent = message;
    this.appState?.addConsoleLog?.({ type: 'INFO', text: message });
  }

  async _sendTcpCommand(command) {
    if (!this.serialManager?.isConnected?.()) {
      this._setTcpStatus('Connect to the robot before changing TCP calibration or offsets.');
      return false;
    }
    await this.serialManager.send(`${command}\n`);
    this.appState?.addConsoleLog?.({ type: 'TX', text: command });
    return true;
  }

  // ==========================================================================
  // STREAMING & THROTTLING HELPERS
  // ==========================================================================

  /**
   * Dispatches a throttled live stream motion packet to connected hardware
   * @param {number} [customDur=null] - Duration override in ms
   * @private
   */
  _queueLiveStream(customDur = null) {
    // TWO-STATE SAFETY GUARD: Only stream physical packets when Direct Drive is ARMED
    if (this.executionMode !== 'direct_drive' || !this.isArmed) return;
    if (!this.autoSend) return;
    if (!this.serialManager || !this.serialManager.isConnected()) return;

    const isEstop = this.appState ? Boolean(this.appState.getState('safety.estop')) : false;
    if (isEstop) return;

    const dur = customDur || this.LIVE_STREAM_DURATION_MS;
    const now = Date.now();

    if (now - this._lastLiveStreamTime >= this.LIVE_STREAM_INTERVAL_MS) {
      if (this._liveStreamTimer) {
        clearTimeout(this._liveStreamTimer);
        this._liveStreamTimer = null;
      }
      this._lastLiveStreamTime = now;
      this.sendMovSpline({ duration: dur }).catch(() => {});
    } else {
      // Schedule trailing edge execution
      if (!this._liveStreamTimer) {
        const delay = Math.max(10, this.LIVE_STREAM_INTERVAL_MS - (now - this._lastLiveStreamTime));
        this._liveStreamTimer = setTimeout(() => {
          this._liveStreamTimer = null;
          this._lastLiveStreamTime = Date.now();
          this.sendMovSpline({ duration: dur }).catch(() => {});
        }, delay);
      }
    }
  }

  /**
   * Flushes final precise position on slider release or discrete commit
   * @param {number} [duration=null]
   */
  flushFinalTarget(duration = null) {
    if (this._liveStreamTimer) {
      clearTimeout(this._liveStreamTimer);
      this._liveStreamTimer = null;
    }
    const dur = duration || this.duration;
    // TWO-STATE SAFETY GUARD: Only dispatch to hardware when Direct Drive is ARMED
    if (this.executionMode === 'direct_drive' && this.isArmed && this.autoSend && this.serialManager && this.serialManager.isConnected()) {
      const isEstop = this.appState ? Boolean(this.appState.getState('safety.estop')) : false;
      if (!isEstop) {
        this.sendMovSpline({ duration: dur }).catch(() => {});
      }
    }
  }

  // ==========================================================================
  // JOINT MOTION API
  // ==========================================================================

  /**
   * Returns true only when Live Direct Drive is explicitly armed and autoSend is enabled
   * (and not inside a high-frequency hold-jog loop that handles its own 20Hz throttling).
   * @returns {boolean}
   */
  isLiveFollowActive() {
    return Boolean(
      this.autoSend &&
      this.executionMode === 'direct_drive' &&
      this.isArmed &&
      !this._activeHoldJog?.isHeld
    );
  }

  /**
   * Sets a specific joint angle, clamps to soft limits, recalculates FK and updates Ghost pose
   * 
   * @param {'j1'|'j2'|'j3'|'j4'|'j5'|'grip'} joint
   * @param {number} value
   * @param {boolean} [sendToRobot=false]
   * @param {boolean} [isContinuous=true] - True if called from continuous drag input event
   * @returns {Object} Updated joints and FK result
   */
  setJointTarget(joint, value, sendToRobot = false, isContinuous = true) {
    const key = String(joint).toLowerCase();
    if (!(key in JOINT_CONFIGS)) {
      throw new Error(`Invalid joint identifier: "${joint}". Valid: j1, j2, j3, j4, j5, j6, grip`);
    }

    const cfg = JOINT_CONFIGS[key];
    const numVal = Number(value);
    const clampedVal = clamp(isNaN(numVal) ? cfg.default : numVal, cfg.min, cfg.max);

    this.joints[key] = clampedVal;

    // Recalculate Cartesian FK
    this._recomputeFK();
    this._evaluateReachability();
    this._syncToAppState();
    this._updateDomUI();

    if (sendToRobot) {
      this.sendGhostToRobot();
    } else if (this.isLiveFollowActive()) {
      if (isContinuous) {
        this._queueLiveStream();
      } else {
        this.flushFinalTarget();
      }
    }

    return {
      joints: { ...this.joints },
      cartesian: { ...this.cartesian },
      reachability: { ...this.reachability }
    };
  }

  /**
   * Sets all joint targets at once
   * @param {Object} targetAngles - { j1, j2, j3, j4, j5, grip }
   * @param {boolean} [sendToRobot=false]
   */
  setAllJointTargets(targetAngles = {}, sendToRobot = false) {
    for (const [k, cfg] of Object.entries(JOINT_CONFIGS)) {
      if (targetAngles[k] !== undefined) {
        const num = Number(targetAngles[k]);
        this.joints[k] = clamp(isNaN(num) ? cfg.default : num, cfg.min, cfg.max);
      }
    }

    this._recomputeFK();
    this._evaluateReachability();
    this._syncToAppState();
    this._updateDomUI();

    if (sendToRobot || this.isLiveFollowActive()) {
      this.sendGhostToRobot();
    }

    return {
      joints: { ...this.joints },
      cartesian: { ...this.cartesian },
      reachability: { ...this.reachability }
    };
  }

  /**
   * Alias for setJointTarget
   * @param {'j1'|'j2'|'j3'|'j4'|'j5'|'grip'} joint
   * @param {number} angle
   * @param {boolean} [sendToRobot=false]
   */
  setJointAngle(joint, angle, sendToRobot = false) {
    return this.setJointTarget(joint, angle, sendToRobot, false);
  }

  /**
   * Nudges a joint by a given angular delta (degrees)
   * 
   * @param {'j1'|'j2'|'j3'|'j4'|'j5'|'grip'} joint
   * @param {number} delta
   * @param {boolean} [sendToRobot=false]
   */
  nudgeJoint(joint, delta, sendToRobot = false) {
    const key = String(joint).toLowerCase();
    const current = this.joints[key] ?? 0;
    const dur = Math.max(250, Math.min(800, Math.round(Math.abs(delta) * 80)));
    const res = this.setJointTarget(key, current + Number(delta), false, false);
    if (sendToRobot || this.isLiveFollowActive()) {
      this.sendMovSpline({ duration: dur }).catch(() => {});
    }
    return res;
  }

  /**
   * Executes a Gripper preset macro (OPEN 180°, CLOSE 0°, HALF 90°)
   * 
   * @param {'OPEN'|'CLOSE'|'HALF'|number} macroOrAngle
   * @param {boolean} [sendToRobot=true]
   */
  setGripperMacro(macroOrAngle, sendToRobot = true) {
    let targetAngle = 0;
    if (typeof macroOrAngle === 'string') {
      const norm = macroOrAngle.toUpperCase().trim();
      if (norm === 'OPEN' || norm === '180') targetAngle = 180;
      else if (norm === 'CLOSE' || norm === '0') targetAngle = 0;
      else if (norm === 'HALF' || norm === '90') targetAngle = 90;
      else targetAngle = Number(macroOrAngle) || 0;
    } else {
      targetAngle = Number(macroOrAngle) || 0;
    }

    this.joints.grip = clamp(targetAngle, 0, 180);
    this._syncToAppState();
    this._updateDomUI();

    if (sendToRobot && this.serialManager && this.serialManager.isConnected()) {
      const isEstop = this.appState ? Boolean(this.appState.getState('safety.estop')) : false;
      if (!isEstop) {
        const cmd = buildGripCommand(this.joints.grip);
        this.serialManager.send(cmd).catch((err) => {
          console.warn('Failed to send gripper command:', err);
        });
        if (this.appState) {
          this.appState.addConsoleLog({ type: 'TX', text: cmd.trim() });
        }
      }
    }

    return this.joints.grip;
  }

  // ==========================================================================
  // CARTESIAN IK MOTION API
  // ==========================================================================

  /**
   * Sets Cartesian target coordinates (mm), solves Analytical IK, evaluates safety, and updates state
   * 
   * @param {number} x - Target X (mm)
   * @param {number} y - Target Y (mm)
   * @param {number} z - Target Z (mm)
   * @param {number} [wristMode=null] - WristMode (0-6)
   * @param {number} [duration=null] - Movement duration in ms
   * @param {boolean} [sendToRobot=false]
   * @param {boolean} [isContinuous=true] - True if continuous dragging
   * @param {Object} [orientation=null] - Optional { roll, pitch, yaw } in degrees
   * @returns {Object} IK solve outcome & Reachability report
   */
  setCartesianTarget(x, y, z, wristMode = null, duration = null, sendToRobot = false, isContinuous = true, orientation = null) {
    this.cartesian.x = Number(x ?? this.cartesian.x);
    this.cartesian.y = Number(y ?? this.cartesian.y);
    this.cartesian.z = Number(z ?? this.cartesian.z);

    if (orientation) {
      if (orientation.roll !== undefined) this.cartesian.roll = Number(orientation.roll);
      if (orientation.pitch !== undefined) this.cartesian.pitch = Number(orientation.pitch);
      if (orientation.yaw !== undefined) this.cartesian.yaw = Number(orientation.yaw);
      this.cartesian.orientationMode = '6dof';
    } else if (wristMode !== null && wristMode !== undefined) {
      this.cartesian.wristMode = clamp(Math.round(Number(wristMode)), 0, 6);
      this.cartesian.orientationMode = 'preset';
    }
    if (duration !== null && duration !== undefined) {
      this.duration = Math.max(1, Math.round(Number(duration)));
    }

    // Solve Analytical IK
    let ik;
    if (this.cartesian.orientationMode === '6dof') {
      ik = solveIK6DOF(
        this.cartesian.x,
        this.cartesian.y,
        this.cartesian.z,
        this.cartesian.roll || 0,
        this.cartesian.pitch || 0,
        this.cartesian.yaw || 0,
        { currentAngles: this.joints, toolOffset: this.toolOffset, workOffset: this.workOffset }
      );
    } else {
      const isManual = this.cartesian.wristMode === WristMode.MANUAL;
      ik = solveIK(this.cartesian.x, this.cartesian.y, this.cartesian.z, {
        wristMode: this.cartesian.wristMode,
        pitch: isManual ? (this.cartesian.pitch ?? this.cartesian.j5Param) : undefined,
        j4Param: isManual ? (this.joints.j4 ?? this.cartesian.j4Param) : this.cartesian.j4Param,
        j5Param: this.cartesian.j5Param,
        j6Param: isManual ? (this.joints.j6 ?? this.cartesian.j6Param) : this.cartesian.j6Param,
        currentAngles: { j2: this.joints.j2, j3: this.joints.j3 },
        toolOffset: this.toolOffset,
        workOffset: this.workOffset
      });
      if (ik && !isNaN(ik.j1) && !isNaN(ik.j2) && !isNaN(ik.j3) && !isNaN(ik.j4) && !isNaN(ik.j5) && !isNaN(ik.j6)) {
        const fk = forwardKinematics6DOF(ik.j1, ik.j2, ik.j3, ik.j4, ik.j5, ik.j6, {
          toolOffset: this.toolOffset,
          workOffset: this.workOffset
        });
        this.cartesian.roll = Number(fk.roll.toFixed(1));
        this.cartesian.pitch = Number(fk.pitch.toFixed(1));
        this.cartesian.yaw = Number(fk.yaw.toFixed(1));
      }
    }

    // Always update joint angles to the IK solution (which contains closest valid / clamped angles)
    if (ik) {
      this.joints.j1 = ik.j1;
      this.joints.j2 = ik.j2;
      this.joints.j3 = ik.j3;
      this.joints.j4 = ik.j4;
      this.joints.j5 = ik.j5;
      this.joints.j6 = ik.j6 !== undefined ? ik.j6 : this.joints.j6;
    }

    // Workspace and safety check
    this._evaluateReachability(ik);
    this._syncToAppState();
    this._updateDomUI();

    if (sendToRobot) {
      this.sendMovSpline();
    } else if (this.isLiveFollowActive()) {
      if (isContinuous) {
        this._queueLiveStream();
      } else {
        this.flushFinalTarget();
      }
    }

    return {
      success: ik ? ik.success : false,
      isReachable: this.reachability.isReachable,
      ik,
      joints: { ...this.joints },
      cartesian: { ...this.cartesian },
      reachability: { ...this.reachability }
    };
  }

  /**
   * Nudges a coordinate axis by a delta.
   * In Cartesian mode (X/Y/Z): jogs X, Y, Z in mm.
   * In Cylindrical mode (R/theta/Z):
   *   - axis 'x' jogs Radius R (mm) radially along current angle
   *   - axis 'y' jogs Polar Angle theta (degrees) in rotation around base
   *   - axis 'z' jogs Height Z (mm) vertically
   * 
   * @param {'x'|'y'|'z'} axis
   * @param {number} delta - Delta in mm (or degrees for theta in cylindrical mode)
   * @param {boolean} [sendToRobot=false]
   */
  nudgeCartesian(axis, delta, sendToRobot = false) {
    const key = String(axis).toLowerCase();
    if (!['x', 'y', 'z'].includes(key)) {
      throw new Error(`Invalid cartesian axis: "${axis}". Valid: x, y, z`);
    }

    let targetX, targetY, targetZ;
    const dur = Math.max(300, Math.min(1000, Math.round(Math.abs(delta) * 20)));

    if (!this.isCylindricalMode) {
      const curr = this.cartesian[key];
      const newTarget = {
        x: this.cartesian.x,
        y: this.cartesian.y,
        z: this.cartesian.z
      };
      if (key === 'z') {
        newTarget.z = clamp(curr + Number(delta), CARTESIAN_CONFIGS.z.min, CARTESIAN_CONFIGS.z.max);
      } else {
        newTarget[key] = clamp(curr + Number(delta), CARTESIAN_CONFIGS[key].min, CARTESIAN_CONFIGS[key].max);
      }
      targetX = newTarget.x;
      targetY = newTarget.y;
      targetZ = newTarget.z;
    } else {
      // Cylindrical Mode: X = Reach R (mm), Y = Angle theta (deg), Z = Height Z (mm)
      const currentX = this.cartesian.x;
      const currentY = this.cartesian.y;
      const currentZ = this.cartesian.z;

      const currentR = Math.hypot(currentX, currentY);
      const currentThetaRad = Math.atan2(currentY, currentX);

      if (key === 'x') {
        // Nudge Reach R (mm) along current radial angle
        const newR = clamp(currentR + Number(delta), CARTESIAN_CONFIGS.r.min, CARTESIAN_CONFIGS.r.max);
        targetX = newR * Math.cos(currentThetaRad);
        targetY = newR * Math.sin(currentThetaRad);
        targetZ = currentZ;
      } else if (key === 'y') {
        // Nudge Polar Angle theta (degrees) in pure rotation around base clamped to base limits [-90, 90]
        const currentThetaDeg = (currentThetaRad * 180) / Math.PI;
        const newThetaDeg = clamp(currentThetaDeg + Number(delta), JOINT_CONFIGS.j1.min, JOINT_CONFIGS.j1.max);
        const newThetaRad = (newThetaDeg * Math.PI) / 180;
        const r = currentR === 0 ? 100 : currentR;
        targetX = r * Math.cos(newThetaRad);
        targetY = r * Math.sin(newThetaRad);
        targetZ = currentZ;
      } else {
        // Nudge Height Z (mm) vertically
        targetX = currentX;
        targetY = currentY;
        targetZ = clamp(currentZ + Number(delta), CARTESIAN_CONFIGS.z.min, CARTESIAN_CONFIGS.z.max);
      }
    }

    const targetWristMode = this.cartesian.orientationMode === 'preset' ? this.cartesian.wristMode : null;
    const res = this.setCartesianTarget(
      targetX,
      targetY,
      targetZ,
      targetWristMode,
      null,
      false,
      false
    );
    if (sendToRobot || this.isLiveFollowActive()) {
      this.sendMovSpline({ duration: dur }).catch(() => {});
    }
    return res;
  }

  /**
   * Nudges a tool orientation axis (Roll, Pitch, Yaw) by a delta in degrees
   * 
   * @param {'roll'|'pitch'|'yaw'} axis
   * @param {number} delta - Delta in degrees
   * @param {boolean} [sendToRobot=false]
   */
  nudgeOrientation(axis, delta, sendToRobot = false) {
    const key = String(axis).toLowerCase();
    if (!['roll', 'pitch', 'yaw'].includes(key)) {
      throw new Error(`Invalid orientation axis: "${axis}". Valid: roll, pitch, yaw`);
    }

    const currentVal = this.cartesian[key] || 0;
    const minVal = key === 'pitch' ? -90 : -180;
    const maxVal = key === 'pitch' ? 90 : 180;
    const newVal = clamp(currentVal + Number(delta), minVal, maxVal);

    this.cartesian[key] = newVal;
    this.cartesian.orientationMode = '6dof';

    const res = this.setCartesianTarget(
      this.cartesian.x,
      this.cartesian.y,
      this.cartesian.z,
      null,
      null,
      false,
      false,
      { [key]: newVal }
    );

    if (sendToRobot || this.isLiveFollowActive()) {
      this.sendMovSpline({ duration: 500 }).catch(() => {});
    }
    return res;
  }

  /**
   * Sets a tool orientation axis (Roll, Pitch, Yaw) in degrees
   * 
   * @param {'roll'|'pitch'|'yaw'} axis
   * @param {number} val - Angle in degrees
   * @param {boolean} [isContinuous=true]
   * @param {boolean} [sendToRobot=false]
   */
  setOrientationTarget(axis, val, isContinuous = true, sendToRobot = false) {
    const key = String(axis).toLowerCase();
    if (!['roll', 'pitch', 'yaw'].includes(key)) {
      throw new Error(`Invalid orientation axis: "${axis}". Valid: roll, pitch, yaw`);
    }

    const minVal = key === 'pitch' ? -90 : -180;
    const maxVal = key === 'pitch' ? 90 : 180;
    const newVal = clamp(Number(val), minVal, maxVal);

    this.cartesian[key] = newVal;
    this.cartesian.orientationMode = '6dof';

    const res = this.setCartesianTarget(
      this.cartesian.x,
      this.cartesian.y,
      this.cartesian.z,
      null,
      null,
      sendToRobot,
      isContinuous,
      { [key]: newVal }
    );
    return res;
  }

  /**
   * Sets orientation control mode ('6dof' or 'preset')
   * @param {'6dof'|'preset'} mode
   */
  setOrientationMode(mode) {
    this.cartesian.orientationMode = mode === 'preset' ? 'preset' : '6dof';
    if (this.cartesian.orientationMode === '6dof') {
      this._recomputeFK();
    } else {
      this.setWristMode(this.cartesian.wristMode);
    }
    this._evaluateReachability();
    this._syncToAppState();
    this._updateDomUI();
    return this.cartesian.orientationMode;
  }

  /**
   * Sets backlash compensation for a given joint (j1..j6) in degrees
   * @param {'j1'|'j2'|'j3'|'j4'|'j5'|'j6'} joint
   * @param {number} deg
   */
  setBacklash(joint, deg) {
    const j = String(joint).toLowerCase();
    if (this.backlash[j] !== undefined) {
      this.backlash[j] = clamp(Number(deg) || 0, 0, 5.0);
      this._updateDomUI();
    }
  }

  /**
   * Queries backlash compensation settings from robot firmware
   */
  async queryBacklash() {
    const cmd = buildBacklashCommand();
    if (this.serialManager && this.serialManager.isConnected()) {
      await this.serialManager.send(cmd);
      if (this.appState) {
        this.appState.addConsoleLog({ type: 'TX', text: cmd.trim() });
      }
    } else {
      if (this.appState) {
        this.appState.addConsoleLog({ type: 'INFO', text: `[OFFLINE] Backlash Query: ${cmd.trim()}` });
      }
    }
    return { success: true, command: cmd };
  }

  /**
   * Saves current backlash compensation settings (J1..J6) to robot firmware / EEPROM
   */
  async saveBacklash() {
    const commands = [];
    for (let i = 1; i <= 6; i++) {
      const val = this.backlash[`j${i}`] ?? 0;
      const cmd = buildBacklashCommand(i, val);
      commands.push(cmd);
      if (this.serialManager && this.serialManager.isConnected()) {
        await this.serialManager.send(cmd);
      }
    }
    if (this.appState) {
      this.appState.addConsoleLog({
        type: this.serialManager && this.serialManager.isConnected() ? 'TX' : 'INFO',
        text: `Backlash saved: J1=${this.backlash.j1}°, J2=${this.backlash.j2}°, J3=${this.backlash.j3}°, J4=${this.backlash.j4}°, J5=${this.backlash.j5}°, J6=${this.backlash.j6}°`
      });
    }
    return { success: true, commands };
  }

  /**
   * Sets the active step size for Cartesian jogging (mm)
   * @param {number} stepMm - Step size (0.1, 1.0, 10.0, 50.0)
   * @returns {number}
   */
  setCartesianStepSize(stepMm) {
    const val = Math.max(0.01, Number(stepMm) || 10.0);
    this.cartesianStepSize = val;
    this._updateStepJogUI('cartesian');
    return this.cartesianStepSize;
  }

  /**
   * Sets the active step size for Joint angle jogging (degrees)
   * @param {number} stepDeg - Step size (0.1, 1.0, 5.0, 10.0, 45.0)
   * @returns {number}
   */
  setJointStepSize(stepDeg) {
    const val = Math.max(0.01, Number(stepDeg) || 1.0);
    this.jointStepSize = val;
    this._updateStepJogUI('joint');
    return this.jointStepSize;
  }

  /**
   * Updates DOM labels, titles, data-step attributes, and active pill states for step joggers
   * @param {'cartesian'|'joint'|'all'} [type='all']
   * @private
   */
  _updateStepJogUI(type = 'all') {
    if (!this.container) return;

    if (type === 'cartesian' || type === 'all') {
      const pills = this.container.querySelectorAll('[data-cart-step]');
      pills.forEach((p) => {
        const pVal = parseFloat(p.getAttribute('data-cart-step'));
        if (p.classList) {
          p.classList.toggle('active', Math.abs(pVal - this.cartesianStepSize) < 1e-4);
        }
      });

      const jogBtns = this.container.querySelectorAll('[data-cartesian-nudge]');
      jogBtns.forEach((btn) => {
        const dir = parseFloat(btn.getAttribute('data-jog-dir') || '0');
        if (dir !== 0) {
          const sign = dir < 0 ? '−' : '+';
          const stepText = (this.cartesianStepSize < 1) ? this.cartesianStepSize.toFixed(1) : String(this.cartesianStepSize);
          const axis = btn.getAttribute('data-cartesian-nudge') || 'x';
          const isTheta = this.isCylindricalMode && axis === 'y';
          btn.setAttribute('data-step', String(dir * this.cartesianStepSize));

          // Trap 4: Do NOT overwrite text on keypad buttons (+X, -X, etc.)
          if (!btn.classList || !btn.classList.contains('btn-keypad')) {
            btn.textContent = `${sign}${stepText}${isTheta ? '°' : ''}`;
          }

          let axisName = axis.toUpperCase();
          let unit = 'mm';
          if (this.isCylindricalMode) {
            if (axis === 'x') { axisName = 'R'; unit = 'mm'; }
            else if (axis === 'y') { axisName = 'θ'; unit = '°'; }
            else if (axis === 'z') { axisName = 'Z'; unit = 'mm'; }
          }
          const shortcut = axis === 'x' ? (dir > 0 ? ' [W]' : ' [S]') : axis === 'y' ? (dir > 0 ? ' [A]' : ' [D]') : (dir > 0 ? ' [▲]' : ' [▼]');
          btn.title = `Jog ${axisName} ${sign}${stepText}${unit}${shortcut}`;
        }
      });

      const keypadHint = (this.container && typeof this.container.querySelector === 'function')
        ? this.container.querySelector('#keypad_active_step_hint')
        : null;
      if (keypadHint) {
        keypadHint.textContent = `±${this.cartesianStepSize}mm`;
      }
    }

    if (type === 'joint' || type === 'all') {
      const pills = this.container.querySelectorAll('[data-joint-step]');
      pills.forEach((p) => {
        const pVal = parseFloat(p.getAttribute('data-joint-step'));
        if (p.classList) {
          p.classList.toggle('active', Math.abs(pVal - this.jointStepSize) < 1e-4);
        }
      });

      const jogBtns = this.container.querySelectorAll('[data-nudge-joint]');
      jogBtns.forEach((btn) => {
        const dir = parseFloat(btn.getAttribute('data-jog-dir') || '0');
        if (dir !== 0) {
          const sign = dir < 0 ? '−' : '+';
          const stepText = (this.jointStepSize < 1) ? this.jointStepSize.toFixed(1) : String(this.jointStepSize);
          btn.setAttribute('data-step', String(dir * this.jointStepSize));

          btn.textContent = `${sign}${stepText}°`;

          const joint = btn.getAttribute('data-nudge-joint') || 'j1';
          btn.title = `Jog ${joint.toUpperCase()} ${sign}${stepText}° [Hold to accelerate]`;
        }
      });
    }
  }

  /**
   * Sets whether coordinate system is in Cylindrical mode (R, theta, Z) or Cartesian (X, Y, Z)
   * @param {boolean} enabled
   * @returns {boolean}
   */
  setCylindricalMode(enabled) {
    this.isCylindricalMode = Boolean(enabled);
    this._updateDomUI();
    this._updateStepJogUI('cartesian');
    return this.isCylindricalMode;
  }

  /**
   * Handles user input from slider or number field in Cartesian (X,Y,Z) or Cylindrical (R,theta,Z) mode
   * @param {'x'|'y'|'z'} axis
   * @param {number} val
   * @param {boolean} [isContinuous=true]
   * @private
   */
  _handleCartesianInput(axis, val, isContinuous = true) {
    if (isNaN(val)) return;

    if (!this.isCylindricalMode) {
      const newTarget = {
        x: this.cartesian.x,
        y: this.cartesian.y,
        z: this.cartesian.z
      };
      if (axis === 'z') {
        newTarget.z = clamp(Number(val), CARTESIAN_CONFIGS.z.min, CARTESIAN_CONFIGS.z.max);
      } else {
        newTarget[axis] = clamp(Number(val), CARTESIAN_CONFIGS[axis].min, CARTESIAN_CONFIGS[axis].max);
      }
      this.setCartesianTarget(newTarget.x, newTarget.y, newTarget.z, null, null, false, isContinuous);
      return;
    }

    // Cylindrical Mode (R, theta, Z)
    const currentX = this.cartesian.x;
    const currentY = this.cartesian.y;
    const currentZ = this.cartesian.z;

    const currentR = Math.hypot(currentX, currentY);
    const currentThetaRad = Math.atan2(currentY, currentX);

    if (axis === 'x') {
      // Axis X slot = Reach (R in mm)
      const newR = clamp(Number(val), CARTESIAN_CONFIGS.r.min, CARTESIAN_CONFIGS.r.max);
      const newX = newR * Math.cos(currentThetaRad);
      const newY = newR * Math.sin(currentThetaRad);
      this.setCartesianTarget(newX, newY, currentZ, null, null, false, isContinuous);
    } else if (axis === 'y') {
      // Axis Y slot = Rotation (theta in degrees) clamped to physical base limits
      const newThetaDeg = clamp(Number(val), JOINT_CONFIGS.j1.min, JOINT_CONFIGS.j1.max);
      const newThetaRad = (newThetaDeg * Math.PI) / 180;
      const r = currentR === 0 ? 100 : currentR;
      const newX = r * Math.cos(newThetaRad);
      const newY = r * Math.sin(newThetaRad);
      this.setCartesianTarget(newX, newY, currentZ, null, null, false, isContinuous);
    } else if (axis === 'z') {
      // Axis Z slot = Height (Z in mm)
      this.setCartesianTarget(currentX, currentY, clamp(Number(val), CARTESIAN_CONFIGS.z.min, CARTESIAN_CONFIGS.z.max), null, null, false, isContinuous);
    }
  }

  /**
   * Updates the Wrist Mode (W0-W6) and optional manual parameters
   * 
   * @param {number} mode - WristMode constant (0-6)
   * @param {number} [j4Param=null]
   * @param {number} [j5Param=null]
   * @param {number} [j6Param=null]
   */
  setWristMode(mode, j4Param = null, j5Param = null, j6Param = null) {
    this.cartesian.wristMode = clamp(Math.round(Number(mode)), 0, 6);
    this.cartesian.orientationMode = 'preset';
    if (j4Param !== null && j4Param !== undefined) {
      this.cartesian.j4Param = clamp(Number(j4Param), -90, 90);
    }
    if (j5Param !== null && j5Param !== undefined) {
      this.cartesian.j5Param = clamp(Number(j5Param), -90, 90);
    }
    if (j6Param !== null && j6Param !== undefined) {
      this.cartesian.j6Param = clamp(Number(j6Param), -180, 180);
    }

    return this.setCartesianTarget(
      this.cartesian.x,
      this.cartesian.y,
      this.cartesian.z,
      this.cartesian.wristMode
    );
  }

  /**
   * Cycles through Wrist Modes (W0 to W6)
   * 
   * @param {number} [direction=1] - (+1 for next mode, -1 for previous mode)
   * @returns {Object} Result of setCartesianTarget
   */
  cycleWristMode(direction = 1) {
    const currentMode = Number(this.cartesian.wristMode ?? 0);
    const newMode = (currentMode + (direction > 0 ? 1 : 6)) % 7;
    return this.setWristMode(newMode);
  }

  /**
   * Jogs Cartesian position or cycles wrist mode via keyboard (WSAD, Arrow Up/Down, Arrow Left/Right)
   * 
   * @param {string} key - 'w', 's', 'a', 'd', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'
   * @param {number} [step=5] - Step size in mm
   * @param {boolean} [sendToRobot=false]
   * @returns {Object|null}
   */
  jogCartesianKeyboard(key, step = null, sendToRobot = false) {
    const k = String(key).toLowerCase();
    const effectiveStep = (typeof step === 'number' && !isNaN(step)) ? step : this.cartesianStepSize;
    if (k === 'w') {
      return this.nudgeCartesian('x', effectiveStep, sendToRobot);
    } else if (k === 's') {
      return this.nudgeCartesian('x', -effectiveStep, sendToRobot);
    } else if (k === 'a') {
      return this.nudgeCartesian('y', effectiveStep, sendToRobot);
    } else if (k === 'd') {
      return this.nudgeCartesian('y', -effectiveStep, sendToRobot);
    } else if (k === 'arrowup' || key === 'ArrowUp') {
      return this.nudgeCartesian('z', effectiveStep, sendToRobot);
    } else if (k === 'arrowdown' || key === 'ArrowDown') {
      return this.nudgeCartesian('z', -effectiveStep, sendToRobot);
    } else if (k === 'arrowleft' || key === 'ArrowLeft') {
      return this.cycleWristMode(-1);
    } else if (k === 'arrowright' || key === 'ArrowRight') {
      return this.cycleWristMode(1);
    }
    return null;
  }

  /**
   * Sets trajectory motion duration (ms)
   * @param {number} ms
   */
  setDuration(ms) {
    this.duration = Math.max(1, Math.round(Number(ms)));
    this._syncToAppState();
    this._updateDomUI();
    return this.duration;
  }

  // ==========================================================================
  // COMMAND EXECUTION & SERIAL TRANSMISSION
  // ==========================================================================

  /**
   * Dispatches a Spline Quintic Move: sends framed CRC-8 packet `<j1,j2,j3,j4,j5,dx,dy,dur,crc>`
   * 
   * @param {Object} [overrideParams] - Optional duration or angle overrides
   * @returns {Promise<{ success: boolean, packet: string }>}
   */
  async sendMovSpline(overrideParams = {}) {
    const isEstop = this.appState ? Boolean(this.appState.getState('safety.estop')) : false;
    if (isEstop) {
      throw new Error('Cannot execute motion: EMERGENCY STOP is active');
    }

    const durParam = overrideParams.duration ?? overrideParams.dur ?? this.duration;
    const dur = Math.max(0, Math.round(durParam));
    const j1 = overrideParams.j1 ?? this.joints.j1;
    const j2 = overrideParams.j2 ?? this.joints.j2;
    const j3 = overrideParams.j3 ?? this.joints.j3;
    const j4 = overrideParams.j4 ?? this.joints.j4;
    const j5 = overrideParams.j5 ?? this.joints.j5;
    const j6 = overrideParams.j6 ?? this.joints.j6;
    const dx = overrideParams.dx ?? this.dx;
    const dy = overrideParams.dy ?? this.dy;

    // Safety workspace verification
    const ws = checkWorkspace(j1, j2, j3, j4, j5, j6, { toolOffset: this.toolOffset, workOffset: this.workOffset });
    if (!ws.safe) {
      const err = new Error(`Motion rejected by Workspace Safety: ${ws.violations.join(', ')}`);
      err.violations = ws.violations;
      if (this.appState) {
        this.appState.addConsoleLog({ type: 'WARN', text: `SPLINE MOVE BLOCKED: ${ws.violations.join(', ')}` });
      }
      throw err;
    }

    // Build framed CRC-8 packet
    const packet = buildFramedPacket(j1, j2, j3, j4, j5, j6, dx, dy, dur);

    if (this.serialManager && this.serialManager.isConnected()) {
      await this.serialManager.send(packet + '\n');
      if (this.appState) {
        this.appState.addConsoleLog({ type: 'TX', text: packet });
      }
    } else {
      if (this.appState) {
        this.appState.addConsoleLog({ type: 'WARN', text: `[OFFLINE] Built packet: ${packet}` });
      }
    }

    return { success: true, packet };
  }

  /**
   * Dispatches a Linear Cartesian Move: sends ASCII `MOVL X# Y# Z# W# J4# J5# D#\n`
   * 
   * @param {Object} [overrideParams]
   * @returns {Promise<{ success: boolean, command: string }>}
   */
  async sendMovLinear(overrideParams = {}) {
    const isEstop = this.appState ? Boolean(this.appState.getState('safety.estop')) : false;
    if (isEstop) {
      throw new Error('Cannot execute motion: EMERGENCY STOP is active');
    }

    const x = Number(overrideParams.x ?? this.cartesian.x);
    const y = Number(overrideParams.y ?? this.cartesian.y);
    const z = Number(overrideParams.z ?? this.cartesian.z);
    const dur = Math.max(1, Math.round(overrideParams.duration || overrideParams.dur || this.duration));

    let cmd;
    if (this.cartesian.orientationMode === '6dof' || overrideParams.roll !== undefined || overrideParams.pitch !== undefined || overrideParams.yaw !== undefined) {
      const roll = Number(overrideParams.roll ?? this.cartesian.roll ?? 0);
      const pitch = Number(overrideParams.pitch ?? this.cartesian.pitch ?? 0);
      const yaw = Number(overrideParams.yaw ?? this.cartesian.yaw ?? 0);

      const ik = solveIK6DOF(x, y, z, roll, pitch, yaw, { currentAngles: this.joints, toolOffset: this.toolOffset, workOffset: this.workOffset });
      if (!ik.success || ik.clamped) {
        throw new Error(`Linear move target unreachable: ${ik.warnings.join(', ')}`);
      }
      cmd = buildMovOrientationCommand(x, y, z, roll, pitch, yaw, dur, true);
      const start = forwardKinematics6DOF(this.joints.j1, this.joints.j2, this.joints.j3, this.joints.j4, this.joints.j5, this.joints.j6, { toolOffset: this.toolOffset, workOffset: this.workOffset });
      const q0 = eulerToQuat(start.roll, start.pitch, start.yaw);
      const q1 = eulerToQuat(roll, pitch, yaw);
      this.linearPreview = Array.from({ length: 21 }, (_, index) => {
        const t = index / 20;
        return {
          x: start.tcp.x + (x - start.tcp.x) * t,
          y: start.tcp.y + (y - start.tcp.y) * t,
          z: start.tcp.z + (z - start.tcp.z) * t,
          ...quatToEuler(quatSlerp(q0, q1, t))
        };
      });
    } else {
      const w = Number(overrideParams.wristMode ?? this.cartesian.wristMode);
      const j4 = Number(overrideParams.j4 ?? this.cartesian.j4Param);
      const j5 = Number(overrideParams.j5 ?? this.cartesian.j5Param);
      const j6 = Number(overrideParams.j6 ?? this.cartesian.j6Param ?? this.joints?.j6 ?? 0);

      // Analytical IK check for linear start/end point
      const ik = solveIK(x, y, z, { wristMode: w, j4Param: j4, j5Param: j5, j6Param: j6 });
      if (!ik.success || ik.clamped) {
        throw new Error(`Linear move target unreachable: ${ik.warnings.join(', ')}`);
      }

      cmd = buildMovLCommand(x, y, z, w, j4, j5, j6, dur);
    }

    if (this.serialManager && this.serialManager.isConnected()) {
      await this.serialManager.send(cmd);
      if (this.appState) {
        this.appState.addConsoleLog({ type: 'TX', text: cmd.trim() });
      }
    } else {
      if (this.appState) {
        this.appState.addConsoleLog({ type: 'WARN', text: `[OFFLINE] Built command: ${cmd.trim()}` });
      }
    }

    return { success: true, command: cmd };
  }

  // ==========================================================================
  // LIVE VS GHOST SYNCHRONIZATION
  // ==========================================================================

  /**
   * Copies current Live Pose telemetry from AppState into Ghost Target Pose
   * @param {boolean} [updateUI=true]
   */
  setGhostToLive(updateUI = true) {
    if (!this.appState) return;

    const live = this.appState.getState('livePose');
    if (!live) return;

    this.joints.j1 = Number(live.j1 ?? this.joints.j1);
    this.joints.j2 = Number(live.j2 ?? this.joints.j2);
    this.joints.j3 = Number(live.j3 ?? this.joints.j3);
    this.joints.j4 = Number(live.j4 ?? this.joints.j4);
    this.joints.j5 = Number(live.j5 ?? this.joints.j5);
    this.joints.j6 = Number(live.j6 ?? this.joints.j6);
    this.joints.grip = Number(live.grip ?? this.joints.grip);
    this.dx = Number(live.dx ?? 0);
    this.dy = Number(live.dy ?? 0);

    this._recomputeFK();
    this._evaluateReachability();
    this._syncToAppState();

    if (updateUI) {
      this._updateDomUI();
    }

    if (this.appState) {
      this.appState.addConsoleLog({ type: 'INFO', text: 'Ghost Target Pose synchronized with Live Robot Pose' });
    }
  }

  /**
   * Sends the current Ghost Target Pose directly to physical robot
   */
  sendGhostToRobot() {
    return this.sendMovSpline();
  }

  /**
   * Toggles continuous auto-sync from live robot to ghost target
   * @param {boolean} enable
   */
  setAutoSync(enable) {
    this._autoSyncWithLive = Boolean(enable);
    if (this._autoSyncWithLive) {
      this.setGhostToLive(true);
    }
  }

  // ==========================================================================
  // INTERNAL CALCULATIONS & STATE HELPERS
  // ==========================================================================

  /**
   * Recomputes Forward Kinematics for current joint values
   * @private
   */
  _recomputeFK() {
    const fk = forwardKinematics6DOF(
      this.joints.j1,
      this.joints.j2,
      this.joints.j3,
      this.joints.j4,
      this.joints.j5,
      this.joints.j6,
      { toolOffset: this.toolOffset, workOffset: this.workOffset }
    );

    this.cartesian.x = Number(fk.tcp.x.toFixed(1));
    this.cartesian.y = Number(fk.tcp.y.toFixed(1));
    this.cartesian.z = Number(fk.tcp.z.toFixed(1));
    this.cartesian.roll = Number(fk.roll.toFixed(1));
    this.cartesian.pitch = Number(fk.pitch.toFixed(1));
    this.cartesian.yaw = Number(fk.yaw.toFixed(1));
    this._lastFk = fk;
  }

  /**
   * Evaluates Reachability and Workspace constraints
   * @private
   */
  _evaluateReachability(cachedIk = null) {
    let ik = cachedIk;
    if (!ik) {
      if (this.cartesian.orientationMode === '6dof') {
        ik = solveIK6DOF(
          this.cartesian.x,
          this.cartesian.y,
          this.cartesian.z,
          this.cartesian.roll || 0,
          this.cartesian.pitch || 0,
          this.cartesian.yaw || 0,
          { currentAngles: this.joints, toolOffset: this.toolOffset, workOffset: this.workOffset }
        );
      } else {
        const isManual = this.cartesian.wristMode === WristMode.MANUAL;
        ik = solveIK(this.cartesian.x, this.cartesian.y, this.cartesian.z, {
          wristMode: this.cartesian.wristMode,
          pitch: isManual ? (this.cartesian.pitch ?? this.cartesian.j5Param) : undefined,
          j4Param: isManual ? (this.joints.j4 ?? this.cartesian.j4Param) : this.cartesian.j4Param,
          j5Param: this.cartesian.j5Param,
          j6Param: isManual ? (this.joints.j6 ?? this.cartesian.j6Param) : this.cartesian.j6Param,
          currentAngles: { j2: this.joints.j2, j3: this.joints.j3 },
          toolOffset: this.toolOffset,
          workOffset: this.workOffset
        });
      }
    }

    const ws = checkWorkspace(
      this.joints.j1,
      this.joints.j2,
      this.joints.j3,
      this.joints.j4,
      this.joints.j5,
      this.joints.j6,
      { toolOffset: this.toolOffset, workOffset: this.workOffset }
    );

    const warnings = [...(ik.warnings || [])];
    for (const v of ws.violations) {
      if (!warnings.includes(v)) warnings.push(v);
    }

    const isReachable = ik.success && !ik.clamped && ws.safe;
    let statusText = 'VALID (Reachable)';

    if (ik.clamped || !ws.safe) {
      statusText = warnings.length > 0 ? `WARNING: ${warnings.join(', ')}` : 'CLAMPED';
    } else if (!ik.success) {
      statusText = 'OUT OF REACH';
    }

    this.reachability = {
      isReachable,
      safe: ws.safe,
      clamped: ik.clamped,
      warnings,
      violations: ws.violations,
      ikResult: ik,
      workspaceResult: ws,
      statusText
    };
  }

  /**
   * Pushes Ghost Pose updates to central AppState
   * @private
   */
  _syncToAppState() {
    if (!this.appState) return;

    this.appState.updateGhostPose({
      j1: this.joints.j1,
      j2: this.joints.j2,
      j3: this.joints.j3,
      j4: this.joints.j4,
      j5: this.joints.j5,
      j6: this.joints.j6,
      grip: this.joints.grip,
      x: this.cartesian.x,
      y: this.cartesian.y,
      z: this.cartesian.z,
      wristMode: this.cartesian.wristMode,
      duration: this.duration,
      isReachable: this.reachability.isReachable,
      warnings: this.reachability.warnings
    });

  }

  // ==========================================================================
  // DOM BINDING & UI EVENT HANDLING
  // ==========================================================================

  /**
   * Binds UI elements inside a container element to this controller
   * @param {HTMLElement} container
   */
  bindDom(container) {
    if (!container || typeof document === 'undefined') return;

    this.unbindDom();
    this.container = container;
    this._domBound = true;

    // 1. Joint Sliders and Number Inputs (J1..J6; gripper handled by _bindPersistentGripper)
    for (const joint of ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']) {
      const slider = container.querySelector(`[data-joint-slider="${joint}"], #slider_${joint}`);
      const valInput = container.querySelector(`[data-joint-val="${joint}"], #val_${joint}`);
      const liveMarker = container.querySelector(`[data-joint-live-marker="${joint}"]`);
      const deltaSpan = container.querySelector(`[data-joint-delta-span="${joint}"]`);
      const deltaBadge = container.querySelector(`[data-joint-delta-badge="${joint}"]`);
      const cardRow = container.querySelector(`[data-joint-card="${joint}"]`);

      if (liveMarker) this._domElements[`live_marker_${joint}`] = liveMarker;
      if (deltaSpan) this._domElements[`delta_span_${joint}`] = deltaSpan;
      if (deltaBadge) this._domElements[`delta_badge_${joint}`] = deltaBadge;

      if (cardRow) {
        this._domElements[`card_row_${joint}`] = cardRow;
        const onEnter = () => {
          cardRow.classList.add('is-highlighted');
          this.setHighlightedJoint(joint);
        };
        const onLeave = () => {
          cardRow.classList.remove('is-highlighted');
          if (this._highlightedJoint === joint) {
            this.setHighlightedJoint(null);
          }
        };
        cardRow.addEventListener('pointerenter', onEnter);
        cardRow.addEventListener('pointerleave', onLeave);
        cardRow.addEventListener('focusin', onEnter);
        cardRow.addEventListener('focusout', onLeave);
        this._eventCleanups.push(() => {
          cardRow.removeEventListener('pointerenter', onEnter);
          cardRow.removeEventListener('pointerleave', onLeave);
          cardRow.removeEventListener('focusin', onEnter);
          cardRow.removeEventListener('focusout', onLeave);
        });
      }

      if (slider) {
        this._domElements[`slider_${joint}`] = slider;
        slider.min = JOINT_CONFIGS[joint].min;
        slider.max = JOINT_CONFIGS[joint].max;
        slider.step = JOINT_CONFIGS[joint].step;
        slider.value = this.joints[joint];

        const onSliderInput = (e) => {
          this.setJointTarget(joint, parseFloat(e.target.value), false, true);
        };
        slider.addEventListener('input', onSliderInput);
        this._eventCleanups.push(() => slider.removeEventListener('input', onSliderInput));

        const onSliderChange = (e) => {
          this.setJointTarget(joint, parseFloat(e.target.value), false, false);
          this.flushFinalTarget();
        };
        slider.addEventListener('change', onSliderChange);
        this._eventCleanups.push(() => slider.removeEventListener('change', onSliderChange));
      }

      if (valInput) {
        this._domElements[`val_${joint}`] = valInput;
        valInput.value = this.joints[joint].toFixed(1);
        this._bindInlineBadgeInput(joint, valInput);
      }
    }

    // 2. Fine-step Joint Nudge & Unified Jog Buttons (Tap Step + Hold-Jog Acceleration)
    const nudgeBtns = container.querySelectorAll('[data-nudge-joint]');
    nudgeBtns.forEach((btn) => {
      const joint = btn.getAttribute('data-nudge-joint');
      const dir = parseFloat(btn.getAttribute('data-jog-dir') || '0');
      this._bindJogRamping(btn, 'joint', joint, dir);
    });

    // 2.1 Joint Step Pill Selector
    const jointStepPills = container.querySelectorAll('[data-joint-step]');
    jointStepPills.forEach((pill) => {
      const stepVal = parseFloat(pill.getAttribute('data-joint-step'));
      const onPillClick = () => {
        this.setJointStepSize(stepVal);
      };
      pill.addEventListener('click', onPillClick);
      this._eventCleanups.push(() => pill.removeEventListener('click', onPillClick));
    });

    // 3.1 Live Control / AutoSend Toggles (Synchronized across all panels)
    const autoSendToggles = container.querySelectorAll('#check_joints_autosend, #check_ik_autosend, [data-auto-send-toggle]');
    autoSendToggles.forEach((toggle) => {
      toggle.checked = Boolean(this.autoSend);
      const onAutoSendToggle = (e) => {
        this.autoSend = Boolean(e.target.checked);
        autoSendToggles.forEach((t) => {
          if (t !== e.target) t.checked = this.autoSend;
        });
      };
      toggle.addEventListener('change', onAutoSendToggle);
      this._eventCleanups.push(() => toggle.removeEventListener('change', onAutoSendToggle));
    });

    // 3.2 Send Joints to Robot & Sync Live Buttons
    const root = (container?.closest && container.closest('#left-panel')) || container;
    const queryEl = (sel) => {
      return (root.querySelector ? root.querySelector(sel) : null) ||
        (container.querySelector ? container.querySelector(sel) : null) ||
        (typeof document !== 'undefined' ? document.querySelector(sel) : null);
    };

    const btnSendJoints = queryEl('#btn_send_joints_robot, [data-action="send-joints-robot"]');
    if (btnSendJoints) {
      const onSendJoints = () => {
        this.sendGhostToRobot();
      };
      btnSendJoints.addEventListener('click', onSendJoints);
      this._eventCleanups.push(() => btnSendJoints.removeEventListener('click', onSendJoints));
    }

    const btnSyncLiveJoints = queryEl('#btn_sync_live_joints, [data-action="sync-live-to-joints"]');
    if (btnSyncLiveJoints) {
      const onSyncLive = () => {
        this.setGhostToLive(true);
      };
      btnSyncLiveJoints.addEventListener('click', onSyncLive);
      this._eventCleanups.push(() => btnSyncLiveJoints.removeEventListener('click', onSyncLive));
    }
    // 3.5 IK Cylindrical Toggle
    const toggleCyl = container.querySelector('#check_ik_cylindrical');
    if (toggleCyl) {
      this._domElements.toggleCyl = toggleCyl;
      toggleCyl.checked = Boolean(this.isCylindricalMode);

      const onToggle = (e) => {
        this.setCylindricalMode(Boolean(e.target.checked));
      };
      toggleCyl.addEventListener('change', onToggle);
      this._eventCleanups.push(() => toggleCyl.removeEventListener('change', onToggle));
    }

    // 4. Cartesian / Cylindrical Coordinate Inputs
    for (const axis of ['x', 'y', 'z']) {
      const slider = container.querySelector(`[data-cartesian-slider="${axis}"], #slider_${axis}`);
      const input = container.querySelector(`[data-cartesian-input="${axis}"], #input_${axis}`);
      
      if (slider) {
        this._domElements[`slider_${axis}`] = slider;

        const onCartSlider = (e) => {
          const val = parseFloat(e.target.value);
          this._handleCartesianInput(axis, val, true);
        };
        slider.addEventListener('input', onCartSlider);
        this._eventCleanups.push(() => slider.removeEventListener('input', onCartSlider));

        const onCartSliderChange = (e) => {
          const val = parseFloat(e.target.value);
          this._handleCartesianInput(axis, val, false);
          this.flushFinalTarget();
        };
        slider.addEventListener('change', onCartSliderChange);
        this._eventCleanups.push(() => slider.removeEventListener('change', onCartSliderChange));
      }

      if (input) {
        this._domElements[`input_${axis}`] = input;

        const onCartChange = (e) => {
          const val = parseFloat(e.target.value);
          this._handleCartesianInput(axis, val, false);
          this.flushFinalTarget();
        };
        input.addEventListener('change', onCartChange);
        this._eventCleanups.push(() => input.removeEventListener('change', onCartChange));
      }
    }

    // 5. Cartesian Step Nudge & Unified Jog Buttons (Tap Step + Hold-Jog Acceleration)
    const cartNudgeBtns = container.querySelectorAll('[data-cartesian-nudge]');
    cartNudgeBtns.forEach((btn) => {
      const axis = btn.getAttribute('data-cartesian-nudge');
      const dir = parseFloat(btn.getAttribute('data-jog-dir') || '0');
      this._bindJogRamping(btn, 'cartesian', axis, dir);
    });

    // 5.1 Cartesian Step Pill Selector
    const cartStepPills = container.querySelectorAll('[data-cart-step]');
    cartStepPills.forEach((pill) => {
      const stepVal = parseFloat(pill.getAttribute('data-cart-step'));
      const onPillClick = () => {
        this.setCartesianStepSize(stepVal);
      };
      pill.addEventListener('click', onPillClick);
      this._eventCleanups.push(() => pill.removeEventListener('click', onPillClick));
    });

    // 5.2 Interactive Numerical Drag Scrubbing on Inputs
    const scrubInputs = container.querySelectorAll('.scrubbable-input');
    scrubInputs.forEach((input) => {
      let isDragging = false;
      let startX = 0;
      let startVal = 0;
      let didDrag = false;
      const dragThreshold = 4; // px before drag activates

      const onPointerDown = (downEvt) => {
        if (downEvt.button !== 0) return;
        startX = downEvt.clientX;
        startVal = parseFloat(input.value) || 0;
        didDrag = false;
        isDragging = false;

        const onPointerMove = (moveEvt) => {
          const dx = moveEvt.clientX - startX;
          if (!didDrag && Math.abs(dx) > dragThreshold) {
            didDrag = true;
            isDragging = true;
            input.classList.add('scrubbing');
          }
          if (isDragging) {
            moveEvt.preventDefault();
            // Multiplier: Shift = 0.1x fine, Ctrl/Alt = 5.0x coarse
            const multiplier = moveEvt.shiftKey ? 0.1 : (moveEvt.ctrlKey || moveEvt.altKey ? 5.0 : 1.0);
            const isCart = input.hasAttribute('data-cartesian-input');
            const sensitivity = isCart ? 0.5 : 0.2; // mm/px or deg/px
            const newVal = startVal + (dx * sensitivity * multiplier);

            if (input.hasAttribute('data-orient-input') || ['input_roll', 'input_pitch', 'input_yaw'].includes(input.id)) {
              const orientAxis = input.getAttribute('data-orient-input') || input.id.replace('input_', '');
              this.setOrientationTarget(orientAxis, newVal, true);
            } else if (isCart) {
              const axis = input.getAttribute('data-cartesian-input');
              this._handleCartesianInput(axis, newVal, true);
            } else {
              const joint = input.getAttribute('data-joint-val');
              if (joint) {
                this.setJointTarget(joint, newVal, false, true);
              }
            }
          }
        };

        const onPointerUp = () => {
          window.removeEventListener('pointermove', onPointerMove);
          window.removeEventListener('pointerup', onPointerUp);
          window.removeEventListener('pointercancel', onPointerUp);
          if (isDragging) {
            input.classList.remove('scrubbing');
            this.flushFinalTarget();
          }
        };

        window.addEventListener('pointermove', onPointerMove, { passive: false });
        window.addEventListener('pointerup', onPointerUp);
        window.addEventListener('pointercancel', onPointerUp);
      };

      input.addEventListener('pointerdown', onPointerDown);
      this._eventCleanups.push(() => input.removeEventListener('pointerdown', onPointerDown));
    });

    // 5.3 Orientation Mode Switcher (6-DOF vs Preset Wrist)
    const btnOrient6dof = container.querySelector('#btn_orient_mode_6dof');
    if (btnOrient6dof) {
      this._domElements.btnOrient6dof = btnOrient6dof;
      const on6dofClick = () => this.setOrientationMode('6dof');
      btnOrient6dof.addEventListener('click', on6dofClick);
      this._eventCleanups.push(() => btnOrient6dof.removeEventListener('click', on6dofClick));
    }

    const btnOrientWrist = container.querySelector('#btn_orient_mode_wrist');
    if (btnOrientWrist) {
      this._domElements.btnOrientWrist = btnOrientWrist;
      const onWristClick = () => this.setOrientationMode('preset');
      btnOrientWrist.addEventListener('click', onWristClick);
      this._eventCleanups.push(() => btnOrientWrist.removeEventListener('click', onWristClick));
    }

    const containerOrient6dof = container.querySelector('#container_orient_6dof');
    if (containerOrient6dof) {
      this._domElements.containerOrient6dof = containerOrient6dof;
    }

    const containerWristMode = container.querySelector('#container_wrist_mode');
    if (containerWristMode) {
      this._domElements.containerWristMode = containerWristMode;
    }

    const wrapperWristMode = container.querySelector('#wrapper_wrist_mode, #group_wrist_mode');
    if (wrapperWristMode) {
      this._domElements.wrapperWristMode = wrapperWristMode;
    }

    // 5.4 6-DOF Tool Orientation Sliders (Roll, Pitch, Yaw)
    for (const axis of ['roll', 'pitch', 'yaw']) {
      const slider = container.querySelector(`[data-orient-slider="${axis}"], #slider_${axis}`);
      const input = container.querySelector(`[data-orient-input="${axis}"], #input_${axis}`);

      if (slider) {
        this._domElements[`slider_${axis}`] = slider;
        const onOrientSlider = (e) => {
          const val = parseFloat(e.target.value);
          this.setOrientationTarget(axis, val, true);
        };
        slider.addEventListener('input', onOrientSlider);
        this._eventCleanups.push(() => slider.removeEventListener('input', onOrientSlider));

        const onOrientSliderChange = (e) => {
          const val = parseFloat(e.target.value);
          this.setOrientationTarget(axis, val, false);
          this.flushFinalTarget();
        };
        slider.addEventListener('change', onOrientSliderChange);
        this._eventCleanups.push(() => slider.removeEventListener('change', onOrientSliderChange));
      }

      if (input) {
        this._domElements[`input_${axis}`] = input;
        const onOrientChange = (e) => {
          const val = parseFloat(e.target.value);
          this.setOrientationTarget(axis, val, false);
          this.flushFinalTarget();
        };
        input.addEventListener('change', onOrientChange);
        this._eventCleanups.push(() => input.removeEventListener('change', onOrientChange));
      }
    }

    // 5.5 Orientation Nudge & Unified Jog Buttons
    const orientNudgeBtns = container.querySelectorAll('[data-orient-nudge]');
    orientNudgeBtns.forEach((btn) => {
      const axis = btn.getAttribute('data-orient-nudge');
      const onClick = () => {
        const dir = parseFloat(btn.getAttribute('data-jog-dir') || '0');
        const step = (dir !== 0)
          ? (dir * 5.0)
          : parseFloat(btn.getAttribute('data-step') || btn.textContent);
        this.nudgeOrientation(axis, step, false);
      };
      btn.addEventListener('click', onClick);
      this._eventCleanups.push(() => btn.removeEventListener('click', onClick));
    });

    // 5.6 Backlash Query & Save Buttons
    const btnBacklashQuery = container.querySelector('#btn_backlash_query');
    if (btnBacklashQuery) {
      const onQuery = () => this.queryBacklash().catch(() => {});
      btnBacklashQuery.addEventListener('click', onQuery);
      this._eventCleanups.push(() => btnBacklashQuery.removeEventListener('click', onQuery));
    }

    const btnBacklashSave = container.querySelector('#btn_backlash_save');
    if (btnBacklashSave) {
      const onSave = () => this.saveBacklash().catch(() => {});
      btnBacklashSave.addEventListener('click', onSave);
      this._eventCleanups.push(() => btnBacklashSave.removeEventListener('click', onSave));
    }

    for (let pose = 1; pose <= 4; pose++) {
      const button = container.querySelector(`#btn_tcp_cal_pose_${pose}`);
      if (!button) continue;
      const onClick = () => this._sendTcpCommand(`TCPCAL ${pose}`).then((sent) => {
        if (sent) this._setTcpStatus(`Captured TCP calibration pose ${pose}.`);
      }).catch((error) => this._setTcpStatus(`TCP capture failed: ${error.message}`));
      button.addEventListener('click', onClick);
      this._eventCleanups.push(() => button.removeEventListener('click', onClick));
    }
    for (const [id, command, message] of [
      ['btn_tcp_cal_solve', 'TCPCAL SOLVE', 'Solving TCP calibration...'],
      ['btn_tcp_cal_clear', 'TCPCAL CLEAR', 'TCP calibration poses cleared.']
    ]) {
      const button = container.querySelector(`#${id}`);
      if (!button) continue;
      const onClick = () => this._sendTcpCommand(command).then((sent) => {
        if (sent) this._setTcpStatus(message);
      }).catch((error) => this._setTcpStatus(`TCP command failed: ${error.message}`));
      button.addEventListener('click', onClick);
      this._eventCleanups.push(() => button.removeEventListener('click', onClick));
    }
    for (const kind of ['tool', 'work']) for (const axis of ['x', 'y', 'z']) {
      const input = container.querySelector(`#input_${kind}_${axis}`);
      if (!input) continue;
      const onChange = () => {
        const value = Number(input.value);
        if (!Number.isFinite(value)) return;
        this._sendTcpCommand(`${kind.toUpperCase()}${axis.toUpperCase()} ${value}`).then((sent) => {
          if (!sent) return;
          this[`${kind}Offset`][axis] = value;
          this._recomputeFK();
          this._evaluateReachability();
          this._syncToAppState();
          this._updateDomUI();
        }).catch((error) => this._setTcpStatus(`Offset update failed: ${error.message}`));
      };
      input.addEventListener('change', onChange);
      this._eventCleanups.push(() => input.removeEventListener('change', onChange));
    }

    // 5.7 Backlash Sliders and Inputs (J1..J6)
    for (let i = 1; i <= 6; i++) {
      const jKey = `j${i}`;
      const slider = container.querySelector(`#slider_backlash_${jKey}`);
      const input = container.querySelector(`#input_backlash_${jKey}`);

      if (slider) {
        this._domElements[`slider_backlash_${jKey}`] = slider;
        const onSliderInput = (e) => {
          const val = parseFloat(e.target.value);
          this.setBacklash(jKey, val);
          if (input) input.value = val.toFixed(2);
        };
        slider.addEventListener('input', onSliderInput);
        this._eventCleanups.push(() => slider.removeEventListener('input', onSliderInput));
      }

      if (input) {
        this._domElements[`input_backlash_${jKey}`] = input;
        const onInputChange = (e) => {
          const val = parseFloat(e.target.value);
          this.setBacklash(jKey, val);
          if (slider) slider.value = val;
        };
        input.addEventListener('change', onInputChange);
        this._eventCleanups.push(() => input.removeEventListener('change', onInputChange));
      }
    }

    // 6. Wrist Mode Selector (Select Dropdown or Radio Chips)
    const wristSelect = container.querySelector('[data-wrist-mode-select], #select_wrist_mode');
    if (wristSelect) {
      this._domElements.wristSelect = wristSelect;
      wristSelect.value = this.cartesian.wristMode;

      const onWristChange = (e) => {
        this.setWristMode(parseInt(e.target.value, 10));
      };
      wristSelect.addEventListener('change', onWristChange);
      this._eventCleanups.push(() => wristSelect.removeEventListener('change', onWristChange));
    }

    const wristChips = container.querySelectorAll('[data-wrist-chip]');
    wristChips.forEach((chip) => {
      const mode = parseInt(chip.getAttribute('data-wrist-chip'), 10);
      const onClick = () => {
        this.setWristMode(mode);
      };
      chip.addEventListener('click', onClick);
      this._eventCleanups.push(() => chip.removeEventListener('click', onClick));
    });

    // 7. Movement Duration Input
    const durInput = container.querySelector('[data-duration-input], #input_duration');
    if (durInput) {
      this._domElements.durInput = durInput;
      durInput.value = this.duration;

      const onDurChange = (e) => {
        this.setDuration(parseInt(e.target.value, 10));
      };
      durInput.addEventListener('change', onDurChange);
      this._eventCleanups.push(() => durInput.removeEventListener('change', onDurChange));
    }

    // 8. Reachability Warning Badge
    const reachBadge = container.querySelector('[data-reachability-badge], #badge_reachability');
    if (reachBadge) {
      this._domElements.reachBadge = reachBadge;
    }

    // 9. Motion Action Buttons (MOV Spline & MOVL Linear)
    const movBtn = container.querySelector('[data-action="mov-spline"], #btn_mov_spline');
    if (movBtn) {
      this._domElements.movBtn = movBtn;
      const onMovClick = () => {
        this.sendMovSpline().catch((err) => {
          console.warn('MOV Spline failed:', err.message);
        });
      };
      movBtn.addEventListener('click', onMovClick);
      this._eventCleanups.push(() => movBtn.removeEventListener('click', onMovClick));
    }

    const movlBtn = container.querySelector('[data-action="mov-linear"], #btn_mov_linear');
    if (movlBtn) {
      this._domElements.movlBtn = movlBtn;
      const onMovlClick = () => {
        this.sendMovLinear().catch((err) => {
          console.warn('MOVL Linear failed:', err.message);
        });
      };
      movlBtn.addEventListener('click', onMovlClick);
      this._eventCleanups.push(() => movlBtn.removeEventListener('click', onMovlClick));
    }

    // 10. Live vs Ghost Sync Buttons
    const syncGhostToLiveBtn = queryEl('[data-action="ghost-to-live"], #btn_ghost_to_live');
    if (syncGhostToLiveBtn) {
      const onSyncClick = () => this.setGhostToLive(true);
      syncGhostToLiveBtn.addEventListener('click', onSyncClick);
      this._eventCleanups.push(() => syncGhostToLiveBtn.removeEventListener('click', onSyncClick));
    }

    const sendGhostBtn = queryEl('[data-action="send-ghost"], #btn_send_ghost');
    if (sendGhostBtn) {
      const onSendGhost = () => this.sendGhostToRobot().catch(() => {});
      sendGhostBtn.addEventListener('click', onSendGhost);
      this._eventCleanups.push(() => sendGhostBtn.removeEventListener('click', onSendGhost));
    }

    // 11. Reference Coordinate Frame Selector (World, Base, Tool)
    const framePills = container.querySelectorAll('[data-coord-frame]');
    framePills.forEach((pill) => {
      const frame = pill.getAttribute('data-coord-frame');
      const onFrameClick = () => {
        this.setCartesianFrame(frame);
      };
      pill.addEventListener('click', onFrameClick);
      this._eventCleanups.push(() => pill.removeEventListener('click', onFrameClick));
    });

    const selectFrame = container.querySelector('#select_coord_frame');
    if (selectFrame) {
      const onSelectChange = (e) => {
        this.setCartesianFrame(e.target.value);
      };
      selectFrame.addEventListener('change', onSelectChange);
      this._eventCleanups.push(() => selectFrame.removeEventListener('change', onSelectChange));
    }

    // 12. Mode Navigation Tabs (IK vs FK vs Trajectory)
    const modeTabs = (root.querySelectorAll ? root.querySelectorAll('[data-mode-tab]') : container.querySelectorAll('[data-mode-tab]'));
    modeTabs.forEach((tabBtn) => {
      const tabKey = tabBtn.getAttribute('data-mode-tab');
      const onTabClick = () => {
        this.setControlModeTab(tabKey);
      };
      tabBtn.addEventListener('click', onTabClick);
      this._eventCleanups.push(() => tabBtn.removeEventListener('click', onTabClick));
    });

    // 13. Quick Pose Presets
    const presetBtns = container.querySelectorAll('[data-pose-preset], #btn_preset_home, #btn_preset_zero, #btn_preset_ready, #btn_preset_stow');
    presetBtns.forEach((btn) => {
      const presetKey = btn.getAttribute('data-pose-preset') || btn.id.replace('btn_preset_', '');
      const onPresetClick = () => {
        this.applyPosePreset(presetKey);
      };
      btn.addEventListener('click', onPresetClick);
      this._eventCleanups.push(() => btn.removeEventListener('click', onPresetClick));
    });

    // 14. Action Bar Plan Trajectory Button
    const btnPlanTraj = queryEl('#btn_plan_trajectory, [data-action="plan-trajectory"]');
    if (btnPlanTraj) {
      const onPlan = () => {
        this.planTrajectory();
      };
      btnPlanTraj.addEventListener('click', onPlan);
      this._eventCleanups.push(() => btnPlanTraj.removeEventListener('click', onPlan));
    }

    // 15. Safety Modal & Direct Drive confirmation
    this._initSafetyModalListeners();

    // 16. Persistent Gripper Card
    this._bindPersistentGripper(container);

    this._updateDomUI();
    this._updateStepJogUI('all');

    // 17. Initial Mode Tab Synchronization
    const initialModeTab = (this.appState?.getState && this.appState.getState('controlMode.activeTab')) ||
      (root.querySelector?.('.panel-mode-tab.active')?.getAttribute('data-mode-tab')) ||
      'ik';
    this.setControlModeTab(initialModeTab);
  }

  /**
   * Updates all DOM inputs and display values to match current state
   * @private
   */
  _updateDomUI() {
    if (!this._domBound || typeof document === 'undefined') return;

    // Update joint sliders and numerical text inputs
    for (const joint of ['j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'grip']) {
      const slider = this._domElements[`slider_${joint}`];
      const valInput = this._domElements[`val_${joint}`];
      const val = Number(this.joints[joint] ?? 0);
      if (joint === 'grip') {
        const isPctSlider = slider && Number(slider.max) <= 100;
        const gripDisp = isPctSlider ? Math.round(val / 1.8) : val;
        if (slider && Number(slider.value) !== gripDisp) {
          slider.value = gripDisp;
        }
        if (valInput) {
          valInput.value = isPctSlider ? String(gripDisp) : val.toFixed(0);
        }
      } else {
        if (slider && Number(slider.value) !== val) {
          slider.value = val;
        }
        if (valInput) {
          valInput.value = val.toFixed(1);
        }
      }
    }

    this._updateJointDualMarkers();
    this._updateGripperJawGraphic();

    // Update Mode Toggle Switch state
    if (this._domElements.toggleCyl && this._domElements.toggleCyl.checked !== this.isCylindricalMode) {
      this._domElements.toggleCyl.checked = this.isCylindricalMode;
    }

    // Update Cartesian / Cylindrical Labels & Inputs
    const labelX = this.container ? this.container.querySelector('#label_ik_x') : null;
    const labelY = this.container ? this.container.querySelector('#label_ik_y') : null;
    const labelZ = this.container ? this.container.querySelector('#label_ik_z') : null;

    const currentX = Number(this.cartesian.x ?? 253.5);
    const currentY = Number(this.cartesian.y ?? 0.0);
    const currentZ = Number(this.cartesian.z ?? 141.0);

    const currentR = Math.hypot(currentX, currentY);
    const currentThetaDeg = (Math.atan2(currentY, currentX) * 180) / Math.PI;

    if (this.isCylindricalMode) {
      if (labelX) labelX.textContent = i18n.t('ik.target.r');
      if (labelY) labelY.textContent = i18n.t('ik.target.theta');
      if (labelZ) labelZ.textContent = i18n.t('ik.target.z');

      const sliderX = this._domElements['slider_x'];
      const inputX = this._domElements['input_x'];
      if (sliderX) {
        sliderX.min = CARTESIAN_CONFIGS.r.min;
        sliderX.max = CARTESIAN_CONFIGS.r.max;
        if (document.activeElement !== sliderX) sliderX.value = currentR;
      }
      if (inputX) {
        inputX.min = CARTESIAN_CONFIGS.r.min;
        inputX.max = CARTESIAN_CONFIGS.r.max;
        if (document.activeElement !== inputX) inputX.value = currentR.toFixed(1);
      }

      const sliderY = this._domElements['slider_y'];
      const inputY = this._domElements['input_y'];
      if (sliderY) {
        sliderY.min = JOINT_CONFIGS.j1.min;
        sliderY.max = JOINT_CONFIGS.j1.max;
        if (document.activeElement !== sliderY) sliderY.value = currentThetaDeg;
      }
      if (inputY) {
        inputY.min = JOINT_CONFIGS.j1.min;
        inputY.max = JOINT_CONFIGS.j1.max;
        if (document.activeElement !== inputY) inputY.value = currentThetaDeg.toFixed(1);
      }

      const sliderZ = this._domElements['slider_z'];
      const inputZ = this._domElements['input_z'];
      if (sliderZ) {
        sliderZ.min = CARTESIAN_CONFIGS.z.min;
        sliderZ.max = CARTESIAN_CONFIGS.z.max;
        if (document.activeElement !== sliderZ) sliderZ.value = currentZ;
      }
      if (inputZ) {
        inputZ.min = CARTESIAN_CONFIGS.z.min;
        inputZ.max = CARTESIAN_CONFIGS.z.max;
        if (document.activeElement !== inputZ) inputZ.value = currentZ.toFixed(1);
      }
    } else {
      if (labelX) labelX.textContent = i18n.t('ik.target.x');
      if (labelY) labelY.textContent = i18n.t('ik.target.y');
      if (labelZ) labelZ.textContent = i18n.t('ik.target.z');

      const sliderX = this._domElements['slider_x'];
      const inputX = this._domElements['input_x'];
      if (sliderX) {
        sliderX.min = CARTESIAN_CONFIGS.x.min;
        sliderX.max = CARTESIAN_CONFIGS.x.max;
        if (document.activeElement !== sliderX) sliderX.value = currentX;
      }
      if (inputX) {
        inputX.min = CARTESIAN_CONFIGS.x.min;
        inputX.max = CARTESIAN_CONFIGS.x.max;
        if (document.activeElement !== inputX) inputX.value = currentX.toFixed(1);
      }

      const sliderY = this._domElements['slider_y'];
      const inputY = this._domElements['input_y'];
      if (sliderY) {
        sliderY.min = CARTESIAN_CONFIGS.y.min;
        sliderY.max = CARTESIAN_CONFIGS.y.max;
        if (document.activeElement !== sliderY) sliderY.value = currentY;
      }
      if (inputY) {
        inputY.min = CARTESIAN_CONFIGS.y.min;
        inputY.max = CARTESIAN_CONFIGS.y.max;
        if (document.activeElement !== inputY) inputY.value = currentY.toFixed(1);
      }

      const sliderZ = this._domElements['slider_z'];
      const inputZ = this._domElements['input_z'];
      if (sliderZ) {
        sliderZ.min = CARTESIAN_CONFIGS.z.min;
        sliderZ.max = CARTESIAN_CONFIGS.z.max;
        if (document.activeElement !== sliderZ) sliderZ.value = currentZ;
      }
      if (inputZ) {
        inputZ.min = CARTESIAN_CONFIGS.z.min;
        inputZ.max = CARTESIAN_CONFIGS.z.max;
        if (document.activeElement !== inputZ) inputZ.value = currentZ.toFixed(1);
      }
    }

    // Update Wrist Mode select
    if (this._domElements.wristSelect) {
      this._domElements.wristSelect.value = this.cartesian.wristMode;
    }

    // Update Duration input
    if (this._domElements.durInput) {
      this._domElements.durInput.value = this.duration;
    }

    // Update Orientation Mode Button active states and visibility
    const is6DOF = this.cartesian.orientationMode === '6dof';
    if (this._domElements.btnOrient6dof) {
      this._domElements.btnOrient6dof.classList.toggle('active', is6DOF);
    }
    if (this._domElements.btnOrientWrist) {
      this._domElements.btnOrientWrist.classList.toggle('active', !is6DOF);
    }

    // Hide sliders when in preset mode, and hide preset wrist mode when in 6dof mode
    if (this._domElements.containerOrient6dof) {
      this._domElements.containerOrient6dof.classList.toggle('hidden', !is6DOF);
    }
    const containerOrientPreset = this.container ? this.container.querySelector('#container_orient_preset') : null;
    if (containerOrientPreset) {
      containerOrientPreset.classList.toggle('hidden', is6DOF);
    }
    const wristModeWrapper = this._domElements.wrapperWristMode ||
      (this._domElements.wristSelect ? this._domElements.wristSelect.closest('#wrapper_wrist_mode, .wrist-mode-group, div') : null);
    if (wristModeWrapper) {
      wristModeWrapper.classList.toggle('hidden', is6DOF);
    }
    if (this._domElements.containerWristMode) {
      this._domElements.containerWristMode.classList.toggle('wrist-mode-hidden', is6DOF);
    }

    // Update active state on wrist preset chips & preset Euler readout
    if (this.container) {
      const activeMode = Number(this.cartesian.wristMode ?? 0);
      const chips = this.container.querySelectorAll('[data-wrist-chip]');
      chips.forEach((c) => {
        const cMode = parseInt(c.getAttribute('data-wrist-chip'), 10);
        c.classList.toggle('active', !is6DOF && cMode === activeMode);
      });
      const pR = this.container.querySelector('#preset_readout_roll');
      const pP = this.container.querySelector('#preset_readout_pitch');
      const pYW = this.container.querySelector('#preset_readout_yaw');
      if (pR) pR.textContent = `${Number(this.cartesian.roll ?? 0).toFixed(1)}°`;
      if (pP) pP.textContent = `${Number(this.cartesian.pitch ?? 0).toFixed(1)}°`;
      if (pYW) pYW.textContent = `${Number(this.cartesian.yaw ?? 0).toFixed(1)}°`;
    }

    // Update Orientation Sliders & Inputs
    for (const axis of ['roll', 'pitch', 'yaw']) {
      const s = this._domElements[`slider_${axis}`];
      const inp = this._domElements[`input_${axis}`];
      const val = this.cartesian[axis] ?? 0.0;
      if (s && document.activeElement !== s) {
        s.value = val;
      }
      if (inp && document.activeElement !== inp) {
        inp.value = Number(val).toFixed(1);
      }
    }

    // Update Backlash Sliders & Inputs
    for (let i = 1; i <= 6; i++) {
      const jKey = `j${i}`;
      const s = this._domElements[`slider_backlash_${jKey}`];
      const inp = this._domElements[`input_backlash_${jKey}`];
      const val = this.backlash[jKey] ?? 0.0;
      if (s && document.activeElement !== s) {
        s.value = val;
      }
      if (inp && document.activeElement !== inp) {
        inp.value = Number(val).toFixed(2);
      }
    }

    // Update Viewport HUD coordinates (X, Y, Z, R, P, YW)
    const hudX = document.getElementById('hud-tcp-x');
    const hudY = document.getElementById('hud-tcp-y');
    const hudZ = document.getElementById('hud-tcp-z');
    const hudR = document.getElementById('hud-tcp-r');
    const hudP = document.getElementById('hud-tcp-p');
    const hudYW = document.getElementById('hud-tcp-yw');
    if (hudX) hudX.textContent = Number(this.cartesian.x ?? 0).toFixed(1);
    if (hudY) hudY.textContent = Number(this.cartesian.y ?? 0).toFixed(1);
    if (hudZ) hudZ.textContent = Number(this.cartesian.z ?? 215).toFixed(1);
    if (hudR) hudR.textContent = `${Number(this.cartesian.roll ?? 0).toFixed(1)}°`;
    if (hudP) hudP.textContent = `${Number(this.cartesian.pitch ?? 0).toFixed(1)}°`;
    if (hudYW) hudYW.textContent = `${Number(this.cartesian.yaw ?? 0).toFixed(1)}°`;

    // Update Reachability Badge
    if (this._domElements.reachBadge) {
      const b = this._domElements.reachBadge;
      if (this.reachability.isReachable) {
        b.textContent = i18n.t('ik.reachability.valid');
      } else if (this.reachability.ikResult && !this.reachability.ikResult.success && !this.reachability.clamped) {
        b.textContent = i18n.t('ik.reachability.invalid');
      } else {
        b.textContent = this.reachability.statusText;
      }
      b.className = 'reachability-badge ' + (this.reachability.isReachable ? 'badge-safe' : (this.reachability.clamped ? 'badge-warning' : 'badge-danger'));
    }

    this._updateExecutionSafetyUI();
  }

  /**
   * Updates Move buttons disabled state when ESTOP changes
   * @private
   */
  _updateMoveButtonsState(estopActive) {
    if (!this._domBound) return;
    if (this._domElements.movBtn) {
      this._domElements.movBtn.disabled = Boolean(estopActive);
    }
    if (this._domElements.movlBtn) {
      this._domElements.movlBtn.disabled = Boolean(estopActive);
    }
  }

  // ==========================================================================
  // MILESTONE 1: MOTION PANEL OVERHAUL METHODS
  // ==========================================================================

  /**
   * Sets active primary mode tab ('ik' vs 'joints' vs 'trajectory')
   * @param {'ik'|'joints'|'trajectory'} tab
   */
  setControlModeTab(tab) {
    const validTab = ['ik', 'joints', 'trajectory'].includes(tab) ? tab : 'ik';
    if (this.appState && typeof this.appState.setControlModeTab === 'function') {
      this.appState.setControlModeTab(validTab);
    }

    const root = (this.container?.closest && this.container.closest('#left-panel')) ||
                 this.container ||
                 (typeof document !== 'undefined' ? document.getElementById('left-panel') || document : null);

    if (root) {
      // Update top tab buttons active class
      const tabs = root.querySelectorAll ? root.querySelectorAll('[data-mode-tab]') : [];
      tabs.forEach((t) => {
        const isCurrent = t.getAttribute('data-mode-tab') === validTab;
        if (t.classList) {
          t.classList.toggle('active', isCurrent);
        }
        if (typeof t.setAttribute === 'function') {
          t.setAttribute('aria-selected', String(isCurrent));
        }
      });

      // Synchronize container cards
      const cards = root.querySelectorAll ? root.querySelectorAll('.accordion-card[data-accordion]') : [];
      cards.forEach((card) => {
        const key = card.getAttribute('data-accordion');
        const isTarget = (key === validTab);
        if (card.classList) {
          card.classList.toggle('open', isTarget);
        }
        const header = card.querySelector ? card.querySelector('.accordion-header') : null;
        if (header && typeof header.setAttribute === 'function') {
          header.setAttribute('aria-expanded', String(isTarget));
        }
      });

      // Reveal / hide joint action buttons depending on whether joints tab is active
      const isJoints = (validTab === 'joints');
      const btnSyncLiveJoints = (root.querySelector ? root.querySelector('#btn_sync_live_joints, [data-action="sync-live-to-joints"]') : null) ||
        (typeof document !== 'undefined' ? document.querySelector('#btn_sync_live_joints, [data-action="sync-live-to-joints"]') : null);
      if (btnSyncLiveJoints) {
        btnSyncLiveJoints.style.display = isJoints ? '' : 'none';
      }
      const btnSendJoints = (root.querySelector ? root.querySelector('#btn_send_joints_robot, [data-action="send-joints-robot"]') : null) ||
        (typeof document !== 'undefined' ? document.querySelector('#btn_send_joints_robot, [data-action="send-joints-robot"]') : null);
      if (btnSendJoints) {
        btnSendJoints.style.display = isJoints ? '' : 'none';
      }
    }

    return validTab;
  }

  /**
   * Sets active Cartesian coordinate frame ('base', 'world', 'tool')
   * @param {'base'|'world'|'tool'} frame
   */
  setCartesianFrame(frame) {
    const validFrame = ['base', 'world', 'tool'].includes(frame) ? frame : 'base';
    this.cartesianFrame = validFrame;

    if (this.appState && typeof this.appState.setCartesianFrame === 'function') {
      this.appState.setCartesianFrame(validFrame);
    }

    if (this.container) {
      const frameSelect = this.container.querySelector('#select_coord_frame');
      if (frameSelect && frameSelect.value !== validFrame) {
        frameSelect.value = validFrame;
      }
      const framePills = this.container.querySelectorAll('[data-coord-frame]');
      framePills.forEach((p) => {
        if (p.classList) {
          p.classList.toggle('active', p.getAttribute('data-coord-frame') === validFrame);
        }
      });
    }

    return this.cartesianFrame;
  }

  /**
   * Steps Cartesian position along chosen axis with coordinate frame transformation
   * 
   * @param {'x'|'y'|'z'} axis
   * @param {number} dir - (+1 or -1)
   * @param {number} [stepSize=null] - Optional override in mm
   * @returns {Object} Result of setCartesianTarget
   */
  stepCartesian(axis, dir, stepSize = null) {
    const effectiveStep = (typeof stepSize === 'number' && !isNaN(stepSize))
      ? stepSize
      : this.cartesianStepSize;
    const delta = (dir >= 0 ? 1 : -1) * effectiveStep;

    if (this.cartesianFrame === 'tool') {
      // Fetch latest R06 rotation matrix
      const fk = forwardKinematics6DOF
        ? forwardKinematics6DOF(
            this.joints.j1,
            this.joints.j2,
            this.joints.j3,
            this.joints.j4,
            this.joints.j5,
            this.joints.j6,
            { toolOffset: this.toolOffset, workOffset: this.workOffset }
          )
        : (this._lastFk || forwardKinematics(this.joints.j1, this.joints.j2, this.joints.j3, {}, null, this.joints.j4, this.joints.j5, this.joints.j6));
      const R06 = fk.R06 || [
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
      ];

      // Column 0: Tool X, Column 1: Tool Y, Column 2: Tool Z
      const col = (axis === 'x') ? 0 : (axis === 'y') ? 1 : 2;

      const deltaX_base = R06[0][col] * delta;
      const deltaY_base = R06[1][col] * delta;
      const deltaZ_base = R06[2][col] * delta;

      // Scale uniformly if an axis bound is encountered so the 3D step vector stays collinear with the tool axis
      let scale = 1.0;
      const axesBounds = [
        { curr: this.cartesian.x, d: deltaX_base, min: CARTESIAN_CONFIGS.x.min, max: CARTESIAN_CONFIGS.x.max },
        { curr: this.cartesian.y, d: deltaY_base, min: CARTESIAN_CONFIGS.y.min, max: CARTESIAN_CONFIGS.y.max },
        { curr: this.cartesian.z, d: deltaZ_base, min: CARTESIAN_CONFIGS.z.min, max: CARTESIAN_CONFIGS.z.max }
      ];
      for (const b of axesBounds) {
        if (b.d > 1e-6 && b.curr + b.d > b.max) {
          scale = Math.min(scale, Math.max(0, (b.max - b.curr) / b.d));
        } else if (b.d < -1e-6 && b.curr + b.d < b.min) {
          scale = Math.min(scale, Math.max(0, (b.min - b.curr) / b.d));
        }
      }

      const targetX = this.cartesian.x + deltaX_base * scale;
      const targetY = this.cartesian.y + deltaY_base * scale;
      const targetZ = this.cartesian.z + deltaZ_base * scale;

      return this.setCartesianTarget(targetX, targetY, targetZ, null, null, false, false);
    } else {
      // Base or World frame: standard Cartesian nudge
      return this.nudgeCartesian(axis, delta, false);
    }
  }

  /**
   * Applies a quick pose preset to the robot
   * In Preview Mode: updates Ghost Target Pose only
   * In Direct Drive Mode: dispatches a smooth coordinated PTP move
   * 
   * @param {'home'|'zero'|'ready'|'stow'} presetKey
   * @returns {Object} Updated joints and FK result
   */
  applyPosePreset(presetKey) {
    const key = String(presetKey).toLowerCase().trim();
    const preset = this.POSE_PRESETS[key];
    if (!preset) {
      throw new Error(`Unknown pose preset: "${presetKey}". Valid: home, zero, ready, stow`);
    }

    const { joints } = preset;

    // Apply joint angles locally
    for (const [jKey, val] of Object.entries(joints)) {
      if (jKey in this.joints) {
        this.joints[jKey] = val;
      }
    }

    this._recomputeFK();
    this._evaluateReachability();
    this._syncToAppState();
    this._updateDomUI();

    if (this.appState) {
      this.appState.addConsoleLog({
        type: 'INFO',
        text: `Applied Pose Preset: "${preset.name}" (${this.executionMode === 'direct_drive' && this.isArmed ? 'LIVE' : 'PREVIEW'})`
      });
    }

    if (this.executionMode === 'direct_drive' && this.isArmed) {
      // Dispatch 2000ms smooth coordinated spline move
      this.sendMovSpline({ duration: 2000 }).catch((err) => {
        console.warn(`Failed to execute preset ${preset.name}:`, err);
      });
    }

    return {
      preset: preset.name,
      joints: { ...this.joints },
      cartesian: { ...this.cartesian },
      reachability: { ...this.reachability }
    };
  }

  /**
   * Sets execution mode ('preview' vs 'direct_drive') and armed state
   * @param {'preview'|'direct_drive'} mode
   * @param {boolean} [armed=false]
   */
  setExecutionMode(mode, armed = false) {
    const validMode = (mode === 'direct_drive') ? 'direct_drive' : 'preview';
    this.executionMode = validMode;
    this.isArmed = Boolean(armed && validMode === 'direct_drive');

    if (this.appState && typeof this.appState.setExecutionMode === 'function') {
      this.appState.setExecutionMode(this.executionMode, this.isArmed);
    }

    this._updateExecutionSafetyUI();
    return { executionMode: this.executionMode, isArmed: this.isArmed };
  }

  /**
   * Dispatches Point-to-Point (PTP) motion to drive physical robot to current Ghost target pose
   * @param {number} [duration=null]
   * @returns {Promise<{ success: boolean, packet: string }>}
   */
  async sendTargetPTP(duration = null) {
    const dur = duration || this.duration;
    if (this.appState) {
      this.appState.addConsoleLog({
        type: 'INFO',
        text: `Executing [Send to Target (PTP)]: moving physical hardware over ${dur}ms`
      });
    }
    return this.sendMovSpline({ duration: dur });
  }

  /**
   * Plans trajectory by appending current Ghost target pose to active waypoint sequence
   * @returns {Object} Newly created waypoint
   */
  planTrajectory() {
    if (!this.appState) return null;
    const ghost = this.appState.getState('ghostPose') || {};
    const count = (this.appState.getState('sequence.waypoints') || []).length + 1;

    const wp = this.appState.addWaypoint({
      label: `Target Point ${count}`,
      j1: ghost.j1 ?? this.joints.j1,
      j2: ghost.j2 ?? this.joints.j2,
      j3: ghost.j3 ?? this.joints.j3,
      j4: ghost.j4 ?? this.joints.j4,
      j5: ghost.j5 ?? this.joints.j5,
      j6: ghost.j6 ?? this.joints.j6,
      grip: ghost.grip ?? this.joints.grip,
      dur: this.duration,
      comment: `Planned from Ghost Pose at X:${this.cartesian.x} Y:${this.cartesian.y} Z:${this.cartesian.z}`
    });

    if (this.appState) {
      this.appState.addConsoleLog({
        type: 'INFO',
        text: `Captured Ghost Pose to Trajectory: "${wp.label}"`
      });
    }

    // Switch dock to timeline view if available
    if (typeof document !== 'undefined') {
      const dockTimelineBtn = document.querySelector('[data-dock-tab="timeline"]');
      if (dockTimelineBtn && typeof dockTimelineBtn.click === 'function') {
        dockTimelineBtn.click();
      }
    }

    return wp;
  }

  /**
   * Updates execution safety banner and toggle UI
   * @private
   */
  _updateExecutionSafetyUI() {
    if (!this.container && typeof document === 'undefined') return;
    const query = (sel) => (this.container && this.container.querySelector ? this.container.querySelector(sel) : null) ||
      (typeof document !== 'undefined' ? document.querySelector(sel) : null);

    const isLive = (this.executionMode === 'direct_drive' && this.isArmed);

    const toggle = query('#toggle_direct_drive');
    if (toggle) {
      toggle.checked = isLive;
    }

    const banner = query('#banner_direct_drive_active');
    if (banner && banner.classList) {
      banner.classList.toggle('active', isLive);
      banner.classList.toggle('hidden', !isLive);
    }

    const previewLabel = query('#label_mode_preview');
    if (previewLabel && previewLabel.classList) {
      previewLabel.classList.toggle('active', !isLive);
    }

    const liveLabel = query('#label_mode_live');
    if (liveLabel && liveLabel.classList) {
      liveLabel.classList.toggle('active', isLive);
    }

    const statusBadge = query('#badge_execution_mode');
    if (statusBadge) {
      if (isLive) {
        statusBadge.textContent = 'DIRECT DRIVE ACTIVE';
        statusBadge.className = 'badge badge-direct-drive-active';
      } else {
        statusBadge.textContent = 'TARGET PREVIEW (SAFE)';
        statusBadge.className = 'badge badge-preview-safe';
      }
    }
  }

  /**
   * Configures confirmation modal dialog for Direct Drive arming
   * @private
   */
  _initSafetyModalListeners() {
    const query = (sel) => (typeof document !== 'undefined' ? document.querySelector(sel) : null) ||
      (this.container && this.container.querySelector ? this.container.querySelector(sel) : null);

    const toggle = query('#toggle_direct_drive');
    const modal = query('#modal_direct_drive_confirm');
    const btnConfirm = query('#btn_confirm_direct_drive');
    const btnCancel = query('#btn_cancel_direct_drive');

    if (toggle && modal) {
      const onToggleClick = (e) => {
        if (toggle.checked) {
          // Prevent immediate arming; show confirmation modal
          e.preventDefault();
          toggle.checked = false;
          modal.classList.remove('hidden');
          modal.classList.add('active');
        } else {
          // Immediately disarm with zero delay
          this.setExecutionMode('preview', false);
          if (this.appState) {
            this.appState.addConsoleLog({
              type: 'INFO',
              text: '[SAFETY] Live Direct Drive disarmed. Restored Safe Preview Mode.'
            });
          }
        }
      };
      toggle.addEventListener('click', onToggleClick);
      this._eventCleanups.push(() => toggle.removeEventListener('click', onToggleClick));
    }

    if (btnConfirm && modal && toggle) {
      const onConfirm = () => {
        modal.classList.add('hidden');
        modal.classList.remove('active');
        toggle.checked = true;
        this.setExecutionMode('direct_drive', true);
        if (this.appState) {
          this.appState.addConsoleLog({
            type: 'WARN',
            text: '[SAFETY] Live Direct Drive ARMED by operator. Hardware follows jog inputs.'
          });
        }
      };
      btnConfirm.addEventListener('click', onConfirm);
      this._eventCleanups.push(() => btnConfirm.removeEventListener('click', onConfirm));
    }

    if (btnCancel && modal && toggle) {
      const onCancel = () => {
        modal.classList.add('hidden');
        modal.classList.remove('active');
        toggle.checked = false;
        this.setExecutionMode('preview', false);
      };
      btnCancel.addEventListener('click', onCancel);
      this._eventCleanups.push(() => btnCancel.removeEventListener('click', onCancel));
    }
  }

  /**
   * Sets gripper position as percentage (0 to 100%)
   * @param {number} percent - 0 to 100
   * @param {boolean} [sendToRobot=null] - If null, respects executionMode
   * @returns {number} Joint grip angle in degrees (0 to 180)
   */
  setGripperPercent(percent, sendToRobot = null) {
    const pct = clamp(Math.round(Number(percent) || 0), 0, 100);
    this.gripperPercent = pct;
    const angle = clamp(Math.round(pct * 1.8), 0, 180);

    const shouldSend = (sendToRobot !== null)
      ? Boolean(sendToRobot)
      : (this.executionMode === 'direct_drive' && this.isArmed);

    return this.setGripperMacro(angle, shouldSend);
  }

  /**
   * Binds persistent gripper card UI elements
   * @private
   */
  _bindPersistentGripper(container) {
    if (!container && typeof document === 'undefined') return;
    const root = container || document;

    // Gripper slider (0-100% or 0-180°)
    const sliderGrip = root.querySelector('#slider_grip, [data-joint-slider="grip"]');
    const valGrip = root.querySelector('#val_grip, [data-joint-val="grip"]');

    if (sliderGrip) {
      this._domElements['slider_grip'] = sliderGrip;
      const onGripInput = (e) => {
        const raw = parseFloat(e.target.value);
        const isPctSlider = Number(sliderGrip.max) <= 100 && Number(sliderGrip.max) > 0;
        const angle = isPctSlider ? Math.round(raw * 1.8) : raw;
        const shouldSend = (this.executionMode === 'direct_drive' && this.isArmed);
        this.setGripperMacro(angle, shouldSend);
      };
      sliderGrip.addEventListener('input', onGripInput);
      this._eventCleanups.push(() => sliderGrip.removeEventListener('input', onGripInput));

      const onGripChange = (e) => {
        const raw = parseFloat(e.target.value);
        const isPctSlider = Number(sliderGrip.max) <= 100 && Number(sliderGrip.max) > 0;
        const angle = isPctSlider ? Math.round(raw * 1.8) : raw;
        const shouldSend = (this.executionMode === 'direct_drive' && this.isArmed);
        this.setGripperMacro(angle, shouldSend);
      };
      sliderGrip.addEventListener('change', onGripChange);
      this._eventCleanups.push(() => sliderGrip.removeEventListener('change', onGripChange));
    }

    if (valGrip) {
      this._domElements['val_grip'] = valGrip;
      const onValChange = (e) => {
        const raw = parseFloat(e.target.value);
        const isPct = Number(sliderGrip?.max || 100) <= 100 && Number(sliderGrip?.max || 100) > 0;
        const angle = isPct ? Math.round(raw * 1.8) : raw;
        const shouldSend = (this.executionMode === 'direct_drive' && this.isArmed);
        this.setGripperMacro(angle, shouldSend);
      };
      valGrip.addEventListener('change', onValChange);
      this._eventCleanups.push(() => valGrip.removeEventListener('change', onValChange));
    }

    // Gripper macro buttons
    const macroBtns = root.querySelectorAll('[data-grip-macro]');
    macroBtns.forEach((btn) => {
      const macro = btn.getAttribute('data-grip-macro');
      const onClick = () => {
        const shouldSend = (this.executionMode === 'direct_drive' && this.isArmed);
        this.setGripperMacro(macro, shouldSend);
      };
      btn.addEventListener('click', onClick);
      this._eventCleanups.push(() => btn.removeEventListener('click', onClick));
    });

    // Subscribe to gripper telemetry updates
    if (this.appState && typeof this.appState.subscribe === 'function') {
      const unsubGrip = this.appState.subscribe('gripper:telemetry', (gripTelem) => {
        this._updateGripperTelemetryUI(gripTelem);
      });
      this._unsubState.push(unsubGrip);
    }
  }

  /**
   * Updates gripper telemetry readout in DOM
   * @param {Object} gripTelem
   * @private
   */
  _updateGripperTelemetryUI(gripTelem) {
    if (!this.container && typeof document === 'undefined') return;
    const root = this.container || document;

    const effortEl = root.querySelector('#gripper_status_effort, #gripper_effort_readout');
    if (effortEl && gripTelem.effort !== undefined) {
      effortEl.textContent = `${gripTelem.effort} mA`;
    }

    const objectEl = root.querySelector('#gripper_status_object, #gripper_object_badge');
    if (objectEl && gripTelem.objectDetected !== undefined) {
      objectEl.textContent = gripTelem.objectDetected ? 'OBJECT DETECTED' : 'CLEAR';
    }

    const badgeText = root.querySelector('#gripper_badge_text');
    if (badgeText && gripTelem.effort !== undefined) {
      const pct = Math.max(0, Math.min(100, Math.round((gripTelem.effort / 500) * 100)));
      badgeText.textContent = `LOAD: ${pct}%`;
    }
  }

  /**
   * Updates dual-marker live vs ghost position indicators and delta badges on J1-J6 tracks
   * @private
   */
  _updateJointDualMarkers() {
    if (!this._domBound) return;
    const livePose = (this.appState && typeof this.appState.getState === 'function')
      ? this.appState.getState('livePose')
      : null;

    for (const joint of ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']) {
      const cfg = JOINT_CONFIGS[joint];
      if (!cfg) continue;
      const range = (cfg.max - cfg.min) || 1;
      const ghostAngle = Number(this.joints[joint] ?? cfg.default);
      const liveAngle = (livePose && typeof livePose[joint] === 'number' && !Number.isNaN(livePose[joint]))
        ? Number(livePose[joint])
        : ghostAngle;

      const ghostPct = Math.max(0, Math.min(100, ((ghostAngle - cfg.min) / range) * 100));
      const livePct = Math.max(0, Math.min(100, ((liveAngle - cfg.min) / range) * 100));
      const deltaDeg = ghostAngle - liveAngle;

      const liveMarker = this._domElements[`live_marker_${joint}`];
      if (liveMarker && liveMarker.style) {
        liveMarker.style.left = `${livePct.toFixed(1)}%`;
      }

      const deltaSpan = this._domElements[`delta_span_${joint}`];
      if (deltaSpan && deltaSpan.style) {
        const leftPct = Math.min(ghostPct, livePct);
        const widthPct = Math.abs(ghostPct - livePct);
        deltaSpan.style.left = `${leftPct.toFixed(1)}%`;
        deltaSpan.style.width = `${widthPct.toFixed(1)}%`;
      }

      const deltaBadge = this._domElements[`delta_badge_${joint}`];
      if (deltaBadge) {
        if (Math.abs(deltaDeg) > 0.1) {
          const sign = deltaDeg > 0 ? '+' : '';
          deltaBadge.textContent = `Δ${sign}${deltaDeg.toFixed(1)}°`;
          deltaBadge.classList?.add('visible');
        } else {
          deltaBadge.textContent = '';
          deltaBadge.classList?.remove('visible');
        }
      }
    }
  }

  /**
   * Updates the inline SVG parallel-jaw indicator and millimeter gap badge in the persistent gripper card
   * @private
   */
  _updateGripperJawGraphic() {
    if (!this.container && typeof document === 'undefined') return;
    const root = (typeof document !== 'undefined' && document) ? document : this.container;
    if (!root || typeof root.querySelector !== 'function') return;

    const gripDeg = Number(this.joints.grip ?? 0);
    const pct = Math.max(0, Math.min(100, gripDeg / 1.8));
    const offset = (pct / 100) * 19.5; // 0 -> 19.5px outward translation from closed center
    const leftX = (27.5 - offset).toFixed(1);
    const rightX = (32.5 + offset).toFixed(1);

    const jawLeft = root.querySelector('#gripper_jaw_left');
    const jawRight = root.querySelector('#gripper_jaw_right');
    const gapLabel = root.querySelector('#gripper_jaw_gap_label');

    if (jawLeft && typeof jawLeft.setAttribute === 'function') {
      jawLeft.setAttribute('transform', `translate(${leftX}, 0)`);
    }
    if (jawRight && typeof jawRight.setAttribute === 'function') {
      jawRight.setAttribute('transform', `translate(${rightX}, 0)`);
    }
    if (gapLabel) {
      const spanMm = (pct * 0.42).toFixed(1);
      gapLabel.textContent = `${spanMm}mm`;
    }
  }

  /**
   * Notifies AppState when a J1-J6 joint card is hovered/focused for 2D<->3D link highlighting
   * @param {'j1'|'j2'|'j3'|'j4'|'j5'|'j6'|null} joint
   */
  setHighlightedJoint(joint) {
    this._highlightedJoint = joint || null;
    if (this.appState) {
      if (typeof this.appState.setState === 'function') {
        this.appState.setState('ui.highlightedJoint', this._highlightedJoint);
      }
      if (typeof this.appState.emit === 'function') {
        this.appState.emit('ui:highlightJoint', this._highlightedJoint);
      }
    }
  }

  /**
   * Sets up inline editable monospace badge input commit and validation
   * @param {'j1'|'j2'|'j3'|'j4'|'j5'|'j6'} joint
   * @param {HTMLInputElement} inputEl
   * @private
   */
  _bindInlineBadgeInput(joint, inputEl) {
    if (!inputEl) return;

    const cfg = JOINT_CONFIGS[joint];

    const commitValue = () => {
      const parsed = parseFloat(inputEl.value);
      if (isNaN(parsed)) {
        inputEl.value = Number(this.joints[joint] ?? cfg.default).toFixed(1);
        return;
      }

      const clamped = clamp(parsed, cfg.min, cfg.max);
      inputEl.value = clamped.toFixed(1);

      this.setJointTarget(joint, clamped, false, false);

      if (this.executionMode === 'direct_drive' && this.isArmed) {
        this.flushFinalTarget();
      }
    };

    const onFocus = () => {
      if (typeof inputEl.select === 'function') inputEl.select();
      if (inputEl.classList) inputEl.classList.add('badge-editing');
    };

    const onKeyDown = (e) => {
      if (e.key === 'Enter') {
        e.preventDefault();
        commitValue();
        if (typeof inputEl.blur === 'function') inputEl.blur();
      } else if (e.key === 'Escape') {
        e.preventDefault();
        inputEl.value = Number(this.joints[joint] ?? cfg.default).toFixed(1);
        if (typeof inputEl.blur === 'function') inputEl.blur();
      }
    };

    const onBlur = () => {
      if (inputEl.classList) inputEl.classList.remove('badge-editing');
      commitValue();
    };

    const onChange = () => {
      commitValue();
    };

    inputEl.addEventListener('focus', onFocus);
    inputEl.addEventListener('keydown', onKeyDown);
    inputEl.addEventListener('blur', onBlur);
    inputEl.addEventListener('change', onChange);

    this._eventCleanups.push(() => {
      inputEl.removeEventListener('focus', onFocus);
      inputEl.removeEventListener('keydown', onKeyDown);
      inputEl.removeEventListener('blur', onBlur);
      inputEl.removeEventListener('change', onChange);
    });
  }

  /**
   * Initializes hold-jog gesture tracking on pointer down
   * @param {'joint'|'cartesian'} type - Target type
   * @param {string} id - Joint ID ('j1'..'j6') or axis ('x','y','z')
   * @param {number} dir - Direction (+1 or -1)
   * @param {PointerEvent} downEvt
   * @private
   */
  _startHoldJog(type, id, dir, downEvt) {
    if (downEvt && downEvt.button !== undefined && downEvt.button !== 0) return;
    if (downEvt && typeof downEvt.preventDefault === 'function') {
      downEvt.preventDefault();
    }

    this._stopHoldJog();

    const startTime = (typeof performance !== 'undefined' && performance.now) ? performance.now() : Date.now();
    const btn = downEvt ? downEvt.currentTarget : null;
    if (btn && typeof btn.setPointerCapture === 'function' && downEvt && downEvt.pointerId !== undefined) {
      try {
        btn.setPointerCapture(downEvt.pointerId);
      } catch (_) {}
    }

    this._activeHoldJog = {
      type,
      id,
      dir: dir >= 0 ? 1 : -1,
      startTime,
      lastFrameTime: startTime,
      lastDispatchTime: 0,
      isHeld: false,
      pointerId: downEvt ? downEvt.pointerId : null,
      element: btn,
      timerId: null,
      rafId: null
    };

    this._activeHoldJog.timerId = setTimeout(() => {
      if (!this._activeHoldJog) return;
      this._activeHoldJog.isHeld = true;
      this._activeHoldJog.lastFrameTime = (typeof performance !== 'undefined' && performance.now) ? performance.now() : Date.now();
      this._runHoldJogLoop();
    }, this.HOLD_JOG_THRESHOLD_MS);
  }

  /**
   * Animation frame loop for continuous smoothstep velocity ramping
   * @private
   */
  _runHoldJogLoop() {
    if (!this._activeHoldJog || !this._activeHoldJog.isHeld) return;

    const now = (typeof performance !== 'undefined' && performance.now) ? performance.now() : Date.now();
    const dt = Math.min(0.05, Math.max(0.001, (now - this._activeHoldJog.lastFrameTime) / 1000.0));
    this._activeHoldJog.lastFrameTime = now;

    // Evaluate smoothstep velocity
    const heldTime = now - this._activeHoldJog.startTime - this.HOLD_JOG_THRESHOLD_MS;
    const u = Math.max(0.0, Math.min(1.0, heldTime / this.HOLD_JOG_RAMP_DURATION_MS));
    const smoothstep = 3.0 * u * u - 2.0 * u * u * u;
    const currentVelocity = this.HOLD_JOG_MIN_VELOCITY + (this.HOLD_JOG_MAX_VELOCITY - this.HOLD_JOG_MIN_VELOCITY) * smoothstep;

    const delta = this._activeHoldJog.dir * currentVelocity * dt;

    if (this._activeHoldJog.type === 'joint') {
      const joint = this._activeHoldJog.id;
      const currVal = this.joints[joint] ?? 0;
      const cfg = JOINT_CONFIGS[joint];
      const clampedVal = clamp(currVal + delta, cfg.min, cfg.max);

      this.joints[joint] = clampedVal;
      this._recomputeFK();
      this._evaluateReachability();
      this._syncToAppState();
      this._updateDomUI();
    } else if (this._activeHoldJog.type === 'cartesian') {
      const axis = this._activeHoldJog.id;
      this.stepCartesian(axis, this._activeHoldJog.dir, Math.abs(delta));
    }

    // 20Hz Throttled live hardware packet dispatch
    if (this.executionMode === 'direct_drive' && this.isArmed) {
      if (now - this._activeHoldJog.lastDispatchTime >= this.HOLD_JOG_DISPATCH_INTERVAL_MS) {
        this._activeHoldJog.lastDispatchTime = now;
        this.sendMovSpline({ duration: 0 }).catch(() => {});
      }
    }

    if (typeof requestAnimationFrame === 'function') {
      this._activeHoldJog.rafId = requestAnimationFrame(this._runHoldJogLoop.bind(this));
    }
  }

  /**
   * Finalizes hold-jog on pointer up, cancel, or leave
   * @private
   */
  _stopHoldJog() {
    if (!this._activeHoldJog) return;

    const { isHeld, timerId, rafId, element, pointerId } = this._activeHoldJog;

    if (timerId) clearTimeout(timerId);
    if (rafId && typeof cancelAnimationFrame === 'function') cancelAnimationFrame(rafId);
    if (element && pointerId && typeof element.releasePointerCapture === 'function') {
      try {
        element.releasePointerCapture(pointerId);
      } catch (_) {}
    }

    this._activeHoldJog = null;

    if (isHeld) {
      this._justFinishedHold = true;
      setTimeout(() => { this._justFinishedHold = false; }, 100);
      if (this.executionMode === 'direct_drive' && this.isArmed) {
        this.flushFinalTarget(100);
      }
    }
  }

  /**
   * Binds single-tap and hold-jog acceleration listeners to a button
   * @param {HTMLElement} btn
   * @param {'joint'|'cartesian'} type
   * @param {string} id
   * @param {number} dir
   * @private
   */
  _bindJogRamping(btn, type, id, dir) {
    if (!btn) return;

    const onPointerDown = (e) => this._startHoldJog(type, id, dir, e);
    const onPointerUp = () => this._stopHoldJog();
    const onPointerCancel = () => this._stopHoldJog();
    const onPointerLeave = (e) => {
      if (this._activeHoldJog && (!this._activeHoldJog.element.hasPointerCapture || !this._activeHoldJog.element.hasPointerCapture(e.pointerId))) {
        this._stopHoldJog();
      }
    };
    const onClick = () => {
      if (this._justFinishedHold) {
        this._justFinishedHold = false;
        return;
      }
      if (type === 'joint') {
        const stepAttr = parseFloat(btn.getAttribute('data-step') || '0');
        const effectiveStep = (stepAttr !== 0 && !isNaN(stepAttr)) ? stepAttr : (dir * this.jointStepSize);
        this.nudgeJoint(id, effectiveStep, false);
      } else if (type === 'cartesian') {
        const stepAttr = parseFloat(btn.getAttribute('data-step') || '0');
        const effectiveStep = (stepAttr !== 0 && !isNaN(stepAttr)) ? Math.abs(stepAttr) : this.cartesianStepSize;
        this.stepCartesian(id, dir, effectiveStep);
      }
    };

    btn.addEventListener('pointerdown', onPointerDown);
    btn.addEventListener('pointerup', onPointerUp);
    btn.addEventListener('pointercancel', onPointerCancel);
    btn.addEventListener('pointerleave', onPointerLeave);
    btn.addEventListener('click', onClick);

    this._eventCleanups.push(() => {
      btn.removeEventListener('pointerdown', onPointerDown);
      btn.removeEventListener('pointerup', onPointerUp);
      btn.removeEventListener('pointercancel', onPointerCancel);
      btn.removeEventListener('pointerleave', onPointerLeave);
      btn.removeEventListener('click', onClick);
    });
  }

  /**
   * Unbinds all DOM event listeners
   */
  unbindDom() {
    for (const cleanup of this._eventCleanups) {
      try {
        cleanup();
      } catch (_) {}
    }
    this._eventCleanups = [];
    this._domElements = {};
    this._domBound = false;
    this.container = null;
  }

  /**
   * Disposes the controller and cleans up all listeners
   */
  dispose() {
    this.unbindDom();
    if (this._serialDataUnsub) this._serialDataUnsub();
    this._serialDataUnsub = null;
    for (const unsub of this._unsubState) {
      try {
        unsub();
      } catch (_) {}
    }
    this._unsubState = [];
  }
}

export default MotionController;
