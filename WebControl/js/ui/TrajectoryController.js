/**
 * TrajectoryController.js - Master Trajectory Waypoint Planner & Teach/Playback Studio Controller
 * 
 * Controls visual timeline waypoints, sequence player engine (Play/Pause/Step/Loop/Speed),
 * live/ghost pose capture, ghost arm 3D preview, bidirectional JSON import/export,
 * LocalStorage preset management, and hardware Teach & Playback synchronization.
 */

import { appState as defaultAppState } from '../state/AppState.js';
import { presetManager as defaultPresetManager, PresetManager } from '../state/PresetManager.js';
import { buildFramedPacket, buildMovOrientationCommand, buildTeachCommand, buildGripCommand, buildStopCommand } from '../comms/PacketBuilder.js';
import { forwardKinematics6DOF } from '../kinematics/RobotKinematics.js';

export const PlaybackState = Object.freeze({
  IDLE: 'IDLE',
  PLAYING: 'PLAYING',
  PAUSED: 'PAUSED',
  STOPPED: 'STOPPED'
});

export const MAX_TEACH_POINTS = 64;

export class TrajectoryController {
  /**
   * @param {Object} [options]
   * @param {HTMLElement|string} [options.container] - DOM element or selector
   * @param {AppState} [options.appState] - Central reactive state store
   * @param {WebSerialManager} [options.serialManager] - WebSerial communication manager
   * @param {PresetManager} [options.presetManager] - Preset library engine
   * @param {number} [options.speedScale=1.0] - Playback speed scaling multiplier
   * @param {boolean} [options.loop=false] - Loop sequence toggle
   */
  constructor(options = {}) {
    this.appState = options.appState || defaultAppState;
    this.serialManager = options.serialManager || null;
    this.presetManager = options.presetManager || defaultPresetManager;

    // Sequence Player Engine State
    this.playerState = PlaybackState.IDLE;
    this.currentIndex = 0;
    this.speedScale = Math.min(5.0, Math.max(0.05, Number(options.speedScale || 1.0)));
    this.loop = Boolean(options.loop);
    this.elapsedTime = 0;
    this.dispatchedPackets = [];

    // FIFO Lookahead pacing: number of waypoints to pre-dispatch into firmware buffer
    this.LOOKAHEAD_DEPTH = 3;
    // Gripper dwell time (ms) after gripper actuation before next motion dispatches
    this.gripperDwellMs = Number(options.gripperDwellMs ?? 150);
    // Tracks how many lookahead waypoints have been pre-fired beyond currentIndex
    this._lookaheadIndex = 0;
    // Timers for pre-dispatched lookahead waypoints
    this._lookaheadTimers = [];
    // Set of waypoint indices already transmitted to prevent duplicate dispatch
    this._dispatchedIndices = new Set();

    // Internal timing controls
    this._stepTimer = null;
    this._stepStartTime = 0;
    this._stepDuration = 0;
    this._stepRemaining = 0;

    // Selected Waypoint for Inspector/Highlight
    this.selectedWaypointId = null;

    // DOM Binding tracking
    this.container = null;
    this._domBound = false;
    this._domElements = {};
    this._eventCleanups = [];
    this._unsubState = [];

    // Load initial sequence into AppState if empty
    this._initInitialSequence();

    // Subscribe to AppState updates
    this._initAppStateListeners();

    // Bind DOM if container is provided and document exists
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
   * Initializes baseline sequence if none exists
   * @private
   */
  _initInitialSequence() {
    if (!this.appState) return;
    const current = this.appState.getState('sequence');
    if (!current || !current.waypoints || current.waypoints.length === 0) {
      const homePreset = this.presetManager.getPreset('Home') || this.presetManager.getPreset('Pick & Place Demo');
      if (homePreset) {
        this.appState.setSequence(homePreset);
      }
    }
  }

  /**
   * Attaches listeners to AppState pub/sub bus
   * @private
   */
  _initAppStateListeners() {
    if (!this.appState) return;

    // Listen for safety emergency stop: IMMEDIATELY halt active playback
    const unsubEstop = this.appState.subscribe('safety:estop', (isEstop) => {
      if (isEstop && this.playerState === PlaybackState.PLAYING) {
        this.stop();
        if (this.appState) {
          this.appState.addConsoleLog({
            type: 'WARN',
            text: 'Trajectory Player halted immediately: EMERGENCY STOP active'
          });
        }
        this._updateDomUI();
      }
    });
    this._unsubState.push(unsubEstop);

    // Listen for sequence updates (e.g. from imports or presets)
    const unsubSeq = this.appState.subscribe('sequence:updated', () => {
      this._updateDomUI();
    });
    this._unsubState.push(unsubSeq);

    // Listen for teach state changes
    const unsubTeach = this.appState.subscribe('stateChange:teach', () => {
      this._updateDomTeachUI();
    });
    this._unsubState.push(unsubTeach);
  }

  /**
   * Sets serial manager instance
   * @param {WebSerialManager} manager
   */
  setSerialManager(manager) {
    this.serialManager = manager;
  }

  // ==========================================================================
  // WAYPOINT TIMELINE CRUD OPERATIONS
  // ==========================================================================

  /**
   * Returns current active sequence from AppState
   * @returns {Object}
   */
  getSequence() {
    return this.appState.getState('sequence') || { sequenceName: 'Sequence', loop: this.loop, waypoints: [] };
  }

  _orientationControls() {
    if (typeof document === 'undefined') return {};
    const number = (id) => {
      const input = document.getElementById(id);
      return input && input.value.trim() !== '' && Number.isFinite(Number(input.value))
        ? Number(input.value) : undefined;
    };
    const wristButton = document.getElementById('btn_orient_mode_wrist');
    const activeFrame = document.querySelector('[data-coord-frame].active');
    const selectedFrame = document.getElementById('select_coord_frame')?.value;
    const frame = activeFrame?.dataset.coordFrame || selectedFrame;
    const w = number('select_wrist_mode');
    return {
      roll: number('input_roll'), pitch: number('input_pitch'), yaw: number('input_yaw'),
      orientationMode: wristButton?.classList.contains('active') ? 'wrist' : '6dof',
      ...(w >= 0 && w <= 6 ? { w } : {}),
      frame: ['world', 'base', 'tool'].includes(frame) ? frame : 'base',
      x: number('input_x'), y: number('input_y'), z: number('input_z')
    };
  }

  _completeWaypoint(source, sanitized, controls = null) {
    const wp = { ...sanitized };
    const joints = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'];
    const hasJoints = joints.every(key => source[key] !== undefined && source[key] !== null && Number.isFinite(Number(source[key])));
    const fk = hasJoints ? forwardKinematics6DOF(...joints.map(key => Number(source[key]))) : null;
    for (const axis of ['x', 'y', 'z']) {
      const value = source[axis] ?? controls?.[axis];
      if (value !== undefined && value !== null && Number.isFinite(Number(value))) wp[axis] = Number(value);
      else if (fk) wp[axis] = fk.tcp[axis];
    }
    const explicit = ['roll', 'pitch', 'yaw'].every(key =>
      source[key] !== undefined && source[key] !== null && Number.isFinite(Number(source[key])));
    const controlEuler = controls && ['roll', 'pitch', 'yaw'].every(key => Number.isFinite(controls[key]));
    if (explicit || controlEuler || fk) {
      for (const key of ['roll', 'pitch', 'yaw']) {
        wp[key] = explicit ? Number(source[key]) : controlEuler ? controls[key] : fk[key];
      }
    }
    const w = source.w ?? controls?.w;
    if (w !== undefined && w !== null && Number.isInteger(Number(w)) && Number(w) >= 0 && Number(w) <= 6) wp.w = Number(w);
    const mode = source.orientationMode ?? controls?.orientationMode ?? (wp.w !== undefined ? 'wrist' : '6dof');
    wp.orientationMode = mode === 'wrist' || mode === 'preset' ? 'wrist' : '6dof';
    const frame = source.frame ?? controls?.frame;
    wp.frame = ['world', 'base', 'tool'].includes(frame) ? frame : 'base';
    return wp;
  }

  /**
   * Sets entire sequence dataset
   * @param {Object} seq
   */
  setSequence(seq) {
    this.stop();
    const valResult = PresetManager.validateSequence(seq);
    if (!valResult.valid) {
      throw new Error(`Invalid sequence: ${valResult.errors.join('; ')}`);
    }
    this.loop = valResult.sequence.loop;
    valResult.sequence.waypoints = valResult.sequence.waypoints.map((wp, index) =>
      this._completeWaypoint(seq.waypoints[index], wp));
    this.appState.setSequence(valResult.sequence);
    this.currentIndex = 0;
    this.selectedWaypointId = valResult.sequence.waypoints.length > 0 ? valResult.sequence.waypoints[0].id : null;
    this._updateDomUI();
  }

  /**
   * Appends or inserts a new waypoint into the timeline
   * 
   * @param {Object} [wpData={}]
   * @param {number} [atIndex=-1]
   * @returns {Object} The created waypoint
   */
  addWaypoint(wpData = {}, atIndex = -1) {
    const seq = this.getSequence();
    const newIndex = atIndex >= 0 ? atIndex : seq.waypoints.length;
    const valResult = PresetManager.validateWaypoint(wpData, newIndex);
    if (!valResult.valid) throw new Error(`Invalid waypoint: ${valResult.errors.join('; ')}`);
    const wp = this._completeWaypoint(wpData, valResult.waypoint, this._orientationControls());
    if (atIndex >= 0 && atIndex < seq.waypoints.length) {
      seq.waypoints.splice(atIndex, 0, wp);
      this.appState.setSequence(seq);
    } else {
      seq.waypoints.push(wp);
      this.appState.setSequence(seq);
    }

    this.selectedWaypointId = wp.id;
    this._updateDomUI();
    return wp;
  }

  /**
   * Updates an existing waypoint by ID or index
   * 
   * @param {string|number} idOrIndex
   * @param {Object} updates
   * @returns {Object|null}
   */
  updateWaypoint(idOrIndex, updates) {
    const seq = this.getSequence();
    let targetWp = null;

    if (typeof idOrIndex === 'number') {
      targetWp = seq.waypoints[idOrIndex];
    } else if (typeof idOrIndex === 'string') {
      targetWp = seq.waypoints.find(w => w.id === idOrIndex);
    }

    if (!targetWp) return null;

    const merged = { ...targetWp, ...updates };
    const valResult = PresetManager.validateWaypoint(merged, 0);
    if (!valResult.valid) throw new Error(`Invalid waypoint: ${valResult.errors.join('; ')}`);
    const captureControls = ['x', 'y', 'z', 'roll', 'pitch', 'yaw', 'w', 'orientationMode', 'frame'].some(key => key in updates);
    const waypoint = this._completeWaypoint(merged, valResult.waypoint, captureControls ? this._orientationControls() : null);
    const updated = this.appState.updateWaypoint(targetWp.id, waypoint);
    this._updateDomUI();
    return updated;
  }

  /**
   * Deletes a waypoint by index or ID
   * 
   * @param {number|string} indexOrId
   * @returns {boolean}
   */
  deleteWaypoint(indexOrId) {
    const seq = this.getSequence();
    let idToDelete = null;

    if (typeof indexOrId === 'number') {
      if (indexOrId >= 0 && indexOrId < seq.waypoints.length) {
        idToDelete = seq.waypoints[indexOrId].id;
      }
    } else if (typeof indexOrId === 'string') {
      idToDelete = indexOrId;
    }

    if (!idToDelete) return false;

    const res = this.appState.removeWaypoint(idToDelete);
    if (this.selectedWaypointId === idToDelete) {
      const updatedSeq = this.getSequence();
      this.selectedWaypointId = updatedSeq.waypoints.length > 0 ? updatedSeq.waypoints[0].id : null;
    }
    if (this.currentIndex >= this.getSequence().waypoints.length) {
      this.currentIndex = Math.max(0, this.getSequence().waypoints.length - 1);
    }
    this._updateDomUI();
    return res;
  }

  /**
   * Alias for deleteWaypoint to support timeline/external callers
   * @param {number|string} indexOrId
   * @returns {boolean}
   */
  removeWaypoint(indexOrId) {
    return this.deleteWaypoint(indexOrId);
  }

  /**
   * Duplicates a waypoint and inserts it directly after the source
   * 
   * @param {number|string} indexOrId
   * @returns {Object|null} The duplicated waypoint
   */
  duplicateWaypoint(indexOrId) {
    const seq = this.getSequence();
    let srcIndex = -1;

    if (typeof indexOrId === 'number') {
      srcIndex = indexOrId;
    } else {
      srcIndex = seq.waypoints.findIndex(w => w.id === indexOrId);
    }

    if (srcIndex < 0 || srcIndex >= seq.waypoints.length) {
      return null;
    }

    const srcWp = seq.waypoints[srcIndex];
    const duplicateData = {
      ...srcWp,
      id: `wp_${Date.now()}_${Math.random().toString(36).substr(2, 6)}`,
      label: `${srcWp.label} (Copy)`
    };

    return this.addWaypoint(duplicateData, srcIndex + 1);
  }

  /**
   * Moves a waypoint from one position to another
   * 
   * @param {number} fromIndex
   * @param {number} toIndex
   * @returns {boolean}
   */
  moveWaypoint(fromIndex, toIndex) {
    const res = this.appState.reorderWaypoints(fromIndex, toIndex);
    if (res) {
      this._updateDomUI();
    }
    return res;
  }

  /**
   * Shifts a waypoint up one slot in timeline
   */
  moveUp(index) {
    if (index > 0) {
      return this.moveWaypoint(index, index - 1);
    }
    return false;
  }

  /**
   * Shifts a waypoint down one slot in timeline
   */
  moveDown(index) {
    const seq = this.getSequence();
    if (index >= 0 && index < seq.waypoints.length - 1) {
      return this.moveWaypoint(index, index + 1);
    }
    return false;
  }

  /**
   * Inverts order of all waypoints in active sequence
   */
  reverseSequence() {
    const seq = this.getSequence();
    seq.waypoints.reverse();
    this.appState.setSequence(seq);
    this._updateDomUI();
  }

  /**
   * Clears all waypoints from active sequence
   */
  clearAllWaypoints() {
    this.stop();
    this.appState.clearWaypoints();
    this.selectedWaypointId = null;
    this.currentIndex = 0;
    this.playerState = PlaybackState.IDLE;
    this._updateDomUI();
  }

  // ==========================================================================
  // POSE CAPTURE & 3D GHOST PREVIEW
  // ==========================================================================

  /**
   * Captures current Live Pose from telemetry and appends as a new waypoint
   * 
   * @param {string} [customLabel]
   * @param {string} [customComment]
   * @returns {Object}
   */
  captureLivePose(customLabel, customComment) {
    const live = this.appState.getState('livePose') || {};
    const seq = this.getSequence();
    const wpCount = seq.waypoints.length + 1;

    const wpData = {
      label: customLabel || `Live Pose ${wpCount}`,
      j1: Number(live.j1 ?? 0.0),
      j2: Number(live.j2 ?? -90.0),
      j3: Number(live.j3 ?? 90.0),
      j4: Number(live.j4 ?? 90.0),
      j5: Number(live.j5 ?? 90.0),
      j6: Number(live.j6 ?? 0.0),
      dx: Number(live.dx ?? 0),
      dy: Number(live.dy ?? 0),
      dur: 1500,
      grip: Number(live.grip ?? 0),
      comment: customComment || 'Captured from live robot telemetry'
    };

    const added = this.addWaypoint(wpData);
    if (this.appState) {
      this.appState.addConsoleLog({
        type: 'INFO',
        text: `Captured Live Pose: J1=${wpData.j1}°, J2=${wpData.j2}°, J3=${wpData.j3}°, Grip=${wpData.grip}°`
      });
    }
    return added;
  }

  /**
   * Captures current Ghost Pose from 3D visualizer / sliders and appends as a waypoint
   * 
   * @param {string} [customLabel]
   * @param {string} [customComment]
   * @returns {Object}
   */
  captureGhostPose(customLabel, customComment) {
    const ghost = this.appState.getState('ghostPose') || {};
    const seq = this.getSequence();
    const wpCount = seq.waypoints.length + 1;

    const wpData = {
      label: customLabel || `Ghost Pose ${wpCount}`,
      j1: Number(ghost.j1 ?? 0.0),
      j2: Number(ghost.j2 ?? -90.0),
      j3: Number(ghost.j3 ?? 90.0),
      j4: Number(ghost.j4 ?? 90.0),
      j5: Number(ghost.j5 ?? 90.0),
      j6: Number(ghost.j6 ?? 0.0),
      dx: 0,
      dy: 0,
      dur: Number(ghost.duration || 1500),
      grip: Number(ghost.grip ?? 90),
      comment: customComment || 'Captured from ghost target pose'
    };

    const added = this.addWaypoint(wpData);
    if (this.appState) {
      this.appState.addConsoleLog({
        type: 'INFO',
        text: `Captured Ghost Pose: J1=${wpData.j1}°, J2=${wpData.j2}°, J3=${wpData.j3}°, Grip=${wpData.grip}°`
      });
    }
    return added;
  }

  /**
   * Updates 3D visualizer ghost arm to match the selected waypoint pose
   * 
   * @param {number|string} indexOrId
   */
  previewWaypointInGhost(indexOrId) {
    const seq = this.getSequence();
    let wp = null;

    if (typeof indexOrId === 'number') {
      wp = seq.waypoints[indexOrId];
    } else {
      wp = seq.waypoints.find(w => w.id === indexOrId);
    }

    if (!wp) return;

    this.selectedWaypointId = wp.id;
    if (this.appState) {
      this.appState.updateGhostPose({
        j1: wp.j1,
        j2: wp.j2,
        j3: wp.j3,
        j4: wp.j4,
        j5: wp.j5,
        j6: wp.j6,
        x: wp.x, y: wp.y, z: wp.z,
        roll: wp.roll, pitch: wp.pitch, yaw: wp.yaw,
        orientationMode: wp.orientationMode, w: wp.w, frame: wp.frame,
        grip: wp.grip,
        duration: wp.dur
      });
      this.appState.addConsoleLog({
        type: 'INFO',
        text: `Previewing Waypoint "${wp.label}" on Ghost Arm (J1=${wp.j1}°, J2=${wp.j2}°, J3=${wp.j3}°)`
      });
    }

    this._updateDomUI();
  }

  // ==========================================================================
  // SEQUENCE PLAYER ENGINE
  // ==========================================================================

  /**
   * Calculates total execution duration of sequence accounting for speed scaling
   * @returns {number} Duration in ms
   */
  getTotalDuration() {
    const seq = this.getSequence();
    const rawSum = seq.waypoints.reduce((sum, wp) => sum + wp.dur, 0);
    return rawSum / Math.max(0.05, this.speedScale);
  }

  /**
   * Retrieves live player status and progress metrics
   * @returns {Object}
   */
  getProgress() {
    const seq = this.getSequence();
    const totalWaypoints = seq.waypoints.length;
    const totalTime = this.getTotalDuration();
    const progressPercent = totalTime > 0 ? Math.min(100, (this.elapsedTime / totalTime) * 100) : 0;

    return {
      currentIndex: this.currentIndex,
      totalWaypoints,
      progressPercent,
      elapsedTime: this.elapsedTime,
      totalTime,
      state: this.playerState,
      isPlaying: this.playerState === PlaybackState.PLAYING,
      isPaused: this.playerState === PlaybackState.PAUSED,
      loop: this.loop,
      speedScale: this.speedScale
    };
  }

  /**
   * Starts or resumes playback of active sequence
   * @returns {boolean}
   */
  play() {
    const seq = this.getSequence();
    if (!seq.waypoints || seq.waypoints.length === 0) {
      if (this.appState) {
        this.appState.addConsoleLog({ type: 'WARN', text: 'Cannot play: Trajectory sequence is empty' });
      }
      return false;
    }

    // Safety check: ESTOP active blocks playback
    const safety = this.appState.getState('safety') || {};
    if (safety.estop) {
      if (this.appState) {
        this.appState.addConsoleLog({ type: 'ERR', text: 'Cannot start playback: EMERGENCY STOP is active!' });
      }
      return false;
    }

    if (this.playerState === PlaybackState.PAUSED) {
      return this.resume();
    }

    // If already playing, do nothing
    if (this.playerState === PlaybackState.PLAYING) {
      return true;
    }

    this.playerState = PlaybackState.PLAYING;
    if (this.currentIndex < 0 || this.currentIndex >= seq.waypoints.length - 1) {
      this.currentIndex = 0;
    }
    this.elapsedTime = 0;
    this.dispatchedPackets = [];
    this._dispatchedIndices.clear();

    this._syncPlayerStateToApp();
    this._dispatchCurrentWaypointAndSchedule();
    this._updateDomUI();
    return true;
  }

  /**
   * Pauses active sequence execution
   * @returns {boolean}
   */
  pause() {
    if (this._isStepping) {
      this._isStepping = false;
      this._clearStepTimer();
      this.playerState = PlaybackState.PAUSED;

      // Send immediate hardware stop to halt motion
      this._sendSerial(buildStopCommand());

      this._syncPlayerStateToApp();
      this._updateDomUI();
      return true;
    }

    if (this.playerState !== PlaybackState.PLAYING) {
      return false;
    }

    this._clearStepTimer();
    const elapsedInStep = Date.now() - this._stepStartTime;
    this._stepRemaining = Math.max(0, this._stepDuration - elapsedInStep);
    this.playerState = PlaybackState.PAUSED;

    // Send immediate hardware stop to halt motion
    this._sendSerial(buildStopCommand());

    this._syncPlayerStateToApp();
    this._updateDomUI();
    return true;
  }

  /**
   * Resumes paused sequence execution.
   * If hardware is connected, re-dispatches the current waypoint to resync
   * from actual telemetry pose. Then re-primes the FIFO lookahead.
   * @returns {boolean}
   */
  resume() {
    if (this.playerState !== PlaybackState.PAUSED) {
      return false;
    }

    // Safety check
    const safety = this.appState.getState('safety') || {};
    if (safety.estop) {
      if (this.appState) {
        this.appState.addConsoleLog({ type: 'ERR', text: 'Cannot resume: EMERGENCY STOP is active!' });
      }
      return false;
    }

    this.playerState = PlaybackState.PLAYING;
    this._syncPlayerStateToApp();

    const resumeDelay = this._stepRemaining > 0 ? this._stepRemaining : 50;
    this._stepStartTime = Date.now();
    this._stepDuration = resumeDelay;

    const seq = this.getSequence();
    const wp = seq.waypoints[this.currentIndex];

    if (!this.serialManager || !this.serialManager.isConnected()) {
      // Offline preview: resume from the current ghost pose.
      if (wp) {
        const currentTelem = this.appState.getState('ghostPose') || {};
        const startPose = {
          j1: Number(currentTelem.j1 ?? 0),
          j2: Number(currentTelem.j2 ?? -90),
          j3: Number(currentTelem.j3 ?? 90),
          j4: Number(currentTelem.j4 ?? 90),
          j5: Number(currentTelem.j5 ?? 90),
          j6: Number(currentTelem.j6 ?? 0),
          grip: Number(currentTelem.grip ?? 0)
        };
        const targetPose = {
          j1: Number(wp.j1 ?? 0),
          j2: Number(wp.j2 ?? -90),
          j3: Number(wp.j3 ?? 90),
          j4: Number(wp.j4 ?? 90),
          j5: Number(wp.j5 ?? 90),
          j6: Number(wp.j6 ?? 0),
          grip: Number(wp.grip ?? 0)
        };
        this._startGhostInterpolation(startPose, targetPose, resumeDelay);
      }
    } else {
      // Hardware connected: re-dispatch current waypoint to resync firmware pose
      if (wp) {
        const scaledDur = Math.max(10, Math.round(wp.dur / this.speedScale));
        const packet = buildFramedPacket({
          j1: wp.j1, j2: wp.j2, j3: wp.j3, j4: wp.j4, j5: wp.j5, j6: wp.j6 ?? 0,
          dx: wp.dx, dy: wp.dy, dur: scaledDur
        });
        this._sendSerial(packet + '\n');
      }
      // Re-prime FIFO lookahead from resume point
      this._prefillLookahead(seq, resumeDelay);
    }

    this._stepTimer = setTimeout(() => {
      this.advanceWaypoint();
    }, resumeDelay);

    this._updateDomUI();
    return true;
  }

  /**
   * Stops sequence playback and resets progress
   */
  stop() {
    this._isStepping = false;
    this._clearStepTimer();
    this.playerState = PlaybackState.STOPPED;
    this.currentIndex = 0;
    this.elapsedTime = 0;
    this._stepRemaining = 0;
    this._dispatchedIndices.clear();

    // Send immediate hardware stop to halt motion
    this._sendSerial(buildStopCommand());

    this._syncPlayerStateToApp();
    this._updateDomUI();
  }

  /**
   * Steps forward to next waypoint and executes it
   * @returns {boolean}
   */
  stepForward() {
    const seq = this.getSequence();
    if (!seq.waypoints || seq.waypoints.length === 0) return false;

    if (this.currentIndex < seq.waypoints.length - 1) {
      this.currentIndex++;
      this._isStepping = true;
      this._dispatchCurrentWaypoint();
      this._updateDomUI();
      return true;
    } else if (this.loop) {
      this.currentIndex = 0;
      this._isStepping = true;
      this._dispatchCurrentWaypoint();
      this._updateDomUI();
      return true;
    }

    return false;
  }

  /**
   * Steps backward to previous waypoint and executes it
   * @returns {boolean}
   */
  stepBackward() {
    const seq = this.getSequence();
    if (!seq.waypoints || seq.waypoints.length === 0) return false;

    if (this.currentIndex > 0) {
      this.currentIndex--;
      this._isStepping = true;
      this._dispatchCurrentWaypoint();
      this._updateDomUI();
      return true;
    } else if (this.loop) {
      this.currentIndex = seq.waypoints.length - 1;
      this._isStepping = true;
      this._dispatchCurrentWaypoint();
      this._updateDomUI();
      return true;
    }

    return false;
  }

  /**
   * Jumps directly to a specific waypoint index
   * @param {number} index
   * @returns {boolean}
   */
  jumpTo(index) {
    const seq = this.getSequence();
    if (index < 0 || index >= seq.waypoints.length) {
      return false;
    }

    this.currentIndex = index;
    this._isStepping = true;
    if (this.playerState === PlaybackState.PLAYING) {
      this._clearStepTimer();
      this._dispatchCurrentWaypointAndSchedule();
    } else {
      this._dispatchCurrentWaypoint();
    }

    this._updateDomUI();
    return true;
  }

  /**
   * Toggles sequence looping mode
   * @param {boolean} loop
   */
  setLoop(loop) {
    this.loop = Boolean(loop);
    const seq = this.getSequence();
    seq.loop = this.loop;
    this.appState.setState('sequence.loop', this.loop);
    this._updateDomUI();
  }

  /**
   * Sets playback speed scaling multiplier (0.05x to 5.0x)
   * @param {number} scale
   */
  setSpeedScale(scale) {
    this.speedScale = Math.min(5.0, Math.max(0.05, Number(scale || 1.0)));
    this._updateDomUI();
  }

  /**
   * Advances waypoint on timer completion
   * @returns {boolean}
   */
  advanceWaypoint() {
    if (this.playerState !== PlaybackState.PLAYING) {
      return false;
    }

    const seq = this.getSequence();
    const currWp = seq.waypoints[this.currentIndex];
    if (currWp) {
      this.elapsedTime += (currWp.dur / this.speedScale);
    }

    if (this.currentIndex < seq.waypoints.length - 1) {
      this.currentIndex++;
      this._dispatchCurrentWaypointAndSchedule();
      this._updateDomUI();
      return true;
    } else if (this.loop) {
      this.currentIndex = 0;
      this._dispatchedIndices.clear();
      this._dispatchCurrentWaypointAndSchedule();
      this._updateDomUI();
      return true;
    } else {
      // Sequence completed
      this.playerState = PlaybackState.IDLE;
      this._clearStepTimer();
      this._syncPlayerStateToApp();
      if (this.appState) {
        this.appState.addConsoleLog({
          type: 'INFO',
          text: `Sequence "${seq.sequenceName}" playback finished successfully (${(this.elapsedTime / 1000).toFixed(2)}s total)`
        });
      }
      this._updateDomUI();
      return false;
    }
  }

  /**
   * Synchronizes active player state (playing, paused, activeIndex, progress) to AppState
   * @private
   */
  _syncPlayerStateToApp() {
    if (!this.appState) return;
    const currentSeq = this.appState.getState('sequence') || {};
    const totalWaypoints = (currentSeq.waypoints || []).length;
    const progress = totalWaypoints > 0 ? ((this.currentIndex + 1) / totalWaypoints) * 100 : 0;

    const isIdleOrStopped = (this.playerState === PlaybackState.IDLE || this.playerState === PlaybackState.STOPPED);
    const activeIndex = isIdleOrStopped ? -1 : this.currentIndex;

    this.appState.setSequenceState({
      isPlaying: this.playerState === PlaybackState.PLAYING,
      isPaused: this.playerState === PlaybackState.PAUSED,
      activeIndex,
      progress: isIdleOrStopped ? 0 : Math.min(100, Math.max(0, progress))
    });

    this.appState.setState('sequence.isPlaying', this.playerState === PlaybackState.PLAYING);
    this.appState.setState('sequence.isPaused', this.playerState === PlaybackState.PAUSED);
    this.appState.setState('sequence.activeIndex', activeIndex);

    // Emit timeline events for timeline scrubbing and playback button sync
    this.appState.emit('trajectory:playbackState', this.playerState);
    this.appState.emit('trajectory:progress', {
      elapsedMs: this.elapsedTime,
      progress: isIdleOrStopped ? 0 : Math.min(100, Math.max(0, progress)),
      activeIndex
    });
  }

  /**
   * Dispatches current waypoint command and schedules next timer tick.
   * Also pre-dispatches the next LOOKAHEAD_DEPTH waypoints into the firmware
   * FIFO using staggered timers so the hardware buffer stays pre-loaded,
   * eliminating the inter-waypoint deceleration stutter.
   * @private
   */
  _dispatchCurrentWaypointAndSchedule() {
    const seq = this.getSequence();
    const wp = seq.waypoints[this.currentIndex];
    if (!wp) return;

    this._clearStepTimer();

    const scaledDur = Math.max(10, Math.round(wp.dur / this.speedScale));
    const dwell = (wp.grip !== undefined) ? Math.max(0, this.gripperDwellMs) : 0;
    const totalStepDur = scaledDur + dwell;
    this._stepStartTime = Date.now();
    this._stepDuration = totalStepDur;
    this._stepRemaining = totalStepDur;

    this._dispatchCurrentWaypoint();

    // FIFO Lookahead pre-fill: dispatch the next LOOKAHEAD_DEPTH waypoints
    // immediately with staggered delays so the firmware buffer is always
    // loaded AHEAD of playback position.
    this._prefillLookahead(seq, totalStepDur);

    this._stepTimer = setTimeout(() => {
      this.advanceWaypoint();
    }, totalStepDur);
  }

  /**
   * Pre-dispatches the next LOOKAHEAD_DEPTH waypoints onto the firmware FIFO
   * using staggered setTimeout calls, keeping firmware buffer depth >= 2.
   * @param {Object} seq - Current sequence
   * @param {number} currentDurMs - Scaled duration of current waypoint
   * @private
   */
  _prefillLookahead(seq, currentDurMs) {
    if (!seq || !Array.isArray(seq.waypoints)) return;
    const waypoints = seq.waypoints;
    let accumulatedDelay = currentDurMs;

    for (let ahead = 1; ahead <= this.LOOKAHEAD_DEPTH; ahead++) {
      let lookIdx = this.currentIndex + ahead;
      // Wrap if looping
      if (lookIdx >= waypoints.length) {
        if (this.loop) {
          lookIdx = lookIdx % waypoints.length;
        } else {
          break; // No more waypoints to pre-load
        }
      }

      const futureWp = waypoints[lookIdx];
      if (!futureWp) break;

      // Do not prefill joint-spline lookahead across a Cartesian linear (MOVL) move
      const futureMoveType = String(futureWp.moveType || futureWp.motionType || '').toLowerCase();
      if (futureMoveType === 'movl') break;

      const futureDur = Math.max(10, Math.round(futureWp.dur / this.speedScale));
      const futureDwell = (futureWp.grip !== undefined) ? Math.max(0, this.gripperDwellMs) : 0;
      const dispatchDelay = accumulatedDelay;

      // Capture lookIdx for closure
      const capturedIdx = lookIdx;
      const t = setTimeout(() => {
        // Only dispatch if still playing and this lookahead matches expectation
        if (this.playerState !== PlaybackState.PLAYING) return;
        if (this._dispatchedIndices.has(capturedIdx)) return;
        const packet = buildFramedPacket({
          j1: futureWp.j1,
          j2: futureWp.j2,
          j3: futureWp.j3,
          j4: futureWp.j4,
          j5: futureWp.j5,
          j6: futureWp.j6 ?? 0,
          dx: futureWp.dx,
          dy: futureWp.dy,
          dur: futureDur
        });
        this._sendSerial(packet + '\n');
        this._dispatchedIndices.add(capturedIdx);
        this._lookaheadIndex = Math.max(this._lookaheadIndex, capturedIdx);

        // Gripper actuation with dwell: delay grip command so arm arrives first
        if (futureWp.grip !== undefined) {
          const totalDelay = futureDur;
          if (totalDelay > 0) {
            const gripTimer = setTimeout(() => {
              if (this.playerState === PlaybackState.PLAYING) {
                this._sendSerial(buildGripCommand(futureWp.grip));
              }
            }, totalDelay);
            this._lookaheadTimers.push(gripTimer);
          } else {
            this._sendSerial(buildGripCommand(futureWp.grip));
          }
        }
      }, dispatchDelay);

      this._lookaheadTimers.push(t);
      accumulatedDelay += futureDur + futureDwell;
    }
  }

  /**
   * Dispatches framed CRC packet (or MOVL linear command) and gripper command for active waypoint
   * @private
   */
  _dispatchCurrentWaypoint() {
    const seq = this.getSequence();
    const wp = seq.waypoints[this.currentIndex];
    if (!wp) return;

    const scaledDur = Math.max(1, Math.round(wp.dur / this.speedScale));
    const isMovl = String(wp.moveType || wp.motionType || '').toLowerCase() === 'movl';

    let packet;
    if (isMovl) {
      const fk = forwardKinematics6DOF(
        wp.j1 ?? 0,
        wp.j2 ?? 0,
        wp.j3 ?? 0,
        wp.j4 ?? 0,
        wp.j5 ?? 0,
        wp.j6 ?? 0
      );
      const tx = Number.isFinite(Number(wp.x)) ? Number(wp.x) : fk.tcp.x;
      const ty = Number.isFinite(Number(wp.y)) ? Number(wp.y) : fk.tcp.y;
      const tz = Number.isFinite(Number(wp.z)) ? Number(wp.z) : fk.tcp.z;
      const tr = Number.isFinite(Number(wp.roll)) ? Number(wp.roll) : fk.roll;
      const tp = Number.isFinite(Number(wp.pitch)) ? Number(wp.pitch) : fk.pitch;
      const tyw = Number.isFinite(Number(wp.yaw)) ? Number(wp.yaw) : fk.yaw;
      packet = buildMovOrientationCommand(tx, ty, tz, tr, tp, tyw, scaledDur, true).trim();
    } else {
      packet = buildFramedPacket({
        j1: wp.j1,
        j2: wp.j2,
        j3: wp.j3,
        j4: wp.j4,
        j5: wp.j5,
        j6: wp.j6 ?? 0,
        dx: wp.dx,
        dy: wp.dy,
        dur: scaledDur
      });
    }

    // Send packet via WebSerial if connected (guard against double dispatch if already sent by lookahead)
    if (!this._dispatchedIndices.has(this.currentIndex)) {
      this._sendSerial(packet + '\n');
      this._dispatchedIndices.add(this.currentIndex);
    }
    // Gripper: delay by scaledDur so the arm finishes motion before gripping, then dwell time elapses before advance
    if (wp.grip !== undefined) {
      const gripDelay = scaledDur;
      if (gripDelay > 0 && (this.playerState === PlaybackState.PLAYING || this._isStepping)) {
        const gripTimer = setTimeout(() => {
          if (this.playerState === PlaybackState.PLAYING || this._isStepping) {
            this._sendSerial(buildGripCommand(wp.grip));
          }
        }, gripDelay);
        this._lookaheadTimers.push(gripTimer);
      } else {
        this._sendSerial(buildGripCommand(wp.grip));
      }
    }

    const record = {
      index: this.currentIndex,
      label: wp.label,
      packet,
      grip: wp.grip,
      dur: scaledDur,
      timestamp: Date.now()
    };
    this.dispatchedPackets.push(record);

    // Preview the target on the ghost arm; live pose is reserved for hardware telemetry.
    if (this.appState) {
      if (!this.serialManager || !this.serialManager.isConnected()) {
        const currentTelem = this.appState.getState('ghostPose') || {};
        const startPose = {
          j1: Number(currentTelem.j1 ?? 0),
          j2: Number(currentTelem.j2 ?? -90),
          j3: Number(currentTelem.j3 ?? 90),
          j4: Number(currentTelem.j4 ?? 90),
          j5: Number(currentTelem.j5 ?? 90),
          j6: Number(currentTelem.j6 ?? 0),
          grip: Number(currentTelem.grip ?? 0)
        };
        const targetPose = {
          j1: Number(wp.j1 ?? 0),
          j2: Number(wp.j2 ?? -90),
          j3: Number(wp.j3 ?? 90),
          j4: Number(wp.j4 ?? 90),
          j5: Number(wp.j5 ?? 90),
          j6: Number(wp.j6 ?? 0),
          grip: Number(wp.grip ?? 0)
        };
        this._startGhostInterpolation(startPose, targetPose, scaledDur);
      } else {
        this.appState.updateGhostPose({
          j1: wp.j1,
          j2: wp.j2,
          j3: wp.j3,
          j4: wp.j4,
          j5: wp.j5,
          j6: wp.j6 ?? 0,
          grip: wp.grip
        });
      }
      this.appState.setState('sequence.activeIndex', this.currentIndex);
    }
  }

  /**
   * Smoothly interpolates ghost arm pose across waypoint duration (offline/preview playback)
   * @private
   */
  _startGhostInterpolation(startPose, targetPose, durationMs) {
    this._clearInterpolation();

    if (!durationMs || durationMs <= 20) {
      if (this.appState) {
        this.appState.updateGhostPose(targetPose);
      }
      return;
    }

    const startTime = Date.now();

    // Quintic ease-in-out curve matching firmware trajectory physics
    const easeQuintic = (t) => {
      const tau = Math.max(0, Math.min(1, t));
      return tau * tau * tau * (10 + tau * (-15 + tau * 6));
    };

    const animate = () => {
      const elapsed = Date.now() - startTime;
      const progress = Math.min(1.0, elapsed / durationMs);
      const s = easeQuintic(progress);

      const interpolated = {
        j1: startPose.j1 + (targetPose.j1 - startPose.j1) * s,
        j2: startPose.j2 + (targetPose.j2 - startPose.j2) * s,
        j3: startPose.j3 + (targetPose.j3 - startPose.j3) * s,
        j4: startPose.j4 + (targetPose.j4 - startPose.j4) * s,
        j5: startPose.j5 + (targetPose.j5 - startPose.j5) * s,
        j6: startPose.j6 + (targetPose.j6 - startPose.j6) * s,
        grip: startPose.grip + (targetPose.grip - startPose.grip) * s
      };

      if (this.appState) {
        this.appState.updateGhostPose(interpolated);
      }

      if (progress < 1.0 && (this.playerState === PlaybackState.PLAYING || this._isStepping)) {
        if (typeof requestAnimationFrame !== 'undefined') {
          this._interpAnimFrame = requestAnimationFrame(animate);
        } else if (typeof setTimeout !== 'undefined') {
          this._interpAnimFrame = setTimeout(animate, 16);
        }
      } else {
        this._interpAnimFrame = null;
        this._isStepping = false;
      }
    };

    if (typeof requestAnimationFrame !== 'undefined') {
      this._interpAnimFrame = requestAnimationFrame(animate);
    } else if (typeof setTimeout !== 'undefined') {
      this._interpAnimFrame = setTimeout(animate, 16);
    }
  }

  /**
   * Clears active ghost pose animation interpolation
   * @private
   */
  _clearInterpolation() {
    if (this._interpAnimFrame) {
      if (typeof cancelAnimationFrame !== 'undefined') {
        cancelAnimationFrame(this._interpAnimFrame);
      }
      if (typeof clearTimeout !== 'undefined') {
        clearTimeout(this._interpAnimFrame);
      }
      this._interpAnimFrame = null;
    }
  }

  /**
   * Safe transmission helper supporting send() and write()
   * @private
   */
  _sendSerial(data) {
    if (this.serialManager && typeof this.serialManager.isConnected === 'function' && this.serialManager.isConnected()) {
      try {
        if (typeof this.serialManager.send === 'function') {
          this.serialManager.send(data);
        } else if (typeof this.serialManager.write === 'function') {
          this.serialManager.write(data);
        }
      } catch (err) {
        console.warn('TrajectoryController: Serial dispatch error:', err);
      }
    }
  }

  /**
   * Clears active step timer
   * @private
   */
  _clearStepTimer() {
    if (this._stepTimer) {
      clearTimeout(this._stepTimer);
      this._stepTimer = null;
    }
    // Cancel all pre-dispatched lookahead timers so firmware doesn't receive
    // stale future waypoints after a stop or pause.
    if (this._lookaheadTimers && this._lookaheadTimers.length > 0) {
      this._lookaheadTimers.forEach(t => clearTimeout(t));
      this._lookaheadTimers = [];
    }
    this._lookaheadIndex = 0;
    if (this._dispatchedIndices) {
      this._dispatchedIndices.clear();
    }
    this._clearInterpolation();
  }

  // ==========================================================================
  // HARDWARE TEACH & PLAYBACK SYNCHRONIZATION
  // ==========================================================================

  /**
   * Sends TEACH START command to firmware
   */
  teachStart() {
    this._sendSerial(buildTeachCommand('START'));
    if (this.appState) {
      this.appState.setState('teach.active', true);
      this.appState.setState('teach.pointCount', 0);
      this.appState.setState('teach.points', []);
      this.appState.addConsoleLog({ type: 'TX', text: 'TEACH START -> Firmware recording active' });
    }
    this._updateDomTeachUI();
  }

  /**
   * Sends TEACH RECORD command and records current live pose
   */
  teachRecord() {
    const live = this.appState.getState('livePose') || {};
    const teachState = this.appState.getState('teach') || { points: [] };
    const currentPoints = teachState.points || [];

    if (currentPoints.length >= MAX_TEACH_POINTS) {
      if (this.appState) {
        this.appState.addConsoleLog({
          type: 'ERR',
          text: `Teach buffer overflow: maximum ${MAX_TEACH_POINTS} points reached!`
        });
      }
      return false;
    }

    this._sendSerial(buildTeachCommand('RECORD'));

    const newPt = {
      j1: Number(live.j1 ?? 0),
      j2: Number(live.j2 ?? -90),
      j3: Number(live.j3 ?? 90),
      j4: Number(live.j4 ?? 90),
      j5: Number(live.j5 ?? 90),
      grip: Number(live.grip ?? 90),
      timestamp: Date.now()
    };

    currentPoints.push(newPt);
    this.appState.setState('teach.points', currentPoints);
    this.appState.setState('teach.pointCount', currentPoints.length);

    if (this.appState) {
      this.appState.addConsoleLog({
        type: 'TX',
        text: `TEACH RECORD (#${currentPoints.length}): J1=${newPt.j1}°, J2=${newPt.j2}°, J3=${newPt.j3}°`
      });
    }

    this._updateDomTeachUI();
    return true;
  }

  /**
   * Sends TEACH STOP command to firmware
   */
  teachStop() {
    this._sendSerial(buildTeachCommand('STOP'));
    if (this.appState) {
      this.appState.setState('teach.active', false);
      const pts = this.appState.getState('teach.points') || [];
      this.appState.addConsoleLog({
        type: 'TX',
        text: `TEACH STOP -> Recorded ${pts.length} teach waypoints`
      });
    }
    this._updateDomTeachUI();
  }

  /**
   * Sends TEACH PLAY command to firmware to execute hardware trajectory playback
   */
  teachPlay() {
    this._sendSerial(buildTeachCommand('PLAY'));
    if (this.appState) {
      this.appState.addConsoleLog({ type: 'TX', text: 'TEACH PLAY -> Triggered firmware hardware playback' });
    }
  }

  /**
   * Sends TEACH CLEAR command to firmware and flushes local buffer
   */
  teachClear() {
    this._sendSerial(buildTeachCommand('CLEAR'));
    if (this.appState) {
      this.appState.setState('teach.active', false);
      this.appState.setState('teach.pointCount', 0);
      this.appState.setState('teach.points', []);
      this.appState.addConsoleLog({ type: 'TX', text: 'TEACH CLEAR -> Hardware and local teach buffers cleared' });
    }
    this._updateDomTeachUI();
  }

  /**
   * Converts recorded Teach points into a full editable Timeline Sequence
   * 
   * @param {string} [name='Hardware Teach Recording']
   * @returns {Object}
   */
  convertTeachToSequence(name = 'Hardware Teach Recording') {
    const teachState = this.appState.getState('teach') || {};
    const points = teachState.points || [];

    if (points.length === 0) {
      if (this.appState) {
        this.appState.addConsoleLog({ type: 'WARN', text: 'No teach points available to convert' });
      }
      return null;
    }

    const waypoints = points.map((pt, i) => ({
      id: `wp_teach_${Date.now()}_${i}`,
      label: `Teach Point ${i + 1}`,
      j1: pt.j1,
      j2: pt.j2,
      j3: pt.j3,
      j4: pt.j4,
      j5: pt.j5,
      dx: 0,
      dy: 0,
      dur: 1500,
      grip: pt.grip ?? 90,
      comment: `Recorded at ${new Date(pt.timestamp || Date.now()).toLocaleTimeString()}`
    }));

    const seq = {
      sequenceName: name,
      version: '1.0',
      loop: false,
      description: `Imported from hardware teach recording (${points.length} points)`,
      waypoints
    };

    this.setSequence(seq);
    if (this.appState) {
      this.appState.addConsoleLog({
        type: 'INFO',
        text: `Converted ${points.length} hardware teach points into active timeline sequence!`
      });
    }
    return seq;
  }

  // ==========================================================================
  // PRESET & JSON IMPORT / EXPORT
  // ==========================================================================

  /**
   * Loads a preset from the PresetManager by name
   * @param {string} presetName
   */
  loadPreset(presetName) {
    const preset = this.presetManager.getPreset(presetName);
    if (!preset) {
      if (this.appState) {
        this.appState.addConsoleLog({ type: 'WARN', text: `Preset "${presetName}" not found` });
      }
      return false;
    }
    this.setSequence(preset);
    if (this.appState) {
      this.appState.addConsoleLog({
        type: 'INFO',
        text: `Loaded preset "${preset.sequenceName}" (${preset.waypoints.length} waypoints)`
      });
    }
    return true;
  }

  /**
   * Saves the current timeline sequence as a preset
   * @param {string} presetName
   */
  saveCurrentAsPreset(presetName) {
    const seq = this.getSequence();
    const res = this.presetManager.savePreset(presetName, seq);
    if (res.success) {
      if (this.appState) {
        this.appState.addConsoleLog({
          type: 'INFO',
          text: `Saved current sequence as preset "${presetName}"`
        });
      }
      this._updateDomPresetsUI();
    } else if (this.appState) {
      this.appState.addConsoleLog({
        type: 'ERR',
        text: `Failed to save preset: ${res.error}`
      });
    }
    return res;
  }

  /**
   * Deletes a preset by name
   * @param {string} presetName
   */
  deletePreset(presetName) {
    const res = this.presetManager.deletePreset(presetName);
    if (res) {
      if (this.appState) {
        this.appState.addConsoleLog({ type: 'INFO', text: `Deleted preset "${presetName}"` });
      }
      this._updateDomPresetsUI();
    }
    return res;
  }

  /**
   * Exports current active sequence to JSON string
   * @param {boolean} [pretty=true]
   * @returns {string}
   */
  exportJSON(pretty = true) {
    const seq = this.getSequence();
    const validated = PresetManager.validateSequence(seq);
    if (!validated.valid) throw new Error(`Export failed validation: ${validated.errors.join('; ')}`);
    const exported = {
      sequenceName: validated.sequence.sequenceName,
      version: validated.sequence.version,
      loop: validated.sequence.loop,
      description: validated.sequence.description,
      waypoints: validated.sequence.waypoints.map((wp, index) =>
        this._completeWaypoint(seq.waypoints[index], wp))
    };
    return JSON.stringify(exported, null, pretty ? 2 : 0);
  }

  /**
   * Imports sequence from JSON string or object
   * @param {string|Object} jsonStrOrObj
   */
  importJSON(jsonStrOrObj) {
    // The preset manager validates the legacy schema but strips orientation fields.
    const parsed = typeof jsonStrOrObj === 'string' ? JSON.parse(jsonStrOrObj) : jsonStrOrObj;
    this.setSequence(parsed);
    const seq = this.getSequence();
    if (this.appState) {
      this.appState.addConsoleLog({
        type: 'INFO',
        text: `Imported JSON sequence "${seq.sequenceName}" (${seq.waypoints.length} waypoints)`
      });
    }
    return seq;
  }

  /**
   * Exports current active sequence to standard CNC G-Code text
   * @param {Object} [options]
   * @returns {string}
   */
  exportGCode(options = {}) {
    return this.presetManager.exportSequenceGCode(this.getSequence(), options);
  }

  /**
   * Imports sequence from standard CNC G-Code text
   * @param {string} gcodeText
   * @param {Object} [options]
   */
  importGCode(gcodeText, options = {}) {
    const seq = this.presetManager.importGCode(gcodeText, options);
    this.setSequence(seq);
    if (this.appState) {
      this.appState.addConsoleLog({
        type: 'INFO',
        text: `Imported G-Code sequence "${seq.sequenceName}" (${seq.waypoints.length} points, span ${seq.boundingBox?.spanX}x${seq.boundingBox?.spanY}mm)`
      });
    }
    return seq;
  }

  // ==========================================================================
  // DOM BINDING, RENDERING & EVENT MANAGEMENT
  // ==========================================================================

  /**
   * Unbinds DOM events and cleanups
   */
  unbindDom() {
    for (const cleanup of this._eventCleanups) {
      if (typeof cleanup === 'function') cleanup();
    }
    this._eventCleanups = [];
    this._domBound = false;
    this.container = null;
  }

  /**
   * Binds controller to a DOM container element and renders interface
   * @param {HTMLElement} container
   */
  bindDom(container) {
    if (!container) return;
    this.unbindDom();
    this.container = container;
    this._renderDomLayout();
    this._bindDomEvents();
    this._domBound = true;
    this._updateDomUI();
  }

  /**
   * Renders the complete HTML template for Trajectory Studio
   * @private
   */
  _renderDomLayout() {
    this.container.innerHTML = `
      <div class="trajectory-studio">
        <!-- Studio Header & Toolbar -->
        <div class="studio-header">
          <div class="studio-title-area">
            <h3 class="studio-title">
              <span>Trajectory Planner</span>
            </h3>
            <span class="badge sequence-badge" id="seq-name-badge">0 PTS</span>
          </div>

          <!-- Presets & File Actions -->
          <div class="studio-toolbar-presets">
            <div class="preset-row-top" style="display: flex; gap: 6px;">
              <select class="select-preset" id="preset-selector" title="Select and automatically load predefined preset" style="flex: 1;">
                <!-- Populated dynamically -->
              </select>
              <button class="btn btn-secondary btn-sm" id="btn-save-preset" title="Save as New Preset">Save</button>
              <button id="btn-load-preset" style="display: none;" aria-hidden="true"></button>
            </div>
            <div class="preset-row-bottom" style="display: flex; gap: 6px; margin-top: 4px;">
              <button class="btn btn-secondary btn-sm" id="btn-import-sequence" data-i18n-title="tooltip.sequence.import" title="Import Trajectory (JSON / G-Code)" data-i18n="sequence.btn.import" style="flex: 1;">Import Path</button>
              <button class="btn btn-secondary btn-sm" id="btn-export-sequence" data-i18n-title="tooltip.sequence.export" title="Export Trajectory (JSON / G-Code)" data-i18n="sequence.btn.export" style="flex: 1;">Export Path</button>

              <!-- Compatibility aliases for tests & automation -->
              <button id="btn-import-json" style="display: none;"></button>
              <button id="btn-export-json" style="display: none;"></button>
              <button id="btn-import-gcode" style="display: none;"></button>
              <button id="btn-export-gcode" style="display: none;"></button>
              <input type="file" id="input-universal-file" accept=".json,.gcode,.nc,.tap,.ngc,.txt" style="display: none;">
              <input type="file" id="input-gcode-file" accept=".gcode,.nc,.tap,.ngc,.txt" style="display: none;">
            </div>
          </div>
        </div>

        <!-- Player Control Bar -->
        <div class="player-control-panel">
          <div class="player-buttons">
            <button class="btn btn-play btn-icon-only" id="btn-player-play" title="Play Sequence" aria-label="Play sequence"><svg class="ico" viewBox="0 0 16 16" fill="currentColor" aria-hidden="true"><path d="M5 3.5v9L12 8l-7-4.5Z"/></svg></button>
            <button class="btn btn-secondary btn-icon-only" id="btn-player-pause" title="Pause Sequence" aria-label="Pause sequence"><svg class="ico" viewBox="0 0 16 16" fill="currentColor" aria-hidden="true"><path d="M4.5 3.5h2.5v9H4.5zM9 3.5h2.5v9H9z"/></svg></button>
            <button class="btn btn-secondary btn-icon-only" id="btn-player-stop" title="Stop Sequence" aria-label="Stop sequence"><svg class="ico" viewBox="0 0 16 16" fill="currentColor" aria-hidden="true"><rect x="4" y="4" width="8" height="8" rx="1"/></svg></button>
            <div class="divider-v"></div>
            <button class="btn btn-sm btn-secondary btn-icon-only" id="btn-player-step-back" title="Previous Waypoint" aria-label="Previous waypoint"><svg class="ico" viewBox="0 0 16 16" fill="currentColor" aria-hidden="true"><rect x="2.5" y="3.5" width="2" height="9" rx="0.5"/><path d="M13.5 3.5v9L6 8l7.5-4.5Z"/></svg></button>
            <button class="btn btn-sm btn-secondary btn-icon-only" id="btn-player-step-fwd" title="Next Waypoint" aria-label="Next waypoint"><svg class="ico" viewBox="0 0 16 16" fill="currentColor" aria-hidden="true"><path d="M2.5 3.5v9L10 8 2.5 3.5Z"/><rect x="11.5" y="3.5" width="2" height="9" rx="0.5"/></svg></button>
            <div class="divider-v"></div>
            <label class="toggle-control" title="Toggle Infinite Loop">
              <input type="checkbox" id="chk-player-loop">
              <span class="toggle-label">Loop</span>
            </label>
          </div>

          <!-- Speed & Time Readout -->
          <div class="player-timing-controls">
            <div class="speed-slider-wrap">
              <span class="label-muted">Speed:</span>
              <input type="range" class="speed-slider" id="slider-player-speed" min="0.1" max="2.5" step="0.05" value="1.0">
              <span class="speed-val" id="val-player-speed">1.00x</span>
            </div>
            <div class="time-display" id="time-display">0.0s / 0.0s (0%)</div>
          </div>
        </div>

        <!-- Dynamic Scrubber & Progress Bar -->
        <div class="scrubber-container">
          <div class="scrubber-track" id="scrubber-track">
            <div class="scrubber-fill" id="scrubber-fill"></div>
            <div class="scrubber-thumb" id="scrubber-thumb"></div>
          </div>
        </div>

        <!-- Pose Capture & Timeline Action Toolbar -->
        <div class="pose-capture-toolbar">
          <div class="capture-group">
            <button class="btn btn-action" id="btn-capture-live" title="Capture current Live Robot Pose into Timeline">
              + Capture Live
            </button>
            <button class="btn btn-action" id="btn-capture-ghost" title="Capture current Ghost Target Pose into Timeline (Press Space)">
              + Capture Ghost <span class="key-hint-badge" style="margin-left: 4px;">Space</span>
            </button>
          </div>
          <div class="timeline-quick-actions">
            <button class="btn btn-secondary" id="btn-add-custom-wp" title="Add Blank Waypoint">+ Add Blank</button>
            <button class="btn btn-sm btn-secondary btn-icon-only" id="btn-reverse-seq" title="Reverse Sequence" aria-label="Reverse sequence"><svg class="ico" viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M4 6a5 5 0 0 1 8.5-1.5L14 6"/><path d="M14 2.8V6h-3.2M12 10a5 5 0 0 1-8.5 1.5L2 10"/><path d="M2 13.2V10h3.2"/></svg></button>
            <button class="btn btn-sm btn-secondary btn-icon-only" id="btn-clear-seq" title="Clear All Waypoints" aria-label="Clear all waypoints"><svg class="ico" viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.4" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M3 4.5h10M6 4.5V3h4v1.5M4.5 5l.5 8h6l.5-8M6.5 7v4m3-4v4"/></svg></button>
          </div>
        </div>

        <!-- Waypoints Interactive Timeline List -->
        <div class="timeline-container">
          <div class="timeline-cards-list" id="timeline-cards-list">
            <!-- Waypoint Cards rendered dynamically -->
          </div>
        </div>

        <!-- Format Picker Modal: Import Selection -->
        <div class="modal-dialog-backdrop hidden" id="format-import-modal-backdrop">
          <div class="modal-dialog" style="max-width: 520px;">
            <div class="modal-header">
              <h4 data-i18n="format.modal.import.title">Import Trajectory / Toolpath</h4>
              <button class="btn-close-modal" id="btn-close-format-import-modal">✕</button>
            </div>
            <div class="modal-body">
              <p class="field-label" style="margin-bottom: 6px;" data-i18n="format.modal.import.desc">Select the file format you would like to import:</p>
              
              <div class="format-choice-grid">
                <div class="format-choice-card" id="card-choice-import-json">
                  <div class="format-card-header">
                    <span class="format-card-title" data-i18n="format.card.json.title">JSON Robotic Sequence (.json)</span>
                    <span class="format-card-badge format-badge-json">{ JSON }</span>
                  </div>
                  <div class="format-card-desc" data-i18n="format.card.json.desc">
                    Native robot sequence containing joint angles, durations, and gripper states.
                  </div>
                </div>

                <div class="format-choice-card" id="card-choice-import-gcode">
                  <div class="format-card-header">
                    <span class="format-card-title" data-i18n="format.card.gcode.title">CNC G-Code Toolpath (.gcode, .nc)</span>
                    <span class="format-card-badge format-badge-gcode">[ G-CODE ]</span>
                  </div>
                  <div class="format-card-desc" data-i18n="format.card.gcode.desc">
                    Cartesian toolpath from CNC/CAD software with auto-centering &amp; scaling.
                  </div>
                </div>
              </div>

              <div class="format-browse-row">
                <button class="btn btn-sm btn-secondary" id="btn-browse-universal-import" data-i18n="format.btn.browse" style="width: 100%;">
                  Browse Any File (.json, .gcode)...
                </button>
              </div>
            </div>
          </div>
        </div>

        <!-- Format Picker Modal: Export Selection -->
        <div class="modal-dialog-backdrop hidden" id="format-export-modal-backdrop">
          <div class="modal-dialog" style="max-width: 520px;">
            <div class="modal-header">
              <h4 data-i18n="format.modal.export.title">Export Trajectory / Toolpath</h4>
              <button class="btn-close-modal" id="btn-close-format-export-modal">✕</button>
            </div>
            <div class="modal-body">
              <p class="field-label" style="margin-bottom: 6px;" data-i18n="format.modal.export.desc">Choose export format for current sequence:</p>
              
              <div class="format-choice-grid">
                <div class="format-choice-card" id="card-choice-export-json">
                  <div class="format-card-header">
                    <span class="format-card-title" data-i18n="format.card.json.title">JSON Robotic Sequence (.json)</span>
                    <span class="format-card-badge format-badge-json">{ JSON }</span>
                  </div>
                  <div class="format-card-desc" data-i18n="format.card.json.desc">
                    Export full robotic sequence with joint angles, durations, and gripper commands.
                  </div>
                </div>

                <div class="format-choice-card" id="card-choice-export-gcode">
                  <div class="format-card-header">
                    <span class="format-card-title" data-i18n="format.card.gcode.title">CNC G-Code Toolpath (.gcode)</span>
                    <span class="format-card-badge format-badge-gcode">[ G-CODE ]</span>
                  </div>
                  <div class="format-card-desc" data-i18n="format.card.gcode.desc">
                    Export sequence converted to standard CNC Cartesian toolpath with G0/G1 moves.
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>

        <!-- JSON Import/Export Modal Container -->
        <div class="modal-dialog-backdrop hidden" id="json-modal-backdrop">
          <div class="modal-dialog">
            <div class="modal-header">
              <h4 id="json-modal-title">JSON Sequence Transfer</h4>
              <button class="btn-close-modal" id="btn-close-json-modal">✕</button>
            </div>
            <div class="modal-body">
              <textarea class="json-textarea" id="json-modal-textarea" spellcheck="false" placeholder="Paste sequence JSON here..."></textarea>
              <div class="modal-error hidden" id="json-modal-error"></div>
            </div>
            <div class="modal-footer">
              <button class="btn btn-secondary" id="btn-download-json" data-i18n="format.modal.download">Download File</button>
              <button class="btn btn-secondary" id="btn-copy-json" data-i18n="gcode.modal.copy">Copy to Clipboard</button>
              <button class="btn btn-primary" id="btn-apply-json">Apply Import</button>
            </div>
          </div>
        </div>

        <!-- G-Code Import/Export Modal Container -->
        <div class="modal-dialog-backdrop hidden" id="gcode-modal-backdrop">
          <div class="modal-dialog" style="max-width: 580px;">
            <div class="modal-header">
              <h4 id="gcode-modal-title" data-i18n="gcode.modal.title">G-Code Toolpath Engine</h4>
              <button class="btn-close-modal" id="btn-close-gcode-modal">✕</button>
            </div>
            <div class="modal-body">
              <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px;">
                <span class="field-label" data-i18n="gcode.modal.label">Paste G-Code or Select File:</span>
                <button class="btn btn-xs btn-outline" id="btn-browse-gcode" data-i18n="gcode.modal.browse">Browse File (.gcode)</button>
              </div>
              <textarea class="json-textarea" id="gcode-modal-textarea" spellcheck="false" placeholder="Paste G-Code (G0, G1, G2, G3, M3, M5)..." style="height: 140px; font-family: monospace; font-size: 11px;"></textarea>
              
              <!-- Auto-Fit & Kinematics Options -->
              <div class="gcode-options-grid" id="gcode-options-grid" style="display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px; margin-top: 8px; background: rgba(0,0,0,0.25); padding: 8px; border-radius: 4px; border: 1px solid var(--border-metallic, #334155);">
                <div>
                  <label class="field-label" style="font-size: 10px;" data-i18n="gcode.opt.centerx">Center X (mm):</label>
                  <input type="number" id="gcode-opt-cx" class="joint-val-input" value="160" step="5" style="width: 100%;">
                </div>
                <div>
                  <label class="field-label" style="font-size: 10px;" data-i18n="gcode.opt.centery">Center Y (mm):</label>
                  <input type="number" id="gcode-opt-cy" class="joint-val-input" value="0" step="5" style="width: 100%;">
                </div>
                <div>
                  <label class="field-label" style="font-size: 10px;" data-i18n="gcode.opt.centerz">Base Z (mm):</label>
                  <input type="number" id="gcode-opt-cz" class="joint-val-input" value="50" step="5" style="width: 100%;">
                </div>
                <div>
                  <label class="field-label" style="font-size: 10px;" data-i18n="gcode.opt.scale">Scale Factor:</label>
                  <input type="number" id="gcode-opt-scale" class="joint-val-input" value="1.0" step="0.1" min="0.05" max="10.0" style="width: 100%;">
                </div>
                <div>
                  <label class="field-label" style="font-size: 10px;" data-i18n="gcode.opt.feedrate">Default Feed (mm/min):</label>
                  <input type="number" id="gcode-opt-feed" class="joint-val-input" value="1200" step="100" style="width: 100%;">
                </div>
                <div>
                  <label class="field-label" style="font-size: 10px;" data-i18n="gcode.opt.wrist">Wrist Mode:</label>
                  <select id="gcode-opt-wrist" class="baud-select" style="width: 100%; height: 26px; padding: 2px;">
                    <option value="1" selected data-i18n="ik.wrist.w1">W1: Down_0 (Vertical)</option>
                    <option value="0" data-i18n="ik.wrist.w0">W0: Level (Floor)</option>
                    <option value="6" data-i18n="ik.wrist.w6">W6: Manual</option>
                  </select>
                </div>
              </div>

              <div class="modal-error hidden" id="gcode-modal-error" style="margin-top: 6px;"></div>
            </div>
            <div class="modal-footer">
              <button class="btn btn-secondary" id="btn-download-gcode" data-i18n="format.modal.download">Download File</button>
              <button class="btn btn-secondary" id="btn-copy-gcode" data-i18n="gcode.modal.copy">Copy to Clipboard</button>
              <button class="btn btn-primary" id="btn-apply-gcode" data-i18n="gcode.modal.apply">Apply Toolpath</button>
            </div>
          </div>
        </div>
      </div>
    `;

    // Cache elements
    this._domElements = {
      presetSelector: this.container.querySelector('#preset-selector'),
      btnLoadPreset: this.container.querySelector('#btn-load-preset'),
      btnSavePreset: this.container.querySelector('#btn-save-preset'),
      btnImportSequence: this.container.querySelector('#btn-import-sequence'),
      btnExportSequence: this.container.querySelector('#btn-export-sequence'),
      btnImportJson: this.container.querySelector('#btn-import-json'),
      btnExportJson: this.container.querySelector('#btn-export-json'),
      btnImportGCode: this.container.querySelector('#btn-import-gcode'),
      btnExportGCode: this.container.querySelector('#btn-export-gcode'),
      inputUniversalFile: this.container.querySelector('#input-universal-file'),
      inputGCodeFile: this.container.querySelector('#input-gcode-file'),
      formatImportModalBackdrop: this.container.querySelector('#format-import-modal-backdrop'),
      btnCloseFormatImportModal: this.container.querySelector('#btn-close-format-import-modal'),
      cardChoiceImportJson: this.container.querySelector('#card-choice-import-json'),
      cardChoiceImportGCode: this.container.querySelector('#card-choice-import-gcode'),
      btnBrowseUniversalImport: this.container.querySelector('#btn-browse-universal-import'),
      formatExportModalBackdrop: this.container.querySelector('#format-export-modal-backdrop'),
      btnCloseFormatExportModal: this.container.querySelector('#btn-close-format-export-modal'),
      cardChoiceExportJson: this.container.querySelector('#card-choice-export-json'),
      cardChoiceExportGCode: this.container.querySelector('#card-choice-export-gcode'),
      btnPlay: this.container.querySelector('#btn-player-play'),
      btnPause: this.container.querySelector('#btn-player-pause'),
      btnStop: this.container.querySelector('#btn-player-stop'),
      btnStepBack: this.container.querySelector('#btn-player-step-back'),
      btnStepFwd: this.container.querySelector('#btn-player-step-fwd'),
      chkLoop: this.container.querySelector('#chk-player-loop'),
      sliderSpeed: this.container.querySelector('#slider-player-speed'),
      valSpeed: this.container.querySelector('#val-player-speed'),
      timeDisplay: this.container.querySelector('#time-display'),
      scrubberTrack: this.container.querySelector('#scrubber-track'),
      scrubberFill: this.container.querySelector('#scrubber-fill'),
      scrubberThumb: this.container.querySelector('#scrubber-thumb'),
      btnCaptureLive: this.container.querySelector('#btn-capture-live'),
      btnCaptureGhost: this.container.querySelector('#btn-capture-ghost'),
      btnAddCustomWp: this.container.querySelector('#btn-add-custom-wp'),
      btnReverseSeq: this.container.querySelector('#btn-reverse-seq'),
      btnClearSeq: this.container.querySelector('#btn-clear-seq'),
      cardsList: this.container.querySelector('#timeline-cards-list'),
      seqNameBadge: this.container.querySelector('#seq-name-badge'),
      jsonModalBackdrop: this.container.querySelector('#json-modal-backdrop'),
      jsonModalTitle: this.container.querySelector('#json-modal-title'),
      jsonModalTextarea: this.container.querySelector('#json-modal-textarea'),
      jsonModalError: this.container.querySelector('#json-modal-error'),
      btnCloseJsonModal: this.container.querySelector('#btn-close-json-modal'),
      btnDownloadJson: this.container.querySelector('#btn-download-json'),
      btnCopyJson: this.container.querySelector('#btn-copy-json'),
      btnApplyJson: this.container.querySelector('#btn-apply-json'),
      gcodeModalBackdrop: this.container.querySelector('#gcode-modal-backdrop'),
      gcodeModalTitle: this.container.querySelector('#gcode-modal-title'),
      gcodeModalTextarea: this.container.querySelector('#gcode-modal-textarea'),
      gcodeModalError: this.container.querySelector('#gcode-modal-error'),
      btnCloseGCodeModal: this.container.querySelector('#btn-close-gcode-modal'),
      btnBrowseGCode: this.container.querySelector('#btn-browse-gcode'),
      btnDownloadGCode: this.container.querySelector('#btn-download-gcode'),
      btnCopyGCode: this.container.querySelector('#btn-copy-gcode'),
      btnApplyGCode: this.container.querySelector('#btn-apply-gcode'),
      gcodeOptCx: this.container.querySelector('#gcode-opt-cx'),
      gcodeOptCy: this.container.querySelector('#gcode-opt-cy'),
      gcodeOptCz: this.container.querySelector('#gcode-opt-cz'),
      gcodeOptScale: this.container.querySelector('#gcode-opt-scale'),
      gcodeOptFeed: this.container.querySelector('#gcode-opt-feed'),
      gcodeOptWrist: this.container.querySelector('#gcode-opt-wrist'),
      gcodeOptionsGrid: this.container.querySelector('#gcode-options-grid')
    };

    this._updateDomPresetsUI();
  }

  /**
   * Attaches event handlers to DOM elements
   * @private
   */
  _bindDomEvents() {
    const el = this._domElements;

    const addEv = (target, evt, handler) => {
      if (target) {
        target.addEventListener(evt, handler);
        this._eventCleanups.push(() => target.removeEventListener(evt, handler));
      }
    };

    // Player Buttons
    addEv(el.btnPlay, 'click', () => this.play());
    addEv(el.btnPause, 'click', () => this.pause());
    addEv(el.btnStop, 'click', () => this.stop());
    addEv(el.btnStepBack, 'click', () => this.stepBackward());
    addEv(el.btnStepFwd, 'click', () => this.stepForward());

    // Loop & Speed
    addEv(el.chkLoop, 'change', (e) => this.setLoop(e.target.checked));
    addEv(el.sliderSpeed, 'input', (e) => this.setSpeedScale(parseFloat(e.target.value)));

    // Pose Capture
    addEv(el.btnCaptureLive, 'click', () => this.captureLivePose());
    addEv(el.btnCaptureGhost, 'click', () => this.captureGhostPose());
    addEv(el.btnAddCustomWp, 'click', () => this.addWaypoint());
    addEv(el.btnReverseSeq, 'click', () => this.reverseSequence());
    addEv(el.btnClearSeq, 'click', () => {
      if (typeof confirm === 'undefined' || confirm('Clear all waypoints in the active sequence?')) {
        this.clearAllWaypoints();
      }
    });

    // Presets: Auto-load on selection change
    addEv(el.presetSelector, 'change', (e) => {
      const val = e.target ? e.target.value : (el.presetSelector ? el.presetSelector.value : null);
      if (val) {
        this.loadPreset(val);
      }
    });
    addEv(el.btnLoadPreset, 'click', () => {
      if (el.presetSelector && el.presetSelector.value) {
        this.loadPreset(el.presetSelector.value);
      }
    });
    addEv(el.btnSavePreset, 'click', () => {
      const name = prompt ? prompt('Enter name for new preset:', 'My Custom Sequence') : 'My Custom Sequence';
      if (name) {
        this.saveCurrentAsPreset(name);
      }
    });

    // Format Picker: Import & Export Action Triggers
    addEv(el.btnImportSequence, 'click', () => this._openImportFormatModal());
    addEv(el.btnExportSequence, 'click', () => this._openExportFormatModal());
    addEv(el.btnCloseFormatImportModal, 'click', () => this._closeImportFormatModal());
    addEv(el.btnCloseFormatExportModal, 'click', () => this._closeExportFormatModal());

    addEv(el.cardChoiceImportJson, 'click', () => {
      this._closeImportFormatModal();
      this._openJsonModal(true);
    });
    addEv(el.cardChoiceImportGCode, 'click', () => {
      this._closeImportFormatModal();
      this._openGCodeModal(true);
    });
    addEv(el.cardChoiceExportJson, 'click', () => {
      this._closeExportFormatModal();
      this._openJsonModal(false);
    });
    addEv(el.cardChoiceExportGCode, 'click', () => {
      this._closeExportFormatModal();
      this._openGCodeModal(false);
    });

    // Universal File Browser
    addEv(el.btnBrowseUniversalImport, 'click', () => {
      if (el.inputUniversalFile) el.inputUniversalFile.click();
    });
    addEv(el.inputUniversalFile, 'change', (e) => {
      const file = e.target.files && e.target.files[0];
      if (!file) return;
      const reader = new FileReader();
      reader.onload = (evt) => {
        const text = evt.target.result;
        const fname = file.name.toLowerCase();
        this._closeImportFormatModal();
        if (fname.endsWith('.json')) {
          try {
            this.importJSON(text);
          } catch (err) {
            this._openJsonModal(true);
            if (el.jsonModalTextarea) el.jsonModalTextarea.value = text;
            if (el.jsonModalError) {
              el.jsonModalError.textContent = err.message;
              el.jsonModalError.classList.remove('hidden');
            }
          }
        } else {
          this._openGCodeModal(true);
          if (el.gcodeModalTextarea) {
            el.gcodeModalTextarea.value = text;
          }
        }
      };
      reader.readAsText(file);
      e.target.value = '';
    });

    // JSON Modals & Compatibility triggers
    addEv(el.btnExportJson, 'click', () => this._openJsonModal(false));
    addEv(el.btnImportJson, 'click', () => this._openJsonModal(true));
    addEv(el.btnCloseJsonModal, 'click', () => this._closeJsonModal());
    addEv(el.btnDownloadJson, 'click', () => {
      const jsonContent = this.exportJSON(true);
      const seq = this.getSequence();
      const filename = `${(seq.sequenceName || 'sequence').replace(/\s+/g, '_').toLowerCase()}.json`;
      this._downloadFile(filename, jsonContent);
    });
    addEv(el.btnCopyJson, 'click', () => {
      if (el.jsonModalTextarea && navigator.clipboard) {
        navigator.clipboard.writeText(el.jsonModalTextarea.value);
        if (el.btnCopyJson) el.btnCopyJson.textContent = 'Copied!';
        setTimeout(() => { if (el.btnCopyJson) el.btnCopyJson.textContent = 'Copy to Clipboard'; }, 1500);
      }
    });
    addEv(el.btnApplyJson, 'click', () => {
      try {
        this.importJSON(el.jsonModalTextarea.value);
        this._closeJsonModal();
      } catch (err) {
        if (el.jsonModalError) {
          el.jsonModalError.textContent = err.message;
          el.jsonModalError.classList.remove('hidden');
        }
      }
    });

    // G-Code Modals & File Upload
    addEv(el.btnExportGCode, 'click', () => this._openGCodeModal(false));
    addEv(el.btnImportGCode, 'click', () => this._openGCodeModal(true));
    addEv(el.btnCloseGCodeModal, 'click', () => this._closeGCodeModal());
    addEv(el.btnDownloadGCode, 'click', () => {
      const gcodeContent = this.exportGCode();
      const seq = this.getSequence();
      const filename = `${(seq.sequenceName || 'toolpath').replace(/\s+/g, '_').toLowerCase()}.gcode`;
      this._downloadFile(filename, gcodeContent);
    });
    addEv(el.btnBrowseGCode, 'click', () => {
      if (el.inputGCodeFile) el.inputGCodeFile.click();
    });
    addEv(el.inputGCodeFile, 'change', (e) => {
      const file = e.target.files && e.target.files[0];
      if (!file) return;
      const reader = new FileReader();
      reader.onload = (evt) => {
        if (el.gcodeModalTextarea) {
          el.gcodeModalTextarea.value = evt.target.result;
        }
      };
      reader.readAsText(file);
      e.target.value = '';
    });
    addEv(el.btnCopyGCode, 'click', () => {
      if (el.gcodeModalTextarea && navigator.clipboard) {
        navigator.clipboard.writeText(el.gcodeModalTextarea.value);
        if (el.btnCopyGCode) el.btnCopyGCode.textContent = 'Copied!';
        setTimeout(() => { if (el.btnCopyGCode) el.btnCopyGCode.textContent = 'Copy to Clipboard'; }, 1500);
      }
    });
    addEv(el.btnApplyGCode, 'click', () => {
      try {
        const text = el.gcodeModalTextarea ? el.gcodeModalTextarea.value : '';
        const cx = el.gcodeOptCx ? parseFloat(el.gcodeOptCx.value) : 160;
        const cy = el.gcodeOptCy ? parseFloat(el.gcodeOptCy.value) : 0;
        const cz = el.gcodeOptCz ? parseFloat(el.gcodeOptCz.value) : 50;
        const scaleVal = el.gcodeOptScale ? parseFloat(el.gcodeOptScale.value) : 1.0;
        const feedVal = el.gcodeOptFeed ? parseFloat(el.gcodeOptFeed.value) : 1200;
        const wristVal = el.gcodeOptWrist ? parseInt(el.gcodeOptWrist.value, 10) : 1;

        this.importGCode(text, {
          autoCenter: true,
          targetCenter: { x: cx, y: cy, z: cz },
          scale: isNaN(scaleVal) || scaleVal <= 0 ? 1.0 : scaleVal,
          defaultFeedrate: isNaN(feedVal) || feedVal <= 0 ? 1200 : feedVal,
          wristMode: isNaN(wristVal) ? 1 : wristVal,
          sequenceName: 'Imported G-Code'
        });

        this._closeGCodeModal();
      } catch (err) {
        if (el.gcodeModalError) {
          el.gcodeModalError.textContent = err.message;
          el.gcodeModalError.classList.remove('hidden');
        }
      }
    });

    // Scrubber click / jump
    addEv(el.scrubberTrack, 'click', (e) => {
      const rect = el.scrubberTrack.getBoundingClientRect();
      const clickX = e.clientX - rect.left;
      const pct = Math.max(0, Math.min(1, clickX / rect.width));
      const seq = this.getSequence();
      if (seq.waypoints.length > 0) {
        const targetIdx = Math.min(seq.waypoints.length - 1, Math.floor(pct * seq.waypoints.length));
        this.jumpTo(targetIdx);
      }
    });
  }

  /**
   * Triggers native browser text file download
   * @private
   */
  _downloadFile(filename, content) {
    if (typeof document === 'undefined') return;
    try {
      const blob = new Blob([content], { type: 'text/plain;charset=utf-8' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = filename;
      document.body.appendChild(a);
      a.click();
      setTimeout(() => {
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
      }, 200);
    } catch (err) {
      console.error('File download error:', err);
    }
  }

  /**
   * Opens Format Picker Modal for Import
   * @private
   */
  _openImportFormatModal() {
    const el = this._domElements;
    if (el.formatImportModalBackdrop) {
      el.formatImportModalBackdrop.classList.remove('hidden');
    }
  }

  /**
   * Closes Format Picker Modal for Import
   * @private
   */
  _closeImportFormatModal() {
    const el = this._domElements;
    if (el.formatImportModalBackdrop) {
      el.formatImportModalBackdrop.classList.add('hidden');
    }
  }

  /**
   * Opens Format Picker Modal for Export
   * @private
   */
  _openExportFormatModal() {
    const el = this._domElements;
    if (el.formatExportModalBackdrop) {
      el.formatExportModalBackdrop.classList.remove('hidden');
    }
  }

  /**
   * Closes Format Picker Modal for Export
   * @private
   */
  _closeExportFormatModal() {
    const el = this._domElements;
    if (el.formatExportModalBackdrop) {
      el.formatExportModalBackdrop.classList.add('hidden');
    }
  }

  /**
   * Opens JSON Import/Export modal
   * @private
   */
  _openJsonModal(isImport = false) {
    const el = this._domElements;
    if (!el.jsonModalBackdrop) return;

    if (el.jsonModalError) {
      el.jsonModalError.classList.add('hidden');
      el.jsonModalError.textContent = '';
    }

    if (isImport) {
      if (el.jsonModalTitle) el.jsonModalTitle.textContent = 'Import Sequence from JSON';
      if (el.jsonModalTextarea) {
        el.jsonModalTextarea.value = '';
        el.jsonModalTextarea.placeholder = 'Paste valid sequence JSON here...';
      }
      if (el.btnApplyJson) el.btnApplyJson.classList.remove('hidden');
      if (el.btnDownloadJson) el.btnDownloadJson.classList.add('hidden');
    } else {
      if (el.jsonModalTitle) el.jsonModalTitle.textContent = 'Export Active Sequence JSON';
      if (el.jsonModalTextarea) {
        el.jsonModalTextarea.value = this.exportJSON(true);
      }
      if (el.btnApplyJson) el.btnApplyJson.classList.add('hidden');
      if (el.btnDownloadJson) el.btnDownloadJson.classList.remove('hidden');
    }

    el.jsonModalBackdrop.classList.remove('hidden');
  }

  /**
   * Closes JSON modal
   * @private
   */
  _closeJsonModal() {
    const el = this._domElements;
    if (el.jsonModalBackdrop) {
      el.jsonModalBackdrop.classList.add('hidden');
    }
  }

  /**
   * Opens G-Code Import/Export modal
   * @param {boolean} [isImport=true]
   * @private
   */
  _openGCodeModal(isImport = true) {
    const el = this._domElements;
    if (!el.gcodeModalBackdrop) return;

    if (el.gcodeModalError) {
      el.gcodeModalError.classList.add('hidden');
      el.gcodeModalError.textContent = '';
    }

    if (isImport) {
      if (el.gcodeModalTitle) el.gcodeModalTitle.textContent = 'Import CNC/3D Toolpath from G-Code';
      if (el.gcodeModalTextarea) {
        el.gcodeModalTextarea.value = '';
        el.gcodeModalTextarea.placeholder = 'Paste G-Code (e.g. G0, G1, G2, G3, M3, M5) or click Browse File...';
      }
      if (el.btnApplyGCode) el.btnApplyGCode.classList.remove('hidden');
      if (el.gcodeOptionsGrid) el.gcodeOptionsGrid.style.display = 'grid';
      if (el.btnBrowseGCode) el.btnBrowseGCode.classList.remove('hidden');
      if (el.btnDownloadGCode) el.btnDownloadGCode.classList.add('hidden');
    } else {
      if (el.gcodeModalTitle) el.gcodeModalTitle.textContent = 'Export Active Sequence to CNC G-Code';
      if (el.gcodeModalTextarea) {
        el.gcodeModalTextarea.value = this.exportGCode();
      }
      if (el.btnApplyGCode) el.btnApplyGCode.classList.add('hidden');
      if (el.gcodeOptionsGrid) el.gcodeOptionsGrid.style.display = 'none';
      if (el.btnBrowseGCode) el.btnBrowseGCode.classList.add('hidden');
      if (el.btnDownloadGCode) el.btnDownloadGCode.classList.remove('hidden');
    }

    el.gcodeModalBackdrop.classList.remove('hidden');
  }

  /**
   * Closes G-Code modal
   * @private
   */
  _closeGCodeModal() {
    const el = this._domElements;
    if (el.gcodeModalBackdrop) {
      el.gcodeModalBackdrop.classList.add('hidden');
    }
  }

  /**
   * Updates preset selection dropdown
   * @private
   */
  _updateDomPresetsUI() {
    const sel = this._domElements.presetSelector;
    if (!sel) return;

    const presets = this.presetManager.getAllPresets();
    const currentVal = sel.value;
    sel.innerHTML = '';

    for (const p of presets) {
      const opt = document.createElement('option');
      opt.value = p.name;
      opt.textContent = `${p.isBuiltin ? '[SYS] ' : '[USER] '}${p.label} (${p.waypointCount} pts)`;
      sel.appendChild(opt);
    }

    if (currentVal && Array.from(sel.options).some(o => o.value === currentVal)) {
      sel.value = currentVal;
    }
  }

  /**
   * Updates Teach panel status and point counts
   * @private
   */
  _updateDomTeachUI() {
    const el = this._domElements;
    if (!this.appState || !el.teachLed) return;

    const teach = this.appState.getState('teach') || {};
    const isActive = Boolean(teach.active);
    const count = Number(teach.pointCount || (teach.points ? teach.points.length : 0));

    if (isActive) {
      el.teachLed.classList.add('recording');
    } else {
      el.teachLed.classList.remove('recording');
    }

    if (el.teachCountBadge) {
      el.teachCountBadge.textContent = `${count} / ${MAX_TEACH_POINTS} pts`;
      if (count >= MAX_TEACH_POINTS) {
        el.teachCountBadge.classList.add('badge-full');
      } else {
        el.teachCountBadge.classList.remove('badge-full');
      }
    }
  }

  /**
   * Full DOM synchronization pass
   * @private
   */
  _updateDomUI() {
    if (!this._domBound || !this.container) return;

    const el = this._domElements;
    const seq = this.getSequence();
    const progress = this.getProgress();

    // Sequence Header Badge
    if (el.seqNameBadge) {
      const count = (seq.waypoints && seq.waypoints.length) || 0;
      el.seqNameBadge.textContent = `${count} ${count === 1 ? 'WAYPOINT' : 'PTS'}`;
      el.seqNameBadge.title = `${seq.sequenceName || 'Sequence'} (${count} waypoints)`;
    }

    // Player Buttons state
    if (el.btnPlay) {
      el.btnPlay.disabled = progress.isPlaying;
      el.btnPlay.classList.toggle('active', progress.isPlaying);
    }
    if (el.btnPause) {
      el.btnPause.disabled = !progress.isPlaying;
      el.btnPause.classList.toggle('active', progress.isPaused);
    }
    if (el.btnStop) {
      el.btnStop.disabled = progress.state === PlaybackState.IDLE || progress.state === PlaybackState.STOPPED;
    }
    if (el.chkLoop) {
      el.chkLoop.checked = this.loop;
    }
    if (el.valSpeed) {
      el.valSpeed.textContent = `${this.speedScale.toFixed(2)}x`;
    }
    if (el.sliderSpeed && document.activeElement !== el.sliderSpeed) {
      el.sliderSpeed.value = this.speedScale;
    }

    // Time & Progress Readouts
    if (el.timeDisplay) {
      const elapsedSec = (progress.elapsedTime / 1000).toFixed(1);
      const totalSec = (progress.totalTime / 1000).toFixed(1);
      el.timeDisplay.textContent = `${elapsedSec}s / ${totalSec}s (${progress.progressPercent.toFixed(0)}%)`;
    }

    if (el.scrubberFill) {
      el.scrubberFill.style.width = `${progress.progressPercent}%`;
    }
    if (el.scrubberThumb) {
      el.scrubberThumb.style.left = `${progress.progressPercent}%`;
    }

    // Render Waypoint Cards
    this._renderWaypointCards(seq.waypoints);
  }

  /**
   * Renders the interactive list of waypoint cards
   * @private
   */
  _renderWaypointCards(waypoints = []) {
    const list = this._domElements.cardsList;
    if (!list) return;

    if (waypoints.length === 0) {
      list.innerHTML = `
        <div class="empty-timeline-msg">
          <span class="empty-icon"></span>
          <div>No waypoints in timeline. Capture a pose or add waypoints to begin planning.</div>
        </div>
      `;
      return;
    }

    list.innerHTML = waypoints.map((wp, idx) => {
      const isActive = this.currentIndex === idx && (this.playerState === PlaybackState.PLAYING || this.playerState === PlaybackState.PAUSED);
      const isSelected = wp.id === this.selectedWaypointId;

      return `
        <div class="waypoint-card ${isActive ? 'active-playing' : ''} ${isSelected ? 'selected' : ''}" data-id="${wp.id}" data-index="${idx}">
          <div class="wp-card-header">
            <div class="wp-card-index">#${idx + 1}</div>
            <input type="text" class="wp-input-label" data-id="${wp.id}" value="${this._escapeHtml(wp.label)}" title="Waypoint Label">
            <div class="wp-card-actions">
              <button class="btn-card-action btn-preview" data-id="${wp.id}" title="Preview on Ghost Arm"><svg class="ico" viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.3" style="width:12px;height:12px;"><path d="M1 8s2.5-5 7-5 7 5 7 5-2.5 5-7 5S1 8 1 8z"/><circle cx="8" cy="8" r="2.5"/></svg></button>
              <button class="btn-card-action btn-up" data-index="${idx}" ${idx === 0 ? 'disabled' : ''} title="Move Up">▲</button>
              <button class="btn-card-action btn-down" data-index="${idx}" ${idx === waypoints.length - 1 ? 'disabled' : ''} title="Move Down">▼</button>
              <button class="btn-card-action btn-dup" data-id="${wp.id}" title="Duplicate Waypoint">⧉</button>
              <button class="btn-card-action btn-del" data-id="${wp.id}" title="Delete Waypoint">✕</button>
            </div>
          </div>

          <div class="wp-card-body">
            <div class="wp-joint-grid">
              <div class="wp-joint-field">
                <span class="joint-tag">J1</span>
                <input type="number" class="wp-num-input" data-id="${wp.id}" data-field="j1" value="${wp.j1}" step="0.5">
              </div>
              <div class="wp-joint-field">
                <span class="joint-tag">J2</span>
                <input type="number" class="wp-num-input" data-id="${wp.id}" data-field="j2" value="${wp.j2}" step="0.5">
              </div>
              <div class="wp-joint-field">
                <span class="joint-tag">J3</span>
                <input type="number" class="wp-num-input" data-id="${wp.id}" data-field="j3" value="${wp.j3}" step="0.5">
              </div>
              <div class="wp-joint-field">
                <span class="joint-tag">J4</span>
                <input type="number" class="wp-num-input" data-id="${wp.id}" data-field="j4" value="${wp.j4}" step="1">
              </div>
              <div class="wp-joint-field">
                <span class="joint-tag">J5</span>
                <input type="number" class="wp-num-input" data-id="${wp.id}" data-field="j5" value="${wp.j5}" step="1">
              </div>
              <div class="wp-joint-field">
                <span class="joint-tag">Grip</span>
                <input type="number" class="wp-num-input" data-id="${wp.id}" data-field="grip" value="${wp.grip}" step="5" min="0" max="180">
              </div>
            </div>

            <div class="wp-card-footer">
              <div class="wp-dur-field">
                <span class="field-label">Duration:</span>
                <input type="number" class="wp-dur-input" data-id="${wp.id}" data-field="dur" value="${wp.dur}" min="10" step="50">
                <span class="field-unit">ms</span>
              </div>
              <input type="text" class="wp-comment-input" data-id="${wp.id}" data-field="comment" value="${this._escapeHtml(wp.comment || '')}" placeholder="Optional note / comment...">
            </div>
          </div>
        </div>
      `;
    }).join('');

    // Attach card event listeners
    this._attachCardEventListeners(list);
  }

  /**
   * Attaches interactive event handlers to rendered waypoint cards
   * @private
   */
  _attachCardEventListeners(container) {
    // Card selection & Preview
    container.querySelectorAll('.btn-preview').forEach(btn => {
      btn.addEventListener('click', (e) => {
        e.stopPropagation();
        this.previewWaypointInGhost(btn.dataset.id);
      });
    });

    // Move Up / Down
    container.querySelectorAll('.btn-up').forEach(btn => {
      btn.addEventListener('click', (e) => {
        e.stopPropagation();
        this.moveUp(parseInt(btn.dataset.index, 10));
      });
    });

    container.querySelectorAll('.btn-down').forEach(btn => {
      btn.addEventListener('click', (e) => {
        e.stopPropagation();
        this.moveDown(parseInt(btn.dataset.index, 10));
      });
    });

    // Duplicate & Delete
    container.querySelectorAll('.btn-dup').forEach(btn => {
      btn.addEventListener('click', (e) => {
        e.stopPropagation();
        this.duplicateWaypoint(btn.dataset.id);
      });
    });

    container.querySelectorAll('.btn-del').forEach(btn => {
      btn.addEventListener('click', (e) => {
        e.stopPropagation();
        this.deleteWaypoint(btn.dataset.id);
      });
    });

    // Label Edit
    container.querySelectorAll('.wp-input-label').forEach(input => {
      input.addEventListener('change', (e) => {
        this.updateWaypoint(input.dataset.id, { label: e.target.value.trim() });
      });
    });

    // Numerical joint/dur/grip changes
    container.querySelectorAll('.wp-num-input, .wp-dur-input').forEach(input => {
      input.addEventListener('change', (e) => {
        const field = input.dataset.field;
        const val = parseFloat(e.target.value);
        if (!isNaN(val)) {
          this.updateWaypoint(input.dataset.id, { [field]: val });
        }
      });
    });

    // Comment field
    container.querySelectorAll('.wp-comment-input').forEach(input => {
      input.addEventListener('change', (e) => {
        this.updateWaypoint(input.dataset.id, { comment: e.target.value.trim() });
      });
    });

    // Card click selection
    container.querySelectorAll('.waypoint-card').forEach(card => {
      card.addEventListener('click', () => {
        this.selectedWaypointId = card.dataset.id;
        container.querySelectorAll('.waypoint-card').forEach(c => c.classList.remove('selected'));
        card.classList.add('selected');
      });
    });
  }

  /**
   * HTML escape helper
   * @private
   */
  _escapeHtml(str) {
    if (typeof str !== 'string') return '';
    return str
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;')
      .replace(/'/g, '&#039;');
  }

  /**
   * Cleans up all event subscriptions and timers
   */
  dispose() {
    this.stop();
    for (const unsub of this._unsubState) {
      if (typeof unsub === 'function') unsub();
    }
    this._unsubState = [];

    for (const cleanup of this._eventCleanups) {
      if (typeof cleanup === 'function') cleanup();
    }
    this._eventCleanups = [];

    if (this.container) {
      this.container.innerHTML = '';
    }
    this._domBound = false;
  }
}

export default TrajectoryController;
