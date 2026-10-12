/**
 * AppState.js - Central Reactive State Store & Pub/Sub Event Bus
 * 
 * Coordinates connection state, live pose telemetry, ghost target pose,
 * safety alerts, waypoint sequences, and console logs.
 */

import { forwardKinematics, forwardKinematics6DOF } from '../kinematics/RobotKinematics.js';

/**
 * Fast recursive clone without intermediate JSON string serialization
 * @param {*} val
 * @returns {*}
 */
export function fastClone(val) {
  if (val === null || typeof val !== 'object') return val;
  if (Array.isArray(val)) {
    const len = val.length;
    const copy = new Array(len);
    for (let i = 0; i < len; i++) {
      const item = val[i];
      copy[i] = typeof item === 'object' && item !== null ? fastClone(item) : item;
    }
    return copy;
  }
  const copy = {};
  for (const k of Object.keys(val)) {
    const v = val[k];
    copy[k] = typeof v === 'object' && v !== null ? fastClone(v) : v;
  }
  return copy;
}

export class AppState {
  constructor() {
    this._state = {
      connection: {
        status: 'DISCONNECTED', // 'DISCONNECTED' | 'CONNECTING' | 'CONNECTED' | 'ERROR' | 'RECONNECTING'
        isMock: false,
        baudRate: 115200,
        portName: '',
        telemRateHz: 0.0,
        lastHeartbeat: null
      },
      livePose: {
        j1: 0.00,
        j2: 0.00,
        j3: 0.00,
        j4: 0.0,
        j5: 0.0,
        j6: 0.0,
        grip: 0,
        x: 253.45,
        y: 0.0,
        z: 141.03,
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
        dx: 0,
        dy: 0,
        lastUpdated: null
      },
      ghostPose: {
        j1: 0.00,
        j2: 0.00,
        j3: 0.00,
        j4: 0.0,
        j5: 0.0,
        j6: 0.0,
        grip: 0,
        x: 253.45,
        y: 0.0,
        z: 141.03,
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
        wristMode: 0,
        duration: 1000,
        isReachable: true,
        warnings: []
      },
      safety: {
        estop: false,
        homing: false,
        closedLoop: false,
        speedOverride: 100,
        collision: { j1: false, j2: false, j3: false },
        hasCollision: false,
        floorBreach: false,
        reachLimit: false
      },
      teach: {
        active: false,
        pointCount: 0,
        points: []
      },
      sequence: {
        name: 'Default Sequence',
        loop: false,
        isPlaying: false,
        isPaused: false,
        activeIndex: -1,
        progress: 0,
        waypoints: []
      },
      history: {
        telemetry: [], // Max 300 entries (30s @ 10Hz)
        logs: []       // Max 200 console logs
      },
      settings: {
        workOffset: { x: 0, y: 0, z: 0 },
        toolOffset: { x: 0, y: 0, z: 0 },
        maxSpeed: { j1: 10000, j2: 35000, j3: 25000 },
        maxAccel: { j1: 15000, j2: 25000, j3: 20000 },
        safetyMargin: 2.50
      },
      execution: {
        mode: 'preview',          // 'preview' (Ghost arm only) | 'direct_drive' (Live follow)
        armed: false,             // boolean: true only after explicit modal confirmation
        directDrivePulsing: false // boolean: visual pulse indicator active state
      },
      controlMode: {
        activeTab: 'ik',          // 'ik' | 'joints' | 'trajectory'
        cartesianFrame: 'base'    // 'base' | 'world' | 'tool'
      },
      gripper: {
        percent: 0,               // 0 to 100 (%)
        effort: 0,                // Telemetry load/current readout (mA or load %)
        objectDetected: false     // Telemetry contact/clamped sensor flag
      }
    };

    this.listeners = new Map();
    this.maxHistoryLength = 300;
    this.maxLogLength = 200;
  }

  /**
   * Returns copy of whole state or specific path
   * @param {string} [path] - e.g. 'livePose', 'safety.estop'
   */
  getState(path = null) {
    if (!path) {
      return fastClone(this._state);
    }
    const parts = path.split('.');
    let curr = this._state;
    for (const part of parts) {
      if (curr === undefined || curr === null) return undefined;
      curr = curr[part];
    }
    if (typeof curr !== 'object' || curr === null) return curr;

    // Flat hot paths: use shallow spread for maximum throughput and zero GC churn
    if (path === 'livePose' || path === 'connection') {
      return { ...curr };
    }
    if (path === 'safety') {
      return { ...curr, collision: curr.collision ? { ...curr.collision } : {} };
    }
    if (path === 'ghostPose') {
      return { ...curr, warnings: Array.isArray(curr.warnings) ? [...curr.warnings] : [] };
    }
    if (path === 'execution' || path === 'controlMode' || path === 'gripper') {
      return { ...curr };
    }
    return fastClone(curr);
  }

  /**
   * Sets value at a specific state path and notifies subscribers
   * @param {string} path - e.g. 'sequence.activeIndex', 'connection.status'
   * @param {*} value - New value
   */
  setState(path, value) {
    if (!path || typeof path !== 'string') return;
    const parts = path.split('.');
    let curr = this._state;
    for (let i = 0; i < parts.length - 1; i++) {
      const part = parts[i];
      if (curr[part] === undefined || curr[part] === null || typeof curr[part] !== 'object') {
        curr[part] = {};
      }
      curr = curr[part];
    }
    const lastPart = parts[parts.length - 1];
    const prevValue = curr[lastPart];
    curr[lastPart] = value;

    const rootKey = parts[0];
    this.emit(`change:${path}`, { path, value, prevValue });
    this.emit('stateChange', { path, value, prevValue });
    this.emit(`stateChange:${rootKey}`, this._state[rootKey]);
    if (rootKey === 'sequence') {
      this.emit('sequence:updated', this._state.sequence);
    }
  }

  /**
   * Updates partial sequence state and emits notifications
   * @param {Object} updates
   */
  setSequenceState(updates) {
    if (!updates || typeof updates !== 'object') return;
    this._state.sequence = {
      ...this._state.sequence,
      ...updates
    };
    this.emit('sequence:updated', this._state.sequence);
    this.emit('stateChange:sequence', this._state.sequence);
  }

  /**
   * Subscribes to events or state changes
   * @param {string} event - e.g. 'stateChange', 'telemetry:updated', 'safety:alert', etc.
   * @param {Function} handler
   */
  subscribe(event, handler) {
    const norm = event.toLowerCase();
    if (!this.listeners.has(norm)) {
      this.listeners.set(norm, new Set());
    }
    this.listeners.get(norm).add(handler);
    return () => this.unsubscribe(norm, handler);
  }

  /**
   * Alias for subscribe
   */
  on(event, handler) {
    return this.subscribe(event, handler);
  }

  /**
   * Unsubscribes from events
   */
  unsubscribe(event, handler) {
    const norm = event.toLowerCase();
    if (this.listeners.has(norm)) {
      this.listeners.get(norm).delete(handler);
    }
  }

  /**
   * Alias for unsubscribe
   */
  off(event, handler) {
    this.unsubscribe(event, handler);
  }

  /**
   * Emits event to subscribers
   * @param {string} event
   * @param {*} data
   */
  emit(event, data) {
    const norm = event.toLowerCase();
    if (this.listeners.has(norm)) {
      for (const fn of this.listeners.get(norm)) {
        try {
          fn(data);
        } catch (err) {
          console.error(`Error in AppState listener for '${event}':`, err);
        }
      }
    }
    // Also trigger wildcard listeners
    if (norm !== '*' && this.listeners.has('*')) {
      for (const fn of this.listeners.get('*')) {
        try {
          fn({ event, data });
        } catch (err) {
          console.error(`Error in AppState wildcard listener:`, err);
        }
      }
    }
  }


  /**
   * Ingests 10Hz JSON telemetry and updates state atomically
   * @param {Object} telem - Parsed JTELEM packet
   */
  updateLiveTelemetry(telem) {
    if (!telem || typeof telem !== 'object') return;

    const now = telem.timestamp || Date.now();

    const j1 = Number(telem.j1 ?? this._state.livePose.j1);
    const j2 = Number(telem.j2 ?? this._state.livePose.j2);
    const j3 = Number(telem.j3 ?? this._state.livePose.j3);
    const j4 = Number(telem.j4 ?? this._state.livePose.j4);
    const j5 = Number(telem.j5 ?? this._state.livePose.j5);
    const j6 = Number(telem.j6 ?? this._state.livePose.j6 ?? 0.0);
    const fk = forwardKinematics6DOF ? forwardKinematics6DOF(j1, j2, j3, j4, j5, j6) : forwardKinematics(j1, j2, j3, {}, null, j4, j5, j6);

    // Update live pose
    this._state.livePose = {
      j1,
      j2,
      j3,
      j4,
      j5,
      j6,
      grip: Number(telem.grip ?? this._state.livePose.grip),
      x: Number(telem.x !== undefined ? telem.x : fk.tcp.x),
      y: Number(telem.y !== undefined ? telem.y : fk.tcp.y),
      z: Number(telem.z !== undefined ? telem.z : fk.tcp.z),
      roll: Number(telem.roll !== undefined ? telem.roll : (fk.roll ?? 0.0)),
      pitch: Number(telem.pitch !== undefined ? telem.pitch : (fk.pitch ?? 0.0)),
      yaw: Number(telem.yaw !== undefined ? telem.yaw : (fk.yaw ?? 0.0)),
      dx: Number(telem.dx ?? this._state.livePose.dx),
      dy: Number(telem.dy ?? this._state.livePose.dy),
      lastUpdated: now
    };

    // Update safety & status flags
    if (telem.estop !== undefined) {
      this._state.safety.estop = Boolean(telem.estop);
    }
    if (telem.homing !== undefined) {
      this._state.safety.homing = Boolean(telem.homing);
    }
    if (telem.spd !== undefined) {
      this._state.safety.speedOverride = Number(telem.spd);
    }
    if (telem.teach !== undefined) {
      this._state.teach.active = Boolean(telem.teach);
    }
    if (telem.tpts !== undefined) {
      this._state.teach.pointCount = Number(telem.tpts);
    }

    this._state.connection.lastHeartbeat = now;

    // Push into rolling telemetry history
    this._state.history.telemetry.push({
      timestamp: now,
      j1: this._state.livePose.j1,
      j2: this._state.livePose.j2,
      j3: this._state.livePose.j3,
      x: this._state.livePose.x,
      y: this._state.livePose.y,
      z: this._state.livePose.z,
      spd: this._state.safety.speedOverride,
      estop: this._state.safety.estop
    });

    if (this._state.history.telemetry.length > this.maxHistoryLength) {
      this._state.history.telemetry.shift();
    }

    if (telem.effort !== undefined || telem.load !== undefined || telem.grip_effort !== undefined) {
      const effortVal = Number(telem.effort ?? telem.load ?? telem.grip_effort ?? 0);
      const isContact = Boolean(telem.contact || telem.clamped || (effortVal > 150));
      this._state.gripper.effort = effortVal;
      this._state.gripper.objectDetected = isContact;
      this.emit('gripper:telemetry', this._state.gripper);
    }

    this.emit('telemetry:updated', this._state.livePose);
    this.emit('stateChange:livePose', this._state.livePose);
    this.emit('stateChange:safety', this._state.safety);
  }

  /**
   * Sets execution safety mode ('preview' vs 'direct_drive') and armed state
   * @param {'preview'|'direct_drive'} mode
   * @param {boolean} [armed=false]
   * @returns {Object} Updated execution state
   */
  setExecutionMode(mode, armed = false) {
    const validMode = (mode === 'direct_drive') ? 'direct_drive' : 'preview';
    const isArmed = Boolean(armed && validMode === 'direct_drive');
    this._state.execution = {
      mode: validMode,
      armed: isArmed,
      directDrivePulsing: isArmed
    };
    this.emit('execution:mode', this._state.execution);
    this.emit('stateChange:execution', this._state.execution);
    return { ...this._state.execution };
  }

  /**
   * Sets active primary control mode tab ('ik', 'joints', 'trajectory')
   * @param {'ik'|'joints'|'trajectory'} tab
   * @returns {string} Active tab
   */
  setControlModeTab(tab) {
    const validTab = ['ik', 'joints', 'trajectory'].includes(tab) ? tab : 'ik';
    this._state.controlMode.activeTab = validTab;
    this.emit('controlMode:tab', validTab);
    this.emit('stateChange:controlMode', this._state.controlMode);
    return validTab;
  }

  /**
   * Sets active Cartesian coordinate frame ('base', 'world', 'tool')
   * @param {'base'|'world'|'tool'} frame
   * @returns {string} Active frame
   */
  setCartesianFrame(frame) {
    const validFrame = ['base', 'world', 'tool'].includes(frame) ? frame : 'base';
    this._state.controlMode.cartesianFrame = validFrame;
    this.emit('controlMode:frame', validFrame);
    this.emit('stateChange:controlMode', this._state.controlMode);
    return validFrame;
  }

  /**
   * Updates gripper status and telemetry values
   * @param {Object} telem - { percent?: number, effort?: number, objectDetected?: boolean }
   */
  updateGripperTelemetry(telem = {}) {
    if (!telem || typeof telem !== 'object') return;
    let changed = false;

    if (telem.percent !== undefined) {
      this._state.gripper.percent = Math.max(0, Math.min(100, Math.round(Number(telem.percent))));
      changed = true;
    }
    if (telem.effort !== undefined) {
      this._state.gripper.effort = Number(telem.effort) || 0;
      changed = true;
    }
    if (telem.objectDetected !== undefined) {
      this._state.gripper.objectDetected = Boolean(telem.objectDetected);
      changed = true;
    }

    if (changed) {
      this.emit('gripper:telemetry', this._state.gripper);
      this.emit('stateChange:gripper', this._state.gripper);
    }
  }

  /**
   * Updates ghost target pose
   * @param {Object} pose - Partial or full ghost pose
   */
  updateGhostPose(pose) {
    if (!pose || typeof pose !== 'object') return;
    this._state.ghostPose = {
      ...this._state.ghostPose,
      ...pose
    };
    this.emit('ghost:updated', this._state.ghostPose);
    this.emit('stateChange:ghostPose', this._state.ghostPose);
  }

  /**
   * Updates connection status
   */
  setConnectionStatus(status, details = {}) {
    this._state.connection = {
      ...this._state.connection,
      status,
      ...details
    };
    this.emit('connection:status', this._state.connection);
    this.emit('stateChange:connection', this._state.connection);
  }

  /**
   * Sets Emergency Stop status
   * @param {boolean} active
   */
  setEstop(active) {
    this._state.safety.estop = Boolean(active);
    this.emit('safety:estop', this._state.safety.estop);
    this.emit('stateChange:safety', this._state.safety);
  }

  /**
   * Sets Homing status
   * @param {boolean} active
   */
  setHoming(active) {
    this._state.safety.homing = Boolean(active);
    this.emit('safety:homing', this._state.safety.homing);
    this.emit('stateChange:safety', this._state.safety);
  }

  /**
   * Sets Speed override percentage
   * @param {number} percent
   */
  setSpeedOverride(percent) {
    this._state.safety.speedOverride = Math.min(100, Math.max(5, Math.round(percent)));
    this.emit('safety:speed', this._state.safety.speedOverride);
    this.emit('stateChange:safety', this._state.safety);
  }

  /**
   * Sets Closed Loop mode status
   * @param {boolean} active
   */
  setClosedLoop(active) {
    this._state.safety.closedLoop = Boolean(active);
    this.emit('safety:closedLoop', this._state.safety.closedLoop);
    this.emit('stateChange:safety', this._state.safety);
  }

  /**
   * Sets Collision alert flags
   */
  setCollision(j1, j2, j3) {
    this._state.safety.collision = {
      j1: Boolean(j1),
      j2: Boolean(j2),
      j3: Boolean(j3)
    };
    this._state.safety.hasCollision = Boolean(j1 || j2 || j3);
    this.emit('safety:collision', this._state.safety.collision);
    this.emit('stateChange:safety', this._state.safety);
  }

  /**
   * Adds entry to console log history
   * @param {Object} logEntry - { type: 'TX'|'RX'|'INFO'|'WARN'|'ERR', text: string, timestamp?: number }
   */
  addConsoleLog(logEntry) {
    const entry = {
      id: 'log_' + Date.now() + '_' + Math.random().toString(36).substring(2, 7),
      type: logEntry.type || 'INFO',
      text: String(logEntry.text || ''),
      timestamp: logEntry.timestamp || Date.now()
    };
    this._state.history.logs.push(entry);
    if (this._state.history.logs.length > this.maxLogLength) {
      this._state.history.logs.shift();
    }
    this.emit('log:added', entry);
    return entry;
  }

  /**
   * Clears console log history
   */
  clearConsoleLogs() {
    this._state.history.logs = [];
    this.emit('log:cleared');
  }

  /**
   * Appends a waypoint to active sequence
   * @param {Object} wp - Waypoint data
   */
  addWaypoint(wp) {
    const newWp = {
      id: wp.id || 'wp_' + Date.now() + '_' + Math.random().toString(36).substring(2, 7),
      label: wp.label || `Point ${this._state.sequence.waypoints.length + 1}`,
      j1: Number(wp.j1 ?? this._state.livePose.j1),
      j2: Number(wp.j2 ?? this._state.livePose.j2),
      j3: Number(wp.j3 ?? this._state.livePose.j3),
      j4: Number(wp.j4 ?? this._state.livePose.j4),
      j5: Number(wp.j5 ?? this._state.livePose.j5),
      j6: Number(wp.j6 ?? this._state.livePose.j6 ?? 0.0),
      dx: Number(wp.dx ?? 0),
      dy: Number(wp.dy ?? 0),
      dur: Number(wp.dur ?? wp.duration ?? 1000),
      grip: Number(wp.grip ?? this._state.livePose.grip),
      comment: wp.comment || ''
    };
    this._state.sequence.waypoints.push(newWp);
    this.emit('sequence:updated', this._state.sequence);
    this.emit('stateChange:sequence', this._state.sequence);
    return newWp;
  }

  /**
   * Updates an existing waypoint by ID
   */
  updateWaypoint(id, updates) {
    const idx = this._state.sequence.waypoints.findIndex((w) => w.id === id);
    if (idx !== -1) {
      const originalId = this._state.sequence.waypoints[idx].id;
      this._state.sequence.waypoints[idx] = {
        ...this._state.sequence.waypoints[idx],
        ...updates,
        id: originalId
      };
      this.emit('sequence:updated', this._state.sequence);
      this.emit('stateChange:sequence', this._state.sequence);
      return this._state.sequence.waypoints[idx];
    }
    return null;
  }

  /**
   * Removes a waypoint by ID
   */
  removeWaypoint(id) {
    const initialLen = this._state.sequence.waypoints.length;
    this._state.sequence.waypoints = this._state.sequence.waypoints.filter((w) => w.id !== id);
    if (this._state.sequence.waypoints.length !== initialLen) {
      this.emit('sequence:updated', this._state.sequence);
      this.emit('stateChange:sequence', this._state.sequence);
      return true;
    }
    return false;
  }

  /**
   * Clears all waypoints
   */
  clearWaypoints() {
    this._state.sequence.waypoints = [];
    this._state.sequence.activeIndex = -1;
    this._state.sequence.isPlaying = false;
    this._state.sequence.progress = 0;
    this.emit('sequence:updated', this._state.sequence);
    this.emit('stateChange:sequence', this._state.sequence);
  }

  /**
   * Reorders a waypoint from one index to another
   */
  reorderWaypoints(fromIndex, toIndex) {
    const list = this._state.sequence.waypoints;
    if (fromIndex < 0 || fromIndex >= list.length || toIndex < 0 || toIndex >= list.length) {
      return false;
    }
    const [moved] = list.splice(fromIndex, 1);
    list.splice(toIndex, 0, moved);
    this.emit('sequence:updated', this._state.sequence);
    this.emit('stateChange:sequence', this._state.sequence);
    return true;
  }

  /**
   * Sets entire sequence dataset
   */
  setSequence(seq) {
    if (!seq || typeof seq !== 'object') return;
    this._state.sequence = {
      ...this._state.sequence,
      name: seq.sequenceName || seq.name || 'Imported Sequence',
      loop: Boolean(seq.loop),
      waypoints: Array.isArray(seq.waypoints) ? fastClone(seq.waypoints) : [],
      isPlaying: false,
      activeIndex: -1,
      progress: 0
    };
    this.emit('sequence:updated', this._state.sequence);
    this.emit('stateChange:sequence', this._state.sequence);
  }
}

// Global Singleton Instance
export const appState = new AppState();
export default appState;
