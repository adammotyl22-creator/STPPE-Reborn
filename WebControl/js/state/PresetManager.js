/**
 * PresetManager.js - Trajectory Preset Library & JSON Import/Export Engine
 * 
 * Manages default and user-defined trajectory sequences, LocalStorage persistence
 * under the key 'robot_arm_studio_presets', and bidirectional JSON sequence
 * import/export with rigorous schema validation.
 * 
 * Default Presets:
 * 1. Home - Safe reference resting pose
 * 2. Calibrate/Diagnostics - Full multi-axis diagnostic and calibration sweep
 * 3. Pick & Place Demo - Complete industrial 8-step pick, lift, transfer, place cycle
 * 4. Scan Sweep - Continuous inspection scanning sweep across the workspace
 * 5. Wave Pose Demo - Dynamic greeting wave and pose sequence
 */

import { GCodeInterpreter } from '../kinematics/GCodeInterpreter.js';

export const STORAGE_KEY = 'robot_arm_studio_presets';

/**
 * Built-in default trajectory sequences
 */
export const DEFAULT_PRESETS = Object.freeze({
  'Home': {
    sequenceName: 'Home',
    version: '1.0',
    loop: false,
    description: 'Safe baseline resting pose with gripper centered',
    isBuiltin: true,
    waypoints: [
      {
        id: 'wp_home_1',
        label: 'Home Pose',
        j1: 0.0,
        j2: 0.0,
        j3: 0.0,
        j4: 0.0,
        j5: 0.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 1500,
        grip: 90,
        comment: 'Default resting reference pose'
      }
    ]
  },
  'Calibrate/Diagnostics': {
    sequenceName: 'Calibrate/Diagnostics',
    version: '1.0',
    loop: false,
    description: 'Diagnostic exercise testing all stepper axes, servos, and gripper',
    isBuiltin: true,
    waypoints: [
      {
        id: 'wp_diag_1',
        label: 'Home Reset',
        j1: 0.0,
        j2: 0.0,
        j3: 0.0,
        j4: 0.0,
        j5: 0.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 1500,
        grip: 90,
        comment: 'Reset to home baseline'
      },
      {
        id: 'wp_diag_2',
        label: 'Base CCW (+45°)',
        j1: 45.0,
        j2: 0.0,
        j3: 0.0,
        j4: 0.0,
        j5: 0.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 1200,
        grip: 90,
        comment: 'Joint 1 counter-clockwise limit check'
      },
      {
        id: 'wp_diag_3',
        label: 'Base CW (-45°)',
        j1: -45.0,
        j2: 0.0,
        j3: 0.0,
        j4: 0.0,
        j5: 0.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 1600,
        grip: 90,
        comment: 'Joint 1 clockwise limit check'
      },
      {
        id: 'wp_diag_4',
        label: 'Base Center',
        j1: 0.0,
        j2: 0.0,
        j3: 0.0,
        j4: 0.0,
        j5: 0.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 1200,
        grip: 90,
        comment: 'Joint 1 return to center'
      },
      {
        id: 'wp_diag_5',
        label: 'Shoulder Pitch Forward',
        j1: 0.0,
        j2: 30.0,
        j3: 0.0,
        j4: 0.0,
        j5: 0.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 1500,
        grip: 90,
        comment: 'Joint 2 forward reach'
      },
      {
        id: 'wp_diag_6',
        label: 'Elbow Pitch Up',
        j1: 0.0,
        j2: 30.0,
        j3: 30.0,
        j4: 0.0,
        j5: 0.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 1200,
        grip: 90,
        comment: 'Joint 3 elevation check'
      },
      {
        id: 'wp_diag_7',
        label: 'Wrist Pitch Test',
        j1: 0.0,
        j2: 30.0,
        j3: 30.0,
        j4: 45.0,
        j5: 0.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 1000,
        grip: 90,
        comment: 'Joint 4 forearm roll range'
      },
      {
        id: 'wp_diag_8',
        label: 'Wrist Roll Test',
        j1: 0.0,
        j2: 30.0,
        j3: 30.0,
        j4: 0.0,
        j5: 30.0,
        j6: 45.0,
        dx: 0,
        dy: 0,
        dur: 1000,
        grip: 90,
        comment: 'Joint 5 and 6 wrist articulation range'
      },
      {
        id: 'wp_diag_9',
        label: 'Gripper Open',
        j1: 0.0,
        j2: 30.0,
        j3: 30.0,
        j4: 0.0,
        j5: 0.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 800,
        grip: 180,
        comment: 'Gripper servo full open'
      },
      {
        id: 'wp_diag_10',
        label: 'Gripper Close',
        j1: 0.0,
        j2: 30.0,
        j3: 30.0,
        j4: 0.0,
        j5: 0.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 800,
        grip: 0,
        comment: 'Gripper servo full close'
      },
      {
        id: 'wp_diag_11',
        label: 'Return to Home',
        j1: 0.0,
        j2: 0.0,
        j3: 0.0,
        j4: 0.0,
        j5: 0.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 1800,
        grip: 90,
        comment: 'Complete diagnostics and park'
      }
    ]
  },
  'Pick & Place Demo': {
    sequenceName: 'Pick & Place Demo',
    version: '1.0',
    loop: false,
    description: 'High-speed 8-step industrial pick and place cycle with gripper coordination',
    isBuiltin: true,
    waypoints: [
      {
        id: 'wp_pnp_1',
        label: 'Approach Bin A',
        j1: 30.0,
        j2: 31.6,
        j3: 7.1,
        j4: 0.0,
        j5: -43.6,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 1200,
        grip: 0,
        comment: 'Position gripper directly above part in Bin A'
      },
      {
        id: 'wp_pnp_2',
        label: 'Descend to Pick',
        j1: 30.0,
        j2: 15.0,
        j3: 5.0,
        j4: 0.0,
        j5: -25.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 800,
        grip: 0,
        comment: 'Lower into pickup envelope'
      },
      {
        id: 'wp_pnp_3',
        label: 'Grip Close',
        j1: 30.0,
        j2: 15.0,
        j3: 5.0,
        j4: 0.0,
        j5: -25.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 500,
        grip: 180,
        comment: 'Actuate gripper to securely clamp part'
      },
      {
        id: 'wp_pnp_4',
        label: 'Lift',
        j1: 30.0,
        j2: 31.6,
        j3: 7.1,
        j4: 0.0,
        j5: -43.6,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 1000,
        grip: 180,
        comment: 'Ascend clear of Bin A walls'
      },
      {
        id: 'wp_pnp_5',
        label: 'Transfer Place',
        j1: -45.0,
        j2: 31.6,
        j3: 7.1,
        j4: 0.0,
        j5: -43.6,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 1500,
        grip: 180,
        comment: 'Rotational transfer to destination Bin B'
      },
      {
        id: 'wp_pnp_6',
        label: 'Descend Place',
        j1: -45.0,
        j2: 15.0,
        j3: 5.0,
        j4: 0.0,
        j5: -25.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 800,
        grip: 180,
        comment: 'Lower into placement target'
      },
      {
        id: 'wp_pnp_7',
        label: 'Release',
        j1: -45.0,
        j2: 15.0,
        j3: 5.0,
        j4: 0.0,
        j5: -25.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 500,
        grip: 0,
        comment: 'Release part into target receptacle'
      },
      {
        id: 'wp_pnp_8',
        label: 'Retract Home',
        j1: 0.0,
        j2: 0.0,
        j3: 0.0,
        j4: 0.0,
        j5: 0.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 1200,
        grip: 0,
        comment: 'Return to home ready state'
      }
    ]
  },
  'Scan Sweep': {
    sequenceName: 'Scan Sweep',
    version: '1.0',
    loop: false,
    description: 'Smooth continuous arc sweep across workspace for vision or sensor inspection',
    isBuiltin: true,
    waypoints: [
      {
        id: 'wp_scan_1',
        label: 'Scan Left Boundary',
        j1: 50.0,
        j2: 20.0,
        j3: 15.0,
        j4: 0.0,
        j5: -20.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 1500,
        grip: 90,
        comment: 'Position sensor at left sector'
      },
      {
        id: 'wp_scan_2',
        label: 'Scan Mid-Left',
        j1: 25.0,
        j2: 20.0,
        j3: 15.0,
        j4: 0.0,
        j5: -20.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 1000,
        grip: 90,
        comment: 'Sweep through mid-left sector'
      },
      {
        id: 'wp_scan_3',
        label: 'Scan Center Ahead',
        j1: 0.0,
        j2: 20.0,
        j3: 15.0,
        j4: 0.0,
        j5: -20.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 1000,
        grip: 90,
        comment: 'Sweep through center sector'
      },
      {
        id: 'wp_scan_4',
        label: 'Scan Mid-Right',
        j1: -25.0,
        j2: 20.0,
        j3: 15.0,
        j4: 0.0,
        j5: -20.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 1000,
        grip: 90,
        comment: 'Sweep through mid-right sector'
      },
      {
        id: 'wp_scan_5',
        label: 'Scan Right Boundary',
        j1: -50.0,
        j2: 20.0,
        j3: 15.0,
        j4: 0.0,
        j5: -20.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 1000,
        grip: 90,
        comment: 'Reach right sector boundary'
      },
      {
        id: 'wp_scan_6',
        label: 'Return Home',
        j1: 0.0,
        j2: 0.0,
        j3: 0.0,
        j4: 0.0,
        j5: 0.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 1800,
        grip: 90,
        comment: 'Return to neutral home stance'
      }
    ]
  },
  'Wave Pose Demo': {
    sequenceName: 'Wave Pose Demo',
    version: '1.0',
    loop: false,
    description: 'Friendly human-robot interaction wave and polite bow motion sequence',
    isBuiltin: true,
    waypoints: [
      {
        id: 'wp_wave_1',
        label: 'Standby Pose',
        j1: 0.0,
        j2: 0.0,
        j3: 0.0,
        j4: 0.0,
        j5: 0.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 1000,
        grip: 90,
        comment: 'Initial standby'
      },
      {
        id: 'wp_wave_2',
        label: 'Raise Arm High',
        j1: 0.0,
        j2: 45.0,
        j3: 20.0,
        j4: 0.0,
        j5: 0.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 1200,
        grip: 180,
        comment: 'Elevate arm into greeting stance'
      },
      {
        id: 'wp_wave_3',
        label: 'Wave Left',
        j1: 15.0,
        j2: 45.0,
        j3: 20.0,
        j4: 0.0,
        j5: 0.0,
        j6: 30.0,
        dx: 0,
        dy: 0,
        dur: 600,
        grip: 180,
        comment: 'Wave wrist to the left'
      },
      {
        id: 'wp_wave_4',
        label: 'Wave Right',
        j1: -15.0,
        j2: 45.0,
        j3: 20.0,
        j4: 0.0,
        j5: 0.0,
        j6: -30.0,
        dx: 0,
        dy: 0,
        dur: 600,
        grip: 180,
        comment: 'Wave wrist to the right'
      },
      {
        id: 'wp_wave_5',
        label: 'Wave Left Again',
        j1: 15.0,
        j2: 45.0,
        j3: 20.0,
        j4: 0.0,
        j5: 0.0,
        j6: 30.0,
        dx: 0,
        dy: 0,
        dur: 600,
        grip: 180,
        comment: 'Second wave left'
      },
      {
        id: 'wp_wave_6',
        label: 'Wave Right Again',
        j1: -15.0,
        j2: 45.0,
        j3: 20.0,
        j4: 0.0,
        j5: 0.0,
        j6: -30.0,
        dx: 0,
        dy: 0,
        dur: 600,
        grip: 180,
        comment: 'Second wave right'
      },
      {
        id: 'wp_wave_7',
        label: 'Bow Forward',
        j1: 0.0,
        j2: 20.0,
        j3: 10.0,
        j4: 0.0,
        j5: -25.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 1200,
        grip: 90,
        comment: 'Courteous bow to audience'
      },
      {
        id: 'wp_wave_8',
        label: 'Return Home',
        j1: 0.0,
        j2: 0.0,
        j3: 0.0,
        j4: 0.0,
        j5: 0.0,
        j6: 0.0,
        dx: 0,
        dy: 0,
        dur: 1500,
        grip: 90,
        comment: 'Return to neutral home stance'
      }
    ]
  },
    'CNC Star (G-Code)': {
    sequenceName: 'CNC Star (G-Code)',
    version: '1.0',
    loop: false,
    description: 'Precision 5-point star CNC toolpath generated from standard G-code with rapid moves and tool plunging',
    isBuiltin: true,
    waypoints: [
      { id: 'wp_cnc_1', label: 'Approach Star Top (G0)', j1: 6.10, j2: 41.13, j3: 41.39, j4: 0.0, j5: -82.52, j6: 0.0, dx: 0, dy: 0, dur: 350, grip: 0, comment: 'G0 X0 Y20 Z10 F3000' },
      { id: 'wp_cnc_2', label: 'Plunge to Surface (G1)', j1: 6.10, j2: 45.08, j3: 40.47, j4: 0.0, j5: -85.55, j6: 0.0, dx: 0, dy: 0, dur: 500, grip: 180, comment: 'G1 Z0 F600 ; Tool on' },
      { id: 'wp_cnc_3', label: 'Inner Corner 1', j1: 1.13, j2: 46.10, j3: 37.98, j4: 0.0, j5: -84.08, j6: 0.0, dx: 0, dy: 0, dur: 750, grip: 180, comment: 'G1 X5.88 Y6.18' },
      { id: 'wp_cnc_4', label: 'Star Tip 2 (Right)', j1: 1.05, j2: 48.96, j3: 31.22, j4: 0.0, j5: -80.18, j6: 0.0, dx: 0, dy: 0, dur: 650, grip: 180, comment: 'G1 X19.02 Y6.18' },
      { id: 'wp_cnc_5', label: 'Inner Corner 2', j1: -1.51, j2: 46.64, j3: 36.69, j4: 0.0, j5: -83.33, j6: 0.0, dx: 0, dy: 0, dur: 650, grip: 180, comment: 'G1 X8.41 Y-1.53' },
      { id: 'wp_cnc_6', label: 'Star Tip 3 (Bottom-Right)', j1: -5.66, j2: 47.66, j3: 34.27, j4: 0.0, j5: -81.93, j6: 0.0, dx: 0, dy: 0, dur: 660, grip: 180, comment: 'G1 X12.36 Y-14.18' },
      { id: 'wp_cnc_7', label: 'Inner Corner 3 (Bottom)', j1: -2.90, j2: 44.94, j3: 40.81, j4: 0.0, j5: -85.75, j6: 0.0, dx: 0, dy: 0, dur: 760, grip: 180, comment: 'G1 X0 Y-5.20' },
      { id: 'wp_cnc_8', label: 'Star Tip 4 (Bottom-Left)', j1: -6.60, j2: 42.73, j3: 46.41, j4: 0.0, j5: -89.14, j6: 0.0, dx: 0, dy: 0, dur: 760, grip: 180, comment: 'G1 X-12.36 Y-14.18' },
      { id: 'wp_cnc_9', label: 'Inner Corner 4', j1: -1.68, j2: 43.29, j3: 44.96, j4: 0.0, j5: -88.25, j6: 0.0, dx: 0, dy: 0, dur: 660, grip: 180, comment: 'G1 X-8.41 Y-1.53' },
      { id: 'wp_cnc_10', label: 'Star Tip 5 (Left)', j1: 1.33, j2: 41.39, j3: 49.98, j4: 0.0, j5: -90.00, j6: 0.0, dx: 0, dy: 0, dur: 650, grip: 180, comment: 'G1 X-19.02 Y6.18' },
      { id: 'wp_cnc_11', label: 'Inner Corner 5', j1: 1.22, j2: 43.76, j3: 43.76, j4: 0.0, j5: -87.52, j6: 0.0, dx: 0, dy: 0, dur: 650, grip: 180, comment: 'G1 X-5.88 Y6.18' },
      { id: 'wp_cnc_12', label: 'Close Star Contour', j1: 6.10, j2: 45.08, j3: 40.47, j4: 0.0, j5: -85.55, j6: 0.0, dx: 0, dy: 0, dur: 750, grip: 180, comment: 'G1 X0 Y20' },
      { id: 'wp_cnc_13', label: 'Retract Tool & Finish (G0)', j1: 6.10, j2: 39.19, j3: 41.74, j4: 0.0, j5: -80.93, j6: 0.0, dx: 0, dy: 0, dur: 300, grip: 0, comment: 'G0 Z15 ; Tool off' }
    ]
  }
});

/**
 * In-memory fallback storage when localStorage is unavailable
 */
class MemoryStorage {
  constructor() {
    this._store = new Map();
  }
  getItem(key) {
    return this._store.has(key) ? this._store.get(key) : null;
  }
  setItem(key, value) {
    this._store.set(key, String(value));
  }
  removeItem(key) {
    this._store.delete(key);
  }
  clear() {
    this._store.clear();
  }
}

/**
 * Master PresetManager Class
 */
export class PresetManager {
  /**
   * @param {Object} [options]
   * @param {string} [options.storageKey=STORAGE_KEY]
   * @param {Storage} [options.storageEngine]
   */
  constructor(options = {}) {
    this.storageKey = options.storageKey || STORAGE_KEY;
    this.storage = options.storageEngine || this._detectStorageEngine();
    this._ensureDefaultsLoaded();
  }

  /**
   * Detects whether browser localStorage is accessible or falls back to MemoryStorage
   * @private
   */
  _detectStorageEngine() {
    if (typeof window !== 'undefined' && window.localStorage) {
      try {
        const testKey = '__storage_test__';
        window.localStorage.setItem(testKey, testKey);
        window.localStorage.removeItem(testKey);
        return window.localStorage;
      } catch (e) {
        return new MemoryStorage();
      }
    }
    return new MemoryStorage();
  }

  /**
   * Ensures default presets exist in storage on first initialization
   * @private
   */
  _ensureDefaultsLoaded() {
    try {
      const existing = this.storage.getItem(this.storageKey);
      if (!existing) {
        const defaults = JSON.parse(JSON.stringify(DEFAULT_PRESETS));
        this.storage.setItem(this.storageKey, JSON.stringify(defaults));
      }
    } catch (err) {
      console.warn('PresetManager: Error initializing default presets:', err);
    }
  }

  /**
   * Validates a single waypoint object against schema rules
   * 
   * @param {Object} wp
   * @param {number} index
   * @returns {{ valid: boolean, errors: string[], waypoint: Object }}
   */
  static validateWaypoint(wp, index = 0) {
    const errors = [];
    if (!wp || typeof wp !== 'object') {
      return { valid: false, errors: [`Waypoint at index ${index} must be an object`], waypoint: null };
    }

    // Required / formatted fields
    const id = typeof wp.id === 'string' && wp.id.trim().length > 0
      ? wp.id.trim()
      : `wp_${Date.now()}_${Math.random().toString(36).substring(2, 8)}`;

    const label = typeof wp.label === 'string' && wp.label.trim().length > 0
      ? wp.label.trim()
      : `Point ${index + 1}`;

    const numOrError = (val, fieldName, fallback) => {
      const num = Number(val ?? fallback);
      if (isNaN(num)) {
        errors.push(`Waypoint ${index} (${label}): field '${fieldName}' must be a valid number, got ${val}`);
        return fallback;
      }
      return num;
    };

    const j1 = numOrError(wp.j1, 'j1', 0.0);
    const j2 = numOrError(wp.j2, 'j2', 0.0);
    const j3 = numOrError(wp.j3, 'j3', 0.0);
    const j4 = numOrError(wp.j4, 'j4', 0.0);
    const j5 = numOrError(wp.j5, 'j5', 90.0);
    const j6 = numOrError(wp.j6, 'j6', 0.0);
    const dx = Math.round(numOrError(wp.dx, 'dx', 0));
    const dy = Math.round(numOrError(wp.dy, 'dy', 0));
    const rawDur = numOrError(wp.dur ?? wp.duration, 'dur', 1000);
    const dur = Math.max(1, Math.round(rawDur));
    const rawGrip = numOrError(wp.grip, 'grip', 90);
    const grip = Math.min(180, Math.max(0, Math.round(rawGrip)));
    const comment = typeof wp.comment === 'string' ? wp.comment : '';
    // Preserve motion type: 'movl' (Cartesian linear) or 'joint' (default joint-space spline)
    const moveType = (typeof wp.moveType === 'string' && wp.moveType.trim().length > 0)
      ? wp.moveType.trim().toLowerCase()
      : 'joint';

    const sanitized = {
      id,
      label,
      j1: Number(j1.toFixed(2)),
      j2: Number(j2.toFixed(2)),
      j3: Number(j3.toFixed(2)),
      j4: Number(j4.toFixed(1)),
      j5: Number(j5.toFixed(1)),
      j6: Number(j6.toFixed(1)),
      dx,
      dy,
      dur,
      grip,
      moveType,
      comment
    };

    return {
      valid: errors.length === 0,
      errors,
      waypoint: sanitized
    };
  }

  /**
   * Validates a complete sequence dataset against schema
   * 
   * Schema:
   * - sequenceName: string
   * - version: string (default "1.0")
   * - loop: boolean (default false)
   * - waypoints: array of waypoint objects
   * 
   * @param {Object} seq
   * @returns {{ valid: boolean, errors: string[], sequence: Object }}
   */
  static validateSequence(seq) {
    const errors = [];
    if (!seq || typeof seq !== 'object' || Array.isArray(seq)) {
      return { valid: false, errors: ['Sequence must be a non-null object'], sequence: null };
    }

    const sequenceName = typeof seq.sequenceName === 'string' && seq.sequenceName.trim().length > 0
      ? seq.sequenceName.trim()
      : (typeof seq.name === 'string' && seq.name.trim().length > 0 ? seq.name.trim() : 'Unnamed Sequence');

    const version = typeof seq.version === 'string' ? seq.version : '1.0';
    const loop = Boolean(seq.loop);
    const description = typeof seq.description === 'string' ? seq.description : '';

    if (!Array.isArray(seq.waypoints)) {
      errors.push('Sequence property "waypoints" must be an array');
      return { valid: false, errors, sequence: null };
    }

    const sanitizedWaypoints = [];
    for (let i = 0; i < seq.waypoints.length; i++) {
      const valResult = PresetManager.validateWaypoint(seq.waypoints[i], i);
      if (!valResult.valid) {
        errors.push(...valResult.errors);
      } else {
        sanitizedWaypoints.push(valResult.waypoint);
      }
    }

    const sanitized = {
      sequenceName,
      version,
      loop,
      description,
      isBuiltin: Boolean(seq.isBuiltin),
      waypoints: sanitizedWaypoints
    };

    return {
      valid: errors.length === 0,
      errors,
      sequence: sanitized
    };
  }

  /**
   * Retrieves raw dictionary of all stored presets (builtins + user custom)
   * @returns {Object.<string, Object>}
   */
  _getRawPresetsMap() {
    try {
      const raw = this.storage.getItem(this.storageKey);
      let parsed = raw ? JSON.parse(raw) : {};
      if (!parsed || typeof parsed !== 'object') {
        parsed = {};
      }
      // Ensure all DEFAULT_PRESETS are populated
      let modified = false;
      for (const [key, defaultSeq] of Object.entries(DEFAULT_PRESETS)) {
        if (!parsed[key]) {
          parsed[key] = JSON.parse(JSON.stringify(defaultSeq));
          modified = true;
        }
      }
      if (modified) {
        this.storage.setItem(this.storageKey, JSON.stringify(parsed));
      }
      return parsed;
    } catch (err) {
      console.warn('PresetManager: Error parsing presets from storage, resetting:', err);
      const defaults = JSON.parse(JSON.stringify(DEFAULT_PRESETS));
      this.storage.setItem(this.storageKey, JSON.stringify(defaults));
      return defaults;
    }
  }

  /**
   * Returns list of all presets with metadata
   * @returns {Array<{ name: string, label: string, loop: boolean, waypointCount: number, totalDuration: number, isBuiltin: boolean, description: string, sequence: Object }>}
   */
  getAllPresets() {
    const rawMap = this._getRawPresetsMap();
    const result = [];

    for (const [key, seq] of Object.entries(rawMap)) {
      const val = PresetManager.validateSequence(seq);
      if (val.valid && val.sequence) {
        const totalDur = val.sequence.waypoints.reduce((sum, wp) => sum + (wp.dur || 0), 0);
        result.push({
          name: key,
          label: val.sequence.sequenceName || key,
          loop: val.sequence.loop,
          waypointCount: val.sequence.waypoints.length,
          totalDuration: totalDur,
          isBuiltin: Boolean(val.sequence.isBuiltin || DEFAULT_PRESETS[key]),
          description: val.sequence.description || '',
          sequence: val.sequence
        });
      }
    }

    return result;
  }

  /**
   * Retrieves a specific preset by name
   * @param {string} name
   * @returns {Object|null} Cloned sequence object or null if not found
   */
  getPreset(name) {
    if (!name) return null;
    const rawMap = this._getRawPresetsMap();
    const seq = rawMap[name];
    if (!seq) return null;

    const val = PresetManager.validateSequence(seq);
    if (!val.valid) return null;
    return JSON.parse(JSON.stringify(val.sequence));
  }

  /**
   * Saves or updates a preset in storage
   * 
   * @param {string} name - Identifier name
   * @param {Object} sequenceData - Sequence data conforming to schema
   * @returns {{ success: boolean, preset: Object, error?: string }}
   */
  savePreset(name, sequenceData) {
    if (!name || typeof name !== 'string' || name.trim().length === 0) {
      return { success: false, error: 'Preset name cannot be empty', preset: null };
    }

    const trimmedName = name.trim();
    const valResult = PresetManager.validateSequence(sequenceData);
    if (!valResult.valid) {
      return { success: false, error: valResult.errors.join('; '), preset: null };
    }

    const rawMap = this._getRawPresetsMap();
    const toSave = {
      ...valResult.sequence,
      sequenceName: valResult.sequence.sequenceName || trimmedName,
      isBuiltin: Boolean(DEFAULT_PRESETS[trimmedName])
    };

    rawMap[trimmedName] = toSave;
    try {
      this.storage.setItem(this.storageKey, JSON.stringify(rawMap));
      return { success: true, preset: toSave };
    } catch (err) {
      return { success: false, error: `Storage write failure: ${err.message}`, preset: null };
    }
  }

  /**
   * Deletes a preset by name
   * 
   * @param {string} name
   * @returns {boolean} True if deleted, false if not found
   */
  deletePreset(name) {
    if (!name) return false;
    const trimmed = name.trim();
    if (DEFAULT_PRESETS[trimmed]) {
      return false; // Cannot delete built-in presets
    }
    const rawMap = this._getRawPresetsMap();

    if (!(trimmed in rawMap)) {
      return false;
    }

    delete rawMap[trimmed];
    this.storage.setItem(this.storageKey, JSON.stringify(rawMap));
    return true;
  }

  /**
   * Duplicates an existing preset under a new name
   * 
   * @param {string} sourceName
   * @param {string} newName
   * @returns {{ success: boolean, preset?: Object, error?: string }}
   */
  duplicatePreset(sourceName, newName) {
    const src = this.getPreset(sourceName);
    if (!src) {
      return { success: false, error: `Source preset '${sourceName}' not found` };
    }
    if (!newName || typeof newName !== 'string' || newName.trim().length === 0) {
      return { success: false, error: 'New preset name cannot be empty' };
    }

    const cloned = JSON.parse(JSON.stringify(src));
    cloned.sequenceName = newName.trim();
    cloned.isBuiltin = false;
    // Generate fresh waypoint IDs for the clone
    for (const wp of cloned.waypoints) {
      wp.id = `wp_${Date.now()}_${Math.random().toString(36).substring(2, 8)}`;
    }

    return this.savePreset(newName.trim(), cloned);
  }

  /**
   * Resets presets storage back to factory defaults
   */
  resetToDefaults() {
    const defaults = JSON.parse(JSON.stringify(DEFAULT_PRESETS));
    this.storage.setItem(this.storageKey, JSON.stringify(defaults));
    return this.getAllPresets();
  }

  /**
   * Exports sequence data to a formatted JSON string
   * 
   * @param {Object} sequence
   * @param {boolean} [pretty=true]
   * @returns {string} Formatted JSON string
   */
  exportSequenceJSON(sequence, pretty = true) {
    const valResult = PresetManager.validateSequence(sequence);
    if (!valResult.valid) {
      throw new Error(`Export failed validation: ${valResult.errors.join('; ')}`);
    }

    const exportObj = {
      sequenceName: valResult.sequence.sequenceName,
      version: valResult.sequence.version,
      loop: valResult.sequence.loop,
      description: valResult.sequence.description || '',
      waypoints: valResult.sequence.waypoints.map(wp => ({
        id: wp.id,
        label: wp.label,
        j1: wp.j1,
        j2: wp.j2,
        j3: wp.j3,
        j4: wp.j4,
        j5: wp.j5,
        j6: wp.j6 ?? 0,
        dx: wp.dx,
        dy: wp.dy,
        dur: wp.dur,
        grip: wp.grip,
        moveType: wp.moveType || 'joint',
        comment: wp.comment
      }))
    };

    return pretty ? JSON.stringify(exportObj, null, 2) : JSON.stringify(exportObj);
  }

  /**
   * Imports and validates sequence data from a JSON string or parsed object
   * 
   * @param {string|Object} jsonStrOrObj
   * @returns {Object} Clean validated sequence object
   * @throws {Error} If JSON syntax is invalid or schema validation fails
   */
  importSequenceJSON(jsonStrOrObj) {
    let parsed;
    if (typeof jsonStrOrObj === 'string') {
      try {
        parsed = JSON.parse(jsonStrOrObj);
      } catch (err) {
        throw new Error(`Invalid JSON syntax: ${err.message}`);
      }
    } else if (typeof jsonStrOrObj === 'object' && jsonStrOrObj !== null) {
      parsed = jsonStrOrObj;
    } else {
      throw new Error('Import failed: Input must be a JSON string or object');
    }

    const valResult = PresetManager.validateSequence(parsed);
    if (!valResult.valid) {
      throw new Error(`Schema validation failed: ${valResult.errors.join('; ')}`);
    }

    return valResult.sequence;
  }

  /**
   * Imports robotic trajectory waypoints from standard G-Code text
   * 
   * @param {string} gcodeText - Raw G-Code (.gcode / .nc / .tap)
   * @param {Object} [options] - Parsing and fitting configuration
   * @returns {Object} Validated sequence object { sequenceName, waypoints, boundingBox, metrics, warnings }
   */
  importGCode(gcodeText, options = {}) {
    const result = GCodeInterpreter.parse(gcodeText, options);
    if (!result.waypoints || result.waypoints.length === 0) {
      throw new Error(`G-Code Import failed: ${result.warnings.join('; ') || 'No valid moves found'}`);
    }
    return result;
  }

  /**
   * Exports sequence data to standard CNC G-Code text
   * 
   * @param {Object|string} sequenceOrName - Sequence object or preset name
   * @param {Object} [options]
   * @returns {string} Standard G-Code text
   */
  exportSequenceGCode(sequenceOrName, options = {}) {
    let seq = sequenceOrName;
    if (typeof sequenceOrName === 'string') {
      seq = this.getPreset(sequenceOrName);
    }
    if (!seq) {
      throw new Error(`Export failed: Sequence "${sequenceOrName}" not found`);
    }
    return GCodeInterpreter.exportGCode(seq, options);
  }
}

// Global Singleton Instance
export const presetManager = new PresetManager();
export default presetManager;
