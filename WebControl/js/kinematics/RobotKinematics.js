/**
 * RobotKinematics.js - Authoritative Kinematics Math Engine for 6-DOF Robot Arm
 * 
 * Physical formulation matching URDF geometry (`stppe_zlozenie_nowe.urdf`):
 * - Physical dimensions:
 *   - BASE_H = 143.896 mm (Floor to Shoulder J2 rotation axis)
 *   - L1 = 185.845 mm (Shoulder J2 to Elbow J3 length)
 *   - L2 = 225.478 mm (Elbow J3 to Wrist J5 length)
 *   - L3 = 36.105 mm (Wrist J5 to J6 flange)
 *   - TOOL_TOTAL_LEN = 31.53 mm (Effective axial extension from J5 to TCP)
 *   - Rest angle offsets: TH1_0 = 25.0097°, TH2_0 = 140.4962°
 * - Forward Kinematics (FK) computing 3D Cartesian coordinates and intermediate joint pivots
 * - Analytical 2-Link Trigonometric Inverse Kinematics (IK) with zero-iteration closed-form solution
 * - Full 6-DOF support (J1 base yaw, J2 shoulder pitch, J3 elbow pitch, J4 forearm roll, J5 wrist pitch, J6 tool roll)
 * - Wrist Modes W0-W6 support (LEVEL, DOWN_0, DOWN_90, PITCH_0, PITCH_90, PITCH_180, MANUAL)
 * - Steps-to-degrees and degrees-to-steps conversion
 * - Coordinate transformations between Robot Frame (Z-up, X-fwd, Y-left) and Three.js Frame (Y-up, X-right, Z-viewer)
 */

export const BASE_H = 143.896;     // Floor to Shoulder J2 rotation axis (mm)
export const L1 = 185.845;         // Shoulder to Elbow length (mm)
export const L2 = 225.478;         // Elbow to Wrist J5 length (mm)
export const L2_EFF = 225.478;     // Effective forearm length (mm)
export const L2_PAR = 225.478;     // Backward compatibility alias
export const L2_PERP = 0.0;        // Backward compatibility alias
export const L3 = 36.105;          // Wrist link J5 to J6 flange (mm)
export const TOOL_LEN = 15.0;      // Tool flange to TCP focal point (mm)
export const TOOL_TOTAL_LEN = 31.53; // Effective radial offset from J5 to TCP (mm)
export const GRIPPER_LEN = 15.0;   // Flange to gripper focal point (mm)
export const L_GRIPPER = 15.0;     // Alias for Gripper length
export const FLOOR_CLR = 20.0;     // Minimum safe clearance above floor plane (mm)
export const EXCL_R = 80.0;        // Base exclusion cylinder radius (mm)
export const EXCL_R2 = 6400.0;     // Base exclusion radius squared (mm^2)

export const DEG_TO_RAD = Math.PI / 180.0;
export const RAD_TO_DEG = 180.0 / Math.PI;

// Rest angle offsets derived from CAD alignment
export const TH1_0 = 25.0097 * DEG_TO_RAD; // Upper arm rest angle from vertical Z-up (rad)
export const TH2_0 = 140.4962 * DEG_TO_RAD; // Forearm rest angle from vertical Z-up (rad)
export const REL_0 = TH2_0 - TH1_0;        // Relative rest angle between upper arm and forearm (115.4865 deg)
export const GAMMA_0 = Math.PI - REL_0;     // Interior angle at elbow triangle at rest (64.5135 deg)
export const GAMMA = GAMMA_0;              // Backward-compatible alias
export const GAMMA_RAD = GAMMA_0;
export const GAMMA_DEG = GAMMA_0 * RAD_TO_DEG;
export const TOOL_REST_ANGLE = 4.896 * DEG_TO_RAD; // Rest tool elevation from horizontal (rad)

export const STEPS_PER_DEG = Object.freeze({
  j1: 74.074,    // TMC2209 8.33:1 ((200 * 16 * 8.333333) / 360)
  j2: 177.778,   // TMC2209 20:1   ((200 * 16 * 20.0) / 360)
  j3: -177.778   // TMC2209 20:1 Inverted
});

export const JOINT_LIMITS = Object.freeze({
  j1: Object.freeze({ min: -90.0, max: 90.0, home: 0.0 }),
  j2: Object.freeze({ min: -65.0, max: 115.0, home: 0.0 }),
  j3: Object.freeze({ min: -35.0, max: 170.0, home: 0.0 }),
  j4: Object.freeze({ min: -90.0, max: 90.0, home: 0.0 }),
  j5: Object.freeze({ min: -90.0, max: 90.0, home: 0.0 }),
  j6: Object.freeze({ min: -180.0, max: 180.0, home: 0.0 }),
  grip: Object.freeze({ min: 0.0, max: 180.0, home: 90.0 })
});

export const KINEMATICS_CONSTANTS = Object.freeze({
  BASE_H,
  L1,
  L2,
  L2_PAR,
  L2_PERP,
  L2_EFF,
  GAMMA_RAD,
  GAMMA_DEG,
  L3,
  L_GRIPPER,
  GRIPPER_LEN,
  TOOL_LEN,
  TOOL_TOTAL_LEN,
  EXCL_R,
  EXCL_R2,
  FLOOR_CLR,
  LIMITS: Object.freeze({
    J1: JOINT_LIMITS.j1,
    J2: JOINT_LIMITS.j2,
    J3: JOINT_LIMITS.j3,
    J4: JOINT_LIMITS.j4,
    J5: JOINT_LIMITS.j5,
    J6: JOINT_LIMITS.j6,
    GRIP: JOINT_LIMITS.grip
  }),
  STEPS_PER_DEG,
  GEARING: Object.freeze({
    STEPS_PER_REV: 200,
    MICROSTEPS: 16,
    STEPS_PER_MOTOR_REV: 3200,
    GEAR_RATIO_J1: 8.333333,
    GEAR_RATIO_J2_J3: 20.0,
    STEPS_PER_DEG_J1: 74.074,
    STEPS_PER_DEG_J2_J3: 177.778
  })
});

export const WristMode = Object.freeze({
  LEVEL: 0,     // Tool horizontal (pitch = 0), J5 neutral
  DOWN_0: 1,    // Tool pointing straight down
  DOWN_90: 2,   // Tool pointing straight down, J6 rotated 90
  PITCH_0: 3,   // Pitch = 0 deg (horizontal), custom roll
  PITCH_90: 4,  // Pitch = 90 deg (downward), custom roll
  PITCH_180: 5, // Pitch = 180 deg (upward), custom roll
  MANUAL: 6     // J4, J5, J6 direct angle values
});

/**
 * Clamp a numeric value to [min, max]
 */
export function clamp(val, min, max) {
  return Math.max(min, Math.min(max, val));
}

/**
 * Convert joint angles to stepper steps
 */
export function degToSteps(j1, j2, j3) {
  return {
    s1: Math.round(j1 * STEPS_PER_DEG.j1),
    s2: Math.round(j2 * STEPS_PER_DEG.j2),
    s3: Math.round(j3 * STEPS_PER_DEG.j3)
  };
}

/**
 * Convert stepper steps to joint angles (deg)
 */
export function stepsToDeg(s1, s2, s3) {
  return {
    j1: s1 / STEPS_PER_DEG.j1,
    j2: s2 / STEPS_PER_DEG.j2,
    j3: s3 / STEPS_PER_DEG.j3
  };
}

/**
 * Coordinate Converter: Robot Frame -> Three.js World Space
 * 
 * Robot Frame (Standard):
 *   +X: Forward (azimuth 0 deg)
 *   +Y: Left (azimuth +90 deg)
 *   +Z: Up (elevation from floor Z=0)
 * 
 * Three.js World Frame:
 *   +X_three: Forward (X_robot)
 *   +Y_three: Up (Z_robot)
 *   +Z_three: Right / Toward viewer (-Y_robot)
 */
export function robotToThreeCoords(x, y, z) {
  return {
    x: Number(x),
    y: Number(z),
    z: -Number(y)
  };
}

/**
 * Coordinate Converter: Three.js World Space -> Robot Frame
 */
export function threeToRobotCoords(x_three, y_three, z_three) {
  return {
    x: Number(x_three),
    y: -Number(z_three),
    z: Number(y_three)
  };
}

/**
 * Forward Kinematics (FK)
 * 
 * Calculates exact Cartesian TCP coordinates and intermediate 3D pivot positions.
 * Supports both object options parameter and positional arguments for backwards compatibility.
 * 
 * @param {number} j1_deg - Joint 1 angle (deg)
 * @param {number} j2_deg - Joint 2 angle (deg)
 * @param {number} j3_deg - Joint 3 angle (deg)
 * @param {Object} [options] - Options or workOffset
 * @param {Object} [toolOffsetArg] - Optional toolOffset when positional arguments are used
 * @param {number} [j4Arg=0] - Optional j4 when positional arguments are used
 * @param {number} [j5Arg=0] - Optional j5 when positional arguments are used
 * @param {number} [j6Arg=0] - Optional j6 when positional arguments are used
 * @returns {Object} FK solution
 */
export function forwardKinematics(j1_deg, j2_deg, j3_deg, options = {}, toolOffsetArg = undefined, j4Arg = undefined, j5Arg = undefined, j6Arg = undefined) {
  let workOffset = { x: 0, y: 0, z: 0 };
  let toolOffset = { x: 0, y: 0, z: 0 };
  let j4_deg = 0.0;
  let j5_deg = 0.0;
  let j6_deg = 0.0;

  if (toolOffsetArg !== undefined || j4Arg !== undefined || j5Arg !== undefined || j6Arg !== undefined) {
    if (options && typeof options === 'object' && ('x' in options || 'y' in options || 'z' in options)) {
      workOffset = options;
    }
    if (toolOffsetArg && typeof toolOffsetArg === 'object') toolOffset = toolOffsetArg;
    if (typeof j4Arg === 'number') j4_deg = j4Arg;
    if (typeof j5Arg === 'number') j5_deg = j5Arg;
    if (typeof j6Arg === 'number') j6_deg = j6Arg;
  } else if (options && typeof options === 'object') {
    if ('x' in options || 'y' in options || 'z' in options) {
      workOffset = options;
    } else {
      if (options.workOffset) workOffset = options.workOffset;
      if (options.toolOffset) toolOffset = options.toolOffset;
      if (options.j4_deg !== undefined) j4_deg = options.j4_deg;
      else if (options.j4 !== undefined) j4_deg = options.j4;
      if (options.j5_deg !== undefined) j5_deg = options.j5_deg;
      else if (options.j5 !== undefined) j5_deg = options.j5;
      if (options.j6_deg !== undefined) j6_deg = options.j6_deg;
      else if (options.j6 !== undefined) j6_deg = options.j6;
    }
  }

  const j1r = Number(j1_deg) * DEG_TO_RAD;
  const j2r = Number(j2_deg) * DEG_TO_RAD;
  const j3r = Number(j3_deg) * DEG_TO_RAD;
  const j5r = Number(j5_deg) * DEG_TO_RAD;

  // Upper arm angle from vertical (Z-up)
  const th1 = TH1_0 - j2r;
  // Forearm angle from vertical (Z-up)
  const th2 = TH2_0 - (j2r + j3r);

  // Base pivot P0 at floor
  const p0 = { x: 0.0, y: 0.0, z: 0.0 };

  // Shoulder pivot P1 at J2 axis
  const p1 = { x: 0.0, y: 0.0, z: BASE_H };

  // Elbow J3 pivot
  const j3_reach = L1 * Math.sin(th1);
  const j3_z = BASE_H + L1 * Math.cos(th1);
  const p2 = {
    x: j3_reach * Math.cos(j1r),
    y: j3_reach * Math.sin(j1r),
    z: j3_z
  };

  // Wrist pitch J5 pivot
  const j5_reach = j3_reach + L2 * Math.sin(th2);
  const j5_z = j3_z + L2 * Math.cos(th2);
  const p3 = {
    x: j5_reach * Math.cos(j1r),
    y: j5_reach * Math.sin(j1r),
    z: j5_z
  };

  // Tool direction vector:
  // Angle of tool relative to forward horizontal (+X)
  const ang_tool = TOOL_REST_ANGLE + (j2r + j3r + j5r);

  // Wrist flange J6 pivot
  const j6_reach = j5_reach + 16.48 * Math.cos(ang_tool);
  const j6_z = j5_z + 16.48 * Math.sin(ang_tool);
  const p4 = {
    x: j6_reach * Math.cos(j1r),
    y: j6_reach * Math.sin(j1r),
    z: j6_z
  };

  // Tool Center Point (TCP / Gripper tip / crosshair focal point)
  const tcp_reach = j5_reach + TOOL_TOTAL_LEN * Math.cos(ang_tool);
  const tcp_z = j5_z + TOOL_TOTAL_LEN * Math.sin(ang_tool);
  const pGripper = {
    x: tcp_reach * Math.cos(j1r),
    y: tcp_reach * Math.sin(j1r),
    z: tcp_z
  };

  // TCP in Work Frame (matching convention: +workOffset -toolOffset)
  const rawTcp = {
    x: pGripper.x,
    y: pGripper.y,
    z: pGripper.z
  };

  const tcp = {
    x: pGripper.x + (workOffset.x || 0) - (toolOffset.x || 0),
    y: pGripper.y + (workOffset.y || 0) - (toolOffset.y || 0),
    z: pGripper.z + (workOffset.z || 0) - (toolOffset.z || 0)
  };

  const ang34_deg = (Number(j2_deg) + Number(j3_deg));

  return {
    tcp,
    rawTcp,
    x: tcp.x,
    y: tcp.y,
    z: tcp.z,
    p0,
    p1,
    p2,
    p3,
    p4,
    pGripper,
    ang34_deg,
    ang34Deg: ang34_deg,
    j3_reach,
    j3_z,
    j4_reach: j5_reach,
    j4_z: j5_z,
    j5_reach,
    j5_z,
    angles: {
      j1: Number(j1_deg),
      j2: Number(j2_deg),
      j3: Number(j3_deg),
      j4: Number(j4_deg),
      j5: Number(j5_deg),
      j6: Number(j6_deg)
    }
  };
}

/**
 * Analytical 2-Link Inverse Kinematics (IK) Solver
 * 
 * Closed-form analytical trigonometric solver matching the 6-DOF URDF arm dimensions.
 * Computes exact Joint 1 (yaw), Joint 2 (shoulder pitch), Joint 3 (elbow pitch),
 * Joint 4 (forearm roll), Joint 5 (wrist pitch), and Joint 6 (tool roll).
 * 
 * @param {number} x_target - Target X in Work Frame (mm)
 * @param {number} y_target - Target Y in Work Frame (mm)
 * @param {number} z_target - Target Z in Work Frame (mm)
 * @param {Object} [options] - IK configuration
 * @returns {Object} IK result
 */
export function solveIK(x_target, y_target, z_target, options = {}) {
  let opts = {};
  if (typeof options === 'number') {
    opts = { wristMode: options };
  } else if (options && typeof options === 'object') {
    opts = options;
  }

  const {
    wristMode = WristMode.LEVEL,
    workOffset = { x: 0, y: 0, z: 0 },
    toolOffset = { x: 0, y: 0, z: 0 },
    j4Param = 0.0,
    j4Manual = 0.0,
    j5Param = 0.0,
    j6Param = 0.0,
    currentAngles = null
  } = opts;

  const warnings = [];
  let clamped = false;

  // 1. Invert Work & Tool offsets to get raw robot base coordinates
  let x = x_target - (workOffset.x || 0) + (toolOffset.x || 0);
  let y = y_target - (workOffset.y || 0) + (toolOffset.y || 0);
  let z = z_target - (workOffset.z || 0) + (toolOffset.z || 0);

  // 2. Floor clearance safety clamping
  if (z < FLOOR_CLR) {
    z = FLOOR_CLR;
    clamped = true;
    warnings.push('TARGET_CLAMPED_FLOOR_CLEARANCE');
  }

  // 3. Base exclusion cylinder safety clamping
  const r_xy = Math.hypot(x, y);
  if (z < BASE_H && r_xy < EXCL_R) {
    clamped = true;
    warnings.push('TARGET_CLAMPED_BASE_EXCLUSION');
    if (r_xy < 1e-4) {
      x = EXCL_R;
      y = 0.0;
    } else {
      const scale = EXCL_R / r_xy;
      x *= scale;
      y *= scale;
    }
  }

  // 4. Azimuth Joint 1 rotation in XY plane
  let j1 = Math.atan2(y, x) * RAD_TO_DEG;

  // Joint 1 limit check & clamp
  if (j1 < JOINT_LIMITS.j1.min || j1 > JOINT_LIMITS.j1.max) {
    warnings.push('J1_LIMIT_EXCEEDED');
    j1 = clamp(j1, JOINT_LIMITS.j1.min, JOINT_LIMITS.j1.max);
    clamped = true;
  }

  // 5. Decouple Wrist Center (J5) from Target TCP based on wrist orientation
  const r_tcp = Math.hypot(x, y);
  let r5 = r_tcp;
  let z5 = z;

  const modeNum = Number(wristMode);
  let pitch_deg = 0.0;
  if (modeNum === WristMode.DOWN_0 || modeNum === WristMode.DOWN_90 || modeNum === WristMode.PITCH_90) {
    // Tool pointing straight down: J5 is directly above TCP by TOOL_TOTAL_LEN
    z5 = z + TOOL_TOTAL_LEN;
  } else if (modeNum === WristMode.PITCH_180) {
    // Tool pointing straight up: J5 is directly below TCP by TOOL_TOTAL_LEN
    z5 = z - TOOL_TOTAL_LEN;
  } else if (modeNum === WristMode.MANUAL) {
    if (opts.pitch !== undefined) {
      pitch_deg = Number(opts.pitch);
    } else if (opts.j5Param !== undefined && opts.j4Param === undefined) {
      pitch_deg = Number(opts.j5Param);
    } else if (opts.j4Param !== undefined && (opts.j5Param === undefined || opts.j5Param === 90.0 || opts.j5Param === 0.0)) {
      pitch_deg = Number(opts.j4Param);
    } else if (opts.j5Param !== undefined) {
      pitch_deg = Number(opts.j5Param);
    }
    const pitch_rad = pitch_deg * DEG_TO_RAD;
    r5 = Math.max(0, r_tcp - TOOL_TOTAL_LEN * Math.cos(pitch_rad));
    z5 = z - TOOL_TOTAL_LEN * Math.sin(pitch_rad);
  } else {
    // LEVEL (default, horizontal tool): J5 is behind TCP horizontally by TOOL_TOTAL_LEN
    r5 = Math.max(0, r_tcp - TOOL_TOTAL_LEN);
  }

  // 6. 2-Link Planar Trigonometry for Shoulder J2 and Elbow J3
  const dr = r5;
  const dz = z5 - BASE_H;
  const D2 = dr * dr + dz * dz;
  const D = Math.sqrt(D2);

  // Law of Cosines for Elbow
  let C_gamma = (L1 * L1 + L2 * L2 - D2) / (2.0 * L1 * L2);
  let success = true;

  if (C_gamma < -1.0) {
    C_gamma = -1.0;
    success = false;
    warnings.push('OUT_OF_REACH_MAXIMUM_EXTENSION');
  } else if (C_gamma > 1.0) {
    C_gamma = 1.0;
    success = false;
    warnings.push('OUT_OF_REACH_MINIMUM_COLLAPSE');
  }

  const gamma = Math.acos(C_gamma); // radians
  let j3 = (gamma - GAMMA_0) * RAD_TO_DEG;

  const beta = Math.atan2(dr, dz);
  let C_alpha = (L1 * L1 + D2 - L2 * L2) / (2.0 * L1 * Math.max(1e-6, D));
  C_alpha = Math.max(-1.0, Math.min(1.0, C_alpha));
  const alpha = Math.acos(C_alpha);

  const th1 = beta - alpha;
  let j2 = (TH1_0 - th1) * RAD_TO_DEG;

  // If currentAngles provided, evaluate alternative branch if valid and closer
  if (currentAngles && currentAngles.j2 !== undefined && currentAngles.j3 !== undefined) {
    const j3_alt = (-gamma - GAMMA_0) * RAD_TO_DEG;
    const th1_alt = beta + alpha;
    const j2_alt = (TH1_0 - th1_alt) * RAD_TO_DEG;

    const j2_in_bounds = j2 >= JOINT_LIMITS.j2.min && j2 <= JOINT_LIMITS.j2.max;
    const j3_in_bounds = j3 >= JOINT_LIMITS.j3.min && j3 <= JOINT_LIMITS.j3.max;
    const alt_j2_in_bounds = j2_alt >= JOINT_LIMITS.j2.min && j2_alt <= JOINT_LIMITS.j2.max;
    const alt_j3_in_bounds = j3_alt >= JOINT_LIMITS.j3.min && j3_alt <= JOINT_LIMITS.j3.max;

    if (alt_j2_in_bounds && alt_j3_in_bounds) {
      const d_primary = Math.abs(j2 - currentAngles.j2) + Math.abs(j3 - currentAngles.j3);
      const d_alt = Math.abs(j2_alt - currentAngles.j2) + Math.abs(j3_alt - currentAngles.j3);
      if ((!j2_in_bounds || !j3_in_bounds) || d_alt < d_primary) {
        j2 = j2_alt;
        j3 = j3_alt;
      }
    }
  }

  // Joint 2 & 3 soft limit verification
  if (j2 < JOINT_LIMITS.j2.min || j2 > JOINT_LIMITS.j2.max) {
    warnings.push('J2_LIMIT_EXCEEDED');
    j2 = clamp(j2, JOINT_LIMITS.j2.min, JOINT_LIMITS.j2.max);
    clamped = true;
  }
  if (j3 < JOINT_LIMITS.j3.min || j3 > JOINT_LIMITS.j3.max) {
    warnings.push('J3_LIMIT_EXCEEDED');
    j3 = clamp(j3, JOINT_LIMITS.j3.min, JOINT_LIMITS.j3.max);
    clamped = true;
  }

  // 7. Wrist Modes & Orientations (J4 Forearm Roll, J5 Wrist Pitch, J6 Tool Roll)
  const ang34 = j2 + j3;
  let j4 = opts.j4Param !== undefined ? Number(opts.j4Param) : (opts.j4Manual !== undefined ? Number(opts.j4Manual) : 0.0);
  let j5 = 0.0;
  let j6 = opts.j6Param !== undefined ? Number(opts.j6Param) : 0.0;

  const basePitchOffsetDeg = -(TOOL_REST_ANGLE * RAD_TO_DEG); // ~ -4.896 deg

  switch (modeNum) {
    case WristMode.LEVEL: // W0: LEVEL (Horizontal)
      j5 = basePitchOffsetDeg - ang34;
      break;

    case WristMode.DOWN_0: // W1: DOWN_0 (Pointing straight down)
      j5 = basePitchOffsetDeg - ang34 - 90.0;
      break;

    case WristMode.DOWN_90: // W2: DOWN_90 (Pointing straight down, roll 90)
      j5 = basePitchOffsetDeg - ang34 - 90.0;
      j6 = 90.0;
      break;

    case WristMode.PITCH_0: // W3: PITCH_0 (Horizontal, custom roll)
      j5 = basePitchOffsetDeg - ang34;
      j6 = Number(opts.j5Param || 0);
      break;

    case WristMode.PITCH_90: // W4: PITCH_90 (Down, custom roll)
      j5 = basePitchOffsetDeg - ang34 - 90.0;
      j6 = Number(opts.j5Param || 0);
      break;

    case WristMode.PITCH_180: // W5: PITCH_180 (Upward 180, custom roll)
      j5 = basePitchOffsetDeg - ang34 + 90.0;
      j6 = Number(opts.j5Param || 0);
      break;

    case WristMode.MANUAL: // W6: MANUAL
      j4 = Number(opts.j4Param !== undefined ? opts.j4Param : (opts.j4Manual !== undefined ? opts.j4Manual : 0.0));
      j5 = pitch_deg - (TOOL_REST_ANGLE * RAD_TO_DEG) - ang34;
      j6 = Number(opts.j6Param || 0.0);
      break;

    default:
      j5 = basePitchOffsetDeg - ang34;
  }

  // Soft limit clamping for J4, J5, J6
  if (j4 < JOINT_LIMITS.j4.min || j4 > JOINT_LIMITS.j4.max) {
    warnings.push('J4_LIMIT_EXCEEDED');
    j4 = clamp(j4, JOINT_LIMITS.j4.min, JOINT_LIMITS.j4.max);
    clamped = true;
  }
  if (j5 < JOINT_LIMITS.j5.min || j5 > JOINT_LIMITS.j5.max) {
    warnings.push('J5_LIMIT_EXCEEDED');
    j5 = clamp(j5, JOINT_LIMITS.j5.min, JOINT_LIMITS.j5.max);
    clamped = true;
  }
  if (j6 < JOINT_LIMITS.j6.min || j6 > JOINT_LIMITS.j6.max) {
    warnings.push('J6_LIMIT_EXCEEDED');
    j6 = clamp(j6, JOINT_LIMITS.j6.min, JOINT_LIMITS.j6.max);
    clamped = true;
  }

  const isActuallySuccess = success && !clamped;

  return {
    success: isActuallySuccess,
    reachable: isActuallySuccess,
    clamped,
    j1,
    j2,
    j3,
    j4,
    j5,
    j6,
    ang34,
    warnings,
    angles: {
      j1,
      j2,
      j3,
      j4,
      j5,
      j6
    }
  };
}

/**
 * Converts Roll, Pitch, Yaw angles (degrees) to 3x3 rotation matrix
 * R = Rz(yaw) * Ry(pitch) * Rx(roll)
 */
export function rpyToRotMat(rollDeg, pitchDeg, yawDeg) {
  const r = (Number(rollDeg) || 0) * DEG_TO_RAD;
  const p = (Number(pitchDeg) || 0) * DEG_TO_RAD;
  const y = (Number(yawDeg) || 0) * DEG_TO_RAD;

  const cr = Math.cos(r), sr = Math.sin(r);
  const cp = Math.cos(p), sp = Math.sin(p);
  const cy = Math.cos(y), sy = Math.sin(y);

  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp,     cp * sr,                cp * cr]
  ];
}

/**
 * Converts 3x3 rotation matrix to Roll, Pitch, Yaw angles (degrees)
 */
export function rotMatToRpy(R) {
  const pitch = Math.atan2(-R[2][0], Math.hypot(R[0][0], R[1][0])) * RAD_TO_DEG;
  const yaw = Math.atan2(R[1][0], R[0][0]) * RAD_TO_DEG;
  const roll = Math.atan2(R[2][1], R[2][2]) * RAD_TO_DEG;
  return { roll, pitch, yaw };
}

/**
 * Full 6-DOF Forward Kinematics
 * Returns TCP position, orientation matrix R06, and tool direction unit vector
 */
export function forwardKinematics6DOF(j1_deg, j2_deg, j3_deg, j4_deg = 0, j5_deg = 0, j6_deg = 0, options = {}) {
  const { workOffset = { x: 0, y: 0, z: 0 }, toolOffset = { x: 0, y: 0, z: 0 }, raw = false } = options;

  const j1r = (Number(j1_deg) || 0) * DEG_TO_RAD;
  const j2r = (Number(j2_deg) || 0) * DEG_TO_RAD;
  const j3r = (Number(j3_deg) || 0) * DEG_TO_RAD;
  const j4r = (Number(j4_deg) || 0) * DEG_TO_RAD;
  const j5r = (Number(j5_deg) || 0) * DEG_TO_RAD;
  const j6r = (Number(j6_deg) || 0) * DEG_TO_RAD;

  const th1 = TH1_0 - j2r;
  const th2 = TH2_0 - (j2r + j3r);

  const j3_reach = L1 * Math.sin(th1);
  const j3_z = BASE_H + L1 * Math.cos(th1);

  const j5_reach = j3_reach + L2 * Math.sin(th2);
  const j5_z = j3_z + L2 * Math.cos(th2);

  const p5 = {
    x: j5_reach * Math.cos(j1r),
    y: j5_reach * Math.sin(j1r),
    z: j5_z
  };

  const forearm_pitch = Math.PI / 2.0 - th2;
  const c1 = Math.cos(j1r), s1 = Math.sin(j1r);
  const cp = Math.cos(forearm_pitch), sp = Math.sin(forearm_pitch);

  const R03 = [
    [c1 * cp, -s1, c1 * sp],
    [s1 * cp,  c1, s1 * sp],
    [-sp,       0, cp]
  ];

  const c4 = Math.cos(j4r), s4 = Math.sin(j4r);
  const c5 = Math.cos(j5r), s5 = Math.sin(j5r);
  const c6 = Math.cos(j6r), s6 = Math.sin(j6r);

  const Rx4 = [
    [1,  0,   0],
    [0, c4, -s4],
    [0, s4,  c4]
  ];
  const Ry5 = [
    [ c5, 0, s5],
    [  0, 1,  0],
    [-s5, 0, c5]
  ];
  const Rx6 = [
    [1,  0,   0],
    [0, c6, -s6],
    [0, s6,  c6]
  ];

  const matMul = (A, B) => {
    const C = [[0,0,0],[0,0,0],[0,0,0]];
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        C[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j];
      }
    }
    return C;
  };

  const R35 = matMul(Rx4, Ry5);
  const R36 = matMul(R35, Rx6);
  const R06 = matMul(R03, R36);

  const u_tool = {
    x: R06[0][0],
    y: R06[1][0],
    z: R06[2][0]
  };

  const rawTcp = {
    x: p5.x + TOOL_TOTAL_LEN * u_tool.x,
    y: p5.y + TOOL_TOTAL_LEN * u_tool.y,
    z: p5.z + TOOL_TOTAL_LEN * u_tool.z
  };

  const tcp = raw ? rawTcp : {
    x: rawTcp.x + (workOffset.x || 0) - (toolOffset.x || 0),
    y: rawTcp.y + (workOffset.y || 0) - (toolOffset.y || 0),
    z: rawTcp.z + (workOffset.z || 0) - (toolOffset.z || 0)
  };

  const rpy = rotMatToRpy(R06);

  return {
    tcp,
    rawTcp,
    x: tcp.x,
    y: tcp.y,
    z: tcp.z,
    p5,
    u_tool,
    R06,
    roll: rpy.roll,
    pitch: rpy.pitch,
    yaw: rpy.yaw,
    angles: {
      j1: Number(j1_deg),
      j2: Number(j2_deg),
      j3: Number(j3_deg),
      j4: Number(j4_deg),
      j5: Number(j5_deg),
      j6: Number(j6_deg)
    }
  };
}

/**
 * Full 6-DOF Analytical Inverse Kinematics with Singularity Dampening
 * Computes exact Joint 1..6 angles for Cartesian TCP (x, y, z) and orientation (roll, pitch, yaw)
 */
export function solveIK6DOF(x_target, y_target, z_target, rollDeg = 0, pitchDeg = 0, yawDeg = 0, options = {}) {
  const {
    workOffset = { x: 0, y: 0, z: 0 },
    toolOffset = { x: 0, y: 0, z: 0 },
    currentAngles = { j1: 0, j2: 0, j3: 0, j4: 0, j5: 0, j6: 0 }
  } = options;

  const warnings = [];
  let clamped = false;

  let x = x_target - (workOffset.x || 0) + (toolOffset.x || 0);
  let y = y_target - (workOffset.y || 0) + (toolOffset.y || 0);
  let z = z_target - (workOffset.z || 0) + (toolOffset.z || 0);

  const R06 = rpyToRotMat(rollDeg, pitchDeg, yawDeg);
  const ux = R06[0][0];
  const uy = R06[1][0];
  const uz = R06[2][0];

  // 1. Decouple Wrist Center P5
  const x5 = x - TOOL_TOTAL_LEN * ux;
  const y5 = y - TOOL_TOTAL_LEN * uy;
  const z5 = z - TOOL_TOTAL_LEN * uz;

  // 2. Base J1 (Shoulder singularity damping when r5 ~ 0)
  const r5 = Math.hypot(x5, y5);
  let j1 = (r5 < 0.5) ? (currentAngles.j1 || 0) : Math.atan2(y5, x5) * RAD_TO_DEG;

  let zc = z5;
  if (zc < FLOOR_CLR) {
    zc = FLOOR_CLR;
    clamped = true;
    warnings.push('TARGET_CLAMPED_FLOOR_CLEARANCE');
  }

  let rc = r5;
  if (rc * rc < EXCL_R2 && zc < BASE_H) {
    rc = Math.sqrt(EXCL_R2);
    clamped = true;
    warnings.push('TARGET_CLAMPED_BASE_EXCLUSION');
  }

  // 3. Planar 2-Link IK for J2, J3
  const dr = rc;
  const dz = zc - BASE_H;
  let D2 = dr * dr + dz * dz;
  let D = Math.sqrt(D2);

  const maxReach = L1 + L2;
  const isNearBoundary = (D / maxReach) > 0.98;
  if (D > maxReach * 0.999) {
    D = maxReach * 0.999;
    D2 = D * D;
    clamped = true;
    warnings.push('OUT_OF_REACH_MAXIMUM_EXTENSION');
  }

  let C_gamma = (L1 * L1 + L2 * L2 - D2) / (2 * L1 * L2);
  C_gamma = clamp(C_gamma, -1.0, 1.0);
  const gamma = Math.acos(C_gamma);

  const beta = Math.atan2(dr, dz);
  let C_alpha = (L1 * L1 + D2 - L2 * L2) / (2 * L1 * Math.max(1e-6, D));
  C_alpha = clamp(C_alpha, -1.0, 1.0);
  const alpha = Math.acos(C_alpha);

  const cand1_j2 = (TH1_0 - (beta - alpha)) * RAD_TO_DEG;
  const cand1_j3 = (gamma - GAMMA_0) * RAD_TO_DEG;
  const cand2_j2 = (TH1_0 - (beta + alpha)) * RAD_TO_DEG;
  const cand2_j3 = (-gamma - GAMMA_0) * RAD_TO_DEG;

  if (j1 < JOINT_LIMITS.j1.min || j1 > JOINT_LIMITS.j1.max) {
    warnings.push('J1_LIMIT_EXCEEDED');
    j1 = clamp(j1, JOINT_LIMITS.j1.min, JOINT_LIMITS.j1.max);
    clamped = true;
  }

  const prevJ2 = currentAngles.j2 || 0;
  const prevJ3 = currentAngles.j3 || 0;

  let sol1_clamped = clamped;
  const c_j2_1 = clamp(cand1_j2, JOINT_LIMITS.j2.min, JOINT_LIMITS.j2.max);
  const c_j3_1 = clamp(cand1_j3, JOINT_LIMITS.j3.min, JOINT_LIMITS.j3.max);
  if (c_j2_1 !== cand1_j2 || c_j3_1 !== cand1_j3) sol1_clamped = true;

  let sol2_clamped = clamped;
  const c_j2_2 = clamp(cand2_j2, JOINT_LIMITS.j2.min, JOINT_LIMITS.j2.max);
  const c_j3_2 = clamp(cand2_j3, JOINT_LIMITS.j3.min, JOINT_LIMITS.j3.max);
  if (c_j2_2 !== cand2_j2 || c_j3_2 !== cand2_j3) sol2_clamped = true;

  let j2 = c_j2_1, j3 = c_j3_1;
  clamped = sol1_clamped;
  if (Math.abs(c_j2_2 - prevJ2) + Math.abs(c_j3_2 - prevJ3) < Math.abs(c_j2_1 - prevJ2) + Math.abs(c_j3_1 - prevJ3)) {
    j2 = c_j2_2;
    j3 = c_j3_2;
    clamped = sol2_clamped;
  }

  // 4. Forearm Frame R03
  const j1r = j1 * DEG_TO_RAD;
  const j2r = j2 * DEG_TO_RAD;
  const j3r = j3 * DEG_TO_RAD;
  const th2 = TH2_0 - (j2r + j3r);
  const forearm_pitch = Math.PI / 2.0 - th2;

  const c1 = Math.cos(j1r), s1 = Math.sin(j1r);
  const cp = Math.cos(forearm_pitch), sp = Math.sin(forearm_pitch);

  const R03 = [
    [c1 * cp, -s1, c1 * sp],
    [s1 * cp,  c1, s1 * sp],
    [-sp,       0, cp]
  ];

  // 5. Wrist Rotation: R36 = R03^T * R06
  const R36 = [
    [
      R03[0][0] * R06[0][0] + R03[1][0] * R06[1][0] + R03[2][0] * R06[2][0],
      R03[0][0] * R06[0][1] + R03[1][0] * R06[1][1] + R03[2][0] * R06[2][1],
      R03[0][0] * R06[0][2] + R03[1][0] * R06[1][2] + R03[2][0] * R06[2][2]
    ],
    [
      R03[0][1] * R06[0][0] + R03[1][1] * R06[1][0] + R03[2][1] * R06[2][0],
      R03[0][1] * R06[0][1] + R03[1][1] * R06[1][1] + R03[2][1] * R06[2][1],
      R03[0][1] * R06[0][2] + R03[1][1] * R06[1][2] + R03[2][1] * R06[2][2]
    ],
    [
      R03[0][2] * R06[0][0] + R03[1][2] * R06[1][0] + R03[2][2] * R06[2][0],
      R03[0][2] * R06[0][1] + R03[1][2] * R06[1][1] + R03[2][2] * R06[2][1],
      R03[0][2] * R06[0][2] + R03[1][2] * R06[1][2] + R03[2][2] * R06[2][2]
    ]
  ];

  const cosJ5 = clamp(R36[0][0], -1.0, 1.0);
  const sinJ5_sq = R36[1][0] * R36[1][0] + R36[2][0] * R36[2][0];
  const sinJ5 = Math.sqrt(sinJ5_sq);

  let j4, j5, j6;
  const WRIST_SINGULARITY_EPS = 0.0436; // ~2.5 deg
  const wristSingular = (sinJ5 < WRIST_SINGULARITY_EPS);

  if (wristSingular) {
    j5 = Math.acos(cosJ5) * RAD_TO_DEG;
    j4 = currentAngles.j4 || 0;
    const totalRoll = Math.atan2(R36[2][1] - R36[1][2], R36[1][1] + R36[2][2]) * RAD_TO_DEG;
    j6 = totalRoll - j4;
    while (j6 > 180.0) j6 -= 360.0;
    while (j6 < -180.0) j6 += 360.0;
  } else {
    const j5_1 = Math.atan2(sinJ5, cosJ5) * RAD_TO_DEG;
    const j4_1 = Math.atan2(R36[1][0], -R36[2][0]) * RAD_TO_DEG;
    const j6_1 = Math.atan2(R36[0][1], R36[0][2]) * RAD_TO_DEG;

    const j5_2 = Math.atan2(-sinJ5, cosJ5) * RAD_TO_DEG;
    const j4_2 = Math.atan2(-R36[1][0], R36[2][0]) * RAD_TO_DEG;
    const j6_2 = Math.atan2(-R36[0][1], -R36[0][2]) * RAD_TO_DEG;

    const inLimit = (val, lim) => val >= lim.min - 1e-3 && val <= lim.max + 1e-3;
    const b1_valid = inLimit(j4_1, JOINT_LIMITS.j4) && inLimit(j5_1, JOINT_LIMITS.j5) && inLimit(j6_1, JOINT_LIMITS.j6);
    const b2_valid = inLimit(j4_2, JOINT_LIMITS.j4) && inLimit(j5_2, JOINT_LIMITS.j5) && inLimit(j6_2, JOINT_LIMITS.j6);

    const prevJ4 = currentAngles.j4 || 0;
    const prevJ5 = currentAngles.j5 || 0;
    const prevJ6 = currentAngles.j6 || 0;

    if (b1_valid && !b2_valid) {
      j4 = j4_1; j5 = j5_1; j6 = j6_1;
    } else if (b2_valid && !b1_valid) {
      j4 = j4_2; j5 = j5_2; j6 = j6_2;
    } else {
      const dist1 = Math.abs(j4_1 - prevJ4) + Math.abs(j5_1 - prevJ5) + Math.abs(j6_1 - prevJ6);
      const dist2 = Math.abs(j4_2 - prevJ4) + Math.abs(j5_2 - prevJ5) + Math.abs(j6_2 - prevJ6);
      if (dist2 < dist1) {
        j4 = j4_2; j5 = j5_2; j6 = j6_2;
      } else {
        j4 = j4_1; j5 = j5_1; j6 = j6_1;
      }
    }
  }

  if (j4 < JOINT_LIMITS.j4.min || j4 > JOINT_LIMITS.j4.max) {
    warnings.push('J4_LIMIT_EXCEEDED');
    j4 = clamp(j4, JOINT_LIMITS.j4.min, JOINT_LIMITS.j4.max);
    clamped = true;
  }
  if (j5 < JOINT_LIMITS.j5.min || j5 > JOINT_LIMITS.j5.max) {
    warnings.push('J5_LIMIT_EXCEEDED');
    j5 = clamp(j5, JOINT_LIMITS.j5.min, JOINT_LIMITS.j5.max);
    clamped = true;
  }
  if (j6 < JOINT_LIMITS.j6.min || j6 > JOINT_LIMITS.j6.max) {
    warnings.push('J6_LIMIT_EXCEEDED');
    j6 = clamp(j6, JOINT_LIMITS.j6.min, JOINT_LIMITS.j6.max);
    clamped = true;
  }

  const success = !clamped;

  return {
    success,
    reachable: success,
    clamped,
    j1,
    j2,
    j3,
    j4,
    j5,
    j6,
    wristSingular,
    isNearBoundary,
    warnings,
    angles: { j1, j2, j3, j4, j5, j6 }
  };
}
