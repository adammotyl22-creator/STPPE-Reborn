/**
 * WorkspaceValidator.js - Safety Boundary and Workspace Validation Engine
 * 
 * Validates workspace safety constraints against URDF arm limits:
 * - Joint soft limits (J1: [-90, 90], J2: [-65, 115], J3: [-35, 170], J4: [-90, 90], J5: [-120, 120], J6: [-180, 180])
 * - Floor clearance safety plane (FLOOR_CLR = 20mm for TCP and Elbow)
 * - Base exclusion cylinder (r < 100mm when Z < BASE_H 143.896mm)
 * - Anti-collision heuristics
 */

import {
  BASE_H,
  L1,
  FLOOR_CLR,
  EXCL_R,
  EXCL_R2,
  JOINT_LIMITS,
  forwardKinematics,
  forwardKinematics6DOF,
  DEG_TO_RAD
} from './RobotKinematics.js';

/**
 * Validates joint angles against physical soft limits
 * @returns {Array<string>} List of limit violation codes
 */
export function validateJointLimits(j1, j2, j3, j4 = 0.0, j5 = 0.0, j6 = 0.0) {
  const violations = [];
  if (j1 < JOINT_LIMITS.j1.min || j1 > JOINT_LIMITS.j1.max) violations.push('J1_LIMIT');
  if (j2 < JOINT_LIMITS.j2.min || j2 > JOINT_LIMITS.j2.max) violations.push('J2_LIMIT');
  if (j3 < JOINT_LIMITS.j3.min || j3 > JOINT_LIMITS.j3.max) violations.push('J3_LIMIT');
  if (j4 !== undefined && (j4 < JOINT_LIMITS.j4.min || j4 > JOINT_LIMITS.j4.max)) violations.push('J4_LIMIT');
  if (j5 !== undefined && (j5 < JOINT_LIMITS.j5.min || j5 > JOINT_LIMITS.j5.max)) violations.push('J5_LIMIT');
  if (j6 !== undefined && (j6 < JOINT_LIMITS.j6.min || j6 > JOINT_LIMITS.j6.max)) violations.push('J6_LIMIT');
  return violations;
}

/**
 * Validates floor clearance for TCP and Elbow
 * @param {number} tcpZ - TCP elevation (mm)
 * @param {number} [elbowZ=null] - Elbow elevation (mm)
 * @param {number} [gripperZ=null] - Gripper tip elevation (mm)
 * @returns {Array<string>} List of clearance violation codes
 */
export function validateFloorClearance(tcpZ, elbowZ = null, gripperZ = null) {
  const violations = [];
  if (tcpZ < FLOOR_CLR) violations.push('FLOOR_BREACH');
  if (gripperZ !== null && (gripperZ < FLOOR_CLR || gripperZ < 0.0)) {
    if (!violations.includes('FLOOR_BREACH')) violations.push('FLOOR_BREACH');
  }
  if (elbowZ !== null && elbowZ < FLOOR_CLR) violations.push('ELBOW_FLOOR_BREACH');
  return violations;
}

/**
 * Validates cylindrical base exclusion zone (r < 100mm when z < BASE_H)
 * @param {number} x - X coordinate (mm)
 * @param {number} y - Y coordinate (mm)
 * @param {number} z - Z coordinate (mm)
 * @returns {boolean} True if within forbidden exclusion zone
 */
export function validateBaseExclusion(x, y, z) {
  const r2 = x * x + y * y;
  return z < BASE_H && r2 < EXCL_R2;
}

/**
 * Validates arm self-collision with base column
 * @param {number} j2 - Joint 2 angle (deg)
 * @param {number} j3 - Joint 3 angle (deg)
 * @returns {boolean} True if self-collision detected
 */
export function validateSelfCollision(j2, j3) {
  return j2 > 90.0 && j3 < -30.0;
}

/**
 * Master workspace validation check matching safety rules
 * 
 * @param {number} j1 - Joint 1 angle (deg)
 * @param {number} j2 - Joint 2 angle (deg)
 * @param {number} j3 - Joint 3 angle (deg)
 * @param {number} [j4=0] - Joint 4 roll angle (deg)
 * @param {number} [j5=0] - Joint 5 pitch angle (deg)
 * @param {number} [j6=0] - Joint 6 tool roll angle (deg)
 * @returns {Object} { safe: boolean, valid: boolean, violations: string[], reason?: string, tcp: {x,y,z} }
 */
export function checkWorkspace(j1, j2, j3, j4 = 0.0, j5 = 0.0, j6 = 0.0, options = {}) {
  const violations = validateJointLimits(j1, j2, j3, j4, j5, j6);

  // Compute forward kinematics to check geometry
  const armFk = forwardKinematics(j1, j2, j3, {}, null, j4, j5, j6);
  const flangeFk = forwardKinematics6DOF(j1, j2, j3, j4, j5, j6, options);
  const fk = { ...armFk, ...flangeFk, p2: armFk.p2, pGripper: flangeFk.rawTcp };

  // Floor clearance check (TCP)
  if (fk.tcp.z < FLOOR_CLR) {
    violations.push('FLOOR_BREACH');
  }

  // Gripper tip floor clearance check (pGripper.z)
  if (fk.pGripper && (fk.pGripper.z < FLOOR_CLR || fk.pGripper.z < 0.0)) {
    if (!violations.includes('FLOOR_BREACH')) {
      violations.push('FLOOR_BREACH');
    }
  }

  // Elbow floor clearance check (P2.z)
  if (fk.p2.z < FLOOR_CLR) {
    violations.push('ELBOW_FLOOR_BREACH');
  }

  // Base exclusion cylinder check (r < 100mm and z < BASE_H)
  const r_xy = Math.hypot(fk.tcp.x, fk.tcp.y);
  if (fk.tcp.z < BASE_H && r_xy < EXCL_R) {
    violations.push('BASE_EXCLUSION_BREACH');
  }

  // Self-collision heuristic check
  if (validateSelfCollision(j2, j3)) {
    violations.push('SELF_COLLISION');
  }

  const safe = violations.length === 0;
  const reason = safe ? undefined : violations.join(', ');

  return {
    safe,
    valid: safe,
    violations,
    reason,
    tcp: fk.tcp,
    fk
  };
}
