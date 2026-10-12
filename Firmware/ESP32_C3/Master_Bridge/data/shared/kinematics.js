/**
 * RobotKinematics — shared FK/IK/workspace model (matches Teensy IK_Control v3.1+ and stppe_zlozenie_nowe URDF)
 * Standalone-capable: used by ESP32 web UI; robot runs full IK on Teensy.
 */
const RobotKinematics = (() => {
  const DEG = 180 / Math.PI;
  const RAD = Math.PI / 180;

  const BASE_H = 143.896;     // Floor to Shoulder J2 rotation axis (mm)
  const L1 = 185.845;         // Shoulder J2 to Elbow J3 length (mm)
  const L2 = 225.478;         // Elbow J3 to Wrist J5 length (mm)
  const L3 = 36.105;          // Wrist J5 to J6 flange (mm)
  const TOOL_TOTAL_LEN = 31.530; // Effective axial extension from J5 to TCP (mm)
  const FLOOR_CLR = 20.0;     // Safe floor clearance (mm)
  const EXCL_R2 = 6400.0;     // Base exclusion radius squared (80 mm)^2

  const TH1_0 = 25.0097 * RAD;
  const TH2_0 = 140.4962 * RAD;
  const REL_0 = TH2_0 - TH1_0;
  const GAMMA_0 = Math.PI - REL_0;
  const TOOL_REST_ANGLE = 4.896 * RAD;

  const G = {
    BASE_H,
    L1,
    L2,
    L2_EFF: L2,
    L2_PAR: L2,
    L2_PERP: 0.0,
    L3,
    TOOL_LEN: 15.0,
    TOOL_TOTAL_LEN,
    TH1_0,
    TH2_0,
    REL_0,
    GAMMA_0,
    GAMMA: GAMMA_0,
    TOOL_REST_ANGLE,
    EXCL_R2,
    FLOOR_CLR,
    IK_POS_TOL_MM: 2.0,
    IK_R_EPS_MM: 0.5,
    LIMIT_J1: [-90.0, 90.0],
    LIMIT_J2: [-65.0, 115.0],
    LIMIT_J3: [-35.0, 170.0],
    LIMIT_J4: [-90.0, 90.0],
    LIMIT_J5: [-90.0, 90.0],
    LIMIT_J6: [-180.0, 180.0],
    WRIST_LEVEL: 0,
    WRIST_DOWN_0: 1,
    WRIST_DOWN_90: 2,
    WRIST_PITCH_0: 3,
    WRIST_PITCH_90: 4,
    WRIST_PITCH_180: 5,
    WRIST_MANUAL: 6,
  };

  function clamp(v, lo, hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  function forwardKinematics(j1, j2, j3, opt = {}, toolOffset = { x: 0, y: 0, z: 0 }, j4_deg = 0, j5_deg = 0, j6_deg = 0) {
    let workOffset = { x: 0, y: 0, z: 0 };
    if (opt && typeof opt === 'object') {
      if ('x' in opt || 'y' in opt || 'z' in opt) workOffset = opt;
      else {
        if (opt.workOffset) workOffset = opt.workOffset;
        if (opt.toolOffset) toolOffset = opt.toolOffset;
        if (opt.j4 !== undefined) j4_deg = opt.j4;
        if (opt.j5 !== undefined) j5_deg = opt.j5;
        if (opt.j6 !== undefined) j6_deg = opt.j6;
      }
    }

    const j1r = Number(j1) * RAD;
    const j2r = Number(j2) * RAD;
    const j3r = Number(j3) * RAD;
    const j5r = Number(j5_deg) * RAD;

    const th1 = TH1_0 - j2r;
    const th2 = TH2_0 - (j2r + j3r);

    const j3_reach = L1 * Math.sin(th1);
    const j3_z = BASE_H + L1 * Math.cos(th1);

    const j5_reach = j3_reach + L2 * Math.sin(th2);
    const j5_z = j3_z + L2 * Math.cos(th2);

    const ang_tool = TOOL_REST_ANGLE + (j2r + j3r + j5r);
    const tcp_reach = j5_reach + TOOL_TOTAL_LEN * Math.cos(ang_tool);
    const tcp_z = j5_z + TOOL_TOTAL_LEN * Math.sin(ang_tool);

    const rawTcp = {
      x: tcp_reach * Math.cos(j1r),
      y: tcp_reach * Math.sin(j1r),
      z: tcp_z
    };

    const tcp = {
      x: rawTcp.x + (workOffset.x || 0) - (toolOffset.x || 0),
      y: rawTcp.y + (workOffset.y || 0) - (toolOffset.y || 0),
      z: rawTcp.z + (workOffset.z || 0) - (toolOffset.z || 0)
    };

    const ang34_deg = Number(j2) + Number(j3);

    return {
      x: tcp.x,
      y: tcp.y,
      z: tcp.z,
      tcp,
      rawTcp,
      ang34_deg,
      j3_reach,
      j3_z,
      j5_reach,
      j5_z
    };
  }

  function computeWrist(wristMode, ang34_deg, j4m = 0, j5m = 0, j6m = 0) {
    let j4 = 0, j5 = 0, j6 = j6m;
    switch (wristMode) {
      case G.WRIST_LEVEL:
        j4 = 0;
        j5 = 0 - ang34_deg;
        j6 = 0;
        break;
      case G.WRIST_DOWN_0:
        j4 = 0;
        j5 = 90 - ang34_deg;
        j6 = 0;
        break;
      case G.WRIST_DOWN_90:
        j4 = 0;
        j5 = 90 - ang34_deg;
        j6 = 90;
        break;
      case G.WRIST_PITCH_0:
        j4 = 0;
        j5 = 0 - ang34_deg;
        j6 = j5m;
        break;
      case G.WRIST_PITCH_90:
        j4 = 0;
        j5 = 90 - ang34_deg;
        j6 = j5m;
        break;
      case G.WRIST_PITCH_180:
        j4 = 0;
        j5 = 180 - ang34_deg;
        j6 = j5m;
        break;
      case G.WRIST_MANUAL:
      default:
        j4 = j4m;
        j5 = j5m;
        j6 = j6m;
        break;
    }
    return {
      j4: clamp(j4, G.LIMIT_J4[0], G.LIMIT_J4[1]),
      j5: clamp(j5, G.LIMIT_J5[0], G.LIMIT_J5[1]),
      j6: clamp(j6, G.LIMIT_J6[0], G.LIMIT_J6[1])
    };
  }

  function checkWorkspace(j1, j2, j3, j4 = 0, j5 = 0, j6 = 0) {
    if (j1 < G.LIMIT_J1[0] || j1 > G.LIMIT_J1[1]) return false;
    if (j2 < G.LIMIT_J2[0] || j2 > G.LIMIT_J2[1]) return false;
    if (j3 < G.LIMIT_J3[0] || j3 > G.LIMIT_J3[1]) return false;
    if (j4 < G.LIMIT_J4[0] || j4 > G.LIMIT_J4[1]) return false;
    if (j5 < G.LIMIT_J5[0] || j5 > G.LIMIT_J5[1]) return false;
    if (j6 < G.LIMIT_J6[0] || j6 > G.LIMIT_J6[1]) return false;

    const p = forwardKinematics(j1, j2, j3, {}, undefined, j4, j5, j6);
    if ((p.x * p.x + p.y * p.y) < G.EXCL_R2 && p.z < G.BASE_H) return false;
    if (p.z < G.FLOOR_CLR || p.j3_z < G.FLOOR_CLR || p.j5_z < G.FLOOR_CLR) return false;
    return true;
  }

  function ikFkError(j1, j2, j3, tx, ty, tz, j4 = 0, j5 = 0, j6 = 0) {
    const p = forwardKinematics(j1, j2, j3, {}, undefined, j4, j5, j6);
    return Math.hypot(p.x - tx, p.y - ty, p.z - tz);
  }

  function solveIK(x, y, z, wristMode = 0, j4m = 0, j5m = 0, currentJ2 = 0, currentJ3 = 0, currentJ1 = 0, j6m = 0) {
    if (![x, y, z].every(Number.isFinite)) {
      return { ok: false, err: Infinity };
    }

    wristMode = clamp(wristMode, G.WRIST_LEVEL, G.WRIST_MANUAL);
    const r = Math.hypot(x, y);
    let deg1;
    if (r < G.IK_R_EPS_MM) {
      deg1 = clamp(currentJ1, G.LIMIT_J1[0], G.LIMIT_J1[1]);
    } else {
      deg1 = clamp(Math.atan2(y, x) * DEG, G.LIMIT_J1[0], G.LIMIT_J1[1]);
    }

    let zc = z;
    if (zc < G.FLOOR_CLR) zc = G.FLOOR_CLR;
    let rc = r;
    if (rc * rc < G.EXCL_R2 && zc < G.BASE_H) rc = Math.sqrt(G.EXCL_R2);

    let r5 = rc;
    let z5 = zc;
    if (wristMode === G.WRIST_DOWN_0 || wristMode === G.WRIST_DOWN_90 || wristMode === G.WRIST_PITCH_90) {
      z5 = zc + TOOL_TOTAL_LEN;
    } else if (wristMode === G.WRIST_PITCH_180) {
      z5 = zc - TOOL_TOTAL_LEN;
    } else if (wristMode === G.WRIST_MANUAL) {
      const pitch_rad = j4m * RAD;
      r5 = Math.max(0, rc - TOOL_TOTAL_LEN * Math.cos(pitch_rad));
      z5 = zc - TOOL_TOTAL_LEN * Math.sin(pitch_rad);
    } else {
      r5 = Math.max(0, rc - TOOL_TOTAL_LEN);
    }

    const dr = r5;
    const dz = z5 - BASE_H;
    const D2 = dr * dr + dz * dz;
    const D = Math.sqrt(D2);

    let C_gamma = (L1 * L1 + L2 * L2 - D2) / (2 * L1 * L2);
    C_gamma = clamp(C_gamma, -1, 1);
    const gamma = Math.acos(C_gamma);

    const beta = Math.atan2(dr, dz);
    let C_alpha = (L1 * L1 + D2 - L2 * L2) / (2 * L1 * Math.max(1e-6, D));
    C_alpha = clamp(C_alpha, -1, 1);
    const alpha = Math.acos(C_alpha);

    const candidates = [
      {
        j2: (TH1_0 - (beta - alpha)) * DEG,
        j3: (gamma - GAMMA_0) * DEG
      },
      {
        j2: (TH1_0 - (beta + alpha)) * DEG,
        j3: (-gamma - GAMMA_0) * DEG
      }
    ];

    const solutions = [];

    for (const cand of candidates) {
      const cj2 = clamp(cand.j2, G.LIMIT_J2[0], G.LIMIT_J2[1]);
      const cj3 = clamp(cand.j3, G.LIMIT_J3[0], G.LIMIT_J3[1]);
      const ang34 = cj2 + cj3;
      const { j4, j5, j6 } = computeWrist(wristMode, ang34, j4m, j5m, j6m);

      if (!checkWorkspace(deg1, cj2, cj3, j4, j5, j6)) continue;
      const err = ikFkError(deg1, cj2, cj3, x, y, z, j4, j5, j6);
      solutions.push({ j1: deg1, j2: cj2, j3: cj3, j4, j5, j6, err });
    }

    if (!solutions.length) {
      return { ok: false, err: Infinity };
    }

    solutions.sort((a, b) => {
      const d1 = Math.abs(a.j2 - currentJ2) + Math.abs(a.j3 - currentJ3);
      const d2 = Math.abs(b.j2 - currentJ2) + Math.abs(b.j3 - currentJ3);
      return d1 - d2;
    });

    const best = solutions[0];
    return { ok: true, ...best };
  }

  function rpyToRotMat(rollDeg, pitchDeg, yawDeg) {
    const r = (Number(rollDeg) || 0) * RAD;
    const p = (Number(pitchDeg) || 0) * RAD;
    const y = (Number(yawDeg) || 0) * RAD;

    const cr = Math.cos(r), sr = Math.sin(r);
    const cp = Math.cos(p), sp = Math.sin(p);
    const cy = Math.cos(y), sy = Math.sin(y);

    return [
      [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
      [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
      [-sp,     cp * sr,                cp * cr]
    ];
  }

  function rotMatToRpy(R) {
    const pitch = Math.atan2(-R[2][0], Math.hypot(R[0][0], R[1][0])) * DEG;
    const yaw = Math.atan2(R[1][0], R[0][0]) * DEG;
    const roll = Math.atan2(R[2][1], R[2][2]) * DEG;
    return { roll, pitch, yaw };
  }

  function forwardKinematics6DOF(j1_deg, j2_deg, j3_deg, j4_deg = 0, j5_deg = 0, j6_deg = 0) {
    const j1r = (Number(j1_deg) || 0) * RAD;
    const j2r = (Number(j2_deg) || 0) * RAD;
    const j3r = (Number(j3_deg) || 0) * RAD;
    const j4r = (Number(j4_deg) || 0) * RAD;
    const j5r = (Number(j5_deg) || 0) * RAD;
    const j6r = (Number(j6_deg) || 0) * RAD;

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

    const Rx4 = [[1,0,0],[0,c4,-s4],[0,s4,c4]];
    const Ry5 = [[c5,0,s5],[0,1,0],[-s5,0,c5]];
    const Rx6 = [[1,0,0],[0,c6,-s6],[0,s6,c6]];

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

    const u_tool = { x: R06[0][0], y: R06[1][0], z: R06[2][0] };
    const tcp = {
      x: p5.x + TOOL_TOTAL_LEN * u_tool.x,
      y: p5.y + TOOL_TOTAL_LEN * u_tool.y,
      z: p5.z + TOOL_TOTAL_LEN * u_tool.z
    };

    const rpy = rotMatToRpy(R06);
    return { tcp, p5, u_tool, R06, ...rpy, j1: j1_deg, j2: j2_deg, j3: j3_deg, j4: j4_deg, j5: j5_deg, j6: j6_deg };
  }

  function solveIK6DOF(x, y, z, rollDeg = 0, pitchDeg = 0, yawDeg = 0, prevJ = { j1: 0, j2: 0, j3: 0, j4: 0, j5: 0, j6: 0 }) {
    const R06 = rpyToRotMat(rollDeg, pitchDeg, yawDeg);
    const ux = R06[0][0], uy = R06[1][0], uz = R06[2][0];

    const x5 = x - TOOL_TOTAL_LEN * ux;
    const y5 = y - TOOL_TOTAL_LEN * uy;
    const z5 = z - TOOL_TOTAL_LEN * uz;

    const r5 = Math.hypot(x5, y5);
    let j1 = (r5 < 0.5) ? (prevJ.j1 || 0) : Math.atan2(y5, x5) * DEG;

    let zc = z5 < FLOOR_CLR ? FLOOR_CLR : z5;
    let rc = r5;
    if (rc * rc < EXCL_R2 && zc < BASE_H) rc = Math.sqrt(EXCL_R2);

    const dr = rc;
    const dz = zc - BASE_H;
    let D2 = dr * dr + dz * dz;
    let D = Math.sqrt(D2);

    const maxReach = L1 + L2;
    if (D > maxReach * 0.999) {
      D = maxReach * 0.999;
      D2 = D * D;
    }

    let C_gamma = (L1 * L1 + L2 * L2 - D2) / (2 * L1 * L2);
    C_gamma = clamp(C_gamma, -1.0, 1.0);
    const gamma = Math.acos(C_gamma);

    const beta = Math.atan2(dr, dz);
    let C_alpha = (L1 * L1 + D2 - L2 * L2) / (2 * L1 * Math.max(1e-6, D));
    C_alpha = clamp(C_alpha, -1.0, 1.0);
    const alpha = Math.acos(C_alpha);

    const cand1_j2 = (TH1_0 - (beta - alpha)) * DEG;
    const cand1_j3 = (gamma - GAMMA_0) * DEG;
    const cand2_j2 = (TH1_0 - (beta + alpha)) * DEG;
    const cand2_j3 = (-gamma - GAMMA_0) * DEG;

    j1 = clamp(j1, G.LIMIT_J1[0], G.LIMIT_J1[1]);
    const prevJ2 = prevJ.j2 || 0;
    const prevJ3 = prevJ.j3 || 0;

    const c_j2_1 = clamp(cand1_j2, G.LIMIT_J2[0], G.LIMIT_J2[1]);
    const c_j3_1 = clamp(cand1_j3, G.LIMIT_J3[0], G.LIMIT_J3[1]);
    const c_j2_2 = clamp(cand2_j2, G.LIMIT_J2[0], G.LIMIT_J2[1]);
    const c_j3_2 = clamp(cand2_j3, G.LIMIT_J3[0], G.LIMIT_J3[1]);

    let j2 = c_j2_1, j3 = c_j3_1;
    if (Math.abs(c_j2_2 - prevJ2) + Math.abs(c_j3_2 - prevJ3) < Math.abs(c_j2_1 - prevJ2) + Math.abs(c_j3_1 - prevJ3)) {
      j2 = c_j2_2;
      j3 = c_j3_2;
    }

    const j1r = j1 * RAD;
    const j2r = j2 * RAD;
    const j3r = j3 * RAD;
    const th2 = TH2_0 - (j2r + j3r);
    const forearm_pitch = Math.PI / 2.0 - th2;

    const c1 = Math.cos(j1r), s1 = Math.sin(j1r);
    const cp = Math.cos(forearm_pitch), sp = Math.sin(forearm_pitch);

    const R03 = [
      [c1 * cp, -s1, c1 * sp],
      [s1 * cp,  c1, s1 * sp],
      [-sp,       0, cp]
    ];

    const R36 = [
      [
        R03[0][0]*R06[0][0] + R03[1][0]*R06[1][0] + R03[2][0]*R06[2][0],
        R03[0][0]*R06[0][1] + R03[1][0]*R06[1][1] + R03[2][0]*R06[2][1],
        R03[0][0]*R06[0][2] + R03[1][0]*R06[1][2] + R03[2][0]*R06[2][2]
      ],
      [
        R03[0][1]*R06[0][0] + R03[1][1]*R06[1][0] + R03[2][1]*R06[2][0],
        R03[0][1]*R06[0][1] + R03[1][1]*R06[1][1] + R03[2][1]*R06[2][1],
        R03[0][1]*R06[0][2] + R03[1][1]*R06[1][2] + R03[2][1]*R06[2][2]
      ],
      [
        R03[0][2]*R06[0][0] + R03[1][2]*R06[1][0] + R03[2][2]*R06[2][0],
        R03[0][2]*R06[0][1] + R03[1][2]*R06[1][1] + R03[2][2]*R06[2][1],
        R03[0][2]*R06[0][2] + R03[1][2]*R06[1][2] + R03[2][2]*R06[2][2]
      ]
    ];

    const cosJ5 = clamp(R36[0][0], -1.0, 1.0);
    const sinJ5_sq = R36[1][0]*R36[1][0] + R36[2][0]*R36[2][0];
    const sinJ5 = Math.sqrt(sinJ5_sq);

    let j4, j5, j6;
    const WRIST_SINGULARITY_EPS = 0.0436;
    if (sinJ5 < WRIST_SINGULARITY_EPS) {
      j5 = Math.acos(cosJ5) * DEG;
      j4 = prevJ.j4 || 0;
      const totalRoll = Math.atan2(R36[2][1] - R36[1][2], R36[1][1] + R36[2][2]) * DEG;
      j6 = totalRoll - j4;
      while (j6 > 180.0) j6 -= 360.0;
      while (j6 < -180.0) j6 += 360.0;
    } else {
      const j5_1 = Math.atan2(sinJ5, cosJ5) * DEG;
      const j4_1 = Math.atan2(R36[1][0], -R36[2][0]) * DEG;
      const j6_1 = Math.atan2(R36[0][1], R36[0][2]) * DEG;

      const j5_2 = Math.atan2(-sinJ5, cosJ5) * DEG;
      const j4_2 = Math.atan2(-R36[1][0], R36[2][0]) * DEG;
      const j6_2 = Math.atan2(-R36[0][1], -R36[0][2]) * DEG;

      const inLimit = (val, lim) => val >= lim[0] - 1e-3 && val <= lim[1] + 1e-3;
      const b1_valid = inLimit(j4_1, G.LIMIT_J4) && inLimit(j5_1, G.LIMIT_J5) && inLimit(j6_1, G.LIMIT_J6);
      const b2_valid = inLimit(j4_2, G.LIMIT_J4) && inLimit(j5_2, G.LIMIT_J5) && inLimit(j6_2, G.LIMIT_J6);

      const prevJ4 = prevJ.j4 || 0;
      const prevJ5 = prevJ.j5 || 0;
      const prevJ6 = prevJ.j6 || 0;

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

    j4 = clamp(j4, G.LIMIT_J4[0], G.LIMIT_J4[1]);
    j5 = clamp(j5, G.LIMIT_J5[0], G.LIMIT_J5[1]);
    j6 = clamp(j6, G.LIMIT_J6[0], G.LIMIT_J6[1]);

    const ok = checkWorkspace(j1, j2, j3, j4, j5, j6);
    return { ok, j1, j2, j3, j4, j5, j6, wristSingular: sinJ5 < WRIST_SINGULARITY_EPS };
  }

  return {
    G,
    clamp,
    forwardKinematics,
    forwardKinematics6DOF,
    computeWrist,
    checkWorkspace,
    solveIK,
    solveIK6DOF,
    ikFkError,
    rpyToRotMat,
    rotMatToRpy,
  };
})();

if (typeof module !== 'undefined') module.exports = RobotKinematics;
