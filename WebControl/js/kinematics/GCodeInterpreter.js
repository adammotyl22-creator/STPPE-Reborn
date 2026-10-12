/**
 * GCodeInterpreter.js - High-Performance CNC & 3D Slicer G-Code Parser with Path Simplification
 * 
 * Translates standard CNC/Laser/3D Slicer G-code into 6-DOF robotic trajectory waypoints.
 * Implements 3D trajectory simplification and collinear decimation to prevent DOM & WebGL
 * freeze on massive G-code files with thousands of lines.
 */

import {
  solveIK,
  forwardKinematics,
  FLOOR_CLR,
  EXCL_R
} from './RobotKinematics.js';

export const GCODE_DEFAULTS = Object.freeze({
  DEFAULT_FEEDRATE: 1200.0, // mm/min (20 mm/s)
  RAPID_FEEDRATE: 3600.0,   // mm/min (60 mm/s)
  DEFAULT_CENTER: Object.freeze({ x: 160.0, y: 0.0, z: 60.0 }),
  MAX_SPAN: 160.0,          // Maximum allowable XY span for auto-scaling (mm)
  MIN_SAFE_Z: 25.0,         // Minimum Z elevation above floor (mm)
  ARC_SEGMENT_LENGTH: 3.0,  // Sub-segment chord resolution for circular arcs (mm)
  DEFAULT_WRIST_MODE: 1,    // W1: Downward tool pointing straight down
  MAX_POINTS_DEFAULT: 250,  // Max waypoints to generate (prevents UI freeze)
  SIMPLIFY_TOLERANCE: 0.25  // 3D Collinear tolerance in mm
});

export class GCodeInterpreter {
  /**
   * Parses raw G-Code text into robotic trajectory waypoints with automatic path simplification
   * 
   * @param {string} gcodeText - Raw G-code string
   * @param {Object} [options] - Parsing and fitting configuration
   * @param {boolean} [options.autoCenter=true] - Center toolpath around target workspace position
   * @param {Object} [options.targetCenter] - { x: 160, y: 0, z: 60 }
   * @param {number|'auto'} [options.scale=1.0] - Scale multiplier or 'auto' to fit workspace
   * @param {number} [options.defaultFeedrate=1200] - Default feedrate in mm/min
   * @param {number} [options.wristMode=1] - Wrist orientation mode (0-6)
   * @param {number|'all'} [options.maxPoints=250] - Maximum waypoints cap for UI performance
   * @param {number} [options.simplifyTolerance=0.25] - Collinear simplification tolerance (mm)
   * @param {string} [options.sequenceName] - Name for generated sequence
   * @returns {Object} Parsed sequence object
   */
  static parse(gcodeText, options = {}) {
    const opts = Object.assign({
      autoCenter: true,
      targetCenter: { ...GCODE_DEFAULTS.DEFAULT_CENTER },
      scale: 1.0,
      defaultFeedrate: GCODE_DEFAULTS.DEFAULT_FEEDRATE,
      rapidFeedrate: GCODE_DEFAULTS.RAPID_FEEDRATE,
      wristMode: GCODE_DEFAULTS.DEFAULT_WRIST_MODE,
      maxPoints: GCODE_DEFAULTS.MAX_POINTS_DEFAULT,
      simplifyTolerance: GCODE_DEFAULTS.SIMPLIFY_TOLERANCE,
      sequenceName: 'G-Code Toolpath'
    }, options);

    if (!gcodeText || typeof gcodeText !== 'string' || !gcodeText.trim()) {
      return {
        sequenceName: opts.sequenceName,
        version: '1.0',
        loop: false,
        waypoints: [],
        boundingBox: { minX: 0, maxX: 0, minY: 0, maxY: 0, minZ: 0, maxZ: 0, spanX: 0, spanY: 0, spanZ: 0 },
        metrics: { totalPoints: 0, rawLines: 0, totalDistance: 0, totalDuration: 0 },
        warnings: ['Empty or invalid G-code input']
      };
    }

    // 1. Pass 1: Parse raw motion commands from lines
    let rawMoves = GCodeInterpreter._parseRawMoves(gcodeText, opts);
    const rawLineCount = rawMoves.length;

    if (rawMoves.length === 0) {
      return {
        sequenceName: opts.sequenceName,
        version: '1.0',
        loop: false,
        waypoints: [],
        boundingBox: { minX: 0, maxX: 0, minY: 0, maxY: 0, minZ: 0, maxZ: 0, spanX: 0, spanY: 0, spanZ: 0 },
        metrics: { totalPoints: 0, rawLines: 0, totalDistance: 0, totalDuration: 0 },
        warnings: ['No valid motion commands (G0/G1/G2/G3) found in G-code']
      };
    }

    // 2. Compute raw bounding box
    const rawBBox = GCodeInterpreter._computeBoundingBox(rawMoves);

    // 3. Path Simplification & Decimation (Collapses collinear micro-moves)
    if (opts.maxPoints !== 'all' && rawMoves.length > (opts.maxPoints || 250)) {
      rawMoves = GCodeInterpreter._simplifyPath(rawMoves, opts.maxPoints, opts.simplifyTolerance);
    }

    // 4. Calculate auto-scaling factor and spatial translation offsets
    let effectiveScale = (typeof opts.scale === 'number' && opts.scale > 0) ? opts.scale : 1.0;
    if (opts.scale === 'auto') {
      const maxRawSpan = Math.max(rawBBox.spanX, rawBBox.spanY);
      if (maxRawSpan > GCODE_DEFAULTS.MAX_SPAN) {
        effectiveScale = GCODE_DEFAULTS.MAX_SPAN / maxRawSpan;
      }
    }

    let offsetX = 0;
    let offsetY = 0;
    let offsetZ = 0;

    if (opts.autoCenter) {
      const scaledCenterX = ((rawBBox.minX + rawBBox.maxX) / 2) * effectiveScale;
      const scaledCenterY = ((rawBBox.minY + rawBBox.maxY) / 2) * effectiveScale;
      const scaledMinZ = rawBBox.minZ * effectiveScale;

      offsetX = opts.targetCenter.x - scaledCenterX;
      offsetY = opts.targetCenter.y - scaledCenterY;
      offsetZ = Math.max(GCODE_DEFAULTS.MIN_SAFE_Z, opts.targetCenter.z) - scaledMinZ;
    }

    // 5. Pass 2: Apply spatial transformation and solve Inverse Kinematics for each waypoint
    const waypoints = [];
    const warnings = [];
    let totalDistance = 0;
    let totalDuration = 0;
    let prevPoint = null;

    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;
    let minZ = Infinity, maxZ = -Infinity;

    rawMoves.forEach((move, idx) => {
      // Apply scale & offset
      const posX = (move.x * effectiveScale) + offsetX;
      const posY = (move.y * effectiveScale) + offsetY;
      const posZ = (move.z * effectiveScale) + offsetZ;

      // Update transformed bounding box
      minX = Math.min(minX, posX);
      maxX = Math.max(maxX, posX);
      minY = Math.min(minY, posY);
      maxY = Math.max(maxY, posY);
      minZ = Math.min(minZ, posZ);
      maxZ = Math.max(maxZ, posZ);

      // Calculate Euclidean move distance
      let dist = 0;
      if (prevPoint) {
        dist = Math.hypot(posX - prevPoint.x, posY - prevPoint.y, posZ - prevPoint.z);
      }
      totalDistance += dist;

      // Calculate step duration in milliseconds based on feedrate
      const feedrate = move.isRapid ? opts.rapidFeedrate : (move.feedrate || opts.defaultFeedrate);
      const feedSpeed = Math.max(0.1, feedrate / 60.0); // mm/sec
      let dur = move.dwellMs > 0 ? move.dwellMs : Math.max(50, Math.round((dist / feedSpeed) * 1000));
      totalDuration += dur;

      // Solve Inverse Kinematics
      const ik = solveIK(posX, posY, posZ, { wristMode: opts.wristMode });
      const j1 = Number.isFinite(ik.j1) ? Number(ik.j1.toFixed(2)) : 0;
      const j2 = Number.isFinite(ik.j2) ? Number(ik.j2.toFixed(2)) : 0;
      const j3 = Number.isFinite(ik.j3) ? Number(ik.j3.toFixed(2)) : 0;
      const j4 = Number.isFinite(ik.j4) ? Number(ik.j4.toFixed(2)) : 0;
      const j5 = Number.isFinite(ik.j5) ? Number(ik.j5.toFixed(2)) : 0;
      const j6 = Number.isFinite(ik.j6) ? Number(ik.j6.toFixed(2)) : 0;

      if (!ik.success || ik.clamped) {
        warnings.push(`Point #${idx + 1} (${posX.toFixed(1)}, ${posY.toFixed(1)}, ${posZ.toFixed(1)}) is out of kinematic reach`);
      }

      const wp = {
        id: `wp_gcode_${idx + 1}_${Math.random().toString(36).substring(2, 6)}`,
        label: move.comment ? move.comment : `GCode Pt ${idx + 1} (${move.cmd})`,
        j1,
        j2,
        j3,
        j4,
        j5,
        j6,
        dx: 0,
        dy: 0,
        dur,
        grip: move.toolActive ? 180 : 0,
        comment: move.rawLine || `X:${posX.toFixed(1)} Y:${posY.toFixed(1)} Z:${posZ.toFixed(1)}`
      };

      waypoints.push(wp);
      prevPoint = { x: posX, y: posY, z: posZ };
    });

    const transformedBBox = {
      minX: Number(minX.toFixed(1)),
      maxX: Number(maxX.toFixed(1)),
      minY: Number(minY.toFixed(1)),
      maxY: Number(maxY.toFixed(1)),
      minZ: Number(minZ.toFixed(1)),
      maxZ: Number(maxZ.toFixed(1)),
      spanX: Number((maxX - minX).toFixed(1)),
      spanY: Number((maxY - minY).toFixed(1)),
      spanZ: Number((maxZ - minZ).toFixed(1))
    };

    return {
      sequenceName: opts.sequenceName,
      version: '1.0',
      loop: false,
      description: `Generated from G-Code (${waypoints.length} waypoints from ${rawLineCount} moves, span: ${transformedBBox.spanX}x${transformedBBox.spanY}mm)`,
      waypoints,
      boundingBox: transformedBBox,
      metrics: {
        rawLines: rawLineCount,
        totalPoints: waypoints.length,
        totalDistance: Number(totalDistance.toFixed(1)),
        totalDuration: Number((totalDuration / 1000).toFixed(2)),
        scaleApplied: effectiveScale
      },
      warnings
    };
  }

  /**
   * Simplifies 3D path by removing collinear intermediate points while keeping sharp corners,
   * tool changes, rapid travels, and vertical plunges/retracts.
   * @private
   */
  static _simplifyPath(moves, maxPoints, tolerance = 0.25) {
    if (moves.length <= maxPoints) return moves;

    // Identify critical inflection points (start, end, rapid transitions, tool toggles, Z jumps)
    const kept = [moves[0]];

    for (let i = 1; i < moves.length - 1; i++) {
      const prev = kept[kept.length - 1];
      const curr = moves[i];
      const next = moves[i + 1];

      // Critical state changes MUST be kept
      const stateChange = (curr.toolActive !== prev.toolActive) ||
                          (curr.isRapid !== prev.isRapid) ||
                          (curr.dwellMs > 0) ||
                          (Math.abs(curr.z - prev.z) > 0.8);

      if (stateChange) {
        kept.push(curr);
        continue;
      }

      // Check distance from line between prev and next (3D collinear deviation)
      const dev = GCodeInterpreter._pointLineDistance3D(curr, prev, next);
      if (dev >= tolerance) {
        kept.push(curr);
      }
    }

    kept.push(moves[moves.length - 1]);

    // If still exceeds maxPoints, downsample evenly while preserving transitions
    if (kept.length > maxPoints) {
      const step = kept.length / maxPoints;
      const sampled = [kept[0]];
      for (let i = 1; i < maxPoints - 1; i++) {
        const idx = Math.floor(i * step);
        sampled.push(kept[idx]);
      }
      sampled.push(kept[kept.length - 1]);
      return sampled;
    }

    return kept;
  }

  /**
   * Distance from point P to line segment AB in 3D
   * @private
   */
  static _pointLineDistance3D(p, a, b) {
    const abX = b.x - a.x, abY = b.y - a.y, abZ = b.z - a.z;
    const apX = p.x - a.x, apY = p.y - a.y, apZ = p.z - a.z;
    const abLenSq = (abX * abX) + (abY * abY) + (abZ * abZ);
    if (abLenSq < 1e-6) return Math.hypot(apX, apY, apZ);

    const t = Math.max(0, Math.min(1, ((apX * abX) + (apY * abY) + (apZ * abZ)) / abLenSq));
    const projX = a.x + t * abX;
    const projY = a.y + t * abY;
    const projZ = a.z + t * abZ;
    return Math.hypot(p.x - projX, p.y - projY, p.z - projZ);
  }

  /**
   * Internal parser extracting structured move commands from G-code lines
   * @private
   */
  static _parseRawMoves(gcodeText, opts) {
    const lines = gcodeText.split(/\r?\n/);
    const moves = [];

    let currentX = 0;
    let currentY = 0;
    let currentZ = GCODE_DEFAULTS.MIN_SAFE_Z;
    let currentFeed = opts.defaultFeedrate;
    let isAbsolute = true;
    let unitScale = 1.0;
    let toolActive = false;

    for (let lineIndex = 0; lineIndex < lines.length; lineIndex++) {
      let rawLine = lines[lineIndex].trim();
      if (!rawLine) continue;

      let line = rawLine.replace(/;.*$/, '').replace(/\(.*?\)/g, '').trim();
      if (!line) continue;

      const words = {};
      const tokens = line.match(/[A-Za-z][-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?/g) || [];
      tokens.forEach((token) => {
        const letter = token[0].toUpperCase();
        const value = parseFloat(token.substring(1));
        if (!isNaN(value)) {
          words[letter] = value;
        }
      });

      if (words.G === 90) isAbsolute = true;
      if (words.G === 91) isAbsolute = false;
      if (words.G === 20) unitScale = 25.4;
      if (words.G === 21) unitScale = 1.0;

      if (words.M === 3 || words.M === 4 || words.M === 106) toolActive = true;
      if (words.M === 5 || words.M === 107) toolActive = false;

      if (words.G === 4) {
        let dwellMs = 500;
        if (words.P !== undefined) dwellMs = Math.round(words.P);
        if (words.S !== undefined) dwellMs = Math.round(words.S * 1000);
        moves.push({
          cmd: 'G4',
          x: currentX,
          y: currentY,
          z: currentZ,
          dwellMs,
          isRapid: false,
          toolActive,
          rawLine,
          comment: `Dwell ${dwellMs}ms`
        });
        continue;
      }

      if (words.F !== undefined) {
        currentFeed = words.F * unitScale;
      }

      const hasX = words.X !== undefined;
      const hasY = words.Y !== undefined;
      const hasZ = words.Z !== undefined;

      if (!hasX && !hasY && !hasZ && words.G === undefined) {
        continue;
      }

      let targetX = currentX;
      let targetY = currentY;
      let targetZ = currentZ;

      if (hasX) targetX = isAbsolute ? words.X * unitScale : currentX + (words.X * unitScale);
      if (hasY) targetY = isAbsolute ? words.Y * unitScale : currentY + (words.Y * unitScale);
      if (hasZ) targetZ = isAbsolute ? words.Z * unitScale : currentZ + (words.Z * unitScale);

      // G0: Rapid
      if (words.G === 0) {
        moves.push({
          cmd: 'G0',
          x: targetX,
          y: targetY,
          z: targetZ,
          feedrate: opts.rapidFeedrate,
          isRapid: true,
          dwellMs: 0,
          toolActive,
          rawLine
        });
        currentX = targetX;
        currentY = targetY;
        currentZ = targetZ;
      }
      // G1: Linear Move
      else if (words.G === 1 || (words.G === undefined && (hasX || hasY || hasZ))) {
        moves.push({
          cmd: 'G1',
          x: targetX,
          y: targetY,
          z: targetZ,
          feedrate: currentFeed,
          isRapid: false,
          dwellMs: 0,
          toolActive,
          rawLine
        });
        currentX = targetX;
        currentY = targetY;
        currentZ = targetZ;
      }
      // G2 / G3: Circular Arcs
      else if (words.G === 2 || words.G === 3) {
        const isCW = (words.G === 2);
        const arcPoints = GCodeInterpreter._interpolateArc({
          startX: currentX,
          startY: currentY,
          endX: targetX,
          endY: targetY,
          z: targetZ,
          i: (words.I || 0) * unitScale,
          j: (words.J || 0) * unitScale,
          r: words.R !== undefined ? words.R * unitScale : null,
          isCW,
          feedrate: currentFeed,
          toolActive,
          rawLine
        });

        arcPoints.forEach(pt => moves.push(pt));
        currentX = targetX;
        currentY = targetY;
        currentZ = targetZ;
      }
    }

    return moves;
  }

  /**
   * Subdivides G2/G3 circular arcs into discrete linear chord points
   * @private
   */
  static _interpolateArc(arcData) {
    const { startX, startY, endX, endY, z, i, j, r, isCW, feedrate, toolActive, rawLine } = arcData;
    let centerX, centerY, radius;

    if (r !== null) {
      const d2 = Math.hypot(endX - startX, endY - startY);
      if (d2 < 1e-7) {
        // Degenerate arc where start and end points coincide: indeterminate with R
        return [];
      }
      const h2 = Math.max(0, (r * r) - (d2 * d2 / 4));
      const h = Math.sqrt(h2);
      const midX = (startX + endX) / 2;
      const midY = (startY + endY) / 2;
      const factor = isCW ? -1 : 1;
      centerX = midX + factor * h * (startY - endY) / d2;
      centerY = midY + factor * h * (endX - startX) / d2;
      radius = Math.abs(r);
    } else {
      centerX = startX + i;
      centerY = startY + j;
      radius = Math.hypot(i, j);
      if (radius < 1e-7) {
        return [];
      }
    }

    const startAngle = Math.atan2(startY - centerY, startX - centerX);
    let endAngle = Math.atan2(endY - centerY, endX - centerX);

    if (isCW) {
      if (endAngle >= startAngle) endAngle -= 2 * Math.PI;
    } else {
      if (endAngle <= startAngle) endAngle += 2 * Math.PI;
    }

    const angularSpan = Math.abs(endAngle - startAngle);
    const arcLength = radius * angularSpan;
    const steps = Math.max(2, Math.ceil(arcLength / GCODE_DEFAULTS.ARC_SEGMENT_LENGTH));

    const points = [];
    for (let s = 1; s <= steps; s++) {
      const t = s / steps;
      const angle = startAngle + (endAngle - startAngle) * t;
      const ptX = centerX + radius * Math.cos(angle);
      const ptY = centerY + radius * Math.sin(angle);

      points.push({
        cmd: isCW ? 'G2' : 'G3',
        x: ptX,
        y: ptY,
        z,
        feedrate,
        isRapid: false,
        dwellMs: 0,
        toolActive,
        rawLine: s === steps ? rawLine : ''
      });
    }

    return points;
  }

  /**
   * Computes min/max 3D bounding box for points array
   * @private
   */
  static _computeBoundingBox(moves) {
    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;
    let minZ = Infinity, maxZ = -Infinity;

    moves.forEach((m) => {
      minX = Math.min(minX, m.x);
      maxX = Math.max(maxX, m.x);
      minY = Math.min(minY, m.y);
      maxY = Math.max(maxY, m.y);
      minZ = Math.min(minZ, m.z);
      maxZ = Math.max(maxZ, m.z);
    });

    if (minX === Infinity) {
      return { minX: 0, maxX: 0, minY: 0, maxY: 0, minZ: 0, maxZ: 0, spanX: 0, spanY: 0, spanZ: 0 };
    }

    return {
      minX: Number(minX.toFixed(2)),
      maxX: Number(maxX.toFixed(2)),
      minY: Number(minY.toFixed(2)),
      maxY: Number(maxY.toFixed(2)),
      minZ: Number(minZ.toFixed(2)),
      maxZ: Number(maxZ.toFixed(2)),
      spanX: Number((maxX - minX).toFixed(2)),
      spanY: Number((maxY - minY).toFixed(2)),
      spanZ: Number((maxZ - minZ).toFixed(2))
    };
  }

  /**
   * Exports a trajectory sequence to standard CNC G-Code text
   */
  static exportGCode(sequence, options = {}) {
    const opts = Object.assign({
      feedrate: GCODE_DEFAULTS.DEFAULT_FEEDRATE,
      rapidFeedrate: GCODE_DEFAULTS.RAPID_FEEDRATE,
      author: 'Teensy 4.1 Robot Arm Studio'
    }, options);

    const waypoints = (sequence && Array.isArray(sequence.waypoints)) ? sequence.waypoints : [];
    const dateStr = new Date().toISOString();

    const lines = [
      `; ==========================================================`,
      `; G-Code Export: ${sequence?.sequenceName || 'Robot Arm Sequence'}`,
      `; Generated: ${dateStr}`,
      `; Author: ${opts.author}`,
      `; Total Waypoints: ${waypoints.length}`,
      `; ==========================================================`,
      `G21 ; Set units to millimeters`,
      `G90 ; Set positioning to absolute`,
      `G92 X0 Y0 Z0 ; Set reference zero`,
      `M5 ; Tool / Gripper Initial State Off`,
      `F${opts.feedrate.toFixed(0)} ; Default Feedrate`,
      ``
    ];

    let prevGrip = 0;

    waypoints.forEach((wp, idx) => {
      const fk = forwardKinematics(wp.j1, wp.j2, wp.j3, {}, null, wp.j4, wp.j5);
      const x = fk.tcp.x.toFixed(2);
      const y = fk.tcp.y.toFixed(2);
      const z = fk.tcp.z.toFixed(2);

      const grip = Number(wp.grip ?? 0);
      if (grip > 90 && prevGrip <= 90) {
        lines.push(`M3 S255 ; Tool / Gripper On (Waypoint #${idx + 1})`);
      } else if (grip <= 90 && prevGrip > 90) {
        lines.push(`M5 ; Tool / Gripper Off (Waypoint #${idx + 1})`);
      }
      prevGrip = grip;

      const dur = Number(wp.dur ?? 1000);
      lines.push(`G1 X${x} Y${y} Z${z} F${opts.feedrate.toFixed(0)} ; Point #${idx + 1} (${wp.label || ''}) [${dur}ms]`);
    });

    lines.push(``);
    lines.push(`M5 ; Final Tool Off`);
    lines.push(`M30 ; Program End and Rewind`);

    return lines.join('\n');
  }
}

export default GCodeInterpreter;
