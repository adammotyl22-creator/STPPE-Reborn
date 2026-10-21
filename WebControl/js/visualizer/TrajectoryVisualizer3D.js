/**
 * TrajectoryVisualizer3D.js - High-Performance 3D Spline Toolpath & Kinematics Safety Visualizer
 * 
 * Renders the 3D Catmull-Rom spline tube connecting sequence waypoints directly in Three.js.
 * Performs real-time sub-segment kinematics clearance checks, color-coding safe paths in
 * sage green (#10b981) and collision/singularity breach segments in alert amber/red (#ef4444).
 * 
 * Includes high-density waypoint sub-sampling to maintain 60 FPS performance without WebGL buffer spikes.
 */

import * as THREE from '../../vendor/three/three.module.js';
import {
  forwardKinematics,
  forwardKinematics6DOF,
  rpyToRotMat,
  solveIK,
  robotToThreeCoords,
  threeToRobotCoords,
  FLOOR_CLR,
  EXCL_R,
  BASE_H
} from '../kinematics/RobotKinematics.js';
import { checkWorkspace } from '../kinematics/WorkspaceValidator.js';

export const PATH_COLORS = Object.freeze({
  SAFE: 0x10b981,         // Soft sage green (#10b981)
  CRUISE: 0x38bdf8,       // Steady cruising cyan
  WARNING: 0xf59e0b,      // Warm amber (#f59e0b)
  COLLISION: 0xef4444,    // Alert red (#ef4444)
  NODE_DEFAULT: 0x38bdf8, // Sky cyan (#38bdf8)
  NODE_ACTIVE: 0xfacc15,  // Bright yellow active cursor (#facc15)
  NODE_HALO: 0x10b981     // Halo ring (#10b981)
});

export class TrajectoryVisualizer3D {
  /**
   * @param {Object} [options]
   * @param {boolean} [options.visible=true]
   * @param {number} [options.tubeRadius=1.4]
   * @param {number} [options.nodeRadius=3.2]
   * @param {number} [options.samplesPerSegment=16]
   * @param {boolean} [options.hidePastWaypoints=true] - Whether already traversed positions disappear
   */
  constructor(options = {}) {
    this.options = Object.assign({
      visible: true,
      tubeRadius: 1.4,
      nodeRadius: 3.2,
      samplesPerSegment: 16,
      hidePastWaypoints: true,
      maxRenderedNodes: 48,
      triadLength: 12,
      maxTubularSegments: 320
    }, options);

    this.root = new THREE.Group();
    this.root.name = 'TrajectoryVisualizer3D_Root';
    this.root.visible = Boolean(this.options.visible);

    // Sub-groups
    this.tubeGroup = new THREE.Group();
    this.tubeGroup.name = 'Trajectory_Tube_Group';
    this.glowGroup = new THREE.Group();
    this.glowGroup.name = 'Trajectory_Glow_Group';
    this.nodesGroup = new THREE.Group();
    this.nodesGroup.name = 'Trajectory_Nodes_Group';
    this.triadsGroup = new THREE.Group();
    this.triadsGroup.name = 'Trajectory_TCP_Orientation_Triads';
    this.activeGroup = new THREE.Group();
    this.activeGroup.name = 'Trajectory_Active_Group';
    // MOVL straight-line segment overlays (renders over spline tube)
    this.linMovlGroup = new THREE.Group();
    this.linMovlGroup.name = 'Trajectory_MOVL_Linear_Group';

    this.root.add(this.tubeGroup);
    this.root.add(this.glowGroup);
    this.root.add(this.linMovlGroup);
    this.root.add(this.nodesGroup);
    this.root.add(this.triadsGroup);
    this.root.add(this.activeGroup);

    // Active playback cursor
    this._initActiveCursor();

    // Cache of current sequence & waypoint metadata
    this.currentSequence = null;
    this._lastSeqKey = '';
    this.waypointPositions = []; // Array of { threePos: THREE.Vector3, robotTcp: {x,y,z}, isReachable: boolean, wp: Object }
    this.activeWaypointIndex = -1;
    this.glowEnabled = true;
  }

  /**
   * Initializes the glowing pulsing active waypoint cursor
   * @private
   */
  _initActiveCursor() {
    const haloGeo = new THREE.RingGeometry(4.5, 6.0, 32);
    const haloMat = new THREE.MeshBasicMaterial({
      color: PATH_COLORS.NODE_ACTIVE,
      transparent: true,
      opacity: 0.85,
      side: THREE.DoubleSide,
      depthWrite: false
    });
    this.activeCursor = new THREE.Mesh(haloGeo, haloMat);
    this.activeCursor.rotation.x = -Math.PI / 2;
    this.activeCursor.visible = false;
    this.activeGroup.add(this.activeCursor);
  }

  /**
   * Updates and builds the 3D trajectory tube and waypoint nodes from a sequence
   * @param {Object} sequence - { waypoints: Array, loop: boolean, activeIndex?: number }
   */
  updateSequence(sequence) {
    const waypoints = (sequence && Array.isArray(sequence.waypoints)) ? sequence.waypoints : [];

    // Include a compact coordinate fingerprint so any in-place waypoint edit
    // (including j4/j5 wrist pitch/roll or moveType) correctly invalidates the mesh.
    const coordHash = waypoints.map(w =>
      `${(+w.j1).toFixed(1)},${(+w.j2).toFixed(1)},${(+w.j3).toFixed(1)},${(+w.j4).toFixed(1)},${(+w.j5).toFixed(1)},${(+w.j6).toFixed(1)},${w.roll},${w.pitch},${w.yaw},${w.frame},${+(w.dur ?? w.duration ?? 1000)},${w.moveType || w.type || 'spline'}`
    ).join('|');
    const seqKey = `${sequence?.sequenceName || 'seq'}_${waypoints.length}_${sequence?.loop ? 1 : 0}_${coordHash}`;
    const structureChanged = (seqKey !== this._lastSeqKey);

    if (structureChanged) {
      this._lastSeqKey = seqKey;
      this.currentSequence = sequence;
      this._rebuildFullSequence(sequence);
    }

    if (sequence && typeof sequence.activeIndex === 'number') {
      this.setActiveWaypoint(sequence.activeIndex);
    } else {
      this.setActiveWaypoint(-1);
    }
  }

  /**
   * Recomputes forward kinematics and rebuilds all 3D waypoint node meshes
   * @private
   */
  _rebuildFullSequence(sequence) {
    this._clearPreviousMeshes();

    if (!sequence || !Array.isArray(sequence.waypoints) || sequence.waypoints.length === 0) {
      this.waypointPositions = [];
      this.activeCursor.visible = false;
      return;
    }

    const waypoints = sequence.waypoints;
    const isLoop = Boolean(sequence.loop);

    // 1. Calculate Forward Kinematics TCP for each waypoint
    this.waypointPositions = waypoints.map((wp, idx) => {
      const j1 = Number(wp.j1 ?? 0);
      const j2 = Number(wp.j2 ?? -90);
      const j3 = Number(wp.j3 ?? 90);
      const j4 = Number(wp.j4 ?? 90);
      const j5 = Number(wp.j5 ?? 90);
      const j6 = Number(wp.j6 ?? 0);

      const fk = forwardKinematics(j1, j2, j3, {}, null, j4, j5);
      const orientation = forwardKinematics6DOF(j1, j2, j3, j4, j5, j6);
      const hasEuler = ['roll', 'pitch', 'yaw'].every(key =>
        wp[key] !== undefined && wp[key] !== null && Number.isFinite(Number(wp[key])));
      const rotation = hasEuler
        ? rpyToRotMat(wp.roll, wp.pitch, wp.yaw)
        : orientation.R06;
      const threeCoords = robotToThreeCoords(fk.tcp.x, fk.tcp.y, fk.tcp.z);
      const threePos = new THREE.Vector3(threeCoords.x, threeCoords.y, threeCoords.z);

      // Check safety of waypoint itself
      const ws = checkWorkspace(j1, j2, j3, j4, j5);
      const isReachable = ws.safe;

      return {
        index: idx,
        wp,
        robotTcp: { x: fk.tcp.x, y: fk.tcp.y, z: fk.tcp.z },
        threePos,
        rotation,
        isReachable
      };
    });

    // 2. Render Waypoint Node Spheres and Rings
    this._renderWaypointNodes();
    this._renderOrientationTriads();

    // 3. Build Smooth Catmull-Rom 3D Spline Tube for full path
    if (this.waypointPositions.length >= 2) {
      this._buildSplineTube(0, isLoop);
    }

    // 4. Build MOVL straight-line overlays on top of spline tube
    this._buildMovlOverlays();
  }

  /**
   * Renders waypoint node spheres and reference rings with density sub-sampling
   * @private
   */
  _renderWaypointNodes() {
    const total = this.waypointPositions.length;
    if (total === 0) return;

    const sphereGeo = new THREE.SphereGeometry(this.options.nodeRadius, 16, 16);
    const dropGeo = new THREE.RingGeometry(2.0, 3.0, 24);
    const dropMat = new THREE.MeshBasicMaterial({
      color: 0x64748b,
      transparent: true,
      opacity: 0.25,
      side: THREE.DoubleSide,
      depthWrite: false
    });

    // Density sub-sampling step
    const maxNodes = this.options.maxRenderedNodes || 48;
    const step = total <= maxNodes ? 1 : Math.ceil(total / maxNodes);

    this.waypointPositions.forEach((item, idx) => {
      // Always include start, end, tool transitions, and modulo step nodes
      const isStartOrEnd = (idx === 0 || idx === total - 1);
      const isSampled = (idx % step === 0);
      const prevWp = idx > 0 ? this.waypointPositions[idx - 1].wp : null;
      const isToolToggle = prevWp && (Number(item.wp.grip) !== Number(prevWp.grip));

      if (!isStartOrEnd && !isSampled && !isToolToggle) {
        return;
      }

      const isSafe = item.isReachable;
      const color = isSafe ? PATH_COLORS.SAFE : PATH_COLORS.COLLISION;

      const nodeMat = new THREE.MeshStandardMaterial({
        color,
        emissive: color,
        emissiveIntensity: 0.35,
        roughness: 0.3,
        metalness: 0.7
      });

      const sphere = new THREE.Mesh(sphereGeo, nodeMat);
      sphere.position.copy(item.threePos);
      sphere.userData = { isWaypointNode: true, waypointIndex: item.index, waypointId: item.wp.id };
      this.nodesGroup.add(sphere);

      // Floor projection drop ring
      const dropRing = new THREE.Mesh(dropGeo, dropMat);
      dropRing.rotation.x = -Math.PI / 2;
      dropRing.position.set(item.threePos.x, 0.2, item.threePos.z);
      dropRing.userData = { isWaypointNode: true, waypointIndex: item.index, waypointId: item.wp.id };
      this.nodesGroup.add(dropRing);
    });
  }

  /** Renders the requested TCP axes at every keyframe, including sampled-out nodes. */
  _renderOrientationTriads() {
    const length = this.options.triadLength;
    const colors = [[1, 0.25, 0.25], [0.25, 1, 0.4], [0.3, 0.55, 1]];
    for (const item of this.waypointPositions) {
      const vertices = [];
      const vertexColors = [];
      for (let axis = 0; axis < 3; axis++) {
        const direction = robotToThreeCoords(
          item.rotation[0][axis], item.rotation[1][axis], item.rotation[2][axis]);
        vertices.push(0, 0, 0, direction.x * length, direction.y * length, direction.z * length);
        vertexColors.push(...colors[axis], ...colors[axis]);
      }
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
      geometry.setAttribute('color', new THREE.Float32BufferAttribute(vertexColors, 3));
      const axes = new THREE.LineSegments(geometry, new THREE.LineBasicMaterial({ vertexColors: true, depthTest: false }));
      axes.position.copy(item.threePos);
      axes.userData = { waypointIndex: item.index, waypointId: item.wp.id, frame: item.wp.frame };
      this.triadsGroup.add(axes);
    }
  }

  /**
   * Constructs the Catmull-Rom 3D Spline Curve and TubeGeometry from a starting index
   * @param {number} [startIndex=0] - Starting waypoint index
   * @param {boolean} [isLoop=false] - Whether trajectory loops back to start
   * @private
   */
  _buildSplineTube(startIndex = 0, isLoop = false) {
    this._clearTubeGroup();

    const remainingItems = this.waypointPositions.slice(Math.max(0, startIndex));
    if (remainingItems.length < 2) {
      return;
    }

    const rawPoints = remainingItems.map((p) => p.threePos.clone());
    const loopFlag = (startIndex === 0) ? isLoop : false;
    
    // Create Catmull-Rom 3D curve with gentle tension (0.2)
    const curve = new THREE.CatmullRomCurve3(rawPoints, loopFlag, 'catmullrom', 0.2);

    // Calculate total sample count with upper safety limit
    const segmentCount = loopFlag ? rawPoints.length : rawPoints.length - 1;
    const requestedSegments = Math.max(32, segmentCount * this.options.samplesPerSegment);
    const tubularSegments = Math.min(this.options.maxTubularSegments || 320, requestedSegments);

    // Create 3D Tube Geometry
    const tubeGeo = new THREE.TubeGeometry(
      curve,
      tubularSegments,
      this.options.tubeRadius,
      8,
      loopFlag
    );

    // Associate each path section with its commanded TCP speed and joint-rate proxy.
    const routeSegments = [];
    const profileSegmentCount = loopFlag ? rawPoints.length : rawPoints.length - 1;
    let totalProfileDistance = 0;
    for (let i = 0; i < profileSegmentCount; i++) {
      const from = remainingItems[i];
      const to = remainingItems[(i + 1) % remainingItems.length];
      const distance = from.threePos.distanceTo(to.threePos);
      const durationSeconds = Math.max(1, Number(to.wp.dur ?? to.wp.duration ?? 1000)) / 1000;
      const jointRate = Math.max(...['j1', 'j2', 'j3', 'j4', 'j5', 'j6'].map((joint) =>
        Math.abs(Number(to.wp[joint] ?? 0) - Number(from.wp[joint] ?? 0)) / durationSeconds
      ));
      routeSegments.push({
        distance,
        speed: distance / durationSeconds,
        jointRate,
        startDistance: totalProfileDistance
      });
      totalProfileDistance += distance;
    }

    // Sample along the curve and color cruise, speed transitions, and safety risk.
    const colors = [];
    const colorCruise = new THREE.Color(PATH_COLORS.CRUISE);
    const colorCollision = new THREE.Color(PATH_COLORS.COLLISION);
    const colorWarning = new THREE.Color(PATH_COLORS.WARNING);

    const pointCount = tubularSegments + 1;
    const sampleSafety = new Array(pointCount);

    for (let i = 0; i < pointCount; i++) {
      const u = i / tubularSegments;
      const ptThree = curve.getPointAt(u);
      const ptRobot = threeToRobotCoords(ptThree.x, ptThree.y, ptThree.z);

      const floorBreach = (ptRobot.z <= FLOOR_CLR);
      const planarDist = Math.hypot(ptRobot.x, ptRobot.y);
      const exclusionBreach = (planarDist <= EXCL_R && ptRobot.z <= BASE_H);

      const ik = solveIK(ptRobot.x, ptRobot.y, ptRobot.z);
      const ikUnreachable = !ik.success;
      const ikClamped = Boolean(ik.clamped);

      const pathDistance = u * totalProfileDistance;
      let segmentIndex = 0;
      while (segmentIndex < routeSegments.length - 1 && pathDistance >= routeSegments[segmentIndex].startDistance + routeSegments[segmentIndex].distance) {
        segmentIndex++;
      }
      const segment = routeSegments[segmentIndex];
      const localProgress = segment && segment.distance > 0
        ? THREE.MathUtils.clamp((pathDistance - segment.startDistance) / segment.distance, 0, 1)
        : 0;
      const previousSegment = segmentIndex > 0 ? routeSegments[segmentIndex - 1] : (loopFlag ? routeSegments[routeSegments.length - 1] : null);
      const nextSegment = segmentIndex < routeSegments.length - 1 ? routeSegments[segmentIndex + 1] : (loopFlag ? routeSegments[0] : null);
      const speedChangeBefore = previousSegment
        ? Math.abs(segment.speed - previousSegment.speed) / Math.max(segment.speed, previousSegment.speed, 1)
        : 0;
      const speedChangeAfter = nextSegment
        ? Math.abs(nextSegment.speed - segment.speed) / Math.max(nextSegment.speed, segment.speed, 1)
        : 0;
      const inSpeedTransition = (speedChangeBefore > 0.25 && localProgress < 0.2)
        || (speedChangeAfter > 0.25 && localProgress > 0.8);
      const highJointRate = segment && segment.jointRate > 120;

      let vertexColor = inSpeedTransition ? colorWarning : colorCruise;
      if (floorBreach || exclusionBreach || ikUnreachable || ik.isNearBoundary || highJointRate) {
        vertexColor = colorCollision;
      } else if (ikClamped) {
        vertexColor = colorWarning;
      }

      sampleSafety[i] = vertexColor;
    }

    const radialSegments = 8;
    const radialVertCount = radialSegments + 1;

    for (let i = 0; i <= tubularSegments; i++) {
      const segmentColor = sampleSafety[i];
      for (let j = 0; j < radialVertCount; j++) {
        colors.push(segmentColor.r, segmentColor.g, segmentColor.b);
      }
    }

    tubeGeo.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));

    const tubeMat = new THREE.MeshStandardMaterial({
      vertexColors: true,
      roughness: 0.3,
      metalness: 0.5,
      emissive: 0x111317,
      emissiveIntensity: 0.15
    });

    const tubeMesh = new THREE.Mesh(tubeGeo, tubeMat);
    tubeMesh.name = 'TrajectorySplineTube';
    this.tubeGroup.add(tubeMesh);

    const glowGeo = new THREE.TubeGeometry(
      curve,
      tubularSegments,
      Math.max(this.options.tubeRadius * 2.5, 3.2),
      8,
      loopFlag
    );
    glowGeo.setAttribute('color', new THREE.Float32BufferAttribute(colors.slice(), 3));
    const glowMesh = new THREE.Mesh(glowGeo, new THREE.MeshBasicMaterial({
      vertexColors: true,
      transparent: true,
      opacity: 0.16,
      blending: THREE.AdditiveBlending,
      depthWrite: false
    }));
    glowMesh.name = 'TrajectorySplineGlow';
    glowMesh.renderOrder = 1;
    glowMesh.visible = this.glowEnabled;
    this.glowGroup.add(glowMesh);
  }

  /**
   * Renders CAD-grade straight-line overlays for waypoints with moveType === 'movl'.
   * Draws a flat bright-white LineSegments mesh directly between successive MOVL
   * TCP positions so operators can visually distinguish Cartesian linear moves
   * from joint-space arcs rendered by the spline tube.
   * @private
   */
  _buildMovlOverlays() {
    this._clearLinMovlGroup();

    if (this.waypointPositions.length < 2) return;

    const linePoints = [];
    for (let i = 1; i < this.waypointPositions.length; i++) {
      const curr = this.waypointPositions[i];
      // 'movl' or 'MOVL' on the waypoint marks a Cartesian linear segment
      const moveType = (curr.wp.moveType || curr.wp.motionType || '').toLowerCase();
      if (moveType === 'movl' || moveType === 'linear') {
        const prev = this.waypointPositions[i - 1];
        linePoints.push(prev.threePos.clone(), curr.threePos.clone());
      }
    }

    if (linePoints.length === 0) return;

    const geo = new THREE.BufferGeometry().setFromPoints(linePoints);
    const mat = new THREE.LineBasicMaterial({
      color: 0xffffff,
      linewidth: 2,
      depthTest: true,
      transparent: true,
      opacity: 0.75
    });
    const lines = new THREE.LineSegments(geo, mat);
    lines.name = 'TrajectoryMOVL_Linear_Overlay';
    this.linMovlGroup.add(lines);
  }

  /**
   * Clears MOVL straight-line overlay group, disposing geometry and materials.
   * @private
   */
  _clearLinMovlGroup() {
    while (this.linMovlGroup.children.length > 0) {
      const child = this.linMovlGroup.children[0];
      this.linMovlGroup.remove(child);
      if (child.geometry && typeof child.geometry.dispose === 'function') {
        child.geometry.dispose();
      }
      if (child.material) {
        if (Array.isArray(child.material)) {
          child.material.forEach(m => m && typeof m.dispose === 'function' && m.dispose());
        } else if (typeof child.material.dispose === 'function') {
          child.material.dispose();
        }
      }
    }
  }

  /**
   * Sets the active waypoint index during sequence playback
   * @param {number} index - Active waypoint index (-1 for none/idle/stopped)
   */
  setActiveWaypoint(index) {
    this.activeWaypointIndex = index;

    if (index >= 0 && index < this.waypointPositions.length) {
      const target = this.waypointPositions[index];
      this.activeCursor.position.copy(target.threePos);
      this.activeCursor.position.y += 0.5;
      this.activeCursor.visible = true;

      if (this.options.hidePastWaypoints) {
        this.nodesGroup.children.forEach((child) => {
          if (child.userData && typeof child.userData.waypointIndex === 'number') {
            child.visible = (child.userData.waypointIndex >= index);
          }
        });

        this._buildSplineTube(index, false);
      }
    } else {
      this.activeCursor.visible = false;
      this.nodesGroup.children.forEach((child) => {
        child.visible = true;
      });
      const isLoop = Boolean(this.currentSequence && this.currentSequence.loop);
      this._buildSplineTube(0, isLoop);
    }
  }

  /**
   * Toggles visibility of the 3D trajectory path and waypoints
   * @param {boolean} visible
   */
  setVisible(visible) {
    this.root.visible = Boolean(visible);
  }

  setGlowEnabled(enabled) {
    this.glowEnabled = Boolean(enabled);
    this.glowGroup.visible = this.glowEnabled;
  }

  /**
   * Cleans up tube meshes and geometry in tubeGroup
   * @private
   */
  _clearTubeGroup() {
    [this.tubeGroup, this.glowGroup].forEach((group) => {
      while (group.children.length > 0) {
        const child = group.children[0];
        group.remove(child);
        if (child.geometry && typeof child.geometry.dispose === 'function') {
          child.geometry.dispose();
        }
        if (child.material) {
          if (Array.isArray(child.material)) {
            child.material.forEach(m => m && typeof m.dispose === 'function' && m.dispose());
          } else if (typeof child.material.dispose === 'function') {
            child.material.dispose();
          }
        }
      }
    });
  }

  /**
   * Cleans up all child meshes, geometries, and materials
   * @private
   */
  _clearPreviousMeshes() {
    this._clearTubeGroup();
    this._clearLinMovlGroup();

    while (this.triadsGroup.children.length > 0) {
      const child = this.triadsGroup.children[0];
      this.triadsGroup.remove(child);
      if (child.geometry && typeof child.geometry.dispose === 'function') child.geometry.dispose();
      if (Array.isArray(child.material)) {
        child.material.forEach(material => material && typeof material.dispose === 'function' && material.dispose());
      } else if (child.material && typeof child.material.dispose === 'function') {
        child.material.dispose();
      }
    }

    while (this.nodesGroup.children.length > 0) {
      const child = this.nodesGroup.children[0];
      this.nodesGroup.remove(child);
      if (child.geometry && typeof child.geometry.dispose === 'function') {
        child.geometry.dispose();
      }
      if (child.material) {
        if (Array.isArray(child.material)) {
          child.material.forEach((m) => m && typeof m.dispose === 'function' && m.dispose());
        } else if (typeof child.material.dispose === 'function') {
          child.material.dispose();
        }
      }
    }
  }

  /**
   * Fully disposes all WebGL memory and geometry allocations
   */
  dispose() {
    this._clearPreviousMeshes();
    if (this.activeCursor) {
      if (this.activeCursor.geometry) this.activeCursor.geometry.dispose();
      if (this.activeCursor.material) this.activeCursor.material.dispose();
    }
    this.waypointPositions = [];
    this.currentSequence = null;
    this._lastSeqKey = '';
  }
}

export default TrajectoryVisualizer3D;
