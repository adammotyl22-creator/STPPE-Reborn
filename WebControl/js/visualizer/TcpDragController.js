/**
 * TcpDragController.js - Interactive 3D TCP Drag & Transform Gizmo Controller
 * 
 * Provides an interactive CAD 3-Axis Transform Gizmo directly at the TCP:
 *   - Linear Axis Arrows (X=Red, Y=Green, Z=Blue) for single-axis dragging.
 *   - Planar Quads (XY, XZ, YZ) for 2D constrained planar dragging.
 *   - Rotation Arcs (Wrist Pitch & Roll) for intuitive orientation tweaking.
 *   - Center Sphere for omnidirectional / camera-plane dragging with Shift/Alt modifiers.
 *   - Snap to Grid (10mm) & Virtual Table Surface (Z=20mm) snapping support.
 */

import * as THREE from '../../vendor/three/three.module.js';
import { threeToRobotCoords, robotToThreeCoords } from '../kinematics/RobotKinematics.js';

// Axis constraint modes
export const AxisConstraint = Object.freeze({
  NONE: 'none',
  X:    'x',     // Linear Robot X
  Y:    'y',     // Linear Robot Y
  Z:    'z',     // Linear Robot Z (or Alt key)
  XY:   'xy',    // Robot XY plane (Shift key)
  XZ:   'xz',    // Robot XZ plane
  YZ:   'yz',    // Robot YZ plane
  PITCH:'pitch', // Wrist pitch rotation
  ROLL: 'roll'   // Wrist roll rotation
});

export class TcpDragController {
  /**
   * @param {ThreeScene} threeScene - Active ThreeScene instance
   * @param {RobotModel} robotModel - Procedural RobotModel instance
   * @param {Function} [onDrag] - Callback fired during dragging: (robotCoords) => void
   * @param {Function} [onDragEnd] - Callback fired when drag ends
   */
  constructor(threeScene, robotModel, onDrag = null, onDragEnd = null) {
    this.threeScene = threeScene;
    this.robotModel = robotModel;
    this.onDrag = onDrag;
    this.onDragEnd = onDragEnd;

    this.raycaster = new THREE.Raycaster();
    this.mouse = new THREE.Vector2();
    this.isDragging = false;
    this.draggedMesh = null;
    this.dragPlane = new THREE.Plane();
    this.planeIntersectPoint = new THREE.Vector3();
    this.tcpWorldPos = new THREE.Vector3();
    this.dragStartWorldPos = new THREE.Vector3();
    this.dragStartPointer = new THREE.Vector2();

    // Axis constraint state
    this._axisConstraint = AxisConstraint.NONE;
    this._frozenRobotCoords = null; // frozen snapshot for constraint axes
    this._activeHandleType = null; // 'arrow', 'plane', 'arc', 'sphere'

    // Snapping configuration
    this.snapEnabled = false;
    this.snapGrid = 10.0; // 10mm increment

    // 3D Transform Gizmo Triad Group
    this.gizmoGroup = new THREE.Group();
    this.gizmoGroup.name = 'tcpTransformGizmo';
    this._gizmoMeshes = [];
    this._hoveredGizmo = null;
    this._dragOffset = new THREE.Vector3();

    this._buildGizmo();
    if (this.threeScene && this.threeScene.scene) {
      this.threeScene.scene.add(this.gizmoGroup);
    }
    this.updateGizmoPosition();

    this._boundOnPointerDown = this._onPointerDown.bind(this);
    this._boundOnPointerMove = this._onPointerMove.bind(this);
    this._boundOnPointerUp = this._onPointerUp.bind(this);
    this._boundOnKeyDown = this._onKeyDown.bind(this);
    this._boundOnKeyUp = this._onKeyUp.bind(this);

    this._initEvents();
  }

  // ==========================================================================
  // GIZMO TRIAD BUILDER
  // ==========================================================================

  _buildGizmo() {
    // Colors matching standard CAD: Red=X, Green=Y, Blue=Z
    const colX = 0xef4444; // Red (Robot X -> Three X)
    const colY = 0x10b981; // Green (Robot Y -> Three -Z)
    const colZ = 0x3b82f6; // Blue (Robot Z -> Three +Y)

    const arrowLength = 40.0;
    const shaftRadius = 1.4;
    const coneRadius = 3.6;
    const coneHeight = 9.0;
    const hitTargetRadius = 6.0;

    // Helper to build a linear arrow
    const createArrow = (axisName, color, dirVector) => {
      const arrowGroup = new THREE.Group();
      arrowGroup.userData = { isGizmoHandle: true, handleType: 'arrow', axis: axisName };

      const mat = new THREE.MeshStandardMaterial({
        color,
        emissive: color,
        emissiveIntensity: 0.25,
        roughness: 0.3,
        metalness: 0.5
      });

      // Shaft
      const shaftGeo = new THREE.CylinderGeometry(shaftRadius, shaftRadius, arrowLength - coneHeight, 12);
      const shaftMesh = new THREE.Mesh(shaftGeo, mat);
      shaftMesh.position.y = (arrowLength - coneHeight) / 2;
      arrowGroup.add(shaftMesh);

      // Cone tip
      const coneGeo = new THREE.ConeGeometry(coneRadius, coneHeight, 16);
      const coneMesh = new THREE.Mesh(coneGeo, mat);
      coneMesh.position.y = arrowLength - coneHeight / 2;
      arrowGroup.add(coneMesh);

      // Invisible fat hit target for effortless mouse picking
      const hitGeo = new THREE.CylinderGeometry(hitTargetRadius, hitTargetRadius, arrowLength + 2, 8);
      const hitMat = new THREE.MeshBasicMaterial({ visible: false });
      const hitMesh = new THREE.Mesh(hitGeo, hitMat);
      hitMesh.position.y = arrowLength / 2;
      hitMesh.userData = { isGizmoHandle: true, handleType: 'arrow', axis: axisName, visualMesh: arrowGroup, baseMat: mat, baseColor: color };
      arrowGroup.add(hitMesh);
      this._gizmoMeshes.push(hitMesh);

      // Orient arrow towards dirVector (default geometry is along +Y)
      if (dirVector.x !== 0) {
        arrowGroup.rotation.z = -Math.PI / 2 * Math.sign(dirVector.x);
      } else if (dirVector.z !== 0) {
        arrowGroup.rotation.x = Math.PI / 2 * Math.sign(dirVector.z);
      }
      // dirVector along +Y requires no extra rotation

      return arrowGroup;
    };

    // 1. Linear Axis Arrows:
    // Robot X -> Three +X (1, 0, 0)
    const arrowX = createArrow('x', colX, new THREE.Vector3(1, 0, 0));
    this.gizmoGroup.add(arrowX);

    // Robot Y -> Three -Z (0, 0, -1)
    const arrowY = createArrow('y', colY, new THREE.Vector3(0, 0, -1));
    this.gizmoGroup.add(arrowY);

    // Robot Z -> Three +Y (0, 1, 0)
    const arrowZ = createArrow('z', colZ, new THREE.Vector3(0, 1, 0));
    this.gizmoGroup.add(arrowZ);

    // 2. Planar Quads (2D dragging):
    // Helper to create small planar square quad
    const createQuad = (planeName, color, pos, normalDir) => {
      const quadGroup = new THREE.Group();
      quadGroup.userData = { isGizmoHandle: true, handleType: 'plane', plane: planeName };

      const quadGeo = new THREE.PlaneGeometry(12, 12);
      const quadMat = new THREE.MeshStandardMaterial({
        color,
        emissive: color,
        emissiveIntensity: 0.2,
        transparent: true,
        opacity: 0.45,
        side: THREE.DoubleSide,
        depthWrite: false
      });

      const quadMesh = new THREE.Mesh(quadGeo, quadMat);
      quadMesh.position.copy(pos);
      if (normalDir.y !== 0) {
        quadMesh.rotation.x = Math.PI / 2;
      } else if (normalDir.x !== 0) {
        quadMesh.rotation.y = Math.PI / 2;
      }

      quadMesh.userData = { isGizmoHandle: true, handleType: 'plane', plane: planeName, visualMesh: quadMesh, baseMat: quadMat, baseColor: color };
      quadGroup.add(quadMesh);
      this._gizmoMeshes.push(quadMesh);
      return quadGroup;
    };

    // Robot XY plane: Three X, -Z plane (normal = Three +Y)
    const quadXY = createQuad('xy', 0xf59e0b, new THREE.Vector3(14, 0, -14), new THREE.Vector3(0, 1, 0));
    this.gizmoGroup.add(quadXY);

    // Robot XZ plane: Three X, +Y plane (normal = Three -Z)
    const quadXZ = createQuad('xz', 0x06b6d4, new THREE.Vector3(14, 14, 0), new THREE.Vector3(0, 0, -1));
    this.gizmoGroup.add(quadXZ);

    // Robot YZ plane: Three -Z, +Y plane (normal = Three +X)
    const quadYZ = createQuad('yz', 0xec4899, new THREE.Vector3(0, 14, -14), new THREE.Vector3(1, 0, 0));
    this.gizmoGroup.add(quadYZ);

    // 3. Rotation Arcs (Pitch & Roll):
    const createRotationArc = (rotateName, color, normalAxis) => {
      const arcGeo = new THREE.TorusGeometry(26, 1.2, 8, 24, Math.PI / 2);
      const arcMat = new THREE.MeshStandardMaterial({
        color,
        emissive: color,
        emissiveIntensity: 0.3,
        transparent: true,
        opacity: 0.65
      });
      const arcMesh = new THREE.Mesh(arcGeo, arcMat);
      if (normalAxis === 'pitch') {
        arcMesh.rotation.y = Math.PI / 4;
      } else {
        arcMesh.rotation.x = Math.PI / 2;
      }

      // Slightly thicker pickable target
      const hitGeo = new THREE.TorusGeometry(26, 3.5, 8, 16, Math.PI / 2);
      const hitMat = new THREE.MeshBasicMaterial({ visible: false });
      const hitMesh = new THREE.Mesh(hitGeo, hitMat);
      hitMesh.rotation.copy(arcMesh.rotation);
      hitMesh.userData = { isGizmoHandle: true, handleType: 'arc', rotate: rotateName, visualMesh: arcMesh, baseMat: arcMat, baseColor: color };
      arcMesh.add(hitMesh);
      this._gizmoMeshes.push(hitMesh);

      return arcMesh;
    };

    const arcPitch = createRotationArc('pitch', 0x14b8a6, 'pitch');
    this.gizmoGroup.add(arcPitch);

    const arcRoll = createRotationArc('roll', 0xa855f7, 'roll');
    this.gizmoGroup.add(arcRoll);

    this.gizmoGroup.visible = true;
  }

  // ==========================================================================
  // EVENT LISTENERS & RAYCASTING
  // ==========================================================================

  _initEvents() {
    if (!this.threeScene || !this.threeScene.renderer || !this.threeScene.renderer.domElement) return;
    const dom = this.threeScene.renderer.domElement;

    dom.addEventListener('pointerdown', this._boundOnPointerDown, { passive: false });
    dom.addEventListener('pointermove', this._boundOnPointerMove, { passive: false });
    window.addEventListener('pointerup', this._boundOnPointerUp, { passive: false });
    window.addEventListener('pointercancel', this._boundOnPointerUp, { passive: false });
    window.addEventListener('keydown', this._boundOnKeyDown, { passive: true });
    window.addEventListener('keyup', this._boundOnKeyUp, { passive: true });
  }

  _updateMouseCoords(e) {
    const rect = this.threeScene.renderer.domElement.getBoundingClientRect();
    this.mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    this.mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
  }

  /**
   * Aggregates all interactive objects: gizmo handles + robot TCP spheres
   */
  _getPickableObjects() {
    const pickables = [...this._gizmoMeshes];
    if (this.robotModel) {
      if (this.robotModel.ghostArm) {
        const ghost = this.robotModel.ghostArm;
        if (ghost.tcp) {
          ghost.tcp.traverse(child => {
            if (child.isMesh) pickables.push(child);
          });
        }
        if (ghost.j5) {
          ghost.j5.traverse(child => {
            if (child.isMesh && !pickables.includes(child)) pickables.push(child);
          });
        }
        if (ghost.j4) {
          ghost.j4.traverse(child => {
            if (child.isMesh && !pickables.includes(child)) pickables.push(child);
          });
        }
      }
      if (this.robotModel.liveArm && (!this.robotModel.ghostArm || !this.robotModel.isGhostVisible)) {
        const live = this.robotModel.liveArm;
        if (live.tcp) {
          live.tcp.traverse(child => {
            if (child.isMesh && !pickables.includes(child)) pickables.push(child);
          });
        }
        if (live.j5) {
          live.j5.traverse(child => {
            if (child.isMesh && !pickables.includes(child)) pickables.push(child);
          });
        }
      }
    }
    return pickables;
  }

  /**
   * Resolves constraint mode from clicked gizmo handle or keyboard modifiers
   */
  _resolveConstraint(e, handleUserData = null) {
    if (handleUserData && handleUserData.isGizmoHandle) {
      if (handleUserData.handleType === 'arrow') {
        if (handleUserData.axis === 'x') return AxisConstraint.X;
        if (handleUserData.axis === 'y') return AxisConstraint.Y;
        if (handleUserData.axis === 'z') return AxisConstraint.Z;
      } else if (handleUserData.handleType === 'plane') {
        if (handleUserData.plane === 'xy') return AxisConstraint.XY;
        if (handleUserData.plane === 'xz') return AxisConstraint.XZ;
        if (handleUserData.plane === 'yz') return AxisConstraint.YZ;
      } else if (handleUserData.handleType === 'arc') {
        if (handleUserData.rotate === 'pitch') return AxisConstraint.PITCH;
        if (handleUserData.rotate === 'roll') return AxisConstraint.ROLL;
      }
    }

    // Fallback to keyboard modifiers if dragging center sphere
    if (e.shiftKey) return AxisConstraint.XY;
    if (e.altKey)   return AxisConstraint.Z;
    return AxisConstraint.NONE;
  }

  /**
   * Sets up drag projection plane normal based on constraint mode
   */
  _updateDragPlane(anchorWorldPos) {
    const normal = new THREE.Vector3();

    if (this._axisConstraint === AxisConstraint.X) {
      const camDir = new THREE.Vector3();
      this.threeScene.camera.getWorldDirection(camDir);
      normal.set(0, camDir.y, camDir.z).normalize();
      if (normal.lengthSq() < 1e-4) normal.set(0, 1, 0);
      this.dragPlane.setFromNormalAndCoplanarPoint(normal, anchorWorldPos);
    } else if (this._axisConstraint === AxisConstraint.Y) {
      const camDir = new THREE.Vector3();
      this.threeScene.camera.getWorldDirection(camDir);
      normal.set(camDir.x, camDir.y, 0).normalize();
      if (normal.lengthSq() < 1e-4) normal.set(0, 1, 0);
      this.dragPlane.setFromNormalAndCoplanarPoint(normal, anchorWorldPos);
    } else if (this._axisConstraint === AxisConstraint.Z) {
      const camFwd = new THREE.Vector3();
      this.threeScene.camera.getWorldDirection(camFwd);
      normal.crossVectors(new THREE.Vector3(0, 1, 0), camFwd).normalize();
      if (normal.lengthSq() < 1e-4) normal.set(1, 0, 0);
      this.dragPlane.setFromNormalAndCoplanarPoint(normal, anchorWorldPos);
    } else if (this._axisConstraint === AxisConstraint.XY) {
      this.dragPlane.setFromNormalAndCoplanarPoint(new THREE.Vector3(0, 1, 0), anchorWorldPos);
    } else if (this._axisConstraint === AxisConstraint.XZ) {
      this.dragPlane.setFromNormalAndCoplanarPoint(new THREE.Vector3(0, 0, 1), anchorWorldPos);
    } else if (this._axisConstraint === AxisConstraint.YZ) {
      this.dragPlane.setFromNormalAndCoplanarPoint(new THREE.Vector3(1, 0, 0), anchorWorldPos);
    } else {
      const camDir = new THREE.Vector3();
      this.threeScene.camera.getWorldDirection(camDir);
      if (Math.abs(camDir.y) > 0.15) {
        // Elevated camera: use horizontal ground plane (Three Y = Robot Z)
        // This keeps Z stable and allows natural arm extension/retraction across the table
        this.dragPlane.setFromNormalAndCoplanarPoint(new THREE.Vector3(0, 1, 0), anchorWorldPos);
      } else {
        // Near eye-level view: use camera-facing plane
        const camFacingNormal = camDir.clone().negate();
        this.dragPlane.setFromNormalAndCoplanarPoint(camFacingNormal, anchorWorldPos);
      }
    }
  }

  // ==========================================================================
  // POINTER HANDLERS
  // ==========================================================================

  _onPointerDown(e) {
    if (e.button !== 0) return; // Primary left button only

    this._updateMouseCoords(e);
    this.raycaster.setFromCamera(this.mouse, this.threeScene.camera);

    const pickables = this._getPickableObjects();
    const intersects = this.raycaster.intersectObjects(pickables, false);

    if (intersects.length > 0) {
      const hit = intersects[0];
      const mesh = hit.object;

      this.isDragging = true;
      this.draggedMesh = mesh;
      this.dragStartPointer.set(e.clientX, e.clientY);

      // Determine constraint mode
      this._axisConstraint = this._resolveConstraint(e, mesh.userData);

      // Disable camera OrbitControls while dragging
      if (this.threeScene.controls) {
        this.threeScene.controls.enabled = false;
      }

      // Anchor world position at Ghost TCP or Gizmo center
      if (this.robotModel && this.robotModel.ghostArm && this.robotModel.ghostArm.tcp) {
        this.robotModel.ghostArm.tcp.getWorldPosition(this.tcpWorldPos);
        this.gizmoGroup.position.copy(this.tcpWorldPos);
      } else {
        this.gizmoGroup.getWorldPosition(this.tcpWorldPos);
      }
      this.dragStartWorldPos.copy(this.tcpWorldPos);

      // Setup drag projection plane
      this._updateDragPlane(this.tcpWorldPos);

      // Store planar offset between raycast intersection and TCP anchor to prevent violent jumps
      const initialHitOnPlane = new THREE.Vector3();
      if (this.raycaster.ray.intersectPlane(this.dragPlane, initialHitOnPlane)) {
        this._dragOffset.copy(initialHitOnPlane).sub(this.tcpWorldPos);
      } else {
        this._dragOffset.copy(hit.point).sub(this.tcpWorldPos);
      }

      // Capture initial robot coords snapshot
      const robotSnap = threeToRobotCoords(this.tcpWorldPos.x, this.tcpWorldPos.y, this.tcpWorldPos.z);
      this._frozenRobotCoords = { x: robotSnap.x, y: robotSnap.y, z: robotSnap.z };

      // Highlight active gizmo handle
      if (mesh.userData && mesh.userData.visualMesh) {
        const v = mesh.userData.visualMesh;
        if (v.material && v.material.emissive) {
          v.material.emissiveIntensity = 0.9;
        }
      }

      document.body.style.cursor = 'grabbing';
      this._showConstraintHint(this._axisConstraint);

      e.preventDefault();
      e.stopPropagation();
    }
  }

  _onPointerMove(e) {
    this._updateMouseCoords(e);
    this.raycaster.setFromCamera(this.mouse, this.threeScene.camera);

    if (this.isDragging && this.draggedMesh) {
      // If dragging center sphere or robot mesh, allow live keyboard modifier updates
      if (!this.draggedMesh.userData.isGizmoHandle) {
        const newConstraint = this._resolveConstraint(e);
        if (newConstraint !== this._axisConstraint) {
          this._axisConstraint = newConstraint;
          this._updateDragPlane(this.tcpWorldPos);
          this._showConstraintHint(newConstraint);
        }
      }

      // Handle rotational arcs
      if (this._axisConstraint === AxisConstraint.PITCH || this._axisConstraint === AxisConstraint.ROLL) {
        const dy = e.clientY - this.dragStartPointer.y;
        const angleDelta = -dy * 0.5; // deg
        if (typeof this.onDrag === 'function' && this._frozenRobotCoords) {
          const out = { ...this._frozenRobotCoords };
          if (this._axisConstraint === AxisConstraint.PITCH) {
            out.pitchDelta = angleDelta;
          } else {
            out.rollDelta = angleDelta;
          }
          this.onDrag(out);
        }
        e.preventDefault();
        return;
      }

      // Handle linear / planar dragging
      if (this.raycaster.ray.intersectPlane(this.dragPlane, this.planeIntersectPoint)) {
        // Offset compensation: ensures grab point stays locked directly under cursor with zero jump
        const targetWorld = this.planeIntersectPoint.clone().sub(this._dragOffset);
        const robotCoords = threeToRobotCoords(
          targetWorld.x,
          targetWorld.y,
          targetWorld.z
        );

        let outX = robotCoords.x;
        let outY = robotCoords.y;
        let outZ = robotCoords.z;

        // Apply strict 1D / 2D axis constraints
        if (this._axisConstraint === AxisConstraint.X && this._frozenRobotCoords) {
          outY = this._frozenRobotCoords.y;
          outZ = this._frozenRobotCoords.z;
        } else if (this._axisConstraint === AxisConstraint.Y && this._frozenRobotCoords) {
          outX = this._frozenRobotCoords.x;
          outZ = this._frozenRobotCoords.z;
        } else if (this._axisConstraint === AxisConstraint.Z && this._frozenRobotCoords) {
          outX = this._frozenRobotCoords.x;
          outY = this._frozenRobotCoords.y;
        } else if (this._axisConstraint === AxisConstraint.XY && this._frozenRobotCoords) {
          outZ = this._frozenRobotCoords.z;
        } else if (this._axisConstraint === AxisConstraint.XZ && this._frozenRobotCoords) {
          outY = this._frozenRobotCoords.y;
        } else if (this._axisConstraint === AxisConstraint.YZ && this._frozenRobotCoords) {
          outX = this._frozenRobotCoords.x;
        }

        // Apply Surface & Grid Snapping if enabled
        if (this.snapEnabled) {
          if (this._axisConstraint === AxisConstraint.X || this._axisConstraint === AxisConstraint.XY || this._axisConstraint === AxisConstraint.XZ || this._axisConstraint === AxisConstraint.NONE) {
            outX = Math.round(outX / this.snapGrid) * this.snapGrid;
          }
          if (this._axisConstraint === AxisConstraint.Y || this._axisConstraint === AxisConstraint.XY || this._axisConstraint === AxisConstraint.YZ || this._axisConstraint === AxisConstraint.NONE) {
            outY = Math.round(outY / this.snapGrid) * this.snapGrid;
          }
          if (this._axisConstraint === AxisConstraint.Z || this._axisConstraint === AxisConstraint.XZ || this._axisConstraint === AxisConstraint.YZ || this._axisConstraint === AxisConstraint.NONE) {
            if (Math.abs(outZ - 20.0) < 6.0) {
              outZ = 20.0;
            } else {
              outZ = Math.round(outZ / this.snapGrid) * this.snapGrid;
            }
          }
        }

        // Working envelope reach clamping (limited to 430.0mm radial)
        const radialDist = Math.hypot(outX, outY);
        const maxReach = 430.0;
        if (radialDist > maxReach && radialDist > 1e-4) {
          const scale = maxReach / radialDist;
          outX *= scale;
          outY *= scale;
        }
        const clampedZ = Math.max(20.0, Math.min(555.0, outZ));

        // Update live Gizmo position to reflect clamped coordinates
        const threeClamped = robotToThreeCoords(outX, outY, clampedZ);
        this.tcpWorldPos.set(threeClamped.x, threeClamped.y, threeClamped.z);
        this.gizmoGroup.position.copy(this.tcpWorldPos);

        if (typeof this.onDrag === 'function') {
          this.onDrag({ x: outX, y: outY, z: clampedZ });
        }
      }
      e.preventDefault();
      return;
    }

    // Hover check
    const pickables = this._getPickableObjects();
    const intersects = this.raycaster.intersectObjects(pickables, false);

    const dom = this.threeScene.renderer.domElement;
    if (intersects.length > 0) {
      const hitObj = intersects[0].object;
      dom.style.cursor = hitObj.userData.isGizmoHandle ? 'pointer' : 'grab';
    } else {
      dom.style.cursor = 'default';
    }
  }

  _onPointerUp(e) {
    if (!this.isDragging) return;

    this.isDragging = false;
    this._hideConstraintHint();

    if (this.draggedMesh && this.draggedMesh.userData && this.draggedMesh.userData.visualMesh) {
      const v = this.draggedMesh.userData.visualMesh;
      if (v.material && v.material.emissive) {
        v.material.emissiveIntensity = 0.25;
      }
    }
    this.draggedMesh = null;
    this._axisConstraint = AxisConstraint.NONE;
    this._frozenRobotCoords = null;

    // Restore camera OrbitControls
    if (this.threeScene.controls) {
      this.threeScene.controls.enabled = true;
    }

    const dom = this.threeScene && this.threeScene.renderer ? this.threeScene.renderer.domElement : null;
    if (dom) dom.style.cursor = 'default';
    if (typeof document !== 'undefined' && document.body) document.body.style.cursor = '';

    if (typeof this.onDragEnd === 'function') {
      this.onDragEnd();
    }
  }

  _onKeyDown(e) {
    if (!this.isDragging || (this.draggedMesh && this.draggedMesh.userData.isGizmoHandle)) return;
    const newConstraint = this._resolveConstraint(e);
    if (newConstraint !== this._axisConstraint) {
      this._axisConstraint = newConstraint;
      this._updateDragPlane(this.tcpWorldPos);
      this._showConstraintHint(newConstraint);
    }
  }

  _onKeyUp(e) {
    if (!this.isDragging || (this.draggedMesh && this.draggedMesh.userData.isGizmoHandle)) return;
    const newConstraint = this._resolveConstraint(e);
    if (newConstraint !== this._axisConstraint) {
      this._axisConstraint = newConstraint;
      this._updateDragPlane(this.tcpWorldPos);
      this._showConstraintHint(newConstraint);
    }
  }

  // ==========================================================================
  // GIZMO TRACKING & SNAP API
  // ==========================================================================

  /**
   * Updates Gizmo position to match the robot's active TCP position
   * @param {THREE.Vector3} [worldPos]
   */
  updateGizmoPosition(worldPos = null) {
    if (this.isDragging) return;
    if (worldPos) {
      this.gizmoGroup.position.copy(worldPos);
      this.tcpWorldPos.copy(worldPos);
    } else if (this.robotModel && this.robotModel.ghostArm && this.robotModel.ghostArm.tcp) {
      this.robotModel.ghostArm.tcp.getWorldPosition(this.gizmoGroup.position);
      this.tcpWorldPos.copy(this.gizmoGroup.position);
    }
  }

  /**
   * Toggles or sets grid snapping
   * @param {boolean} [enable]
   */
  setSnapEnabled(enable = null) {
    this.snapEnabled = enable === null ? !this.snapEnabled : Boolean(enable);
    return this.snapEnabled;
  }

  isSnapEnabled() {
    return this.snapEnabled;
  }

  setSnapGrid(gridSizeMm) {
    this.snapGrid = Math.max(1.0, Number(gridSizeMm || 10.0));
  }

  getGizmo() {
    return this.gizmoGroup;
  }

  /**
   * Shows on-screen constraint hint overlay
   */
  _showConstraintHint(constraint) {
    let el = document.getElementById('tcp-drag-constraint-hint');
    if (!el && typeof document !== 'undefined') {
      el = document.createElement('div');
      el.id = 'tcp-drag-constraint-hint';
      el.style.cssText = [
        'position:fixed', 'bottom:80px', 'left:50%', 'transform:translateX(-50%)',
        'background:rgba(15,23,42,0.85)', 'color:#e2e8f0', 'font:600 12px/1.5 monospace',
        'padding:5px 14px', 'border-radius:6px', 'pointer-events:none',
        'border:1px solid rgba(255,255,255,0.15)', 'box-shadow:0 4px 12px rgba(0,0,0,0.5)',
        'z-index:9999', 'letter-spacing:0.04em', 'white-space:nowrap'
      ].join(';');
      document.body.appendChild(el);
    }
    if (!el) return;

    const snapSuffix = this.snapEnabled ? ` | [SNAP: ${this.snapGrid}mm]` : '';
    const labels = {
      [AxisConstraint.X]:    `[X-AXIS] Triad Locked (YZ frozen)${snapSuffix}`,
      [AxisConstraint.Y]:    `[Y-AXIS] Triad Locked (XZ frozen)${snapSuffix}`,
      [AxisConstraint.Z]:    `[Z-ELEV] Triad Locked (XY frozen)${snapSuffix}`,
      [AxisConstraint.XY]:   `[XY-PLANE] Triad Locked (Z frozen)${snapSuffix}`,
      [AxisConstraint.XZ]:   `[XZ-PLANE] Triad Locked (Y frozen)${snapSuffix}`,
      [AxisConstraint.YZ]:   `[YZ-PLANE] Triad Locked (X frozen)${snapSuffix}`,
      [AxisConstraint.PITCH]:`[WRIST PITCH] Rotation Arc`,
      [AxisConstraint.ROLL]: `[WRIST ROLL] Rotation Arc`,
      [AxisConstraint.NONE]: `[FREE] Omnidirectional Drag${snapSuffix}`
    };
    el.textContent = labels[constraint] || '';
    el.style.display = 'block';
  }

  _hideConstraintHint() {
    if (typeof document === 'undefined') return;
    const el = document.getElementById('tcp-drag-constraint-hint');
    if (el) el.style.display = 'none';
  }

  dispose() {
    if (this.threeScene && this.threeScene.renderer && this.threeScene.renderer.domElement) {
      const dom = this.threeScene.renderer.domElement;
      dom.removeEventListener('pointerdown', this._boundOnPointerDown);
      dom.removeEventListener('pointermove', this._boundOnPointerMove);
    }
    if (typeof window !== 'undefined') {
      window.removeEventListener('pointerup', this._boundOnPointerUp);
      window.removeEventListener('pointercancel', this._boundOnPointerUp);
      window.removeEventListener('keydown', this._boundOnKeyDown);
      window.removeEventListener('keyup', this._boundOnKeyUp);
    }
    if (this.gizmoGroup && this.threeScene && this.threeScene.scene) {
      this.threeScene.scene.remove(this.gizmoGroup);
    }
    this._hideConstraintHint();
  }
}

export default TcpDragController;
