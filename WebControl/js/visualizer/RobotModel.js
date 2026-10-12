/**
 * RobotModel.js - URDF-Based STL Mesh Robot Model
 * 
 * Loads real STL meshes from the stppe_zlozenie_nowe URDF description and
 * assembles them into an articulated 6-DOF robot arm.
 * 
 * 6-DOF Joint Chain:
 *   base_link (fixed)
 *   → joint_1 (base azimuth yaw around vertical axis)
 *   → joint_2 (shoulder pitch)
 *   → joint_3 (elbow pitch)
 *   → joint_4 (forearm roll)
 *   → joint_5 (wrist pitch)
 *   → joint_6 (tool roll)
 * 
 * Manages both Live Arm (Satin Metal PBR) and Ghost Target Arm (Translucent Holographic Amber).
 */

import * as THREE from '../../vendor/three/three.module.js';
import { STLLoader } from '../../vendor/three/STLLoader.js';

// Mesh file paths (relative to index.html)
const MESH_PATH = 'meshes/';
const MESH_FILES = [
  'base_link.stl',
  'link_1.stl',
  'link_2.stl',
  'link_3.stl',
  'link_4.stl',
  'link_5.stl',
  'link_6.stl'
];

// Pivot centers in CAD coordinates (mm), derived from URDF cumulative joint origins
const PIVOTS = {
  j1: new THREE.Vector3(-63.538, -188.851, 62.999),
  j2: new THREE.Vector3(-66.787, -197.703, 143.896),
  j3: new THREE.Vector3(-134.639, -154.402, 312.315),
  j4: new THREE.Vector3(-204.094, -137.683, 288.986),
  j5: new THREE.Vector3(-260.958, -82.089, 138.340),
  j6: new THREE.Vector3(-287.419, -106.614, 139.725)
};

// Rotation axes in CAD coordinates, defined in the URDF description
const AXES = {
  j1: new THREE.Vector3(0, 0, 1),
  j2: new THREE.Vector3(0.34202, 0.939693, 0).normalize(),
  j3: new THREE.Vector3(0.34202, 0.939693, 0).normalize(),
  j4: new THREE.Vector3(0.3971, -0.1445, 0.9063).normalize(),
  j5: new THREE.Vector3(0.34202, 0.939693, 0).normalize(),
  j6: new THREE.Vector3(0.9361, -0.3407, -0.0872).normalize()
};

export class RobotModel {
  /**
   * @param {Object} [options]
   * @param {boolean} [options.showGhost=true]
   * @param {boolean} [options.shadows=true]
   */
  constructor(options = {}) {
    this.options = Object.assign({
      showGhost: true,
      shadows: true
    }, options);

    // Pose state: 6 joints (j1-j6) matching URDF kinematic assembly.
    // Public API still accepts (j1, j2, j3, j4, j5, grip) for backward compatibility.
    this.livePose = { j1: 0.0, j2: 0.0, j3: 0.0, j4: 0.0, j5: 90.0, j6: 0.0 };
    this.targetLivePose = { j1: 0.0, j2: 0.0, j3: 0.0, j4: 0.0, j5: 90.0, j6: 0.0 };
    this.currentLivePose = { j1: 0.0, j2: 0.0, j3: 0.0, j4: 0.0, j5: 90.0, j6: 0.0 };
    this.ghostPose = { j1: 0.0, j2: 0.0, j3: 0.0, j4: 0.0, j5: 90.0, j6: 0.0 };

    this.root = new THREE.Group();
    this.root.name = 'robotModelRoot';
    this._meshesLoaded = false;
    this._initMaterials();

    this.liveArm = null;
    this.ghostArm = null;
    this._cachedGeometries = null;
  }

  get isGhostVisible() {
    return Boolean(this.options.showGhost);
  }

  _initMaterials() {
    this.materials = {
      // Live Arm: Satin Titanium / Slate Engineering PBR Palette
      metalBody: new THREE.MeshStandardMaterial({
        color: 0x94a3b8, // Satin Slate
        metalness: 0.40,
        roughness: 0.35
      }),
      metalDark: new THREE.MeshStandardMaterial({
        color: 0x334155, // Charcoal Titanium Accent
        metalness: 0.50,
        roughness: 0.30
      }),

      // Ghost Target Arm: Industrial Holographic Amber Palette
      ghostBody: new THREE.MeshStandardMaterial({
        color: 0xd97706,
        emissive: 0x78350f,
        emissiveIntensity: 0.25,
        transparent: true,
        opacity: 0.45,
        metalness: 0.1,
        roughness: 0.5,
        depthWrite: false,
        side: THREE.DoubleSide
      }),
      ghostAccent: new THREE.MeshStandardMaterial({
        color: 0xb45309,
        emissive: 0x451a03,
        emissiveIntensity: 0.3,
        transparent: true,
        opacity: 0.55,
        depthWrite: false
      })
    };
  }

  /**
   * Load all STL meshes and assemble the 3D articulated arms.
   * @param {Function} [onComplete]
   * @param {Function} [onProgress]
   * @returns {Promise<void>}
   */
  async loadMeshes(onComplete = null, onProgress = null) {
    const loader = new STLLoader();
    const geometries = {};
    let loaded = 0;
    const total = MESH_FILES.length;

    const loadPromises = MESH_FILES.map((filename) => {
      return new Promise((resolve, reject) => {
        const url = MESH_PATH + filename;
        loader.load(
          url,
          (geometry) => {
            const linkName = filename.replace('.stl', '');
            geometries[linkName] = geometry;
            loaded++;
            if (onProgress) onProgress(loaded, total);
            resolve();
          },
          undefined,
          (error) => {
            console.error(`[RobotModel] Error loading mesh: ${url}`, error);
            reject(error);
          }
        );
      });
    });

    await Promise.all(loadPromises);
    this._cachedGeometries = geometries;

    // 1. Build Live Arm Assembly
    this.liveArm = this._buildArmAssembly(geometries, false);
    this.root.add(this.liveArm.root);

    // 2. Build Ghost Target Arm Assembly
    this.ghostArm = this._buildArmAssembly(geometries, true);
    this.ghostArm.root.visible = this.options.showGhost;
    this.ghostArm.root.renderOrder = 10;
    this.ghostArm.root.traverse((obj) => {
      if (obj && obj.isMesh) obj.renderOrder = 10;
    });
    this.root.add(this.ghostArm.root);

    this._meshesLoaded = true;

    // Apply initial zero / home poses
    this._applyPoseToAssembly(this.liveArm, this.currentLivePose);
    this._applyPoseToAssembly(this.ghostArm, this.ghostPose);

    if (onComplete) onComplete();
  }

  /**
   * Constructs an articulated arm kinematic hierarchy.
   * Coordinates:
   *   The robot CAD assembly is oriented with Z-up. In Three.js, Y-up is the world vertical.
   *   Rotating armRoot by -90 deg around X aligns the robot vertically with the floor grid (Y=0).
   *   Centering offset (63.538, 188.851, 0) centers the base turntable at (0, 0, 0).
   * 
   * @param {Object} geometries - Map of link_name -> BufferGeometry
   * @param {boolean} isGhost - Whether this assembly is the ghost arm
   * @returns {Object} Kinematic references
   */
  _buildArmAssembly(geometries, isGhost = false) {
    const bodyMat = isGhost ? this.materials.ghostBody : this.materials.metalBody;
    const accentMat = isGhost ? this.materials.ghostAccent : this.materials.metalDark;

    // Master wrapper: maps CAD Z-up to Three.js Y-up
    const armRoot = new THREE.Group();
    armRoot.name = isGhost ? 'ghostArmRoot' : 'liveArmRoot';
    armRoot.rotation.x = -Math.PI / 2;

    // ── BASE GROUP (Fixed to floor grid) ───────────────────────────────────────
    // Rotates -180 deg around vertical CAD Z axis so the rectangular base enclosure
    // aligns straight along the Three.js -X axis, perfectly square with the workspace grid.
    const baseGroup = new THREE.Group();
    baseGroup.name = isGhost ? 'ghostBaseGroup' : 'liveBaseGroup';
    baseGroup.rotation.z = -Math.PI;
    armRoot.add(baseGroup);

    // Centering group: positions Joint 1 turntable axis at world X=0, Z=0
    const baseOffset = new THREE.Group();
    baseOffset.position.set(63.538, 188.851, 0);
    baseGroup.add(baseOffset);

    // ── BASE LINK ─────────────────────────────────────────────────────────────
    const baseMesh = new THREE.Mesh(geometries.base_link, bodyMat);
    baseMesh.name = isGhost ? 'ghost_base_link' : 'live_base_link';
    baseMesh.castShadow = !isGhost && this.options.shadows;
    baseMesh.receiveShadow = !isGhost && this.options.shadows;
    baseOffset.add(baseMesh);

    // ── ARTICULATING ARM CHAIN (Joint 1 through Joint 6) ──────────────────────
    // Alignment group: rotates -160 deg around vertical CAD Z axis
    // to align CAD motion plane (at 160 deg in CAD) with forward +X axis in Three.js
    const alignGroup = new THREE.Group();
    alignGroup.name = isGhost ? 'ghostAlignGroup' : 'liveAlignGroup';
    alignGroup.rotation.z = -160 * (Math.PI / 180);
    armRoot.add(alignGroup);

    // Centering group for arm: positions Joint 1 turntable axis at world X=0, Z=0
    const armOffset = new THREE.Group();
    armOffset.position.set(63.538, 188.851, 0);
    alignGroup.add(armOffset);

    // ── JOINT 1 → LINK 1 ──────────────────────────────────────────────────────
    const j1Group = new THREE.Group();
    j1Group.name = isGhost ? 'ghost_joint_1' : 'live_joint_1';
    j1Group.position.copy(PIVOTS.j1);
    armOffset.add(j1Group);

    const link1Mesh = new THREE.Mesh(geometries.link_1, accentMat);
    link1Mesh.name = isGhost ? 'ghost_link_1' : 'live_link_1';
    link1Mesh.position.copy(PIVOTS.j1).negate();
    link1Mesh.castShadow = !isGhost && this.options.shadows;
    link1Mesh.receiveShadow = !isGhost && this.options.shadows;
    j1Group.add(link1Mesh);

    // ── JOINT 2 → LINK 2 ──────────────────────────────────────────────────────
    const j2Group = new THREE.Group();
    j2Group.name = isGhost ? 'ghost_joint_2' : 'live_joint_2';
    j2Group.position.copy(PIVOTS.j2).sub(PIVOTS.j1);
    j1Group.add(j2Group);

    const link2Mesh = new THREE.Mesh(geometries.link_2, bodyMat);
    link2Mesh.name = isGhost ? 'ghost_link_2' : 'live_link_2';
    link2Mesh.position.copy(PIVOTS.j2).negate();
    link2Mesh.castShadow = !isGhost && this.options.shadows;
    link2Mesh.receiveShadow = !isGhost && this.options.shadows;
    j2Group.add(link2Mesh);

    // ── JOINT 3 → LINK 3 ──────────────────────────────────────────────────────
    const j3Group = new THREE.Group();
    j3Group.name = isGhost ? 'ghost_joint_3' : 'live_joint_3';
    j3Group.position.copy(PIVOTS.j3).sub(PIVOTS.j2);
    j2Group.add(j3Group);

    const link3Mesh = new THREE.Mesh(geometries.link_3, accentMat);
    link3Mesh.name = isGhost ? 'ghost_link_3' : 'live_link_3';
    link3Mesh.position.copy(PIVOTS.j3).negate();
    link3Mesh.castShadow = !isGhost && this.options.shadows;
    link3Mesh.receiveShadow = !isGhost && this.options.shadows;
    j3Group.add(link3Mesh);

    // ── JOINT 4 → LINK 4 ──────────────────────────────────────────────────────
    const j4Group = new THREE.Group();
    j4Group.name = isGhost ? 'ghost_joint_4' : 'live_joint_4';
    j4Group.position.copy(PIVOTS.j4).sub(PIVOTS.j3);
    j3Group.add(j4Group);

    const link4Mesh = new THREE.Mesh(geometries.link_4, bodyMat);
    link4Mesh.name = isGhost ? 'ghost_link_4' : 'live_link_4';
    link4Mesh.position.copy(PIVOTS.j4).negate();
    link4Mesh.castShadow = !isGhost && this.options.shadows;
    link4Mesh.receiveShadow = !isGhost && this.options.shadows;
    j4Group.add(link4Mesh);

    // ── JOINT 5 → LINK 5 ──────────────────────────────────────────────────────
    const j5Group = new THREE.Group();
    j5Group.name = isGhost ? 'ghost_joint_5' : 'live_joint_5';
    j5Group.position.copy(PIVOTS.j5).sub(PIVOTS.j4);
    j4Group.add(j5Group);

    const link5Mesh = new THREE.Mesh(geometries.link_5, accentMat);
    link5Mesh.name = isGhost ? 'ghost_link_5' : 'live_link_5';
    link5Mesh.position.copy(PIVOTS.j5).negate();
    link5Mesh.castShadow = !isGhost && this.options.shadows;
    link5Mesh.receiveShadow = !isGhost && this.options.shadows;
    j5Group.add(link5Mesh);

    // ── JOINT 6 → LINK 6 ──────────────────────────────────────────────────────
    const j6Group = new THREE.Group();
    j6Group.name = isGhost ? 'ghost_joint_6' : 'live_joint_6';
    j6Group.position.copy(PIVOTS.j6).sub(PIVOTS.j5);
    j5Group.add(j6Group);

    const link6Mesh = new THREE.Mesh(geometries.link_6, accentMat);
    link6Mesh.name = isGhost ? 'ghost_link_6' : 'live_link_6';
    link6Mesh.position.copy(PIVOTS.j6).negate();
    link6Mesh.castShadow = !isGhost && this.options.shadows;
    link6Mesh.receiveShadow = !isGhost && this.options.shadows;
    j6Group.add(link6Mesh);

    // ── TCP INDICATOR & HIT SPHERES ───────────────────────────────────────────
    const tcpGroup = new THREE.Group();
    tcpGroup.name = isGhost ? 'tcpGhost' : 'tcpLive';
    // Offset along tool axis to the outer flange mounting surface
    tcpGroup.position.copy(AXES.j6).multiplyScalar(-15);

    const crosshairGeo = new THREE.SphereGeometry(4, 16, 16);
    const crosshairMat = new THREE.MeshStandardMaterial({
      color: isGhost ? 0xd97706 : 0x94a3b8,
      metalness: 0.5,
      roughness: 0.3,
      emissive: isGhost ? 0xd97706 : 0x475569,
      emissiveIntensity: 0.2
    });
    const tcpMesh = new THREE.Mesh(crosshairGeo, crosshairMat);
    tcpMesh.name = isGhost ? 'tcpGhostSphere' : 'tcpLiveSphere';
    tcpMesh.userData = { isTcpSphere: true, isGhost };
    tcpGroup.add(tcpMesh);

    // Invisible hit sphere for mouse raycasting
    const hitGeo = new THREE.SphereGeometry(14, 16, 16);
    const hitMat = new THREE.MeshBasicMaterial({ visible: false });
    const hitMesh = new THREE.Mesh(hitGeo, hitMat);
    hitMesh.name = isGhost ? 'tcpGhostHitSphere' : 'tcpLiveHitSphere';
    hitMesh.userData = { isTcpSphere: true, isGhost, visualSphere: tcpMesh };
    tcpGroup.add(hitMesh);

    const tcpAxes = new THREE.AxesHelper(15);
    tcpGroup.add(tcpAxes);
    j6Group.add(tcpGroup);

    // Backward-compatibility finger stubs
    const leftFinger = new THREE.Group();
    const rightFinger = new THREE.Group();
    tcpGroup.add(leftFinger);
    tcpGroup.add(rightFinger);

    return {
      root: armRoot,
      baseOffset,
      baseLink: baseMesh,
      j1: j1Group,
      j2: j2Group,
      j3: j3Group,
      j4: j4Group,
      j5: j5Group,
      j6: j6Group,
      tcp: tcpGroup,
      leftFinger,
      rightFinger
    };
  }

  /**
   * Advances smooth visual damping toward live telemetry target
   */
  stepAnimation() {
    if (!this._meshesLoaded) return false;

    let changed = false;
    for (const key of ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']) {
      const diff = this.targetLivePose[key] - this.currentLivePose[key];
      if (Math.abs(diff) > 0.0001) {
        this.currentLivePose[key] += diff * 0.35;
        changed = true;
      } else {
        this.currentLivePose[key] = this.targetLivePose[key];
      }
    }
    if (changed) {
      this._applyPoseToAssembly(this.liveArm, this.currentLivePose);
    }
    return changed;
  }

  /**
   * Apply joint angles to live robot arm.
   * Supports (j1, j2, j3, j4, j5, j6, grip) or object { j1, j2, j3, j4, j5, j6, grip }.
   */
  updateLivePose(j1, j2, j3, j4 = 0.0, j5 = 90.0, j6 = 0.0, grip = 0.0) {
    let p = { j1: 0, j2: 0, j3: 0, j4: 0, j5: 90.0, j6: 0 };
    if (typeof j1 === 'object' && j1 !== null) {
      p = {
        j1: Number(j1.j1 ?? 0),
        j2: Number(j1.j2 ?? 0),
        j3: Number(j1.j3 ?? 0),
        j4: Number(j1.j4 ?? 0),
        j5: Number(j1.j5 ?? 90.0),
        j6: Number(j1.j6 ?? (j1.grip ?? 0))
      };
    } else {
      p = {
        j1: Number(j1 ?? 0),
        j2: Number(j2 ?? 0),
        j3: Number(j3 ?? 0),
        j4: Number(j4 ?? 0),
        j5: Number(j5 ?? 90.0),
        j6: Number(j6 ?? (grip ?? 0))
      };
    }
    this.livePose = { ...p };
    this.targetLivePose = { ...p };
    if (this._meshesLoaded) {
      this._applyPoseToAssembly(this.liveArm, this.targetLivePose);
    }
    this.currentLivePose = { ...this.targetLivePose };
  }

  /**
   * Apply joint angles to ghost target robot arm.
   * Supports (j1, j2, j3, j4, j5, j6, grip) or object { j1, j2, j3, j4, j5, j6, grip }.
   */
  updateGhostPose(j1, j2, j3, j4 = 0.0, j5 = 90.0, j6 = 0.0, grip = 0.0) {
    let p = { j1: 0, j2: 0, j3: 0, j4: 0, j5: 90.0, j6: 0 };
    if (typeof j1 === 'object' && j1 !== null) {
      p = {
        j1: Number(j1.j1 ?? 0),
        j2: Number(j1.j2 ?? 0),
        j3: Number(j1.j3 ?? 0),
        j4: Number(j1.j4 ?? 0),
        j5: Number(j1.j5 ?? 90.0),
        j6: Number(j1.j6 ?? (j1.grip ?? 0))
      };
    } else {
      p = {
        j1: Number(j1 ?? 0),
        j2: Number(j2 ?? 0),
        j3: Number(j3 ?? 0),
        j4: Number(j4 ?? 0),
        j5: Number(j5 ?? 90.0),
        j6: Number(j6 ?? (grip ?? 0))
      };
    }
    this.ghostPose = { ...p };
    if (this._meshesLoaded) {
      this._applyPoseToAssembly(this.ghostArm, this.ghostPose);
    }
  }

  /**
   * Rotates each joint around its CAD axis.
   * @param {Object} arm - Arm assembly returned by _buildArmAssembly
   * @param {Object} pose - { j1, j2, j3, j4, j5, j6 } in degrees
   */
  _applyPoseToAssembly(arm, pose) {
    if (!arm) return;
    const DEG2RAD = Math.PI / 180.0;

    arm.j1.quaternion.setFromAxisAngle(AXES.j1, pose.j1 * DEG2RAD);
    arm.j2.quaternion.setFromAxisAngle(AXES.j2, pose.j2 * DEG2RAD);
    arm.j3.quaternion.setFromAxisAngle(AXES.j3, pose.j3 * DEG2RAD);
    arm.j4.quaternion.setFromAxisAngle(AXES.j4, pose.j4 * DEG2RAD);
    arm.j5.quaternion.setFromAxisAngle(AXES.j5, (pose.j5 - 90.0) * DEG2RAD);
    arm.j6.quaternion.setFromAxisAngle(AXES.j6, pose.j6 * DEG2RAD);
  }

  setGhostVisible(visible) {
    this.options.showGhost = visible;
    if (this.ghostArm && this.ghostArm.root) {
      this.ghostArm.root.visible = visible;
    }
  }

  getLiveTcpWorldPosition(target = new THREE.Vector3()) {
    if (this.liveArm && this.liveArm.tcp) {
      return this.liveArm.tcp.getWorldPosition(target);
    }
    return target;
  }

  getLiveElbowWorldPosition(target = new THREE.Vector3()) {
    if (this.liveArm && this.liveArm.j3) {
      return this.liveArm.j3.getWorldPosition(target);
    }
    return target;
  }

  getGhostTcpWorldPosition(target = new THREE.Vector3()) {
    if (this.ghostArm && this.ghostArm.tcp) {
      return this.ghostArm.tcp.getWorldPosition(target);
    }
    return target;
  }

  /**
   * Traverse all child meshes and dispose of geometries and materials
   */
  dispose() {
    if (this.root) {
      this.root.traverse((child) => {
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
      });
    }
    if (this.materials) {
      Object.values(this.materials).forEach((mat) => {
        if (mat && typeof mat.dispose === 'function') {
          mat.dispose();
        }
      });
    }
  }
}
