/**
 * WorkspaceVisualizer.js - Environment, Safety Envelopes & Dynamic Alert Visualizer
 * 
 * Renders the 3D floor grid, radial reach circles, TCP orientation triad,
 * Workspace reachability envelope dome (R=310mm), Base exclusion cylinder (R=80mm),
 * and dynamic Floor Clearance Alert plane (Z=20mm, glows pulsing red on proximity breach).
 */

import * as THREE from '../../vendor/three/three.module.js';
import {
  BASE_H,
  L1,
  L2,
  TOOL_TOTAL_LEN,
  EXCL_R,
  FLOOR_CLR
} from '../kinematics/RobotKinematics.js';

// Physical reachable radius envelope (~443mm)
export const MAX_REACH_RADIUS = Math.round(L1 + L2 + TOOL_TOTAL_LEN); // 443 mm
export const GRID_SPAN = 1000;       // 1000mm total span (-500mm to +500mm), fully enclosing the 443mm reach
export const GRID_DIVISIONS = 100;   // 100 divisions = exactly 10mm grid spacing

export class WorkspaceVisualizer {
  /**
   * @param {Object} [options]
   * @param {boolean} [options.showGrid=true]
   * @param {boolean} [options.showWorkspace=true]
   * @param {boolean} [options.showFloorAlert=true]
   * @param {boolean} [options.showExclusionZone=true]
   * @param {boolean} [options.shadows=true]
   */
  constructor(options = {}) {
    this.options = Object.assign({
      showGrid: true,
      showWorkspace: true,
      showFloorAlert: true,
      showExclusionZone: true,
      showReachEnvelope: false,
      shadows: true
    }, options);

    this.root = new THREE.Group();
    this._wasBreached = false;
    this._initFloorGrid();
    this._initFloorClearancePlane();
    this._initBaseExclusionCylinder();
    this._initWorldAxes();
  }

  _initFloorGrid() {
    this.gridGroup = new THREE.Group();

    // 1. Grid helper (1000mm span, 100 divisions = 10mm grid spacing)
    const gridHelper = new THREE.GridHelper(GRID_SPAN, GRID_DIVISIONS, 0x64748b, 0x242832);
    gridHelper.position.y = 0.1;

    const gridPositions = gridHelper.geometry.getAttribute('position');
    const gridColors = gridHelper.geometry.getAttribute('color');
    const floorTone = new THREE.Color(0x111317);
    for (let i = 0; i < gridPositions.count; i++) {
      const radius = Math.hypot(gridPositions.getX(i), gridPositions.getZ(i));
      const t = THREE.MathUtils.clamp((radius - 300) / 260, 0, 1);
      const edgeFade = t * t * (3 - 2 * t);
      const lineColor = new THREE.Color().fromBufferAttribute(gridColors, i);
      lineColor.lerp(floorTone, edgeFade * 0.92);
      gridColors.setXYZ(i, lineColor.r, lineColor.g, lineColor.b);
    }
    gridColors.needsUpdate = true;
    this.gridGroup.add(gridHelper);

    // Soft radial contact shadow beneath the centered robot base.
    const shadowCanvas = document.createElement('canvas');
    shadowCanvas.width = 256;
    shadowCanvas.height = 256;
    const shadowContext = shadowCanvas.getContext('2d');
    if (shadowContext) {
      const shadowGradient = shadowContext.createRadialGradient(128, 128, 6, 128, 128, 128);
      shadowGradient.addColorStop(0, 'rgba(0, 0, 0, 0.38)');
      shadowGradient.addColorStop(0.35, 'rgba(0, 0, 0, 0.25)');
      shadowGradient.addColorStop(0.72, 'rgba(0, 0, 0, 0.08)');
      shadowGradient.addColorStop(1, 'rgba(0, 0, 0, 0)');
      shadowContext.fillStyle = shadowGradient;
      shadowContext.fillRect(0, 0, shadowCanvas.width, shadowCanvas.height);

      const contactShadow = new THREE.Mesh(
        new THREE.PlaneGeometry(260, 260),
        new THREE.MeshBasicMaterial({
          map: new THREE.CanvasTexture(shadowCanvas),
          transparent: true,
          depthWrite: false,
          side: THREE.DoubleSide,
          toneMapped: false
        })
      );
      contactShadow.rotation.x = -Math.PI / 2;
      contactShadow.position.y = 0.16;
      contactShadow.renderOrder = 1;
      this.gridGroup.add(contactShadow);
    }

    // 2. Neutral matte floor plinth plane (1100mm x 1100mm, with 50mm outer bezel)
    const floorGeo = new THREE.PlaneGeometry(1100, 1100);
    const floorMat = new THREE.MeshStandardMaterial({
      color: 0x111317,
      metalness: 0.2,
      roughness: 0.8,
      depthWrite: true
    });
    const floorMesh = new THREE.Mesh(floorGeo, floorMat);
    floorMesh.rotation.x = -Math.PI / 2;
    floorMesh.receiveShadow = this.options.shadows;
    this.gridGroup.add(floorMesh);

    // 3. Radial reach rings at 100mm, 200mm, 300mm, 400mm radii
    [100, 200, 300, 400].forEach((radius) => {
      const ringGeo = new THREE.RingGeometry(radius - 0.75, radius + 0.75, 64);
      const ringMat = new THREE.MeshBasicMaterial({
        color: 0x64748b,
        transparent: true,
        opacity: 0.22,
        side: THREE.DoubleSide
      });
      const ringMesh = new THREE.Mesh(ringGeo, ringMat);
      ringMesh.rotation.x = -Math.PI / 2;
      ringMesh.position.y = 0.2;
      this.gridGroup.add(ringMesh);
    });

    // 3b. Cardinal & Intercardinal Polar Degree Ticks (0°, 45°, 90°, 135°, 180°, 225°, 270°, 315°)
    const tickVertices = [];
    for (let deg = 0; deg < 360; deg += 45) {
      const rad = (deg * Math.PI) / 180;
      const isCardinal = (deg % 90 === 0);
      const cosA = Math.cos(rad);
      const sinA = Math.sin(rad);

      // Radial spoke ticks crossing each 100mm ring and outer boundary
      [100, 200, 300, 400, MAX_REACH_RADIUS].forEach((ringR) => {
        const halfLen = isCardinal ? 8 : 4.5;
        const rInner = ringR - halfLen;
        const rOuter = ringR + halfLen;
        tickVertices.push(
          cosA * rInner, 0.24, sinA * rInner,
          cosA * rOuter, 0.24, sinA * rOuter
        );
      });
    }
    const tickGeo = new THREE.BufferGeometry();
    tickGeo.setAttribute('position', new THREE.Float32BufferAttribute(tickVertices, 3));
    const tickMat = new THREE.LineBasicMaterial({
      color: 0x94a3b8,
      transparent: true,
      opacity: 0.38
    });
    const tickLines = new THREE.LineSegments(tickGeo, tickMat);
    tickLines.name = 'PolarCardinalTicks';
    this.gridGroup.add(tickLines);

    // 4. Maximum physical reach boundary ring (443mm envelope perimeter in calibrated titanium)
    const maxReachGeo = new THREE.RingGeometry(MAX_REACH_RADIUS - 1.1, MAX_REACH_RADIUS + 1.1, 96);
    const maxReachMat = new THREE.MeshBasicMaterial({
      color: 0xe2e8f0, // Soft Titanium reach perimeter accent
      transparent: true,
      opacity: 0.32,
      side: THREE.DoubleSide
    });
    const maxReachMesh = new THREE.Mesh(maxReachGeo, maxReachMat);
    maxReachMesh.rotation.x = -Math.PI / 2;
    maxReachMesh.position.y = 0.25;
    this.gridGroup.add(maxReachMesh);

    this.gridGroup.visible = this.options.showGrid;
    this.root.add(this.gridGroup);
  }

  _initFloorClearancePlane() {
    // Floor Clearance Alert Plane at Y = FLOOR_CLR = 20mm (1000mm x 1000mm)
    const clearanceGeo = new THREE.PlaneGeometry(1000, 1000);
    this.clearanceMat = new THREE.MeshBasicMaterial({
      color: 0x94a3b8,
      transparent: true,
      opacity: 0.04,
      side: THREE.DoubleSide,
      depthWrite: false
    });
    this.clearancePlane = new THREE.Mesh(clearanceGeo, this.clearanceMat);
    this.clearancePlane.rotation.x = -Math.PI / 2;
    this.clearancePlane.position.y = FLOOR_CLR;
    this.clearancePlane.visible = this.options.showFloorAlert;
    this.root.add(this.clearancePlane);
  }

  _initBaseExclusionCylinder() {
    // Visual keep-out cylinder is rounded to the requested 143mm height.
    const exclusionHeight = 143;
    const exclGeo = new THREE.CylinderGeometry(
      EXCL_R,
      EXCL_R,
      exclusionHeight,
      32, 1, true
    );
    this.exclMat = new THREE.MeshBasicMaterial({
      color: 0xd97706,
      wireframe: true,
      transparent: true,
      opacity: 0.15,
      depthWrite: false
    });
    this.exclCylinder = new THREE.Mesh(exclGeo, this.exclMat);
    this.exclCylinder.position.y = exclusionHeight / 2;
    this.exclCylinder.visible = this.options.showExclusionZone;
    this.root.add(this.exclCylinder);
  }

  _initReachEnvelope() {
    this.reachEnvelope = new THREE.Group();
    this.reachEnvelope.name = 'MaximumReachEnvelope';
    this.reachEnvelope.position.y = BASE_H;
    this.reachEnvelope.visible = Boolean(this.options.showReachEnvelope);

    const sphereGeometry = new THREE.SphereGeometry(MAX_REACH_RADIUS, 24, 16);
    const sphereFill = new THREE.Mesh(
      sphereGeometry,
      new THREE.MeshBasicMaterial({
        color: 0x38bdf8,
        transparent: true,
        opacity: 0.035,
        side: THREE.DoubleSide,
        depthWrite: false
      })
    );
    sphereFill.name = 'MaximumReachEnvelopeFill';
    sphereFill.renderOrder = 1;
    this.reachEnvelope.add(sphereFill);

    const wireframe = new THREE.LineSegments(
      new THREE.WireframeGeometry(sphereGeometry),
      new THREE.LineBasicMaterial({
        color: 0x38bdf8,
        transparent: true,
        opacity: 0.24,
        depthWrite: false
      })
    );
    wireframe.name = 'MaximumReachEnvelopeWireframe';
    wireframe.renderOrder = 2;
    this.reachEnvelope.add(wireframe);
    this.root.add(this.reachEnvelope);
  }

  _initWorldAxes() {
    this.axesHelper = new THREE.AxesHelper(100);
    this.axesHelper.position.set(0, 0.5, 0);
    this.root.add(this.axesHelper);
  }

  /**
   * Update floor clearance visualizer alert state based on arm position
   * @param {THREE.Vector3} tcpWorldPos - World position of TCP
   * @param {THREE.Vector3} [elbowWorldPos] - World position of Elbow
   */
  updateFloorClearanceAlert(tcpWorldPos, elbowWorldPos = null) {
    if (!this.clearancePlane || !this.options.showFloorAlert) return false;

    const threshold = FLOOR_CLR + 5.0; // 25mm warning boundary
    const tcpBreach = tcpWorldPos && tcpWorldPos.y <= threshold;
    const elbowBreach = elbowWorldPos && elbowWorldPos.y <= threshold;

    let needsRender = false;
    if (tcpBreach || elbowBreach) {
      const time = (typeof performance !== 'undefined' ? performance.now() : Date.now()) * 0.008;
      const pulseOpacity = 0.35 + Math.sin(time) * 0.15;
      this.clearanceMat.color.setHex(0xdc2626);
      this.clearanceMat.opacity = pulseOpacity;
      this._wasBreached = true;
      needsRender = true;
    } else {
      if (this._wasBreached) {
        this.clearanceMat.color.setHex(0x94a3b8);
        this.clearanceMat.opacity = 0.04;
        this._wasBreached = false;
        needsRender = true;
      }
    }
    return needsRender;
  }

  setGridVisible(visible) {
    this.options.showGrid = visible;
    if (this.gridGroup) this.gridGroup.visible = visible;
  }

  setFloorAlertVisible(visible) {
    this.options.showFloorAlert = visible;
    if (this.clearancePlane) this.clearancePlane.visible = visible;
  }

  setExclusionZoneVisible(visible) {
    this.options.showExclusionZone = visible;
    if (this.exclCylinder) this.exclCylinder.visible = visible;
  }

  setReachEnvelopeVisible(visible) {
    this.options.showReachEnvelope = Boolean(visible);
    if (this.reachEnvelope) this.reachEnvelope.visible = this.options.showReachEnvelope;
  }

  /**
   * Traverse and dispose of all geometries and materials
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
  }
}

