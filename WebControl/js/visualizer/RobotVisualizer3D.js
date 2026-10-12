/**
 * RobotVisualizer3D.js - Master 3D Kinematics Visualizer Controller
 * 
 * Integrates ThreeScene, RobotModel (URDF STL meshes), and WorkspaceVisualizer
 * into a cohesive high-performance visual control suite for the 6-DOF Robot Arm.
 */

import * as THREE from '../../vendor/three/three.module.js';
import { ThreeScene } from './ThreeScene.js';
import { RobotModel } from './RobotModel.js';
import { WorkspaceVisualizer } from './WorkspaceVisualizer.js';
import { TrajectoryVisualizer3D } from './TrajectoryVisualizer3D.js';
import { TcpDragController } from './TcpDragController.js';

export class RobotVisualizer3D {
  /**
   * @param {HTMLElement} container - DOM container element for the WebGL canvas
   * @param {Object} [options] - Configuration options
   */
  constructor(container, options = {}) {
    this.container = container;
    this.options = Object.assign({
      antialias: true,
      shadows: true,
      showGhost: true,
      showWorkspace: true,
      showFloorAlert: true,
      showExclusionZone: true,
      showGrid: true,
      showTrajectoryPath: true
    }, options);

    // 1. Initialize Scene & Camera Manager
    this.threeScene = new ThreeScene(this.container, this.options);

    // 2. Initialize Robot Model (STL meshes loaded asynchronously)
    this.robotModel = new RobotModel(this.options);
    this.threeScene.add(this.robotModel.root);

    // Start async STL mesh loading
    this._meshesReady = false;
    this.robotModel.loadMeshes(
      () => {
        // Meshes loaded — trigger initial render
        this._meshesReady = true;
        console.log('[RobotVisualizer3D] STL meshes loaded successfully');
        this.threeScene.requestRender();
      },
      (loaded, total) => {
        console.log(`[RobotVisualizer3D] Loading meshes: ${loaded}/${total}`);
      }
    ).catch((err) => {
      console.error('[RobotVisualizer3D] Failed to load STL meshes:', err);
    });

    // 3. Initialize Workspace & Safety Envelopes Visualizer
    this.workspaceVisualizer = new WorkspaceVisualizer(this.options);
    this.threeScene.add(this.workspaceVisualizer.root);

    // 4. Initialize 3D Trajectory Path Visualizer
    this.trajectoryVisualizer = new TrajectoryVisualizer3D({
      visible: this.options.showTrajectoryPath !== false,
      tubeRadius: 1.4,
      nodeRadius: 3.2
    });
    this.threeScene.add(this.trajectoryVisualizer.root);

    // 5. Register continuous frame render hook for safety alert plane
    this._tcpWorldPos = new THREE.Vector3();
    this._elbowWorldPos = new THREE.Vector3();

    this.threeScene.onRender(() => {
      let needsRender = false;
      if (this.robotModel) {
        if (typeof this.robotModel.stepAnimation === 'function') {
          if (this.robotModel.stepAnimation()) needsRender = true;
        }
        if (this.robotModel.root && needsRender) {
          this.robotModel.root.updateMatrixWorld(true);
        }
      }
      this.robotModel.getLiveTcpWorldPosition(this._tcpWorldPos);
      this.robotModel.getLiveElbowWorldPosition(this._elbowWorldPos);
      if (this.workspaceVisualizer.updateFloorClearanceAlert(this._tcpWorldPos, this._elbowWorldPos)) {
        needsRender = true;
      }
      return needsRender;
    });

    // 6. Initialize Interactive TCP Drag Controller
    this.onTcpDrag = null;
    this.onTcpDragEnd = null;
    this.tcpDragController = new TcpDragController(
      this.threeScene,
      this.robotModel,
      (coords) => {
        if (typeof this.onTcpDrag === 'function') {
          this.onTcpDrag(coords);
        }
      },
      () => {
        if (typeof this.onTcpDragEnd === 'function') {
          this.onTcpDragEnd();
        }
      }
    );
  }

  /**
   * Update joint angles for the Live Arm (driven by 10Hz JTELEM stream)
   * @param {number} j1 - Joint 1 angle (deg)
   * @param {number} j2 - Joint 2 angle (deg)
   * @param {number} j3 - Joint 3 angle (deg)
   * @param {number} [j4=90] - Joint 4 angle (deg)
   * @param {number} [j5=90] - Joint 5 angle (deg)
   * @param {number} [grip=90] - Gripper angle (deg, 0=Open, 180=Closed)
   */
  updateLivePose(j1, j2, j3, j4 = 0.0, j5 = 0.0, j6 = 0.0, grip = 0.0) {
    this.robotModel.updateLivePose(j1, j2, j3, j4, j5, j6, grip);
    this.threeScene.requestRender();
  }

  updateGhostPose(j1, j2, j3, j4 = 0.0, j5 = 0.0, j6 = 0.0, grip = 0.0) {
    this.robotModel.updateGhostPose(j1, j2, j3, j4, j5, j6, grip);
    if (this.tcpDragController && typeof this.tcpDragController.updateGizmoPosition === 'function') {
      this.tcpDragController.updateGizmoPosition();
    }
    this.threeScene.requestRender();
  }

  setSnapEnabled(enabled = null) {
    if (this.tcpDragController) {
      return this.tcpDragController.setSnapEnabled(enabled);
    }
    return false;
  }

  isSnapEnabled() {
    return this.tcpDragController ? this.tcpDragController.isSnapEnabled() : false;
  }

  setSnapGrid(gridMm) {
    if (this.tcpDragController) {
      this.tcpDragController.setSnapGrid(gridMm);
    }
  }

  setCameraPreset(preset) {
    this.threeScene.setCameraPreset(preset);
    this.threeScene.requestRender();
  }

  setGhostVisible(visible) {
    this.robotModel.setGhostVisible(visible);
    this.threeScene.requestRender();
  }

  setFloorAlertVisible(visible) {
    this.workspaceVisualizer.setFloorAlertVisible(visible);
    this.threeScene.requestRender();
  }

  setExclusionZoneVisible(visible) {
    this.workspaceVisualizer.setExclusionZoneVisible(visible);
    this.threeScene.requestRender();
  }

  setReachEnvelopeVisible(visible) {
    this.workspaceVisualizer.setReachEnvelopeVisible(visible);
    this.threeScene.requestRender();
  }

  setGridVisible(visible) {
    this.workspaceVisualizer.setGridVisible(visible);
    this.threeScene.requestRender();
  }

  setFog(enable) {
    if (this.threeScene) {
      this.threeScene.setFog(enable);
    }
  }

  updateTrajectoryPath(sequence) {
    if (this.trajectoryVisualizer) {
      this.trajectoryVisualizer.updateSequence(sequence);
      this.threeScene.requestRender();
    }
  }

  setActiveTrajectoryWaypoint(index) {
    if (this.trajectoryVisualizer) {
      this.trajectoryVisualizer.setActiveWaypoint(index);
      this.threeScene.requestRender();
    }
  }

  setTrajectoryPathVisible(visible) {
    if (this.trajectoryVisualizer) {
      this.trajectoryVisualizer.setVisible(visible);
      this.threeScene.requestRender();
    }
  }

  setMobileOptimized(enabled) {
    this.threeScene.setMobileOptimized(enabled);
    this.trajectoryVisualizer.setGlowEnabled(!enabled);
    this.threeScene.requestRender();
  }

  onResize() {
    this.threeScene.onResize();
    this.threeScene.requestRender();
  }

  /**
   * Clean up WebGL resources and event listeners
   */
  dispose() {
    if (this.tcpDragController && typeof this.tcpDragController.dispose === 'function') {
      this.tcpDragController.dispose();
    }
    if (this.robotModel && typeof this.robotModel.dispose === 'function') {
      this.robotModel.dispose();
    }
    if (this.workspaceVisualizer && typeof this.workspaceVisualizer.dispose === 'function') {
      this.workspaceVisualizer.dispose();
    }
    if (this.trajectoryVisualizer && typeof this.trajectoryVisualizer.dispose === 'function') {
      this.trajectoryVisualizer.dispose();
    }
    this.threeScene.dispose();
  }
}
