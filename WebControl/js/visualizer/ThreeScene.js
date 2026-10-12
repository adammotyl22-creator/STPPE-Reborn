/**
 * ThreeScene.js - Scene, Camera, Lighting & OrbitControls Manager
 * 
 * Manages Three.js WebGL scene, lighting hierarchy, camera projection,
 * OrbitControls with inertia damping, and smooth camera preset transitions.
 */

import * as THREE from '../../vendor/three/three.module.js';
import { OrbitControls } from '../../vendor/three/OrbitControls.js';

export class ThreeScene {
  /**
   * @param {HTMLElement} container - DOM element to mount the WebGL canvas
   * @param {Object} [options] - Scene configuration options
   */
  constructor(container, options = {}) {
    this.container = container;
    this.options = Object.assign({
      antialias: true,
      shadows: true,
      fog: false
    }, options);

    this.scene = null;
    this.camera = null;
    this.renderer = null;
    this.controls = null;
    this.keyLight = null;
    this.mobileOptimized = false;
    this.animationFrameId = null;
    this._renderCallbacks = [];
    this.needsRender = true;

    this._initScene();
    this._initLights();
    this._initCameraControls();

    this.onResize = this.onResize.bind(this);
    this._animate = this._animate.bind(this);

    this._resizeObserver = null;
    if (typeof ResizeObserver !== 'undefined' && this.container) {
      this._resizeObserver = new ResizeObserver(() => {
        this.onResize();
      });
      this._resizeObserver.observe(this.container);
    }

    if (typeof window !== 'undefined') {
      window.addEventListener('resize', this.onResize);
    }
    this._startAnimationLoop();
  }

  _initScene() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0b0d13);
    if (this.options.fog) {
      this.scene.fog = new THREE.FogExp2(0x0b0d13, 0.0012);
    }

    const width = (this.container && this.container.clientWidth) ? this.container.clientWidth : 800;
    const height = (this.container && this.container.clientHeight) ? this.container.clientHeight : 600;

    this.camera = new THREE.PerspectiveCamera(45, width / height, 1, 4000);
    this.camera.position.set(450, 360, 450);

    this.renderer = new THREE.WebGLRenderer({
      antialias: this.options.antialias,
      powerPreference: 'high-performance'
    });
    this.renderer.setSize(width, height);
    if (typeof window !== 'undefined' && window.devicePixelRatio) {
      this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    }
    this.renderer.shadowMap.enabled = this.options.shadows;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.15;

    if (this.container && this.container.appendChild && this.renderer.domElement) {
      this.container.appendChild(this.renderer.domElement);
    }
  }

  _initLights() {
    // 1. Ambient baseline
    const ambientLight = new THREE.AmbientLight(0x272b35, 1.2);
    this.scene.add(ambientLight);

    // 2. Hemisphere ambient (Warm Slate / Neutral Titanium)
    const hemiLight = new THREE.HemisphereLight(0x4b5563, 0x181a20, 0.8);
    this.scene.add(hemiLight);

    // 3. Key directional light with soft shadows
    const keyLight = new THREE.DirectionalLight(0xffffff, 2.2);
    this.keyLight = keyLight;
    keyLight.position.set(300, 500, 300);
    keyLight.castShadow = this.options.shadows;
    keyLight.shadow.mapSize.width = 2048;
    keyLight.shadow.mapSize.height = 2048;
    keyLight.shadow.camera.near = 50;
    keyLight.shadow.camera.far = 1500;
    keyLight.shadow.camera.left = -550;
    keyLight.shadow.camera.right = 550;
    keyLight.shadow.camera.top = 550;
    keyLight.shadow.camera.bottom = -550;
    keyLight.shadow.bias = -0.0005;
    keyLight.shadow.radius = 4;
    this.scene.add(keyLight);

    // 4. Fill cyan directional light
    const fillLight = new THREE.DirectionalLight(0x00d9e6, 1.0);
    fillLight.position.set(-250, 200, -250);
    this.scene.add(fillLight);

    // 5. Cool top-back rim light separates the arm from the dark viewport.
    const rimLight = new THREE.DirectionalLight(0x8bdcff, 0.9);
    rimLight.position.set(-180, 460, -420);
    this.scene.add(rimLight);
  }

  setMobileOptimized(enabled) {
    this.mobileOptimized = Boolean(enabled);
    const devicePixelRatio = (typeof window !== 'undefined' && window.devicePixelRatio) || 1;
    const pixelRatioCap = this.mobileOptimized ? 1.25 : 2;
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, pixelRatioCap));

    const shadowsEnabled = Boolean(this.options.shadows && !this.mobileOptimized);
    this.renderer.shadowMap.enabled = shadowsEnabled;
    if (this.keyLight) this.keyLight.castShadow = shadowsEnabled;
    this.renderer.shadowMap.needsUpdate = true;
    this.requestRender();
  }

  _initCameraControls() {
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.06;
    this.controls.target.set(0, 150, 0);
    this.controls.maxPolarAngle = Math.PI / 2 + 0.05; // Prevent camera sinking under floor
    this.controls.minDistance = 80;
    this.controls.maxDistance = 2500;
    
    this.controls.addEventListener('change', () => this.requestRender());
  }

  /**
   * Register a render callback to execute before each frame
   * @param {Function} cb - (time) => void
   */
  onRender(cb) {
    if (typeof cb === 'function') {
      this._renderCallbacks.push(cb);
    }
  }

  /**
   * Animate camera smoothly to preset viewpoint
   * @param {'isometric' | 'top' | 'front' | 'side' | 'reset'} preset
   * @param {number} [durationMs=600]
   */
  setCameraPreset(preset, durationMs = 600) {
    const presets = {
      isometric: { pos: new THREE.Vector3(450, 360, 450), target: new THREE.Vector3(0, 150, 0) },
      top: { pos: new THREE.Vector3(0, 850, 0.1), target: new THREE.Vector3(0, 0, 0) },
      front: { pos: new THREE.Vector3(650, 200, 0), target: new THREE.Vector3(0, 160, 0) },
      side: { pos: new THREE.Vector3(0, 200, 650), target: new THREE.Vector3(0, 160, 0) },
      reset: { pos: new THREE.Vector3(450, 360, 450), target: new THREE.Vector3(0, 150, 0) }
    };

    const targetPreset = presets[preset] || presets.isometric;
    this._animateCameraTransition(targetPreset.pos, targetPreset.target, durationMs);
  }

  _animateCameraTransition(endPos, endTarget, durationMs = 600) {
    if (this._cameraTransitionId && typeof cancelAnimationFrame !== 'undefined') {
      cancelAnimationFrame(this._cameraTransitionId);
      this._cameraTransitionId = null;
    }

    const startPos = this.camera.position.clone();
    const startTarget = this.controls.target.clone();
    let startTime = null;

    const updateCamera = (currentTime) => {
      if (startTime === null) startTime = currentTime;
      const elapsed = currentTime - startTime;
      const progress = Math.max(0, Math.min(elapsed / durationMs, 1.0));
      // Smooth cubic ease-out
      const ease = 1 - Math.pow(1 - progress, 3);

      this.camera.position.lerpVectors(startPos, endPos, ease);
      this.controls.target.lerpVectors(startTarget, endTarget, ease);
      this.controls.update();

      if (progress < 1.0 && typeof requestAnimationFrame !== 'undefined') {
        this._cameraTransitionId = requestAnimationFrame(updateCamera);
      } else {
        this._cameraTransitionId = null;
      }
    };

    if (typeof requestAnimationFrame !== 'undefined') {
      this._cameraTransitionId = requestAnimationFrame(updateCamera);
    } else {
      this.camera.position.copy(endPos);
      this.controls.target.copy(endTarget);
      this.controls.update();
    }
  }

  onResize() {
    if (!this.container || !this.renderer || !this.camera) return;
    const width = this.container.clientWidth || 800;
    const height = this.container.clientHeight || 600;
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  /**
   * Request a new frame to be rendered
   */
  requestRender() {
    this.needsRender = true;
  }

  _startAnimationLoop() {
    this.needsRender = true;
    if (typeof requestAnimationFrame !== 'undefined') {
      this.animationFrameId = requestAnimationFrame(this._animate);
    }
  }

  _animate(time) {
    if (typeof requestAnimationFrame !== 'undefined') {
      this.animationFrameId = requestAnimationFrame(this._animate);
    }
    
    let shouldRender = false;
    if (this.needsRender) {
      shouldRender = true;
      this.needsRender = false;
    }

    if (this.controls) {
      // OrbitControls.update() returns true if the camera actually moved this frame
      if (this.controls.update()) {
        shouldRender = true;
      }
    }
    
    for (let i = 0; i < this._renderCallbacks.length; i++) {
      const cbChanged = this._renderCallbacks[i](time);
      if (cbChanged) {
        shouldRender = true;
      }
    }
    
    if (this._cameraTransitionId) {
      shouldRender = true;
    }

    if (shouldRender && this.renderer && this.scene && this.camera) {
      this.renderer.render(this.scene, this.camera);
    }
  }

  /**
   * Add object to scene
   */
  add(object) {
    if (this.scene) this.scene.add(object);
  }

  /**
   * Remove object from scene
   */
  remove(object) {
    if (this.scene) this.scene.remove(object);
  }

  /**
   * Toggles distance fog in the 3D scene
   * @param {boolean} enable
   */
  setFog(enable) {
    this.options.fog = Boolean(enable);
    if (this.scene) {
      this.scene.fog = this.options.fog ? new THREE.FogExp2(0x0d0e12, 0.0012) : null;
      this.requestRender();
    }
  }

  dispose() {
    if (this._resizeObserver) {
      this._resizeObserver.disconnect();
      this._resizeObserver = null;
    }
    if (this.animationFrameId && typeof cancelAnimationFrame !== 'undefined') {
      cancelAnimationFrame(this.animationFrameId);
    }
    if (this._cameraTransitionId && typeof cancelAnimationFrame !== 'undefined') {
      cancelAnimationFrame(this._cameraTransitionId);
      this._cameraTransitionId = null;
    }
    if (typeof window !== 'undefined') {
      window.removeEventListener('resize', this.onResize);
    }
    if (this.controls) this.controls.dispose();
    if (this.renderer) {
      if (this.renderer.domElement && this.container && this.renderer.domElement.parentNode === this.container) {
        this.container.removeChild(this.renderer.domElement);
      }
      this.renderer.dispose();
    }
    this._renderCallbacks = [];
  }
}
