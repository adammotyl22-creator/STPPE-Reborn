/**
 * CameraController.js - J5 Eye-in-Hand USB Camera & Vision HUD Controller
 * 
 * Manages WebRTC camera streams (UVC USB cameras), real-time canvas overlays,
 * precision TCP targeting crosshairs, dynamic gripper opening guides,
 * rotation/flip transforms, color tracking, and snapshot capture.
 */

export class CameraController {
  /**
   * @param {Object} options
   * @param {import('../state/AppState.js').AppState} options.appState
   * @param {HTMLElement} [options.pipContainer]
   * @param {HTMLElement} [options.dockContainer]
   */
  constructor(options = {}) {
    this.appState = options.appState;
    this.pipContainer = options.pipContainer || (typeof document !== 'undefined' ? document.getElementById('camera-hud-container') : null);
    this.dockContainer = options.dockContainer || (typeof document !== 'undefined' ? document.getElementById('vision-split-container') : null);

    // Stream & Hardware State
    this.stream = null;
    this.videoElement = null;
    this.devices = [];
    this.selectedDeviceId = '';
    this.isActive = false;
    this.isPipVisible = true;
    this.pipMode = 'minimized';
    this.fps = 0;
    this._frameCount = 0;
    this._lastFpsTime = (typeof performance !== 'undefined' ? performance.now() : Date.now());

    // Configuration & Transforms
    this.config = {
      resolution: '1280x720', // '1920x1080', '1280x720', '640x480'
      rotation: 0,            // 0, 90, 180, 270 degrees
      flipH: false,
      flipV: false,
      showReticle: true,
      showGripperGuides: true,
      showConcentricRings: true,
      concentricStepMm: 20,
      showDiagnostics: true,
      autoCenterAssist: false,
      colorTracking: {
        enabled: false,
        targetColor: '#ef4444', // Red default
        tolerance: 30,
        detected: false,
        cx: 0,
        cy: 0,
        dx: 0,
        dy: 0
      }
    };

    // Render loop handle
    this._animHandle = null;
    this._isDragging = false;
    this._dragOffset = { x: 0, y: 0 };
    this._domBound = false;
    this._snapshots = [];

    // Initialize hidden offscreen video element
    if (typeof document !== 'undefined') {
      this.videoElement = document.createElement('video');
      this.videoElement.setAttribute('autoplay', '');
      this.videoElement.setAttribute('muted', '');
      this.videoElement.setAttribute('playsinline', '');
      this.videoElement.style.display = 'none';
      document.body.appendChild(this.videoElement);
    }
  }

  /**
   * Initializes DOM bindings, enumerates connected USB cameras, and loads preferences
   */
  async init() {
    this._loadSavedPreferences();
    this.bindDom();
    await this.enumerateDevices();
  }

  /**
   * Enumerates connected video input devices (USB Webcams, Endoscopes, Virtual Cameras)
   * @returns {Promise<Array<MediaDeviceInfo>>}
   */
  async enumerateDevices() {
    if (typeof navigator === 'undefined' || !navigator.mediaDevices || !navigator.mediaDevices.enumerateDevices) {
      return [];
    }

    try {
      const allDevices = await navigator.mediaDevices.enumerateDevices();
      this.devices = allDevices.filter((d) => d.kind === 'videoinput');
      this._updateDeviceSelectors();
      return this.devices;
    } catch (err) {
      console.warn('[CameraController] Failed to enumerate media devices:', err);
      return [];
    }
  }

  /**
   * Starts camera video stream with current device and resolution settings
   * @param {string} [deviceId]
   * @returns {Promise<boolean>}
   */
  async startStream(deviceId = null) {
    if (deviceId) {
      this.selectedDeviceId = deviceId;
    }

    this.stopStream();

    if (typeof navigator === 'undefined' || !navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
      console.warn('[CameraController] getUserMedia is not supported in this environment');
      this._startSimulatedStream();
      return true;
    }

    const [reqW, reqH] = this._parseResolution(this.config.resolution);
    const constraints = {
      video: {
        width: { ideal: reqW },
        height: { ideal: reqH },
        frameRate: { ideal: 30, max: 60 }
      },
      audio: false
    };

    if (this.selectedDeviceId) {
      constraints.video.deviceId = { exact: this.selectedDeviceId };
    }

    try {
      this.stream = await navigator.mediaDevices.getUserMedia(constraints);
      if (this.videoElement) {
        this.videoElement.srcObject = this.stream;
        await this.videoElement.play();
      }
      this.isActive = true;
      this._startRenderLoop();
      this._updateStatusUI();

      if (this.appState) {
        this.appState.addConsoleLog({
          type: 'INFO',
          text: `Eye-in-Hand Camera Started: ${this.config.resolution} @ J5 End-Effector`
        });
      }
      return true;
    } catch (err) {
      console.warn('[CameraController] Error accessing camera stream, switching to simulated test feed:', err);
      this._startSimulatedStream();
      return false;
    }
  }

  /**
   * Stops the active video stream
   */
  stopStream() {
    if (this.stream) {
      this.stream.getTracks().forEach((track) => track.stop());
      this.stream = null;
    }
    if (this.videoElement) {
      this.videoElement.srcObject = null;
    }
    this.isActive = false;
    this._stopRenderLoop();
    this._updateStatusUI();
  }

  /**
   * Toggles camera stream on or off
   */
  toggleStream() {
    if (this.isActive) {
      this.stopStream();
    } else {
      this.startStream();
    }
  }

  /**
   * Starts a fallback test canvas generator if physical hardware is not present
   * @private
   */
  _startSimulatedStream() {
    this.isActive = true;
    this._startRenderLoop();
    this._updateStatusUI();
  }

  /**
   * Sets camera stream resolution
   * @param {string} res - '1920x1080' | '1280x720' | '640x480'
   */
  setResolution(res) {
    this.config.resolution = res;
    this._savePreferences();
    if (this.isActive && this.stream) {
      this.startStream();
    }
  }

  /**
   * Sets camera orientation rotation (0, 90, 180, 270)
   * @param {number} deg
   */
  setRotation(deg) {
    this.config.rotation = (Number(deg) % 360 + 360) % 360;
    this._savePreferences();
  }

  /**
   * Rotates camera orientation 90 degrees clockwise
   */
  rotateClockwise() {
    this.setRotation(this.config.rotation + 90);
  }

  /**
   * Toggles horizontal mirror flip
   */
  toggleFlipH() {
    this.config.flipH = !this.config.flipH;
    this._savePreferences();
  }

  /**
   * Toggles vertical flip
   */
  toggleFlipV() {
    this.config.flipV = !this.config.flipV;
    this._savePreferences();
  }

  /**
   * Sets Visibility of PIP Overlay Window
   * @param {boolean} visible
   */
  setPipVisible(visible) {
    this.isPipVisible = Boolean(visible);
    if (this.pipContainer) {
      this.pipContainer.style.display = this.isPipVisible ? 'flex' : 'none';
    }
    const chkToggle = (typeof document !== 'undefined') ? document.getElementById('chk-camera-pip-toggle') : null;
    if (chkToggle) {
      chkToggle.checked = this.isPipVisible;
    }
    this._savePreferences();
  }

  /**
   * Sets Picture-in-Picture display mode
   * @param {'floating'|'docked'|'minimized'} mode
   */
  setPipMode(mode) {
    this.pipMode = mode;
    if (!this.pipContainer) return;

    const minBadge = (typeof document !== 'undefined') ? document.getElementById('camera-pip-minimized-badge') : null;

    if (mode === 'docked') {
      this.setPipVisible(true);
      this.pipContainer.classList.remove('camera-hud-minimized');
      this.pipContainer.classList.add('camera-hud-docked');
      this.pipContainer.style.left = '';
      this.pipContainer.style.top = '';
      this.pipContainer.style.right = '16px';
      this.pipContainer.style.bottom = '48px';
      if (minBadge) minBadge.classList.add('hidden');
    } else if (mode === 'minimized') {
      this.pipContainer.classList.add('camera-hud-minimized');
      this.pipContainer.classList.remove('camera-hud-docked');
      if (minBadge) minBadge.classList.remove('hidden');
    } else {
      // floating
      this.setPipVisible(true);
      this.pipContainer.classList.remove('camera-hud-docked');
      this.pipContainer.classList.remove('camera-hud-minimized');
      if (minBadge) minBadge.classList.add('hidden');
    }
  }

  toggleDock() {
    this.setPipMode(this.pipMode === 'docked' ? 'floating' : 'docked');
  }

  toggleMinimize() {
    this.setPipMode(this.pipMode === 'minimized' ? 'floating' : 'minimized');
  }

  /**
   * Captures a high-resolution snapshot from the video feed
   * @param {string} [filename]
   * @returns {string} DataURL of captured frame
   */
  captureSnapshot(filename = null) {
    if (typeof document === 'undefined') return null;

    const snapCanvas = document.createElement('canvas');
    const width = (this.videoElement && this.videoElement.videoWidth) ? this.videoElement.videoWidth : 1280;
    const height = (this.videoElement && this.videoElement.videoHeight) ? this.videoElement.videoHeight : 720;

    snapCanvas.width = width;
    snapCanvas.height = height;
    const ctx = snapCanvas.getContext('2d');

    // Draw transformed video frame
    this._drawFrameToContext(ctx, width, height, true);

    const dataUrl = snapCanvas.toDataURL('image/png');
    const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
    const actualName = filename || `j5_eye_in_hand_snap_${timestamp}.png`;

    const snapEntry = {
      id: `snap_${Date.now()}`,
      timestamp: new Date().toLocaleTimeString(),
      name: actualName,
      dataUrl,
      ghostPose: this.appState ? this.appState.getState('ghostPose') : null
    };

    this._snapshots.unshift(snapEntry);
    if (this._snapshots.length > 20) this._snapshots.pop();

    this._renderSnapshotGallery();

    // Trigger instant browser download if called by user action
    const link = document.createElement('a');
    link.download = actualName;
    link.href = dataUrl;
    link.click();

    if (this.appState) {
      this.appState.addConsoleLog({
        type: 'INFO',
        text: `Snapshot Saved: ${actualName} (${width}x${height})`
      });
    }

    return dataUrl;
  }

  // ==========================================================================
  // RENDER & VISION OVERLAYS
  // ==========================================================================

  /**
   * Starts requestAnimationFrame render loop
   * @private
   */
  _startRenderLoop() {
    if (this._animHandle) {
      if (typeof cancelAnimationFrame !== 'undefined') {
        cancelAnimationFrame(this._animHandle);
      } else {
        clearTimeout(this._animHandle);
      }
      this._animHandle = null;
    }

    const reqFrame = (typeof requestAnimationFrame !== 'undefined')
      ? requestAnimationFrame
      : (cb) => setTimeout(() => cb(Date.now()), 16);

    const render = (time) => {
      this._frameCount++;
      if (time - this._lastFpsTime >= 1000) {
        this.fps = Math.round((this._frameCount * 1000) / (time - this._lastFpsTime));
        this._frameCount = 0;
        this._lastFpsTime = time;
        this._updateFpsDisplay();
      }

      this._renderCanvases();

      if (this.isActive) {
        this._animHandle = reqFrame(render);
      }
    };

    this._animHandle = reqFrame(render);
  }

  /**
   * Stops requestAnimationFrame render loop
   * @private
   */
  _stopRenderLoop() {
    if (this._animHandle) {
      if (typeof cancelAnimationFrame !== 'undefined') {
        cancelAnimationFrame(this._animHandle);
      } else {
        clearTimeout(this._animHandle);
      }
      this._animHandle = null;
    }
  }

  /**
   * Renders active frame to both PIP canvas and Dock Vision canvas
   * @private
   */
  _renderCanvases() {
    if (typeof document === 'undefined') return;

    const pipCanvas = document.getElementById('camera-pip-canvas');
    if (pipCanvas && this.isPipVisible) {
      this._renderCanvas(pipCanvas);
    }

    const dockCanvas = document.getElementById('camera-dock-canvas');
    if (dockCanvas && this._isDockTabActive()) {
      this._renderCanvas(dockCanvas);
    }
  }

  /**
   * Renders a single canvas instance with transforms and reticle overlays
   * @private
   * @param {HTMLCanvasElement} canvas
   */
  _renderCanvas(canvas) {
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;

    ctx.clearRect(0, 0, width, height);

    // 1. Draw transformed video stream or test pattern
    this._drawFrameToContext(ctx, width, height, false);

    // 2. Perform color blob tracking if enabled
    if (this.config.colorTracking.enabled) {
      this._processColorTracking(ctx, width, height);
    }

    // 3. Draw Precision Gripper Reticle & Scale Overlays
    if (this.config.showReticle) {
      this._drawReticleOverlay(ctx, width, height);
    }

    // 4. Draw Diagnostics Badge (FPS, Resolution, J5 Gripper Angle)
    if (this.config.showDiagnostics) {
      this._drawDiagnosticsOverlay(ctx, width, height);
    }
  }

  /**
   * Draws the video feed or synthetic test feed with rotation & flips
   * @private
   */
  _drawFrameToContext(ctx, width, height, fullRes = false) {
    ctx.save();

    // Center coordinates for rotation
    ctx.translate(width / 2, height / 2);

    // Apply rotation
    if (this.config.rotation !== 0) {
      ctx.rotate((this.config.rotation * Math.PI) / 180);
    }

    // Apply flips
    const scaleX = this.config.flipH ? -1 : 1;
    const scaleY = this.config.flipV ? -1 : 1;
    ctx.scale(scaleX, scaleY);

    if (this.videoElement && this.videoElement.readyState >= 2 && this.stream) {
      // Draw live video frame
      ctx.drawImage(this.videoElement, -width / 2, -height / 2, width, height);
    } else {
      // Draw realistic Eye-in-Hand camera test pattern
      this._drawSimulatedTestPattern(ctx, width, height);
    }

    ctx.restore();
  }

  /**
   * Synthetic test pattern for testing without physical camera plugged in
   * @private
   */
  _drawSimulatedTestPattern(ctx, width, height) {
    // Dark metallic background
    ctx.fillStyle = '#0f172a';
    ctx.fillRect(-width / 2, -height / 2, width, height);

    // Subtle optical circular vignette
    const grad = ctx.createRadialGradient(0, 0, 10, 0, 0, width * 0.6);
    grad.addColorStop(0, '#1e293b');
    grad.addColorStop(1, '#090d16');
    ctx.fillStyle = grad;
    ctx.fillRect(-width / 2, -height / 2, width, height);

    // Grid markings
    ctx.strokeStyle = 'rgba(71, 85, 105, 0.4)';
    ctx.lineWidth = 1;
    const step = 40;
    for (let x = -width / 2; x <= width / 2; x += step) {
      ctx.beginPath();
      ctx.moveTo(x, -height / 2);
      ctx.lineTo(x, height / 2);
      ctx.stroke();
    }
    for (let y = -height / 2; y <= height / 2; y += step) {
      ctx.beginPath();
      ctx.moveTo(-width / 2, y);
      ctx.lineTo(width / 2, y);
      ctx.stroke();
    }

    // Simulated target workpiece block in center
    ctx.fillStyle = '#e11d48';
    ctx.fillRect(-25, -25, 50, 50);
    ctx.strokeStyle = '#fda4af';
    ctx.strokeRect(-25, -25, 50, 50);

    ctx.fillStyle = '#f8fafc';
    ctx.font = '11px monospace';
    ctx.textAlign = 'center';
    ctx.fillText('TARGET WORKPIECE (25mm)', 0, -32);
  }

  /**
   * Draws precision TCP crosshair, dynamic gripper finger guides, and millimeter rings
   * @private
   */
  _drawReticleOverlay(ctx, width, height) {
    const cx = width / 2;
    const cy = height / 2;

    ctx.save();

    // Get current gripper angle from AppState (0 = closed, 180 = full open)
    let gripAngle = 90;
    if (this.appState) {
      const ghost = this.appState.getState('ghostPose');
      if (ghost && ghost.grip !== undefined) gripAngle = Number(ghost.grip);
    }
    const gripSpanPx = 20 + (gripAngle / 180) * 120; // Simulated jaw span in pixels

    // 1. Concentric Millimeter Reference Rings
    if (this.config.showConcentricRings) {
      const ringRadii = [30, 60, 100, 140];
      const ringLabels = ['10mm', '20mm', '35mm', '50mm'];

      ctx.strokeStyle = 'rgba(148, 163, 184, 0.35)';
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 4]);

      ringRadii.forEach((r, idx) => {
        ctx.beginPath();
        ctx.arc(cx, cy, r, 0, Math.PI * 2);
        ctx.stroke();

        ctx.fillStyle = 'rgba(148, 163, 184, 0.6)';
        ctx.font = '9px monospace';
        ctx.fillText(ringLabels[idx], cx + r + 3, cy - 3);
      });
      ctx.setLineDash([]);
    }

    // 2. Dynamic Gripper Finger Guides (Left & Right Jaws)
    if (this.config.showGripperGuides) {
      ctx.strokeStyle = '#f59e0b'; // Amber
      ctx.lineWidth = 2;
      ctx.fillStyle = 'rgba(245, 158, 11, 0.15)';

      const jawHalf = gripSpanPx / 2;
      const jawHeight = 70;

      // Left Jaw
      ctx.strokeRect(cx - jawHalf - 6, cy - jawHeight / 2, 6, jawHeight);
      ctx.fillRect(cx - jawHalf - 6, cy - jawHeight / 2, 6, jawHeight);

      // Right Jaw
      ctx.strokeRect(cx + jawHalf, cy - jawHeight / 2, 6, jawHeight);
      ctx.fillRect(cx + jawHalf, cy - jawHeight / 2, 6, jawHeight);

      // Gripper span dimension line
      ctx.beginPath();
      ctx.moveTo(cx - jawHalf, cy - jawHeight / 2 - 8);
      ctx.lineTo(cx + jawHalf, cy - jawHeight / 2 - 8);
      ctx.stroke();

      ctx.fillStyle = '#f59e0b';
      ctx.font = '10px monospace';
      ctx.textAlign = 'center';
      ctx.fillText(`GRIP: ${gripAngle.toFixed(0)}°`, cx, cy - jawHeight / 2 - 12);
    }

    // 3. Precision Central Tool Center Point (TCP) Crosshairs
    ctx.strokeStyle = '#38bdf8'; // Sky cyan
    ctx.lineWidth = 1.5;

    // Crosshair arms
    const armLen = 22;
    const gap = 6;

    // North
    ctx.beginPath();
    ctx.moveTo(cx, cy - gap);
    ctx.lineTo(cx, cy - gap - armLen);
    ctx.stroke();

    // South
    ctx.beginPath();
    ctx.moveTo(cx, cy + gap);
    ctx.lineTo(cx, cy + gap - armLen);
    ctx.stroke();

    // West
    ctx.beginPath();
    ctx.moveTo(cx - gap, cy);
    ctx.lineTo(cx - gap - armLen, cy);
    ctx.stroke();

    // East
    ctx.beginPath();
    ctx.moveTo(cx + gap, cy);
    ctx.lineTo(cx + gap + armLen, cy);
    ctx.stroke();

    // Center focal point
    ctx.beginPath();
    ctx.arc(cx, cy, 3, 0, Math.PI * 2);
    ctx.fillStyle = '#38bdf8';
    ctx.fill();

    ctx.restore();
  }

  /**
   * Fast canvas color blob tracking
   * @private
   */
  _processColorTracking(ctx, width, height) {
    try {
      const imgData = ctx.getImageData(0, 0, width, height);
      const data = imgData.data;

      let sumX = 0;
      let sumY = 0;
      let count = 0;

      // Extract RGB targets
      const targetHex = this.config.colorTracking.targetColor || '#ef4444';
      const tr = parseInt(targetHex.slice(1, 3), 16) || 239;
      const tg = parseInt(targetHex.slice(3, 5), 16) || 68;
      const tb = parseInt(targetHex.slice(5, 7), 16) || 68;
      const tol = this.config.colorTracking.tolerance || 35;

      // Fast stride sampling
      const stride = 4;
      for (let y = 0; y < height; y += stride) {
        for (let x = 0; x < width; x += stride) {
          const idx = (y * width + x) * 4;
          const r = data[idx];
          const g = data[idx + 1];
          const b = data[idx + 2];

          const diff = Math.sqrt((r - tr) ** 2 + (g - tg) ** 2 + (b - tb) ** 2);
          if (diff < tol * 3) {
            sumX += x;
            sumY += y;
            count++;
          }
        }
      }

      const tracking = this.config.colorTracking;
      if (count > 20) {
        tracking.detected = true;
        tracking.cx = Math.round(sumX / count);
        tracking.cy = Math.round(sumY / count);
        tracking.dx = tracking.cx - width / 2;
        tracking.dy = tracking.cy - height / 2;

        // Draw target bounding box
        ctx.save();
        ctx.strokeStyle = '#22c55e'; // Green
        ctx.lineWidth = 2;
        ctx.strokeRect(tracking.cx - 20, tracking.cy - 20, 40, 40);

        ctx.fillStyle = '#22c55e';
        ctx.beginPath();
        ctx.arc(tracking.cx, tracking.cy, 4, 0, Math.PI * 2);
        ctx.fill();

        ctx.font = '10px monospace';
        ctx.fillText(`LOCKED (dx:${tracking.dx}, dy:${tracking.dy})`, tracking.cx + 25, tracking.cy);
        ctx.restore();
      } else {
        tracking.detected = false;
      }
    } catch (e) {
      // Ignore context read limits if any
    }
  }

  /**
   * Draws on-screen telemetry overlay (FPS, Resolution, TCP coordinates)
   * @private
   */
  _drawDiagnosticsOverlay(ctx, width, height) {
    ctx.save();
    ctx.fillStyle = 'rgba(15, 23, 42, 0.75)';
    ctx.fillRect(8, 8, 140, 44);
    ctx.strokeStyle = '#334155';
    ctx.strokeRect(8, 8, 140, 44);

    ctx.fillStyle = '#38bdf8';
    ctx.font = '10px monospace';
    ctx.fillText(`EYE-IN-HAND (J5)`, 14, 22);

    ctx.fillStyle = '#94a3b8';
    ctx.fillText(`${this.config.resolution} @ ${this.fps} FPS`, 14, 34);
    ctx.fillText(`ROT: ${this.config.rotation}° | FLIP: ${this.config.flipH ? 'H' : '-'}${this.config.flipV ? 'V' : '-'}`, 14, 46);

    ctx.restore();
  }

  // ==========================================================================
  // DOM BINDING & INTERACTION
  // ==========================================================================

  /**
   * Binds UI controls, draggable handles, and tab switchers
   */
  bindDom() {
    if (this._domBound || typeof document === 'undefined') return;

    // 1. Draggable PIP HUD Header
    const pipHeader = document.getElementById('camera-hud-header');
    if (pipHeader && this.pipContainer) {
      pipHeader.addEventListener('mousedown', (e) => {
        if (e.target.closest('.hud-control-btn')) return;
        this._isDragging = true;
        const rect = this.pipContainer.getBoundingClientRect();
        this._dragOffset = {
          x: e.clientX - rect.left,
          y: e.clientY - rect.top
        };
        document.body.style.userSelect = 'none';
      });

      window.addEventListener('mousemove', (e) => {
        if (!this._isDragging || !this.pipContainer) return;
        const parent = this.pipContainer.parentElement || document.body;
        const parentRect = parent.getBoundingClientRect();

        let newX = e.clientX - parentRect.left - this._dragOffset.x;
        let newY = e.clientY - parentRect.top - this._dragOffset.y;

        newX = Math.max(0, Math.min(newX, parentRect.width - this.pipContainer.offsetWidth));
        newY = Math.max(0, Math.min(newY, parentRect.height - this.pipContainer.offsetHeight));

        this.pipContainer.style.left = `${newX}px`;
        this.pipContainer.style.top = `${newY}px`;
        this.pipContainer.style.right = 'auto';
        this.pipContainer.style.bottom = 'auto';
      });

      window.addEventListener('mouseup', () => {
        this._isDragging = false;
        document.body.style.userSelect = '';
      });
    }

    // 2. PIP Control Buttons
    const btnToggleStream = document.getElementById('btn-camera-toggle-stream');
    if (btnToggleStream) {
      btnToggleStream.addEventListener('click', () => this.toggleStream());
    }

    const btnSnap = document.getElementById('btn-camera-snapshot');
    if (btnSnap) {
      btnSnap.addEventListener('click', () => this.captureSnapshot());
    }

    const btnRotate = document.getElementById('btn-camera-rotate');
    if (btnRotate) {
      btnRotate.addEventListener('click', () => this.rotateClockwise());
    }

    const btnFlipH = document.getElementById('btn-camera-fliph');
    if (btnFlipH) {
      btnFlipH.addEventListener('click', () => this.toggleFlipH());
    }

    const btnFlipV = document.getElementById('btn-camera-flipv');
    if (btnFlipV) {
      btnFlipV.addEventListener('click', () => this.toggleFlipV());
    }

    const btnClosePip = document.getElementById('btn-camera-pip-close');
    if (btnClosePip) {
      btnClosePip.addEventListener('click', () => this.setPipVisible(false));
    }

    const btnDockPip = document.getElementById('btn-camera-pip-dock');
    if (btnDockPip) {
      btnDockPip.addEventListener('click', () => this.toggleDock());
    }

    const btnMinPip = document.getElementById('btn-camera-pip-min');
    if (btnMinPip) {
      btnMinPip.addEventListener('click', () => this.toggleMinimize());
    }

    const minBadge = document.getElementById('camera-pip-minimized-badge');
    if (minBadge) {
      minBadge.addEventListener('click', () => this.setPipMode('docked'));
    }

    // 3. Viewport Layer Toggle
    const chkPip = document.getElementById('chk-camera-pip-toggle');
    if (chkPip) {
      chkPip.addEventListener('change', (e) => this.setPipVisible(e.target.checked));
    }

    // 4. Device & Resolution Selectors
    const devSelects = document.querySelectorAll('[data-camera-device-select]');
    devSelects.forEach((sel) => {
      sel.addEventListener('change', (e) => {
        this.selectedDeviceId = e.target.value;
        if (this.isActive) this.startStream();
      });
    });

    const resSelects = document.querySelectorAll('[data-camera-res-select]');
    resSelects.forEach((sel) => {
      sel.addEventListener('change', (e) => this.setResolution(e.target.value));
    });

    // 5. Reticle Toggles
    const chkReticle = document.getElementById('chk-camera-reticle');
    if (chkReticle) {
      chkReticle.addEventListener('change', (e) => {
        this.config.showReticle = e.target.checked;
        this._savePreferences();
      });
    }

    const chkGripper = document.getElementById('chk-camera-gripper-guides');
    if (chkGripper) {
      chkGripper.addEventListener('change', (e) => {
        this.config.showGripperGuides = e.target.checked;
        this._savePreferences();
      });
    }

    const chkRings = document.getElementById('chk-camera-concentric-rings');
    if (chkRings) {
      chkRings.addEventListener('change', (e) => {
        this.config.showConcentricRings = e.target.checked;
        this._savePreferences();
      });
    }

    // 6. Color Tracking Controls
    const chkColor = document.getElementById('chk-camera-color-track');
    if (chkColor) {
      chkColor.addEventListener('change', (e) => {
        this.config.colorTracking.enabled = e.target.checked;
      });
    }

    const colorPicker = document.getElementById('input-camera-target-color');
    if (colorPicker) {
      colorPicker.addEventListener('input', (e) => {
        this.config.colorTracking.targetColor = e.target.value;
      });
    }

    this._domBound = true;
    this._updateStatusUI();
  }

  /**
   * Updates dropdowns with enumerated USB cameras
   * @private
   */
  _updateDeviceSelectors() {
    const devSelects = document.querySelectorAll('[data-camera-device-select]');
    devSelects.forEach((sel) => {
      sel.innerHTML = '';
      if (this.devices.length === 0) {
        const opt = document.createElement('option');
        opt.value = '';
        opt.textContent = 'USB Camera (Auto / Default)';
        sel.appendChild(opt);
        return;
      }

      this.devices.forEach((dev, idx) => {
        const opt = document.createElement('option');
        opt.value = dev.deviceId;
        opt.textContent = dev.label || `USB Camera ${idx + 1}`;
        if (dev.deviceId === this.selectedDeviceId) opt.selected = true;
        sel.appendChild(opt);
      });
    });
  }

  /**
   * Updates status badge and buttons
   * @private
   */
  _updateStatusUI() {
    if (typeof document === 'undefined') return;

    const badges = document.querySelectorAll('[data-camera-status-badge]');
    badges.forEach((b) => {
      if (this.isActive) {
        b.textContent = 'STREAMING';
        b.className = 'badge badge-green';
      } else {
        b.textContent = 'STANDBY';
        b.className = 'badge badge-slate';
      }
    });

    const toggleBtns = document.querySelectorAll('[data-camera-toggle-btn]');
    toggleBtns.forEach((btn) => {
      btn.textContent = this.isActive ? 'Stop Stream' : 'Start Stream';
      if (this.isActive) {
        btn.classList.add('btn-active');
      } else {
        btn.classList.remove('btn-active');
      }
    });
  }

  /**
   * Updates FPS display
   * @private
   */
  _updateFpsDisplay() {
    if (typeof document === 'undefined') return;

    const fpsLabels = document.querySelectorAll('[data-camera-fps]');
    fpsLabels.forEach((el) => {
      el.textContent = `${this.fps} FPS`;
    });
  }

  /**
   * Checks if bottom dock vision tab is currently active
   * @private
   */
  _isDockTabActive() {
    if (typeof document === 'undefined') return false;
    const dockVision = document.getElementById('vision-split-container');
    return Boolean(dockVision && dockVision.style.display !== 'none' && !dockVision.hidden);
  }

  /**
   * Renders recent snapshots thumbnail gallery
   * @private
   */
  _renderSnapshotGallery() {
    const gallery = document.getElementById('camera-snapshot-gallery');
    if (!gallery) return;

    if (this._snapshots.length === 0) {
      gallery.innerHTML = '<div class="gallery-empty">No snapshots captured yet. Press "Snap Photo" to record.</div>';
      return;
    }

    gallery.innerHTML = this._snapshots.slice(0, 8).map((s) => `
      <div class="snap-thumb-card" title="${s.name}">
        <img src="${s.dataUrl}" alt="${s.name}" class="snap-thumb-img">
        <div class="snap-thumb-meta">
          <span class="snap-time">${s.timestamp}</span>
          <a href="${s.dataUrl}" download="${s.name}" class="btn-snap-dl" title="Download">Save</a>
        </div>
      </div>
    `).join('');
  }

  /**
   * Parses resolution string into [width, height]
   * @private
   */
  _parseResolution(resStr) {
    if (resStr === '1920x1080') return [1920, 1080];
    if (resStr === '640x480') return [640, 480];
    return [1280, 720]; // Default 720p
  }

  /**
   * Saves user settings to LocalStorage
   * @private
   */
  _savePreferences() {
    if (typeof localStorage === 'undefined') return;
    try {
      localStorage.setItem('webcontrol_camera_config', JSON.stringify({
        resolution: this.config.resolution,
        rotation: this.config.rotation,
        flipH: this.config.flipH,
        flipV: this.config.flipV,
        showReticle: this.config.showReticle,
        showGripperGuides: this.config.showGripperGuides,
        showConcentricRings: this.config.showConcentricRings,
        isPipVisible: this.isPipVisible
      }));
    } catch (e) {}
  }

  /**
   * Loads saved settings from LocalStorage
   * @private
   */
  _loadSavedPreferences() {
    if (typeof localStorage === 'undefined') return;
    try {
      const saved = localStorage.getItem('webcontrol_camera_config');
      if (saved) {
        const parsed = JSON.parse(saved);
        Object.assign(this.config, parsed);
        if (parsed.isPipVisible !== undefined) {
          this.isPipVisible = parsed.isPipVisible;
        }
      }
    } catch (e) {}
  }
}
