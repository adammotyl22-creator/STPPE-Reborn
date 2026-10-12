/**
 * GamepadController.js - Web Gamepad API Native Hardware Teach Pendant
 * 
 * Maps Xbox, PlayStation, and generic USB gamepads/joysticks to robot controls:
 *   - Left Stick:   X / Y Cartesian plane jogging (or R/θ in cylindrical mode)
 *   - Right Stick:  Z elevation (Up/Down) + Base yaw rotation (J1)
 *   - Triggers:     L2 = Gripper Open, R2 = Gripper Close
 *   - Bumpers:      L1 / R1 = Cycle Step-Size Multiplier (1x, 5x, 10x)
 *   - A Button (0): Capture Pose into Timeline
 *   - B Button (1): Emergency Stop (ESTOP)
 */

export class GamepadController {
  /**
   * @param {Object} options
   * @param {import('../ui/MotionController.js').MotionController} [options.motionController]
   * @param {import('./TrajectoryController.js').TrajectoryController} [options.trajectoryController]
   * @param {import('../state/AppState.js').AppState} [options.appState]
   */
  constructor(options = {}) {
    this.motionController = options.motionController || null;
    this.trajectoryController = options.trajectoryController || null;
    this.appState = options.appState || null;

    this.isConnected = false;
    this.gamepadIndex = null;
    this.gamepadId = '';

    // Step multipliers: 1x, 5x, 10x
    this.stepScales = [1.0, 5.0, 10.0];
    this.stepScaleIndex = 0;

    // Deadzone and timing configuration
    this.deadzone = 0.18;
    this.pollIntervalMs = 50; // 20 Hz
    this._pollTimer = null;
    this._lastButtonStates = new Map();
    this._lastJogTimestamp = 0;

    this._boundOnConnected = this._onConnected.bind(this);
    this._boundOnDisconnected = this._onDisconnected.bind(this);

    this.init();
  }

  init() {
    if (typeof window === 'undefined') return;

    window.addEventListener('gamepadconnected', this._boundOnConnected);
    window.addEventListener('gamepaddisconnected', this._boundOnDisconnected);

    // Initial check in case gamepad is already plugged in
    this._checkExistingGamepads();
  }

  _checkExistingGamepads() {
    if (typeof navigator === 'undefined' || typeof navigator.getGamepads !== 'function') return;
    const gamepads = navigator.getGamepads();
    for (let i = 0; i < gamepads.length; i++) {
      if (gamepads[i]) {
        this._attachGamepad(gamepads[i]);
        break;
      }
    }
  }

  _onConnected(e) {
    if (e.gamepad) {
      this._attachGamepad(e.gamepad);
    }
  }

  _onDisconnected(e) {
    if (this.gamepadIndex === e.gamepad.index) {
      this.disconnect();
    }
  }

  _attachGamepad(gp) {
    this.isConnected = true;
    this.gamepadIndex = gp.index;
    this.gamepadId = gp.id || 'USB Gamepad';

    this._updateUIBadge(true, this.gamepadId);
    this._startPolling();

    if (this.appState && typeof this.appState.addConsoleLog === 'function') {
      this.appState.addConsoleLog({
        type: 'INFO',
        text: `Gamepad Connected: ${this.gamepadId} (Teach Pendant Active)`
      });
    }
  }

  disconnect() {
    this.isConnected = false;
    this.gamepadIndex = null;
    this.gamepadId = '';
    this._stopPolling();
    this._updateUIBadge(false);

    if (this.appState && typeof this.appState.addConsoleLog === 'function') {
      this.appState.addConsoleLog({
        type: 'WARN',
        text: 'Gamepad Disconnected (Teach Pendant Inactive)'
      });
    }
  }

  _startPolling() {
    this._stopPolling();
    this._pollTimer = setInterval(() => this._poll(), this.pollIntervalMs);
  }

  _stopPolling() {
    if (this._pollTimer) {
      clearInterval(this._pollTimer);
      this._pollTimer = null;
    }
  }

  /**
   * Applies deadzone filter with normalized linear ramp
   * @param {number} value - Axis value [-1.0 .. 1.0]
   * @param {number} [deadzone=0.18]
   * @returns {number}
   */
  applyDeadzone(value, deadzone = this.deadzone) {
    if (Math.abs(value) < deadzone) return 0.0;
    const sign = Math.sign(value);
    return sign * ((Math.abs(value) - deadzone) / (1.0 - deadzone));
  }

  _poll() {
    if (!this.isConnected || typeof navigator === 'undefined' || typeof navigator.getGamepads !== 'function') return;

    const gamepads = navigator.getGamepads();
    const gp = gamepads[this.gamepadIndex];
    if (!gp) return;

    const now = Date.now();
    const multiplier = this.stepScales[this.stepScaleIndex];

    // Read Left Stick: Axis 0 (X), Axis 1 (Y)
    const rawLeftX = gp.axes[0] ?? 0;
    const rawLeftY = gp.axes[1] ?? 0;
    const leftX = this.applyDeadzone(rawLeftX);
    const leftY = this.applyDeadzone(rawLeftY);

    // Read Right Stick: Axis 2 (Yaw/X), Axis 3 (Z/Y)
    const rawRightX = gp.axes[2] ?? 0;
    const rawRightY = gp.axes[3] ?? 0;
    const rightX = this.applyDeadzone(rawRightX);
    const rightY = this.applyDeadzone(rawRightY);

    // Detect Active Jogging
    const isActivelyJogging = Math.abs(leftX) > 0 || Math.abs(leftY) > 0 || Math.abs(rightX) > 0 || Math.abs(rightY) > 0;
    this._setJogGlow(isActivelyJogging);

    if (isActivelyJogging && this.motionController && (now - this._lastJogTimestamp >= 60)) {
      this._lastJogTimestamp = now;
      const baseStep = (this.motionController.cartesianStepSize || 5.0) * multiplier;

      // 1. Left stick Y -> Robot X (Forward / Backward)
      if (Math.abs(leftY) > 0.05) {
        const stepX = -leftY * baseStep * 0.5;
        this.motionController.nudgeCartesian('x', stepX);
      }

      // 2. Left stick X -> Robot Y (Left / Right)
      if (Math.abs(leftX) > 0.05) {
        const stepY = leftX * baseStep * 0.5;
        this.motionController.nudgeCartesian('y', stepY);
      }

      // 3. Right stick Y -> Robot Z (Elevation Up / Down)
      if (Math.abs(rightY) > 0.05) {
        const stepZ = -rightY * baseStep * 0.5;
        this.motionController.nudgeCartesian('z', stepZ);
      }

      // 4. Right stick X -> Base J1 Rotation (Yaw)
      if (Math.abs(rightX) > 0.05) {
        const stepJ1 = rightX * 2.0 * multiplier;
        this.motionController.nudgeJoint('j1', stepJ1);
      }
    }

    // Read Trigger Buttons: Button 6 (L2 Gripper Open), Button 7 (R2 Gripper Close)
    const l2Pressed = (gp.buttons[6] && gp.buttons[6].value > 0.3);
    const r2Pressed = (gp.buttons[7] && gp.buttons[7].value > 0.3);

    if (l2Pressed && this.motionController) {
      this.motionController.nudgeJoint('grip', 4.0);
    } else if (r2Pressed && this.motionController) {
      this.motionController.nudgeJoint('grip', -4.0);
    }

    // Handle Edge-Triggered Buttons (Bumpers, A, B)
    const isButtonPressed = (btnIndex) => {
      const btn = gp.buttons[btnIndex];
      return Boolean(btn && (btn.pressed || btn.value > 0.5));
    };

    // Bumpers: L1 (Button 4) / R1 (Button 5) -> Cycle Step Multiplier
    const r1 = isButtonPressed(5);
    const prevR1 = this._lastButtonStates.get(5) || false;
    if (r1 && !prevR1) {
      this.cycleStepScale();
    }
    this._lastButtonStates.set(5, r1);

    // Button 0 (A / Cross) -> Capture Waypoint / Ghost Pose
    const btnA = isButtonPressed(0);
    const prevA = this._lastButtonStates.get(0) || false;
    if (btnA && !prevA && this.trajectoryController) {
      this.trajectoryController.captureGhostPose();
    }
    this._lastButtonStates.set(0, btnA);

    // Button 1 (B / Circle) -> Emergency Stop
    const btnB = isButtonPressed(1);
    const prevB = this._lastButtonStates.get(1) || false;
    if (btnB && !prevB && this.appState) {
      this.appState.setEstop(true);
    }
    this._lastButtonStates.set(1, btnB);
  }

  cycleStepScale() {
    this.stepScaleIndex = (this.stepScaleIndex + 1) % this.stepScales.length;
    const mult = this.stepScales[this.stepScaleIndex];
    if (this.appState && typeof this.appState.addConsoleLog === 'function') {
      this.appState.addConsoleLog({
        type: 'INFO',
        text: `Gamepad Step Scale: ${mult}x`
      });
    }
    this._updateUIBadge(true, `${this.gamepadId} (${mult}x)`);
    return mult;
  }

  getStepScale() {
    return this.stepScales[this.stepScaleIndex];
  }

  _updateUIBadge(connected, label = '') {
    if (typeof document === 'undefined') return;
    const badge = document.getElementById('gamepad-status-indicator');
    if (!badge) return;

    if (connected) {
      badge.classList.add('connected');
      badge.classList.remove('disconnected');
      badge.title = `Gamepad Connected: ${label}`;
      const textEl = badge.querySelector('.gamepad-text');
      if (textEl) {
        textEl.textContent = `GP: ${this.stepScales[this.stepScaleIndex]}x`;
      }
    } else {
      badge.classList.remove('connected');
      badge.classList.add('disconnected');
      badge.title = 'Gamepad Teach Pendant: Disconnected';
      const textEl = badge.querySelector('.gamepad-text');
      if (textEl) {
        textEl.textContent = 'Gamepad';
      }
    }
  }

  _setJogGlow(active) {
    if (typeof document === 'undefined') return;
    const badge = document.getElementById('gamepad-status-indicator');
    if (badge) {
      badge.classList.toggle('active-jog', active);
    }
  }

  dispose() {
    this._stopPolling();
    if (typeof window !== 'undefined') {
      window.removeEventListener('gamepadconnected', this._boundOnConnected);
      window.removeEventListener('gamepaddisconnected', this._boundOnDisconnected);
    }
  }
}

export default GamepadController;
