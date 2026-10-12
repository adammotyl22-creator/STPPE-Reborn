/**
 * HeaderController.js - Master Navigation, Connection & Safety UI Controller
 * 
 * Manages WebSerial lifecycle, Baud rate selection, Mock mode toggle,
 * Glowing Emergency Stop (ESTOP/REENABLE) with keyboard shortcuts (Spacebar/Esc),
 * Homing routines, Speed Override, and Micro-Gantry step jogging.
 */

import { appState as defaultAppState } from '../state/AppState.js';
import { ConnectionState } from '../comms/WebSerialManager.js';
import {
  buildEstopCommand,
  buildReenableCommand,
  buildHomeCommand,
  buildSpeedCommand,
  buildGantryJogCommand
} from '../comms/PacketBuilder.js';
import { i18n } from '../i18n/I18nManager.js';

export const SUPPORTED_BAUD_RATES = Object.freeze([
  9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600
]);

export const GANTRY_NUDGE_STEPS = Object.freeze([-10, -1, 1, 10]);

export class HeaderController {
  /**
   * @param {Object} [options]
   * @param {HTMLElement|string} [options.container] - Header container DOM element
   * @param {AppState} [options.appState] - Central state store
   * @param {WebSerialManager} [options.serialManager] - Serial communication manager
   * @param {number} [options.defaultBaudRate=115200]
   * @param {boolean} [options.enableShortcuts=true] - Enable Spacebar and Escape keyboard shortcuts
   */
  constructor(options = {}) {
    this.appState = options.appState || defaultAppState;
    this.serialManager = null;
    this.baudRate = Number(options.defaultBaudRate || 115200);
    this.autoReconnect = Boolean(options.autoReconnect);
    this.enableShortcuts = options.enableShortcuts !== false;

    // Transport configuration (USB Serial vs ESP32 Wi-Fi WebSocket)
    let initialTransport = options.transportMode;
    if (!initialTransport && typeof localStorage !== 'undefined') {
      try {
        initialTransport = localStorage.getItem('robotarm_transport_mode');
      } catch (_) {}
    }
    if (!initialTransport) {
      // Auto-detect mobile / non-WebSerial environments
      const hasSerial = typeof navigator !== 'undefined' && 'serial' in navigator;
      initialTransport = hasSerial ? 'serial' : 'websocket';
    }
    this.transportMode = initialTransport;

    let initialWsUrl = options.wsUrl;
    if (!initialWsUrl && typeof localStorage !== 'undefined') {
      try {
        initialWsUrl = localStorage.getItem('robotarm_ws_url');
      } catch (_) {}
    }
    this.wsUrl = initialWsUrl || '192.168.4.1:81';

    this.connectionStatus = ConnectionState.DISCONNECTED;
    this.isMock = false;
    this.telemetryRateHz = 0.0;
    this.estopActive = false;
    this.homingActive = false;
    this.speedOverride = 100;

    // DOM Binding tracking
    this.container = null;
    this._domBound = false;
    this._domElements = {};
    this._eventCleanups = [];
    this._unsubState = [];
    this._keyHandler = null;
    this._popoverOpen = false;
    this._reenableHoldTimer = null;
    this._backdrop = null;

    if (options.serialManager) {
      this.setSerialManager(options.serialManager);
    }

    // Initialize AppState subscribers
    this._initAppStateListeners();

    // Attach global keyboard shortcuts if enabled
    if (this.enableShortcuts && typeof window !== 'undefined') {
      this._bindKeyboardShortcuts();
    }

    // Bind DOM if container is provided
    if (options.container && typeof document !== 'undefined') {
      const el = typeof options.container === 'string'
        ? document.querySelector(options.container)
        : options.container;
      if (el) {
        this.bindDom(el);
      }
    }
  }

  /**
   * Returns fully qualified WebSocket URL
   * @private
   */
  _getFullWsUrl() {
    let url = String(this.wsUrl || '192.168.4.1:81').trim();
    if (!url.startsWith('ws://') && !url.startsWith('wss://')) {
      url = `ws://${url}`;
    }
    if (!url.endsWith('/')) {
      url = `${url}/`;
    }
    return url;
  }

  /**
   * Sets active transport mode ('serial' or 'websocket')
   * @param {'serial'|'websocket'} mode
   */
  setTransportMode(mode) {
    const norm = String(mode).toLowerCase();
    if (norm !== 'serial' && norm !== 'websocket') return;
    this.transportMode = norm;

    if (typeof localStorage !== 'undefined') {
      try {
        localStorage.setItem('robotarm_transport_mode', norm);
      } catch (_) {}
    }

    if (this.serialManager && this.serialManager.setTransportMode && !this.isConnected()) {
      try {
        this.serialManager.setTransportMode(norm, this._getFullWsUrl());
      } catch (err) {
        console.warn('Could not update serialManager transport mode:', err);
      }
    }

    if (this.appState) {
      this.appState.setState('connection.transportMode', norm);
    }

    this._updateDomConnectionUI();
  }

  /**
   * Sets target WebSocket endpoint
   * @param {string} url
   */
  setWsUrl(url) {
    if (!url) return;
    this.wsUrl = String(url).trim();

    if (typeof localStorage !== 'undefined') {
      try {
        localStorage.setItem('robotarm_ws_url', this.wsUrl);
      } catch (_) {}
    }

    if (this.serialManager && this.serialManager.setWsUrl) {
      this.serialManager.setWsUrl(this._getFullWsUrl());
    }

    if (this.appState) {
      this.appState.setState('connection.wsUrl', this.wsUrl);
    }
  }

  /**
   * Toggles Fullscreen Mobile Teach Pendant mode
   */
  async toggleFullscreenPendant() {
    if (typeof document === 'undefined') return;
    try {
      if (!document.fullscreenElement) {
        await document.documentElement.requestFullscreen();
      } else {
        await document.exitFullscreen();
      }
    } catch (err) {
      console.warn('Fullscreen request failed:', err);
    }
  }

  /**
   * Sets serial communication manager and forwards events
   * @param {WebSerialManager} manager
   */
  setSerialManager(manager) {
    this.serialManager = manager;
    if (this.serialManager) {
      if (this.serialManager.setTransportMode && !this.isConnected()) {
        try {
          this.serialManager.setTransportMode(this.transportMode, this._getFullWsUrl());
        } catch (_) {}
      }

      this.serialManager.on('statusChange', ({ newState }) => {
        this.connectionStatus = newState;
        if (this.appState) {
          this.appState.setConnectionStatus(newState, { baudRate: this.baudRate, transportMode: this.transportMode, isMock: this.isMock });
        }
        this._updateDomConnectionUI();
      });

      this.serialManager.on('rateChange', ({ rateHz }) => {
        this.telemetryRateHz = Number(rateHz || 0);
        if (this.appState) {
          this.appState.setState('connection.telemRateHz', this.telemetryRateHz);
        }
        this._updateDomConnectionUI();
      });

      this.serialManager.on('disconnected', () => {
        if (this.autoReconnect && !this.isMock && this.transportMode === 'serial') {
          this._attemptAutoReconnect();
        }
      });
    }
  }

  /**
   * Subscribes to AppState state updates
   * @private
   */
  _initAppStateListeners() {
    if (!this.appState) return;

    const unsubSafety = this.appState.subscribe('stateChange:safety', (safety) => {
      if (safety.estop !== undefined && safety.estop !== this.estopActive) {
        this.estopActive = Boolean(safety.estop);
        this._updateDomEstopUI();
      }
      if (safety.homing !== undefined && safety.homing !== this.homingActive) {
        this.homingActive = Boolean(safety.homing);
        this._updateDomHomingUI();
      }
      if (safety.speedOverride !== undefined && safety.speedOverride !== this.speedOverride) {
        this.speedOverride = Number(safety.speedOverride);
        this._updateDomSpeedUI();
      }
    });
    this._unsubState.push(unsubSafety);

    const unsubConn = this.appState.subscribe('stateChange:connection', (conn) => {
      if (conn.status) this.connectionStatus = conn.status;
      if (conn.isMock !== undefined) this.isMock = Boolean(conn.isMock);
      if (conn.baudRate) this.baudRate = Number(conn.baudRate);
      if (conn.telemRateHz !== undefined) this.telemetryRateHz = Number(conn.telemRateHz);
      this._updateDomConnectionUI();
    });
    this._unsubState.push(unsubConn);

    const unsubI18n = i18n.subscribe(() => {
      this._updateDomConnectionUI();
      this._updateDomHomingUI();
    });
    this._unsubState.push(unsubI18n);
  }

  // ==========================================================================
  // WEBSERIAL CONNECTION CONTROLS
  // ==========================================================================

  /**
   * Connects to WebSerial hardware port or MockSerialPort
   * @param {Object} [customPort=null]
   */
  async connect(customPort = null) {
    if (!this.serialManager) {
      throw new Error('WebSerialManager instance not configured in HeaderController');
    }

    this.connectionStatus = ConnectionState.CONNECTING;
    this._updateDomConnectionUI();

    try {
      const connectOpts = {
        baudRate: this.baudRate,
        autoEnableJTelem: true,
        transport: this.transportMode,
        transportMode: this.transportMode,
        wsUrl: this._getFullWsUrl()
      };
      await this.serialManager.connect(customPort, connectOpts);
      this.connectionStatus = ConnectionState.CONNECTED;
      if (this.appState) {
        this.appState.setConnectionStatus(ConnectionState.CONNECTED, {
          baudRate: this.baudRate,
          transportMode: this.transportMode,
          isMock: this.serialManager.isMock
        });
        const connDesc = this.serialManager.isMock
          ? 'Mock Mode'
          : (this.transportMode === 'websocket' ? `Wi-Fi Bridge (${this._getFullWsUrl()})` : `USB Serial at ${this.baudRate} baud`);
        this.appState.addConsoleLog({
          type: 'INFO',
          text: `Connection established via ${connDesc}`
        });
      }
    } catch (err) {
      this.connectionStatus = ConnectionState.ERROR;
      if (this.appState) {
        this.appState.setConnectionStatus(ConnectionState.ERROR, { error: err.message });
        this.appState.addConsoleLog({ type: 'ERR', text: `Connection failed: ${err.message}` });
      }
      throw err;
    } finally {
      this._updateDomConnectionUI();
    }
  }

  /**
   * Disconnects active serial port
   */
  async disconnect() {
    if (this.serialManager && this.serialManager.isConnected()) {
      await this.serialManager.disconnect();
    }
    this.connectionStatus = ConnectionState.DISCONNECTED;
    this.telemetryRateHz = 0.0;
    if (this.appState) {
      this.appState.setConnectionStatus(ConnectionState.DISCONNECTED, { telemRateHz: 0.0 });
      this.appState.addConsoleLog({ type: 'INFO', text: 'Serial port disconnected' });
    }
    this._updateDomConnectionUI();
  }

  /**
   * Toggles Connect / Disconnect
   */
  async toggleConnect() {
    if (this.isConnected()) {
      return await this.disconnect();
    } else {
      return await this.connect();
    }
  }

  /**
   * Checks if currently connected
   * @returns {boolean}
   */
  isConnected() {
    return this.serialManager ? this.serialManager.isConnected() : (this.connectionStatus === ConnectionState.CONNECTED);
  }

  /**
   * Sets serial baud rate
   * @param {number} baud
   */
  setBaudRate(baud) {
    const numBaud = Number(baud);
    if (!SUPPORTED_BAUD_RATES.includes(numBaud)) {
      console.warn(`Non-standard baud rate: ${baud}`);
    }
    this.baudRate = numBaud;
    if (this.appState) {
      this.appState.setState('connection.baudRate', this.baudRate);
    }
    this._updateDomConnectionUI();
  }

  /**
   * Toggles Mock Serial simulator mode
   * @param {boolean} enable
   */
  setMockMode(enable) {
    this.isMock = Boolean(enable);
    if (this.serialManager) {
      this.serialManager.setMockMode(this.isMock);
    }
    if (this.appState) {
      this.appState.setState('connection.isMock', this.isMock);
      this.appState.addConsoleLog({
        type: 'INFO',
        text: `Mock Serial Simulator ${this.isMock ? 'ENABLED' : 'DISABLED'}`
      });
    }
    this._updateDomConnectionUI();
  }

  /**
   * Sets auto-reconnect state
   * @param {boolean} enable
   */
  setAutoReconnect(enable) {
    this.autoReconnect = Boolean(enable);
  }

  /**
   * Attempts auto-reconnection with backoff
   * @private
   */
  _attemptAutoReconnect() {
    if (this._reconnecting || !this.autoReconnect) return;
    this._reconnecting = true;
    this.connectionStatus = ConnectionState.RECONNECTING;
    this._updateDomConnectionUI();

    setTimeout(async () => {
      try {
        if (this.autoReconnect && !this.isConnected()) {
          await this.connect();
        }
      } catch (_) {}
      this._reconnecting = false;
    }, 2000);
  }

  // ==========================================================================
  // SAFETY & EMERGENCY STOP (ESTOP / REENABLE)
  // ==========================================================================

  /**
   * Dispatches instant EMERGENCY STOP command and transitions to alert state
   * @returns {string} Dispatched ESTOP command
   */
  triggerEstop() {
    this.estopActive = true;
    const cmd = buildEstopCommand();

    if (this.serialManager && this.serialManager.isConnected()) {
      this.serialManager.send(cmd).catch((err) => {
        console.error('Failed to dispatch ESTOP:', err);
      });
    }

    if (this.appState) {
      this.appState.setEstop(true);
      this.appState.addConsoleLog({
        type: 'ERR',
        text: '!!! EMERGENCY STOP TRIGGERED: MOTION HALTED, DRIVERS DISABLED !!!'
      });
    }

    this._updateDomEstopUI();
    return cmd;
  }

  /**
   * Dispatches REENABLE command to clear ESTOP and restore normal operation
   * @returns {string} Dispatched REENABLE command
   */
  triggerReenable() {
    this.estopActive = false;
    const cmd = buildReenableCommand();

    if (this.serialManager && this.serialManager.isConnected()) {
      this.serialManager.send(cmd).catch((err) => {
        console.error('Failed to dispatch REENABLE:', err);
      });
    }

    if (this.appState) {
      this.appState.setEstop(false);
      this.appState.addConsoleLog({
        type: 'INFO',
        text: 'System Re-enabled: Stepper drivers energized, normal operation restored'
      });
    }

    this._updateDomEstopUI();
    return cmd;
  }

  // ==========================================================================
  // HOMING CONTROLS
  // ==========================================================================

  /**
   * Dispatches a Homing routine command (ALL, J1, J2, J3, GANTRY, MANUAL, SWITCH)
   * @param {string} [target='ALL']
   * @returns {string} Dispatched home command
   */
  triggerHome(target = 'ALL') {
    if (this.estopActive) {
      throw new Error('Cannot execute homing: EMERGENCY STOP is active');
    }

    const cmd = buildHomeCommand(target);

    if (this.serialManager && this.serialManager.isConnected()) {
      this.serialManager.send(cmd).catch((err) => {
        console.error('Homing command failed:', err);
      });
    }

    this.homingActive = true;
    if (this.appState) {
      this.appState.setHoming(true);
      this.appState.addConsoleLog({
        type: 'TX',
        text: cmd.trim()
      });
    }

    this._updateDomHomingUI();
    return cmd;
  }

  // ==========================================================================
  // SPEED OVERRIDE & MICRO-GANTRY JOG
  // ==========================================================================

  /**
   * Sets Speed Override percentage (5% to 100%) and dispatches `SPEED <val>` command
   * @param {number} percent
   * @returns {number} Clamped speed percentage
   */
  setSpeedOverride(percent) {
    const clamped = Math.min(100, Math.max(5, Math.round(Number(percent))));
    this.speedOverride = clamped;

    const cmd = buildSpeedCommand(clamped);

    if (this.serialManager && this.serialManager.isConnected()) {
      this.serialManager.send(cmd).catch(() => {});
    }

    if (this.appState) {
      this.appState.setSpeedOverride(clamped);
      this.appState.addConsoleLog({
        type: 'TX',
        text: cmd.trim()
      });
    }

    this._updateDomSpeedUI();
    return clamped;
  }

  /**
   * Jogs the micro-gantry by delta steps or mm (DX or DY)
   * 
   * @param {'X'|'DX'|'Y'|'DY'} axis
   * @param {number} stepsOrMm
   * @returns {string} Dispatched command string
   */
  jogGantry(axis, stepsOrMm) {
    if (this.estopActive) {
      throw new Error('Cannot jog gantry: EMERGENCY STOP is active');
    }

    const num = Math.round(Number(stepsOrMm));
    const cmd = buildGantryJogCommand(axis, num);

    if (this.serialManager && this.serialManager.isConnected()) {
      this.serialManager.send(cmd).catch(() => {});
    }

    if (this.appState) {
      this.appState.addConsoleLog({
        type: 'TX',
        text: cmd.trim()
      });
    }

    return cmd;
  }

  // ==========================================================================
  // KEYBOARD SHORTCUTS
  // ==========================================================================

  /**
   * Binds global Escape key shortcut for instant ESTOP triggering
   * @private
   */
  _bindKeyboardShortcuts() {
    if (typeof window === 'undefined') return;

    this._keyHandler = (e) => {
      // Don't trigger if user is actively typing in a form input or textarea
      const target = e.target;
      const isInput = target && (target.tagName === 'INPUT' || target.tagName === 'TEXTAREA' || target.tagName === 'SELECT' || target.isContentEditable);
      
      if (e.code === 'Escape' || e.key === 'Escape') {
        // If connection popover is open, close it instead of triggering ESTOP
        if (this._popoverOpen) {
          e.preventDefault();
          e.stopPropagation();
          this._closePopover();
          return;
        }
        if (!isInput) {
          e.preventDefault();
        }
        this.triggerEstop();
      }
    };

    window.addEventListener('keydown', this._keyHandler, { capture: true });
  }

  // ==========================================================================
  // DOM BINDING & UI EVENT HANDLING
  // ==========================================================================

  /**
   * Binds UI controls in a header DOM container
   * @param {HTMLElement} container
   */
  bindDom(container) {
    if (!container || typeof document === 'undefined') return;

    this.unbindDom();
    this.container = container;
    this._domBound = true;

    // ── Status Pill & Connection Popover ──────────────────────────────
    const statusPill = container.querySelector('#status-pill');
    const popover = container.querySelector('#connection-popover');
    const statusDot = container.querySelector('#status-dot');

    if (statusPill) {
      this._domElements.statusPill = statusPill;
    }
    if (popover) {
      this._domElements.popover = popover;
    }
    if (statusDot) {
      this._domElements.statusDot = statusDot;
    }

    if (statusPill && popover) {
      const onPillClick = (e) => {
        e.stopPropagation();
        this._togglePopover();
      };
      statusPill.addEventListener('click', onPillClick);
      this._eventCleanups.push(() => statusPill.removeEventListener('click', onPillClick));
    }

    // ── Safety Zone ──────────────────────────────────────────────────
    const safetyZone = container.querySelector('#header-safety-zone');
    if (safetyZone) {
      this._domElements.safetyZone = safetyZone;
    }

    // 1. Connect / Disconnect Toggle Button
    const connectBtn = container.querySelector('[data-action="connect-serial"], #btn_serial_connect');
    if (connectBtn) {
      this._domElements.connectBtn = connectBtn;
      const onConnectClick = () => this.toggleConnect().catch(() => {});
      connectBtn.addEventListener('click', onConnectClick);
      this._eventCleanups.push(() => connectBtn.removeEventListener('click', onConnectClick));
    }

    // 1b. Transport Toggle Buttons (USB WebSerial vs Wi-Fi WebSocket)
    const transportBtns = container.querySelectorAll('[data-transport]');
    transportBtns.forEach((btn) => {
      const mode = btn.getAttribute('data-transport');
      const onTransportClick = () => this.setTransportMode(mode);
      btn.addEventListener('click', onTransportClick);
      this._eventCleanups.push(() => btn.removeEventListener('click', onTransportClick));
    });

    // 1c. WebSocket Bridge Endpoint Input
    const wsInput = container.querySelector('[data-ws-url-input], #input_ws_url');
    if (wsInput) {
      this._domElements.wsInput = wsInput;
      wsInput.value = this.wsUrl;
      const onWsChange = (e) => this.setWsUrl(e.target.value);
      wsInput.addEventListener('change', onWsChange);
      wsInput.addEventListener('input', onWsChange);
      this._eventCleanups.push(() => {
        wsInput.removeEventListener('change', onWsChange);
        wsInput.removeEventListener('input', onWsChange);
      });
    }

    // 1d. Fullscreen Teach Pendant Button
    const pendantBtn = container.querySelector('[data-pendant-toggle], #btn_fullscreen_pendant');
    if (pendantBtn) {
      this._domElements.pendantBtn = pendantBtn;
      const onPendantClick = () => this.toggleFullscreenPendant();
      pendantBtn.addEventListener('click', onPendantClick);
      this._eventCleanups.push(() => pendantBtn.removeEventListener('click', onPendantClick));
    }

    // 2. Baud Rate Dropdown Select
    const baudSelect = container.querySelector('[data-baud-select], #select_baud_rate');
    if (baudSelect) {
      this._domElements.baudSelect = baudSelect;
      baudSelect.value = this.baudRate;
      const onBaudChange = (e) => this.setBaudRate(parseInt(e.target.value, 10));
      baudSelect.addEventListener('change', onBaudChange);
      this._eventCleanups.push(() => baudSelect.removeEventListener('change', onBaudChange));
    }

    // 3. Mock Serial Switch / Checkbox
    const mockToggle = container.querySelector('[data-mock-toggle], #check_mock_mode');
    if (mockToggle) {
      this._domElements.mockToggle = mockToggle;
      mockToggle.checked = this.isMock;
      const onMockChange = (e) => this.setMockMode(e.target.checked);
      mockToggle.addEventListener('change', onMockChange);
      this._eventCleanups.push(() => mockToggle.removeEventListener('change', onMockChange));
    }

    // 4. Auto-Reconnect Checkbox
    const autoReconnToggle = container.querySelector('[data-auto-reconnect], #check_auto_reconnect');
    if (autoReconnToggle) {
      this._domElements.autoReconnToggle = autoReconnToggle;
      autoReconnToggle.checked = this.autoReconnect;
      const onAutoReconnChange = (e) => this.setAutoReconnect(e.target.checked);
      autoReconnToggle.addEventListener('change', onAutoReconnChange);
      this._eventCleanups.push(() => autoReconnToggle.removeEventListener('change', onAutoReconnChange));
    }

    // 5. Connection Status Badge & Telemetry Rate Gauge
    const statusBadge = container.querySelector('[data-connection-status], #badge_connection_status');
    if (statusBadge) {
      this._domElements.statusBadge = statusBadge;
    }
    const rateDisplay = container.querySelector('[data-telemetry-rate], #display_telemetry_rate');
    if (rateDisplay) {
      this._domElements.rateDisplay = rateDisplay;
    }

    // 6. Glowing Crimson ESTOP Button
    const estopBtn = container.querySelector('[data-action="estop"], #btn_estop');
    if (estopBtn) {
      this._domElements.estopBtn = estopBtn;
      const onEstopClick = () => this.triggerEstop();
      estopBtn.addEventListener('click', onEstopClick);
      this._eventCleanups.push(() => estopBtn.removeEventListener('click', onEstopClick));
    }

    // 7. REENABLE Recovery Button
    const reenableBtn = container.querySelector('[data-action="reenable"], #btn_reenable');
    if (reenableBtn) {
      this._domElements.reenableBtn = reenableBtn;
      const onReenableClick = (e) => {
        e.preventDefault();
        this.triggerReenable();
      };
      reenableBtn.addEventListener('click', onReenableClick);
      this._eventCleanups.push(() => reenableBtn.removeEventListener('click', onReenableClick));
    }

    // 8. Homing Controls: Primary Home All + Dropdown Menu for Joint-specific Homing
    const homeBtns = container.querySelectorAll('[data-home-target]');
    const homeMenu = container.querySelector('#home_dropdown_menu');
    const homeTrigger = container.querySelector('#btn_home_menu_trigger');

    if (homeTrigger && homeMenu) {
      const toggleHomeMenu = (e) => {
        e.stopPropagation();
        const isOpen = homeMenu.classList.toggle('open');
        homeTrigger.setAttribute('aria-expanded', String(isOpen));
      };
      homeTrigger.addEventListener('click', toggleHomeMenu);
      this._eventCleanups.push(() => homeTrigger.removeEventListener('click', toggleHomeMenu));

      const closeHomeMenu = () => {
        homeMenu.classList.remove('open');
        homeTrigger.setAttribute('aria-expanded', 'false');
      };

      const onDocClick = (e) => {
        if (!homeMenu.contains(e.target) && !homeTrigger.contains(e.target)) {
          closeHomeMenu();
        }
      };
      document.addEventListener('click', onDocClick);
      this._eventCleanups.push(() => document.removeEventListener('click', onDocClick));
    }

    homeBtns.forEach((btn) => {
      const target = btn.getAttribute('data-home-target') || 'ALL';
      const onClick = () => {
        if (homeMenu) {
          homeMenu.classList.remove('open');
          if (homeTrigger) homeTrigger.setAttribute('aria-expanded', 'false');
        }
        this.triggerHome(target);
      };
      btn.addEventListener('click', onClick);
      this._eventCleanups.push(() => btn.removeEventListener('click', onClick));
    });

    const homingBadge = container.querySelector('[data-homing-status], #badge_homing_status');
    if (homingBadge) {
      this._domElements.homingBadge = homingBadge;
    }

    // 9. Speed Override Slider & Label
    const speedSlider = container.querySelector('[data-speed-slider], #slider_speed_override');
    const speedLabel = container.querySelector('[data-speed-label], #label_speed_val');
    if (speedSlider) {
      this._domElements.speedSlider = speedSlider;
      speedSlider.min = 5;
      speedSlider.max = 100;
      speedSlider.value = this.speedOverride;

      const onSpeedInput = (e) => this.setSpeedOverride(parseInt(e.target.value, 10));
      speedSlider.addEventListener('input', onSpeedInput);
      this._eventCleanups.push(() => speedSlider.removeEventListener('input', onSpeedInput));
    }
    if (speedLabel) {
      this._domElements.speedLabel = speedLabel;
    }

    // 10. Micro-Gantry Jog Nudge Buttons
    const gantryJogBtns = container.querySelectorAll('[data-gantry-jog]');
    gantryJogBtns.forEach((btn) => {
      const axis = btn.getAttribute('data-gantry-jog') || 'X';
      const delta = parseFloat(btn.getAttribute('data-step') || btn.textContent);
      const onClick = () => this.jogGantry(axis, delta);
      btn.addEventListener('click', onClick);
      this._eventCleanups.push(() => btn.removeEventListener('click', onClick));
    });

    this._updateDomConnectionUI();
    this._updateDomEstopUI();
    this._updateDomHomingUI();
    this._updateDomSpeedUI();
  }

  /**
   * Updates Connection Status UI elements
   * @private
   */
  _updateDomConnectionUI() {
    if (!this._domBound || typeof document === 'undefined') return;

    const isConn = this.isConnected();
    if (this._domElements.connectBtn) {
      this._domElements.connectBtn.textContent = isConn ? i18n.t('header.disconnect') : i18n.t('header.connect');
      this._domElements.connectBtn.className = 'btn popover-connect-btn ' + (isConn ? 'btn-danger' : 'btn-primary');
    }

    // Determine status state for both badge label and dot color
    let label = this.connectionStatus;
    let dotClass = 'dot-disconnected';

    const telemRate = Number(this.telemetryRateHz || 0);
    if (this.estopActive) {
      dotClass = 'dot-estop';
    } else if (this.isMock && isConn) {
      dotClass = 'dot-simulation';
    } else if (this.connectionStatus === ConnectionState.CONNECTED) {
      if (telemRate > 0.5) {
        label = `${i18n.t('header.status.streaming')} (${telemRate.toFixed(1)} Hz)`;
        dotClass = 'dot-streaming';
      } else {
        label = i18n.t('header.status.connected');
        dotClass = 'dot-connected';
      }
    } else if (this.connectionStatus === ConnectionState.CONNECTING || this.connectionStatus === ConnectionState.RECONNECTING) {
      label = i18n.t('header.status.connecting');
      dotClass = 'dot-connecting';
    } else if (this.connectionStatus === ConnectionState.ERROR) {
      label = i18n.t('header.status.error');
      dotClass = 'dot-error';
    } else if (this.connectionStatus === ConnectionState.DISCONNECTED) {
      label = i18n.t('header.status.disconnected');
      dotClass = 'dot-disconnected';
    }

    // Update status pill label
    if (this._domElements.statusBadge) {
      this._domElements.statusBadge.textContent = label;
    }

    // Update status dot color
    if (this._domElements.statusDot) {
      this._domElements.statusDot.className = `status-dot ${dotClass}`;
    }

    if (this._domElements.rateDisplay) {
      this._domElements.rateDisplay.textContent = `${telemRate.toFixed(1)} Hz`;
    }

    if (this._domElements.mockToggle && this._domElements.mockToggle.checked !== this.isMock) {
      this._domElements.mockToggle.checked = this.isMock;
    }

    // Sync transport toggle buttons active state
    if (this.container) {
      const transportBtns = this.container.querySelectorAll('[data-transport]');
      transportBtns.forEach((btn) => {
        const mode = btn.getAttribute('data-transport');
        if (mode === this.transportMode) {
          btn.classList.add('active');
        } else {
          btn.classList.remove('active');
        }
      });

      const wsGroup = this.container.querySelector('#ws_config_group');
      if (wsGroup) {
        wsGroup.style.display = (this.transportMode === 'websocket') ? '' : 'none';
      }
    }

    if (this._domElements.baudSelect) {
      this._domElements.baudSelect.style.display = (this.transportMode === 'websocket') ? 'none' : '';
    }

    if (this._domElements.wsInput && this._domElements.wsInput.value !== this.wsUrl) {
      this._domElements.wsInput.value = this.wsUrl;
    }
  }

  /**
   * Updates ESTOP button visual glowing/alert state
   * @private
   */
  _updateDomEstopUI() {
    if (!this._domBound || typeof document === 'undefined') return;

    if (this._domElements.estopBtn) {
      if (this.estopActive) {
        this._domElements.estopBtn.classList.add('estop-active', 'flashing-alert');
      } else {
        this._domElements.estopBtn.classList.remove('estop-active', 'flashing-alert');
      }
    }

    // Toggle safety zone hazard state (shows/hides REENABLE, adds red background)
    if (this._domElements.safetyZone) {
      if (this.estopActive) {
        this._domElements.safetyZone.classList.add('estop-tripped');
      } else {
        this._domElements.safetyZone.classList.remove('estop-tripped');
      }
    }

    if (this._domElements.reenableBtn) {
      this._domElements.reenableBtn.disabled = false;
      if (this.estopActive) {
        this._domElements.reenableBtn.classList.add('recovery-mode');
        this._domElements.reenableBtn.title = 'Clear Emergency Stop and re-energize steppers';
      } else {
        this._domElements.reenableBtn.classList.remove('recovery-mode');
        this._domElements.reenableBtn.title = 'Re-enable & energize stepper motors';
      }
    }

    // Sync status dot to reflect ESTOP state
    this._updateDomConnectionUI();
  }

  /**
   * Updates Homing status UI elements
   * @private
   */
  _updateDomHomingUI() {
    if (!this._domBound || typeof document === 'undefined') return;

    if (this._domElements.homingBadge) {
      this._domElements.homingBadge.textContent = this.homingActive ? i18n.t('header.home.status.homing') : i18n.t('header.home.status.idle');
      this._domElements.homingBadge.className = 'badge ' + (this.homingActive ? 'badge-warning pulsing' : 'badge-idle');
    }
  }

  /**
   * Updates Speed Override slider and value text
   * @private
   */
  _updateDomSpeedUI() {
    if (!this._domBound || typeof document === 'undefined') return;

    if (this._domElements.speedSlider && Number(this._domElements.speedSlider.value) !== this.speedOverride) {
      this._domElements.speedSlider.value = this.speedOverride;
    }
    if (this._domElements.speedLabel) {
      this._domElements.speedLabel.textContent = `${this.speedOverride}%`;
    }
  }

  // ==========================================================================
  // CONNECTION POPOVER MANAGEMENT
  // ==========================================================================

  /**
   * Toggles the connection settings popover open/closed
   * @private
   */
  _togglePopover() {
    if (this._popoverOpen) {
      this._closePopover();
    } else {
      this._openPopover();
    }
  }

  /**
   * Opens the connection settings popover with backdrop for click-outside dismiss
   * @private
   */
  _openPopover() {
    if (!this._domElements.popover || !this._domElements.statusPill) return;

    this._popoverOpen = true;
    this._domElements.popover.classList.add('popover-open');
    this._domElements.statusPill.setAttribute('aria-expanded', 'true');

    // Click-outside dismissal via document listener (delayed to skip the opening click)
    this._popoverOutsideHandler = (e) => {
      const popover = this._domElements.popover;
      const pill = this._domElements.statusPill;
      if (popover && !popover.contains(e.target) && pill && !pill.contains(e.target)) {
        this._closePopover();
      }
    };
    setTimeout(() => {
      document.addEventListener('click', this._popoverOutsideHandler, true);
    }, 0);

    // Escape key to close (but not trigger ESTOP)
    this._popoverEscHandler = (e) => {
      if (e.key === 'Escape' || e.code === 'Escape') {
        e.stopPropagation();
        e.preventDefault();
        this._closePopover();
      }
    };
    window.addEventListener('keydown', this._popoverEscHandler, { capture: true });
  }

  /**
   * Closes the connection settings popover and cleans up listeners
   * @private
   */
  _closePopover() {
    if (!this._domElements.popover) return;

    this._popoverOpen = false;
    this._domElements.popover.classList.remove('popover-open');

    if (this._domElements.statusPill) {
      this._domElements.statusPill.setAttribute('aria-expanded', 'false');
    }

    if (this._popoverOutsideHandler) {
      document.removeEventListener('click', this._popoverOutsideHandler, true);
      this._popoverOutsideHandler = null;
    }

    if (this._popoverEscHandler) {
      window.removeEventListener('keydown', this._popoverEscHandler, { capture: true });
      this._popoverEscHandler = null;
    }
  }

  /**
   * Unbinds DOM elements
   */
  unbindDom() {
    this._closePopover();

    if (this._reenableHoldTimer) {
      clearTimeout(this._reenableHoldTimer);
      this._reenableHoldTimer = null;
    }

    for (const cleanup of this._eventCleanups) {
      try {
        cleanup();
      } catch (_) {}
    }
    this._eventCleanups = [];
    this._domElements = {};
    this._domBound = false;
    this.container = null;
  }

  /**
   * Cleans up all event listeners and keyboard shortcuts
   */
  dispose() {
    this.unbindDom();

    if (this._keyHandler && typeof window !== 'undefined') {
      window.removeEventListener('keydown', this._keyHandler, { capture: true });
      this._keyHandler = null;
    }

    for (const unsub of this._unsubState) {
      try {
        unsub();
      } catch (_) {}
    }
    this._unsubState = [];
  }
}

export default HeaderController;
