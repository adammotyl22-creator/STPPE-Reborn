/**
 * app.js - Master Application Bootstrap & Lifecycle Orchestrator
 * Teensy 4.1 6-DOF Robot Arm Automation Studio
 */

import { appState, AppState } from './state/AppState.js';
import { presetManager } from './state/PresetManager.js';
import { WebSerialManager, ConnectionState } from './comms/WebSerialManager.js';
import { RobotVisualizer3D } from './visualizer/RobotVisualizer3D.js';
import { HeaderController } from './ui/HeaderController.js';
import { MotionController } from './ui/MotionController.js';
import { TrajectoryController } from './ui/TrajectoryController.js';
import { TelemetryController } from './ui/TelemetryController.js';
import { ConsoleController } from './ui/ConsoleController.js';
import { CameraController } from './camera/CameraController.js';
import { HorizontalTimelineController } from './ui/HorizontalTimelineController.js';
import { GamepadController } from './comms/GamepadController.js';
import { i18n } from './i18n/I18nManager.js';

export const STORAGE_KEYS = Object.freeze({
  ACCORDION_STATE: 'robot_arm_accordion_state',
  DOCK_COLLAPSED: 'robot_arm_dock_collapsed',
  DOCK_ACTIVE_TAB: 'robot_arm_dock_active_tab',
  MOBILE_OPTIMIZATIONS: 'robot_arm_mobile_optimizations'
});

export class RobotArmApp {
  constructor(options = {}) {
    this.appState = options.appState || appState;
    this.presetManager = options.presetManager || presetManager;
    this.storage = options.storageEngine || options.storage || this._detectStorageEngine();
    this.serialManager = null;
    this.visualizer = null;
    this.headerController = null;
    this.motionController = null;
    this.trajectoryController = null;
    this.telemetryController = null;
    this.consoleController = null;
    this.cameraController = null;
    this.timelineController = null;
    this.gamepadController = null;
    this._initialized = false;
  }

  /**
   * Detects whether browser localStorage is accessible or returns null
   * @private
   */
  _detectStorageEngine() {
    if (typeof window !== 'undefined' && window.localStorage) {
      try {
        const testKey = '__arm_storage_test__';
        window.localStorage.setItem(testKey, testKey);
        window.localStorage.removeItem(testKey);
        return window.localStorage;
      } catch (e) {}
    }
    if (typeof localStorage !== 'undefined') {
      try {
        const testKey = '__arm_storage_test__';
        localStorage.setItem(testKey, testKey);
        localStorage.removeItem(testKey);
        return localStorage;
      } catch (e) {}
    }
    return null;
  }

  /**
   * Safe getter for storage engine
   * @private
   */
  _getItem(key) {
    try {
      if (this.storage) return this.storage.getItem(key);
    } catch (e) {
      console.warn(`RobotArmApp: Failed to read key "${key}" from storage:`, e);
    }
    return null;
  }

  /**
   * Safe setter for storage engine
   * @private
   */
  _setItem(key, value) {
    try {
      if (this.storage) this.storage.setItem(key, String(value));
    } catch (e) {
      console.warn(`RobotArmApp: Failed to write key "${key}" to storage:`, e);
    }
  }

  /**
   * Initializes and boots all application controllers and visualizers
   */
  async init() {
    if (this._initialized) return;
    this._initialized = true;

    console.log('%cTEENSY 4.1 ROBOT ARM CAD WORKSPACE INITIALIZING...', 'color: #10b981; font-weight: bold; font-size: 14px;');

    // 1. Initialize WebSerial Manager
    this.serialManager = new WebSerialManager({
      baudRate: 115200,
      autoEnableJTelem: true
    });

    // Wire up serialManager events to AppState
    this.serialManager.on('statusChange', ({ newState, error }) => {
      this.appState.setConnectionStatus(newState, {
        baudRate: this.serialManager.options.baudRate,
        isMock: this.serialManager.isMock
      });
      if (newState === 'CONNECTED') {
        this.appState.setClosedLoop(false);
      }
      if (error) {
        this.appState.addConsoleLog({ type: 'ERR', text: `Serial error: ${error.message || error}` });
      }
    });

    this.serialManager.on('jtelem', (pkt) => {
      this.appState.updateLiveTelemetry(pkt);
    });

    this.serialManager.on('ar4', (pkt) => {
      this.appState.updateLiveTelemetry(pkt);
      this.appState.addConsoleLog({ type: 'RX', text: pkt.raw || 'AR4 TELEM' });
    });

    this.serialManager.on('collision', (pkt) => {
      this.appState.setCollision(pkt.j1, pkt.j2, pkt.j3);
      this.appState.addConsoleLog({
        type: 'COLLISION',
        text: `COLLISION DETECTED: J1=${pkt.j1} J2=${pkt.j2} J3=${pkt.j3} (${pkt.raw || ''})`
      });
    });

    this.serialManager.on('rateChange', ({ rateHz }) => {
      this.appState.setState('connection.telemRateHz', rateHz);
    });

    // 2. Initialize 3D Kinematics Visualizer
    const canvasContainer = document.getElementById('visualizer-canvas-wrapper') || document.getElementById('viewport-container');
    if (canvasContainer) {
      try {
        this.visualizer = new RobotVisualizer3D(canvasContainer, {
          antialias: true,
          shadows: true,
          showGhost: true,
          showWorkspace: true,
          showFloorAlert: true,
          showExclusionZone: true,
          showGrid: true
        });

        this._bindVisualizerEvents();
      } catch (err) {
        console.error('Failed to initialize 3D Visualizer:', err);
      }
    }
    this._initMobileOptimizationsToggle();

    // 3. Initialize Header Controller (Navigation, Serial Connection, ESTOP)
    const headerContainer = document.getElementById('header-container');
    if (headerContainer) {
      this.headerController = new HeaderController({
        container: headerContainer,
        appState: this.appState,
        serialManager: this.serialManager,
        defaultBaudRate: 115200,
        enableShortcuts: true
      });
    }

    // 4. Initialize Motion Controller (Joint Sliders, Cartesian IK, MOV/MOVL)
    const motionContainer = document.getElementById('motion-panel-container') || document.getElementById('left-panel');
    if (motionContainer) {
      this.motionController = new MotionController({
        container: motionContainer,
        appState: this.appState,
        serialManager: this.serialManager
      });
    }

    const settingsPanel = document.getElementById('settings-panel-content');
    const motionEngineCard = document.getElementById('card-motion-engine');
    const cartesianKeypad = document.getElementById('cartesian-keypad-card');
    if (settingsPanel && motionContainer) {
      motionContainer.appendChild(settingsPanel);
    }
    if (settingsPanel && motionEngineCard) {
      settingsPanel.appendChild(motionEngineCard);
      motionEngineCard.classList.add('open');
      motionEngineCard.querySelector('.accordion-header')?.setAttribute('aria-expanded', 'true');
    }
    if (settingsPanel && cartesianKeypad) {
      settingsPanel.appendChild(cartesianKeypad);
    }

    // 5. Initialize Trajectory Controller (Waypoint Timeline, Player, Teach/Playback)
    const trajectoryContainer = document.getElementById('trajectory-container');
    if (trajectoryContainer) {
      this.trajectoryController = new TrajectoryController({
        container: trajectoryContainer,
        appState: this.appState,
        serialManager: this.serialManager,
        presetManager: this.presetManager
      });
    }

    // 6. Initialize Bottom Dock: Telemetry Charts & Console Terminal
    this._initBottomDock();

    // Bind layout controls before camera device enumeration can delay startup.
    this._initPanelToggles();
    this._initFocusView();

    // 7. Initialize Eye-in-Hand USB Vision Camera Controller
    const pipHud = (typeof document !== 'undefined') ? document.getElementById('camera-hud-container') : null;
    const dockVision = (typeof document !== 'undefined') ? document.getElementById('vision-split-container') : null;
    if (pipHud || dockVision) {
      try {
        this.cameraController = new CameraController({
          appState: this.appState,
          pipContainer: pipHud,
          dockContainer: dockVision
        });
        await this.cameraController.init();
      } catch (err) {
        console.warn('Failed to initialize CameraController:', err);
      }
    }

    // 8. Initialize CAD Dock Panel Accordions
    this._initAccordions();

    // 8b. Initialize Native Gamepad Controller
    this.gamepadController = new GamepadController({
      motionController: this.motionController,
      trajectoryController: this.trajectoryController,
      appState: this.appState
    });

    // 9. Bind Global Viewport HUD & Keyboard Shortcuts
    this._bindGlobalHUD();
    this._bindGlobalKeyboardShortcuts();

    // 10. Initialize language toggle (must be after all DOM is ready)
    this._initI18n();

    // Startup banner in console
    this.appState.addConsoleLog({
      type: 'INFO',
      text: i18n.t('console.ready')
    });

    console.log('%cALL SUBSYSTEMS OPERATIONAL', 'color: #10b981; font-weight: bold;');
  }

  /**
   * Initializes Left and Right collapsible inspector panels
   * @private
   */
  _initPanelToggles() {
    // Left Panel (Kinematics Inspector)
    const leftPanel = document.getElementById('left-panel');
    const btnLeftCollapse = document.getElementById('btn-left-collapse');
    const btnLeftExpand = document.getElementById('btn-left-expand');

    if (leftPanel && btnLeftCollapse) {
      btnLeftCollapse.addEventListener('click', () => {
        this._leaveFocusView(true);
        leftPanel.classList.add('panel-collapsed');
        this._handleLayoutResize();
      });
    }

    if (leftPanel && btnLeftExpand) {
      btnLeftExpand.addEventListener('click', () => {
        this._leaveFocusView(true);
        leftPanel.classList.remove('panel-collapsed');
        this._handleLayoutResize();
      });
    }

    // Right Panel (Trajectory Studio)
    const rightPanel = document.getElementById('right-panel');
    const btnRightCollapse = document.getElementById('btn-right-collapse');
    const btnRightExpand = document.getElementById('btn-right-expand');

    if (rightPanel && btnRightCollapse) {
      btnRightCollapse.addEventListener('click', () => {
        rightPanel.classList.add('panel-collapsed');
        this._handleLayoutResize();
      });
    }

    if (rightPanel && btnRightExpand) {
      btnRightExpand.addEventListener('click', () => {
        rightPanel.classList.remove('panel-collapsed');
        this._handleLayoutResize();
      });
    }
  }

  /** Temporarily gives the viewport the space occupied by the controls and dock. */
  _initFocusView() {
    const button = document.getElementById('btn-focus-view');
    const leftPanel = document.getElementById('left-panel');
    const bottomDock = document.getElementById('bottom-dock');
    if (!button || !leftPanel || !bottomDock) return;

    button.addEventListener('click', () => {
      if (this._focusViewState) {
        this._leaveFocusView(true);
        return;
      }
      this._focusViewState = {
        leftCollapsed: leftPanel.classList.contains('panel-collapsed'),
        dockCollapsed: bottomDock.classList.contains('dock-collapsed')
      };
      leftPanel.classList.add('panel-collapsed', 'focus-view-collapsed');
      bottomDock.classList.add('dock-collapsed', 'focus-view-collapsed');
      this._updateFocusViewButton();
      const dockToggle = document.getElementById('btn-dock-toggle');
      if (dockToggle) dockToggle.textContent = i18n.t('dock.expand');
      this._handleLayoutResize();
    });
    this._updateFocusViewButton();
  }

  _updateFocusViewButton() {
    const button = document.getElementById('btn-focus-view');
    if (!button) return;
    const active = Boolean(this._focusViewState);
    const label = i18n.t(active ? 'viewport.restore' : 'viewport.focus');
    button.setAttribute('aria-pressed', String(active));
    button.setAttribute('aria-label', label);
    button.title = label;
    const text = button.querySelector('.focus-view-label');
    if (text) text.textContent = label;
  }

  _leaveFocusView(restore) {
    if (!this._focusViewState) return;
    const { leftCollapsed, dockCollapsed } = this._focusViewState;
    this._focusViewState = null;
    const leftPanel = document.getElementById('left-panel');
    const bottomDock = document.getElementById('bottom-dock');
    leftPanel?.classList.remove('focus-view-collapsed');
    bottomDock?.classList.remove('focus-view-collapsed');
    if (restore) {
      leftPanel?.classList.toggle('panel-collapsed', leftCollapsed);
      bottomDock?.classList.toggle('dock-collapsed', dockCollapsed);
    }
    this._updateFocusViewButton();
    const dockToggle = document.getElementById('btn-dock-toggle');
    if (dockToggle && bottomDock) {
      dockToggle.textContent = i18n.t(bottomDock.classList.contains('dock-collapsed') ? 'dock.expand' : 'dock.collapse');
    }
    this._handleLayoutResize();
  }

  _initMobileOptimizationsToggle() {
    const toggle = document.getElementById('toggle-mobile-optimizations');
    if (!toggle) return;

    const savedSetting = this._getItem(STORAGE_KEYS.MOBILE_OPTIMIZATIONS);
    const prefersMobileQuality = typeof window !== 'undefined'
      && window.matchMedia('(max-width: 760px)').matches;
    const enabled = savedSetting === null ? prefersMobileQuality : savedSetting === 'true';

    toggle.checked = enabled;
    toggle.addEventListener('change', () => {
      this._setItem(STORAGE_KEYS.MOBILE_OPTIMIZATIONS, toggle.checked ? 'true' : 'false');
      if (this.visualizer) this.visualizer.setMobileOptimized(toggle.checked);
    });
    if (this.visualizer) this.visualizer.setMobileOptimized(enabled);
  }

  /**
   * Initializes left panel accordion cards and restores persistence state
   * @private
   */
  _initAccordions() {
    // 1. Read and apply saved accordion state
    let savedState = null;
    try {
      const raw = this._getItem(STORAGE_KEYS.ACCORDION_STATE);
      if (raw) {
        savedState = JSON.parse(raw);
      }
    } catch (err) {
      console.warn('RobotArmApp: Error parsing accordion state JSON from storage:', err);
      savedState = null;
    }

    const cards = (typeof document !== 'undefined') ? document.querySelectorAll('.accordion-card[data-accordion]') : [];
    if (savedState && typeof savedState === 'object') {
      cards.forEach(card => {
        const key = card.getAttribute('data-accordion');
        let isOpen;
        if (key === 'ik') isOpen = savedState.ik;
        else if (key === 'joints') isOpen = (savedState.joints !== undefined ? savedState.joints : savedState.fk);
        else if (key === 'trajectory') isOpen = (savedState.trajectory !== undefined ? savedState.trajectory : savedState.traj);
        else if (key && key in savedState) isOpen = savedState[key];

        if (typeof isOpen === 'boolean') {
          card.classList.toggle('open', isOpen);
          const header = card.querySelector ? card.querySelector('.accordion-header') : null;
          if (header) {
            header.setAttribute('aria-expanded', String(isOpen));
          }
        }
      });
    }

    // 2. Wire accordion toggle event listeners
    const headers = (typeof document !== 'undefined') ? document.querySelectorAll('.accordion-header') : [];
    headers.forEach(header => {
      header.addEventListener('click', () => {
        const card = header.closest ? header.closest('.accordion-card') : null;
        if (!card) return;
        const isOpen = card.classList.toggle('open');
        header.setAttribute('aria-expanded', String(isOpen));

        // Synchronize mode tabs with the newly open card
        if (isOpen) {
          const cardKey = card.getAttribute('data-accordion');
          modeTabs.forEach(t => {
            const isCurrent = (t.getAttribute('data-mode-tab') === cardKey);
            t.classList.toggle('active', isCurrent);
            t.setAttribute('aria-selected', String(isCurrent));
          });
        }

        this._saveAccordionState();
        this._handleLayoutResize();
      });
      header.addEventListener('keydown', (e) => {
        if (e.key === 'Enter' || e.key === ' ') {
          e.preventDefault();
          header.click();
        }
      });
    });

    // 3. Wire mode tab switches for quick uncluttered navigation
    const modeTabs = (typeof document !== 'undefined') ? document.querySelectorAll('[data-mode-tab]') : [];

    // Synchronize initial mode tab active state with the first open card
    const initialOpenCard = Array.from(cards).find(c => c.classList.contains('open'));
    if (initialOpenCard) {
      const initialKey = initialOpenCard.getAttribute('data-accordion');
      modeTabs.forEach(t => {
        const isCurrent = (t.getAttribute('data-mode-tab') === initialKey);
        t.classList.toggle('active', isCurrent);
        t.setAttribute('aria-selected', String(isCurrent));
      });
      if (this.motionController) {
        this.motionController.setControlModeTab(initialKey);
      }
    }
    modeTabs.forEach(tabBtn => {
      tabBtn.addEventListener('click', () => {
        const targetMode = tabBtn.getAttribute('data-mode-tab');
        if (this.motionController) {
          this.motionController.setControlModeTab(targetMode);
        }
        modeTabs.forEach(t => {
          const isCurrent = t === tabBtn;
          t.classList.toggle('active', isCurrent);
          t.setAttribute('aria-selected', String(isCurrent));
        });

        cards.forEach(card => {
          const key = card.getAttribute('data-accordion');
          const isTarget = (key === targetMode);
          card.classList.toggle('open', isTarget);
          const header = card.querySelector ? card.querySelector('.accordion-header') : null;
          if (header) {
            header.setAttribute('aria-expanded', String(isTarget));
          }
          if (isTarget && typeof card.scrollIntoView === 'function') {
            card.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
          }
        });

        this._saveAccordionState();
        this._handleLayoutResize();
      });
    });
  }

  /**
   * Serializes current accordion open/closed states and writes to storage
   * @private
   */
  _saveAccordionState() {
    try {
      const state = {};
      const cards = (typeof document !== 'undefined') ? document.querySelectorAll('.accordion-card[data-accordion]') : [];
      cards.forEach(card => {
        const key = card.getAttribute('data-accordion');
        if (key) {
          state[key] = card.classList.contains('open');
        }
      });
      this._setItem(STORAGE_KEYS.ACCORDION_STATE, JSON.stringify(state));
    } catch (err) {
      console.warn('RobotArmApp: Failed to serialize and save accordion state:', err);
    }
  }

  /**
   * Triggers viewport and chart reflows after panel animations
   * @private
   */
  _handleLayoutResize() {
    if (this.visualizer) {
      this.visualizer.onResize();
      setTimeout(() => this.visualizer && this.visualizer.onResize(), 260);
    }
    if (this.telemetryController && typeof this.telemetryController.renderCharts === 'function') {
      setTimeout(() => this.telemetryController.renderCharts(), 260);
    }
  }

  /**
   * Binds AppState reactive updates to 3D Robot Visualizer
   * @private
   */
  _bindVisualizerEvents() {
    if (!this.visualizer) return;

    // Bind Interactive 3D TCP Drag & Drop to MotionController IK
    this.visualizer.onTcpDrag = (coords) => {
      if (!this.motionController || !coords) return;
      if (coords.pitch !== undefined) {
        this.motionController.setOrientationTarget('pitch', coords.pitch, true, false);
      } else if (coords.roll !== undefined) {
        this.motionController.setOrientationTarget('roll', coords.roll, true, false);
      } else if (coords.pitchDelta !== undefined) {
        this.motionController.nudgeOrientation('pitch', coords.pitchDelta, false);
      } else if (coords.rollDelta !== undefined) {
        this.motionController.nudgeOrientation('roll', coords.rollDelta, false);
      } else {
        this.motionController.setCartesianTarget(
          coords.x,
          coords.y,
          coords.z,
          null,
          null,
          false,
          true
        );
      }
    };

    this.visualizer.onTcpDragEnd = () => {
      if (this.motionController && typeof this.motionController.flushFinalTarget === 'function') {
        this.motionController.flushFinalTarget();
      }
    };

    // Update Live Arm Pose on Telemetry Stream and State Change
    const updateLive = (live) => {
      if (live && this.visualizer) {
        this.visualizer.updateLivePose(
          Number(live.j1 ?? 0),
          Number(live.j2 ?? 0),
          Number(live.j3 ?? 0),
          Number(live.j4 ?? 0),
          Number(live.j5 ?? 0),
          Number(live.j6 ?? 0),
          Number(live.grip ?? 0)
        );
      }
    };
    this.appState.subscribe('telemetry:updated', updateLive);
    this.appState.subscribe('stateChange:livePose', updateLive);

    // Active Status Feedback: Breathing pulse & Mode-Aware Perimeter Framing on 3D Viewport
    const viewportContainer = document.getElementById('viewport-container');
    let isFloorBreach = false;
    const syncViewportPerimeter = () => {
      if (!viewportContainer) return;
      const isEstop = Boolean(this.appState.getState('safety.estop'));
      const execMode = this.appState.getState('executionMode.mode') ||
        (this.motionController && this.motionController.executionMode) ||
        'preview';
      const isDirect = (execMode === 'direct_drive' && (!this.motionController || this.motionController.isArmed));
      viewportContainer.classList.toggle('mode-estop', isEstop);
      viewportContainer.classList.toggle('mode-floor-breach', !isEstop && isFloorBreach);
      viewportContainer.classList.toggle('mode-direct-drive', !isEstop && !isFloorBreach && isDirect);
      viewportContainer.classList.toggle('mode-preview', !isEstop && !isFloorBreach && !isDirect);
    };
    syncViewportPerimeter();

    const updateMotionPulse = (telem) => {
      if (viewportContainer) {
        const isMoving = Boolean(telem && (telem.isMoving || (typeof telem.status === 'string' && telem.status.toUpperCase() === 'MOVING')));
        viewportContainer.classList.toggle('robot-moving', isMoving);
        if (telem && telem.z !== undefined) {
          isFloorBreach = Number(telem.z) <= 20;
        }
        syncViewportPerimeter();
      }
    };
    this.appState.subscribe('telemetry:updated', updateMotionPulse);
    this.appState.subscribe('safety:estop', syncViewportPerimeter);
    this.appState.subscribe('executionMode:changed', syncViewportPerimeter);
    this.appState.subscribe('stateChange:executionMode', syncViewportPerimeter);

    // 2D <-> 3D Joint Hover Highlighting
    const onHighlightJoint = (joint) => {
      if (this.visualizer && typeof this.visualizer.setHighlightedJoint === 'function') {
        this.visualizer.setHighlightedJoint(joint);
      }
    };
    this.appState.subscribe('ui:highlightJoint', onHighlightJoint);

    // Update Ghost Target Arm Pose on Motion/IK Changes
    const updateGhost = (ghost) => {
      if (ghost && this.visualizer) {
        this.visualizer.updateGhostPose(
          Number(ghost.j1 ?? 0),
          Number(ghost.j2 ?? 0),
          Number(ghost.j3 ?? 0),
          Number(ghost.j4 ?? 0),
          Number(ghost.j5 ?? 0),
          Number(ghost.j6 ?? 0),
          Number(ghost.grip ?? 0)
        );
      }
    };
    this.appState.subscribe('ghost:updated', updateGhost);
    this.appState.subscribe('ghostPose:updated', updateGhost);
    this.appState.subscribe('stateChange:ghostPose', updateGhost);

    // Update 3D Trajectory Toolpath on Sequence Changes
    const updateTrajectory = (seq) => {
      if (seq && this.visualizer) {
        this.visualizer.updateTrajectoryPath(seq);
        if (typeof seq.activeIndex === 'number') {
          this.visualizer.setActiveTrajectoryWaypoint(seq.activeIndex);
        }
      }
    };
    this.appState.subscribe('sequence:updated', updateTrajectory);
    this.appState.subscribe('stateChange:sequence', updateTrajectory);

    // Initial trajectory path render
    const initialSeq = this.appState.getState('sequence');
    if (initialSeq) {
      this.visualizer.updateTrajectoryPath(initialSeq);
    }

    // Camera preset buttons
    const cameraBtns = document.querySelectorAll('[data-camera-preset]');
    cameraBtns.forEach((btn) => {
      const preset = btn.getAttribute('data-camera-preset');
      btn.addEventListener('click', () => {
        if (this.visualizer) {
          this.visualizer.setCameraPreset(preset);
          cameraBtns.forEach(b => b.classList.remove('active'));
          btn.classList.add('active');
        }
      });
    });

    // Viewport Vision & Layer Controls Popover Menu
    const btnVision = document.getElementById('btn-viewport-vision');
    const popoverVision = document.getElementById('viewport-vision-popover');
    const btnCloseVision = document.getElementById('btn-vision-popover-close');

    if (btnVision && popoverVision) {
      const toggleVisionPopover = (force) => {
        const isHidden = popoverVision.classList.contains('hidden');
        const show = force !== undefined ? force : isHidden;
        popoverVision.classList.toggle('hidden', !show);
        btnVision.classList.toggle('active', show);
        btnVision.setAttribute('aria-expanded', String(show));
      };

      btnVision.addEventListener('click', (e) => {
        e.stopPropagation();
        toggleVisionPopover();
      });

      if (btnCloseVision) {
        btnCloseVision.addEventListener('click', (e) => {
          e.stopPropagation();
          toggleVisionPopover(false);
        });
      }

      // Dismiss on click outside
      document.addEventListener('click', (e) => {
        if (!popoverVision.classList.contains('hidden')) {
          const menuWrapper = document.getElementById('viewport-vision-menu');
          if (menuWrapper && !menuWrapper.contains(e.target)) {
            toggleVisionPopover(false);
          }
        }
      });

      // Dismiss on Escape key
      document.addEventListener('keydown', (e) => {
        if (e.key === 'Escape' && !popoverVision.classList.contains('hidden')) {
          toggleVisionPopover(false);
        }
      });
    }

    // 3D Layer visibility toggles
    const chkLive = document.getElementById('chk-live-visible');
    if (chkLive) {
      chkLive.addEventListener('change', (e) => this.visualizer.setLiveVisible(e.target.checked));
    }

    const chkGhost = document.getElementById('chk-ghost-visible');
    if (chkGhost) {
      chkGhost.addEventListener('change', (e) => this.visualizer.setGhostVisible(e.target.checked));
    }

    const chkPath = document.getElementById('chk-path-visible');
    if (chkPath) {
      chkPath.addEventListener('change', (e) => this.visualizer.setTrajectoryPathVisible(e.target.checked));
    }

    const chkFloorAlert = document.getElementById('chk-floor-alert-visible');
    if (chkFloorAlert) {
      chkFloorAlert.addEventListener('change', (e) => this.visualizer.setFloorAlertVisible(e.target.checked));
    }

    const chkExclusion = document.getElementById('chk-exclusion-visible');
    if (chkExclusion) {
      chkExclusion.addEventListener('change', (e) => this.visualizer.setExclusionZoneVisible(e.target.checked));
    }

    const chkReachEnvelope = document.getElementById('chk-reach-envelope-visible');
    if (chkReachEnvelope) {
      chkReachEnvelope.addEventListener('change', (e) => this.visualizer.setReachEnvelopeVisible(e.target.checked));
    }

    const chkGrid = document.getElementById('chk-grid-visible');
    if (chkGrid) {
      chkGrid.addEventListener('change', (e) => this.visualizer.setGridVisible(e.target.checked));
    }

    // Handle Window Resize
    window.addEventListener('resize', () => {
      this._handleLayoutResize();
    });
  }

  /**
   * Initializes Telemetry and Console Controllers in the Bottom Dock and restores persistence state
   * @private
   */
  _initBottomDock() {
    const telemContainer = (typeof document !== 'undefined') ? document.getElementById('telemetry-split-container') : null;
    if (telemContainer) {
      this.telemetryController = new TelemetryController({
        container: telemContainer,
        appState: this.appState,
        serialManager: this.serialManager,
        windowDurationSec: 30
      });
    }

    const consoleContainer = (typeof document !== 'undefined') ? document.getElementById('console-split-container') : null;
    if (consoleContainer) {
      this.consoleController = new ConsoleController({
        container: consoleContainer,
        appState: this.appState,
        serialManager: this.serialManager,
        autoScroll: true
      });
    }

    // Horizontal Waypoint Timeline Controller
    const timelineContainer = (typeof document !== 'undefined') ? document.getElementById('timeline-component-container') : null;
    if (timelineContainer) {
      this.timelineController = new HorizontalTimelineController({
        container: timelineContainer,
        appState: this.appState,
        trajectoryController: this.trajectoryController
      });
    }

    // Dock Tabs Switching
    const tabBtns = (typeof document !== 'undefined') ? document.querySelectorAll('[data-dock-tab]') : [];
    const viewSplit = (typeof document !== 'undefined') ? document.getElementById('dock-view-split') : null;
    const viewTelem = (typeof document !== 'undefined') ? document.getElementById('dock-view-telemetry') : null;
    const viewConsole = (typeof document !== 'undefined') ? document.getElementById('dock-view-console') : null;
    const viewVision = (typeof document !== 'undefined') ? document.getElementById('dock-view-vision') : null;
    const viewTimeline = (typeof document !== 'undefined') ? document.getElementById('dock-view-timeline') : null;

    const btnDockCamSnap = (typeof document !== 'undefined') ? document.getElementById('btn-dock-cam-snap') : null;
    if (btnDockCamSnap) {
      btnDockCamSnap.addEventListener('click', () => {
        if (this.cameraController) this.cameraController.captureSnapshot();
      });
    }

    const switchTab = (tab, save = true) => {
      tabBtns.forEach((btn) => {
        btn.classList.toggle('active', btn.getAttribute('data-dock-tab') === tab);
      });

      const telemEl = this.telemetryController && this.telemetryController.container 
        ? this.telemetryController.container.querySelector('.telemetry-panel') 
        : null;
      const consoleEl = this.consoleController && this.consoleController.container 
        ? this.consoleController.container.querySelector('.console-panel') 
        : null;

      if (tab === 'split') {
        if (viewSplit) viewSplit.className = 'dock-view-split active';
        if (viewTelem) viewTelem.className = 'dock-view-pane';
        if (viewConsole) viewConsole.className = 'dock-view-pane';
        if (viewVision) viewVision.className = 'dock-view-pane';
        if (viewTimeline) viewTimeline.className = 'dock-view-pane';
        
        if (telemContainer && telemEl && !telemContainer.contains(telemEl)) {
          telemContainer.appendChild(telemEl);
        }
        if (consoleContainer && consoleEl && !consoleContainer.contains(consoleEl)) {
          consoleContainer.appendChild(consoleEl);
        }
      } else if (tab === 'telemetry') {
        if (viewSplit) viewSplit.className = 'dock-view-split';
        if (viewTelem) viewTelem.className = 'dock-view-pane active';
        if (viewConsole) viewConsole.className = 'dock-view-pane';
        if (viewVision) viewVision.className = 'dock-view-pane';
        if (viewTimeline) viewTimeline.className = 'dock-view-pane';
        
        const fullTelem = (typeof document !== 'undefined') ? document.getElementById('telemetry-full-container') : null;
        if (fullTelem && telemEl && !fullTelem.contains(telemEl)) {
          fullTelem.appendChild(telemEl);
        }
      } else if (tab === 'console') {
        if (viewSplit) viewSplit.className = 'dock-view-split';
        if (viewTelem) viewTelem.className = 'dock-view-pane';
        if (viewConsole) viewConsole.className = 'dock-view-pane active';
        if (viewVision) viewVision.className = 'dock-view-pane';
        if (viewTimeline) viewTimeline.className = 'dock-view-pane';
        
        const fullConsole = (typeof document !== 'undefined') ? document.getElementById('console-full-container') : null;
        if (fullConsole && consoleEl && !fullConsole.contains(consoleEl)) {
          fullConsole.appendChild(consoleEl);
        }
      } else if (tab === 'vision') {
        if (viewSplit) viewSplit.className = 'dock-view-split';
        if (viewTelem) viewTelem.className = 'dock-view-pane';
        if (viewConsole) viewConsole.className = 'dock-view-pane';
        if (viewVision) viewVision.className = 'dock-view-pane active';
        if (viewTimeline) viewTimeline.className = 'dock-view-pane';
      } else if (tab === 'timeline') {
        if (viewSplit) viewSplit.className = 'dock-view-split';
        if (viewTelem) viewTelem.className = 'dock-view-pane';
        if (viewConsole) viewConsole.className = 'dock-view-pane';
        if (viewVision) viewVision.className = 'dock-view-pane';
        if (viewTimeline) viewTimeline.className = 'dock-view-pane active';
        if (this.timelineController) this.timelineController.render();
      }

      if (save) {
        this._setItem(STORAGE_KEYS.DOCK_ACTIVE_TAB, tab);
      }
      this._handleLayoutResize();
    };

    this.switchDockTab = switchTab;

    tabBtns.forEach((btn) => {
      btn.addEventListener('click', () => {
        const tab = btn.getAttribute('data-dock-tab');
        if (tab) {
          switchTab(tab, true);
        }
      });
    });

    // 1. Restore Active Dock Tab from storage (default: 'split')
    const savedTab = this._getItem(STORAGE_KEYS.DOCK_ACTIVE_TAB);
    if (savedTab && ['split', 'telemetry', 'console', 'vision', 'timeline'].includes(savedTab)) {
      switchTab(savedTab, false);
    }

    // 2. Dock Collapse / Expand Toggle & Restoration
    const bottomDock = (typeof document !== 'undefined') ? document.getElementById('bottom-dock') : null;
    const btnDockToggle = (typeof document !== 'undefined') ? document.getElementById('btn-dock-toggle') : null;

    // Restore saved collapsed state
    const savedCollapsed = this._getItem(STORAGE_KEYS.DOCK_COLLAPSED);
    if (savedCollapsed === 'true' && bottomDock) {
      bottomDock.classList.add('dock-collapsed');
      if (btnDockToggle) {
        btnDockToggle.textContent = i18n.t('dock.expand');
      }
    } else if (savedCollapsed === 'false' && bottomDock) {
      bottomDock.classList.remove('dock-collapsed');
      if (btnDockToggle) {
        btnDockToggle.textContent = i18n.t('dock.collapse');
      }
    }

    if (bottomDock && btnDockToggle) {
      btnDockToggle.addEventListener('click', () => {
        const wasFocused = Boolean(this._focusViewState);
        this._leaveFocusView(true);
        // Focus shows the dock as collapsed; its visible action is always Expand.
        const isCollapsed = wasFocused ? false : bottomDock.classList.toggle('dock-collapsed');
        if (wasFocused) bottomDock.classList.remove('dock-collapsed');
        btnDockToggle.textContent = isCollapsed ? i18n.t('dock.expand') : i18n.t('dock.collapse');
        this._setItem(STORAGE_KEYS.DOCK_COLLAPSED, isCollapsed ? 'true' : 'false');
        this._handleLayoutResize();
      });
    }

    const workspaceNavBtns = document.querySelectorAll('[data-workspace-view]');
    const setWorkspaceView = (view) => {
      workspaceNavBtns.forEach((btn) => {
        const isActive = btn.getAttribute('data-workspace-view') === view;
        btn.classList.toggle('active', isActive);
        if (isActive) btn.setAttribute('aria-current', 'page');
        else btn.removeAttribute('aria-current');
      });
      const leftPanel = document.getElementById('left-panel');
      if (leftPanel) {
        leftPanel.classList.toggle('settings-view', view === 'settings');
        leftPanel.classList.toggle('motion-view', view === 'motion');
        leftPanel.classList.toggle('program-view', view === 'program');
        leftPanel.classList.toggle('io-view', view === 'io');
      }
      if (view === 'settings') {
        const motionEngineCard = document.getElementById('card-motion-engine');
        motionEngineCard?.classList.add('open');
        motionEngineCard?.querySelector('.accordion-header')?.setAttribute('aria-expanded', 'true');
      }
    };
    const openDockView = (tab) => {
      switchTab(tab, true);
      if (bottomDock && bottomDock.classList.contains('dock-collapsed') && btnDockToggle) {
        btnDockToggle.click();
      }
    };

    workspaceNavBtns.forEach((btn) => {
      btn.addEventListener('click', () => {
        this._leaveFocusView(true);
        document.getElementById('left-panel')?.classList.remove('panel-collapsed');
        const view = btn.getAttribute('data-workspace-view');
        setWorkspaceView(view);

        if (view === 'motion') {
          document.querySelector('[data-mode-tab="ik"]')?.click();
        } else if (view === 'program') {
          document.querySelector('[data-mode-tab="trajectory"]')?.click();
          openDockView('timeline');
        } else if (view === 'io') {
          // Dedicated hardware I/O workspace; left clean for actual digital/analog I/O
        } else if (view === 'status') {
          openDockView('telemetry');
        }
      });
    });

    const settingsConnectionBtn = document.getElementById('btn-settings-connection');
    if (settingsConnectionBtn) {
      settingsConnectionBtn.addEventListener('click', () => {
        const statusPill = document.getElementById('status-pill');
        if (statusPill && statusPill.getAttribute('aria-expanded') !== 'true') {
          statusPill.click();
        }
      });
    }

    document.querySelectorAll('[data-mode-tab]').forEach((btn) => {
      btn.addEventListener('click', () => {
        const mode = btn.getAttribute('data-mode-tab');
        setWorkspaceView(mode === 'trajectory' ? 'program' : 'motion');
      });
    });

    tabBtns.forEach((btn) => {
      btn.addEventListener('click', () => {
        const tab = btn.getAttribute('data-dock-tab');
        if (tab === 'timeline') setWorkspaceView('program');
        else if (tab === 'telemetry') setWorkspaceView('status');
      });
    });
  }

  /**
   * Binds live pose to HUD overlay elements
   * @private
   */
  _bindGlobalHUD() {
    const elX = document.getElementById('hud-tcp-x');
    const elY = document.getElementById('hud-tcp-y');
    const elZ = document.getElementById('hud-tcp-z');
    const elR = document.getElementById('hud-tcp-r');
    const elP = document.getElementById('hud-tcp-p');
    const elYW = document.getElementById('hud-tcp-yw');
    const elFloor = document.getElementById('hud-floor-alert');

    this.appState.subscribe('telemetry:updated', (live) => {
      if (!live) return;
      if (elX) elX.textContent = Number(live.x ?? 0).toFixed(1);
      if (elY) elY.textContent = Number(live.y ?? 0).toFixed(1);
      if (elZ) elZ.textContent = Number(live.z ?? 215).toFixed(1);
      if (elR) elR.textContent = `${Number(live.roll ?? 0).toFixed(1)}°`;
      if (elP) elP.textContent = `${Number(live.pitch ?? 0).toFixed(1)}°`;
      if (elYW) elYW.textContent = `${Number(live.yaw ?? 0).toFixed(1)}°`;

      if (elFloor) {
        const z = Number(live.z ?? 215);
        if (z <= 20) {
          elFloor.textContent = i18n.t('hud.floor.breach');
          elFloor.className = 'hud-alert-indicator hud-alert-breach';
        } else {
          elFloor.textContent = i18n.t('hud.floor.ok');
          elFloor.className = 'hud-alert-indicator hud-alert-ok';
        }
      }
    });
  }

  /**
   * Initializes language localization and toggle button
   * @private
   */
  _initI18n() {
    const applyTranslations = () => {
      if (typeof document === 'undefined') return;

      // 1. Text content on elements with data-i18n
      document.querySelectorAll('[data-i18n]').forEach((el) => {
        const key = el.getAttribute('data-i18n');
        if (!key) return;
        // If element contains child elements with data-i18n, only translate leaf nodes
        if (el.tagName === 'OPTION' || el.children.length === 0 || !el.querySelector('[data-i18n]')) {
          el.textContent = i18n.t(key);
        }
      });

      // 2. Tooltips on elements with data-i18n-title
      document.querySelectorAll('[data-i18n-title]').forEach((el) => {
        const key = el.getAttribute('data-i18n-title');
        if (key) {
          el.setAttribute('title', i18n.t(key));
        }
      });

      this._updateFocusViewButton();

      // 3. Update Language Toggle Button visual state
      const btnToggle = document.getElementById('btn_lang_toggle');
      const langLabel = document.getElementById('lang-code-label');
      const flagEl = btnToggle ? btnToggle.querySelector('.lang-flag') : null;
      if (btnToggle) {
        btnToggle.setAttribute('data-lang', i18n.lang);
        btnToggle.title = i18n.lang === 'en' ? 'Switch to Polish' : 'Przełącz na angielski';
      }
      if (langLabel) {
        langLabel.textContent = i18n.lang === 'en' ? 'EN' : 'PL';
      }
      if (flagEl) {
        const flagEn = '<svg class="ico flag-ico" viewBox="0 0 16 11" width="16" height="11"><rect width="16" height="11" fill="#1e3a8a" rx="1"/><path d="M0 0L16 11M16 0L0 11" stroke="#ffffff" stroke-width="1.8"/><path d="M0 0L16 11M16 0L0 11" stroke="#dc2626" stroke-width="0.9"/><path d="M8 0v11M0 5.5h16" stroke="#ffffff" stroke-width="3"/><path d="M8 0v11M0 5.5h16" stroke="#dc2626" stroke-width="1.6"/></svg>';
        const flagPl = '<svg class="ico flag-ico" viewBox="0 0 16 11" width="16" height="11"><rect width="16" height="11" fill="#ffffff" rx="1"/><path d="M0 5.5h16v5.5H0z" fill="#dc2626"/><rect width="16" height="11" fill="none" stroke="rgba(255,255,255,0.2)" stroke-width="1" rx="1"/></svg>';
        flagEl.innerHTML = i18n.lang === 'en' ? flagEn : flagPl;
      }

      // 4. Update Dock collapse/expand button text
      const bottomDock = document.getElementById('bottom-dock');
      const btnDockToggle = document.getElementById('btn-dock-toggle');
      if (bottomDock && btnDockToggle) {
        const isCollapsed = bottomDock.classList.contains('dock-collapsed');
        btnDockToggle.textContent = isCollapsed ? i18n.t('dock.expand') : i18n.t('dock.collapse');
      }

      // 5. Update HUD floor alert text if visible
      const elFloor = document.getElementById('hud-floor-alert');
      if (elFloor && elFloor.style.display !== 'none') {
        const isBreach = elFloor.classList.contains('hud-alert-breach');
        elFloor.textContent = isBreach ? i18n.t('hud.floor.breach') : i18n.t('hud.floor.ok');
      }
    };

    // Bind Language Toggle Button Click
    const btnToggle = (typeof document !== 'undefined') ? document.getElementById('btn_lang_toggle') : null;
    if (btnToggle) {
      btnToggle.addEventListener('click', () => {
        i18n.toggle();
      });
    }

    // Subscribe to language changes
    i18n.subscribe(() => {
      applyTranslations();
    });

    // Apply translations on initial boot
    applyTranslations();
  }

  /**
   * Binds global keyboard shortcuts
   * @private
   */
  _bindGlobalKeyboardShortcuts() {
    window.addEventListener('keydown', (e) => {
      const target = e.target;
      const isInput = target && (target.tagName === 'INPUT' || target.tagName === 'TEXTAREA' || target.tagName === 'SELECT' || target.isContentEditable);

      if (!isInput) {
        // Camera presets (1-5)
        if (e.key === '1') {
          e.preventDefault();
          this._triggerCameraPreset('isometric');
        } else if (e.key === '2') {
          e.preventDefault();
          this._triggerCameraPreset('top');
        } else if (e.key === '3') {
          e.preventDefault();
          this._triggerCameraPreset('front');
        } else if (e.key === '4') {
          e.preventDefault();
          this._triggerCameraPreset('side');
        } else if (e.key === '5') {
          e.preventDefault();
          this._triggerCameraPreset('reset');
        }
        // Inspector dock toggles: [ for Left, ] for Right, \ for Bottom
        else if (e.key === '[' || (e.altKey && e.key.toLowerCase() === 'left')) {
          e.preventDefault();
          const leftPanel = document.getElementById('left-panel');
          if (leftPanel) {
            const wasFocused = Boolean(this._focusViewState);
            this._leaveFocusView(true);
            if (wasFocused) leftPanel.classList.remove('panel-collapsed');
            else leftPanel.classList.toggle('panel-collapsed');
            this._handleLayoutResize();
          }
        } else if (e.key === ']' || (e.altKey && e.key.toLowerCase() === 'right')) {
          e.preventDefault();
          const rightPanel = document.getElementById('right-panel');
          if (rightPanel) {
            rightPanel.classList.toggle('panel-collapsed');
            this._handleLayoutResize();
          }
        } else if (e.key === '\\') {
          e.preventDefault();
          const bottomDock = document.getElementById('bottom-dock');
          const btnDockToggle = document.getElementById('btn-dock-toggle');
          if (bottomDock) {
            const wasFocused = Boolean(this._focusViewState);
            this._leaveFocusView(true);
            const isCollapsed = wasFocused ? false : bottomDock.classList.toggle('dock-collapsed');
            if (wasFocused) bottomDock.classList.remove('dock-collapsed');
            if (btnDockToggle) {
              btnDockToggle.textContent = isCollapsed ? i18n.t('dock.expand') : i18n.t('dock.collapse');
            }
            this._setItem(STORAGE_KEYS.DOCK_COLLAPSED, isCollapsed ? 'true' : 'false');
            this._handleLayoutResize();
          }
        }
        // -------------------------------------------------------------------
        // Spacebar: Capture Ghost Target Pose into Timeline Sequence
        // -------------------------------------------------------------------
        else if (e.code === 'Space' || e.key === ' ') {
          e.preventDefault();
          if (this.trajectoryController) {
            this.trajectoryController.captureGhostPose();
            const btnGhost = document.getElementById('btn-capture-ghost');
            if (btnGhost) {
              btnGhost.classList.add('btn-pulse');
              setTimeout(() => btnGhost.classList.remove('btn-pulse'), 300);
            }
          }
        }
        // -------------------------------------------------------------------
        // Cartesian Jogging (WSAD & Arrow Up/Down) & Wrist Mode (Arrow Left/Right)
        // -------------------------------------------------------------------
        else if (this.motionController) {
          const kLower = e.key.toLowerCase();
          if (kLower === 'w') {
            e.preventDefault();
            this.motionController.jogCartesianKeyboard('w');
          } else if (kLower === 's') {
            e.preventDefault();
            this.motionController.jogCartesianKeyboard('s');
          } else if (kLower === 'a') {
            e.preventDefault();
            this.motionController.jogCartesianKeyboard('a');
          } else if (kLower === 'd') {
            e.preventDefault();
            this.motionController.jogCartesianKeyboard('d');
          } else if (e.key === 'ArrowUp') {
            e.preventDefault();
            this.motionController.jogCartesianKeyboard('ArrowUp');
          } else if (e.key === 'ArrowDown') {
            e.preventDefault();
            this.motionController.jogCartesianKeyboard('ArrowDown');
          } else if (e.key === 'ArrowLeft') {
            e.preventDefault();
            this.motionController.jogCartesianKeyboard('ArrowLeft');
          } else if (e.key === 'ArrowRight') {
            e.preventDefault();
            this.motionController.jogCartesianKeyboard('ArrowRight');
          }
        }
      }
    });
  }

  /**
   * Triggers camera preset and updates button state
   * @private
   */
  _triggerCameraPreset(preset) {
    if (!this.visualizer) return;
    this.visualizer.setCameraPreset(preset);
    const cameraBtns = document.querySelectorAll('[data-camera-preset]');
    cameraBtns.forEach((btn) => {
      btn.classList.toggle('active', btn.getAttribute('data-camera-preset') === preset);
    });
  }
}

// Instantiate and start on DOMContentLoaded
export const app = new RobotArmApp();

if (typeof window !== 'undefined') {
  window.addEventListener('DOMContentLoaded', () => {
    app.init().catch((err) => {
      console.error('Fatal initialization error:', err);
    });

    // Register Progressive Web App (PWA) Service Worker
    if ('serviceWorker' in navigator) {
      navigator.serviceWorker.register('./sw.js')
        .then((reg) => console.log('Service Worker registered with scope:', reg.scope))
        .catch((err) => console.warn('Service Worker registration failed:', err));
    }
  });
}

export default app;
