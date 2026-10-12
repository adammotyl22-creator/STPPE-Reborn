/**
 * TelemetryController.js - Real-Time Telemetry Analytics, Rolling Strip Charts & Status Gauges
 * 
 * Manages 10Hz live telemetry ingestion, 30-second rolling ring buffer,
 * high-performance Canvas line charts for Joint Angles (J1, J2, J3) and
 * TCP Coordinates (X, Y, Z), status indicator badges (ESTOP, Homing, Closed Loop),
 * and gauges (Speed %, Hz Telemetry Rate, EC Collision Flags).
 */

import { appState as defaultAppState } from '../state/AppState.js';

export const CHART_COLORS = Object.freeze({
  j1: '#10b981', // Soft Sage Green
  j2: '#cbd5e1', // Soft Titanium Silver
  j3: '#f59e0b', // Warm Amber Sand
  x: '#10b981',  // Soft Sage Green
  y: '#cbd5e1',  // Soft Titanium Silver
  z: '#f59e0b',  // Warm Amber Sand
  grid: 'rgba(255, 255, 255, 0.06)',
  text: '#94a3b8',
  bg: '#0d0f12'
});

export const DEFAULT_WINDOW_SEC = 30;
export const MAX_TELEMETRY_BUFFER_SIZE = 600; // Up to 60s @ 10Hz

export class TelemetryController {
  /**
   * @param {Object} [options]
   * @param {HTMLElement|string} [options.container] - DOM element or selector
   * @param {AppState} [options.appState] - Reactive state store
   * @param {WebSerialManager} [options.serialManager] - WebSerial communication manager
   * @param {number} [options.windowDurationSec=30] - Rolling chart window in seconds
   * @param {boolean} [options.useChartJs=false] - Whether to use Chart.js if available
   */
  constructor(options = {}) {
    this.appState = options.appState || defaultAppState;
    this.serialManager = options.serialManager || null;
    this.windowDurationSec = Math.max(5, Math.min(120, Number(options.windowDurationSec || DEFAULT_WINDOW_SEC)));
    this.maxBufferSize = Math.max(100, this.windowDurationSec * 20); // 10Hz default + margin
    this.useChartJs = Boolean(options.useChartJs);

    // Rolling Ring Buffer of Telemetry Points
    this.history = [];

    // Rendering State
    this.isPaused = false;
    this.channels = {
      joints: { j1: true, j2: true, j3: true },
      tcp: { x: true, y: true, z: true }
    };

    // Status Indicators & Gauges Cache
    this.status = {
      estop: false,
      homing: false,
      closedLoop: false,
      speedOverride: 100,
      telemetryRateHz: 0.0,
      collision: { j1: false, j2: false, j3: false, hasCollision: false }
    };

    // DOM & Canvas Elements
    this.container = null;
    this.dom = {};
    this._eventCleanups = [];
    this._unsubState = [];
    this._animFrameId = null;
    this._chartJsInstances = { joints: null, tcp: null };

    if (options.serialManager) {
      this.setSerialManager(options.serialManager);
    }

    // Initialize AppState Subscribers
    this._initAppStateListeners();

    // Bind DOM if container provided
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
   * Sets serial manager and subscribes to events
   * @param {WebSerialManager} manager
   */
  setSerialManager(manager) {
    this.serialManager = manager;
    if (this.serialManager) {
      this.serialManager.on('jtelem', (pkt) => {
        if (this.appState) {
          this.appState.updateLiveTelemetry(pkt);
        } else {
          this.ingestTelemetry(pkt);
        }
      });
      this.serialManager.on('collision', (pkt) => {
        if (this.appState) {
          this.appState.setCollision(pkt.j1, pkt.j2, pkt.j3);
        } else {
          this.updateCollision(pkt);
        }
      });
      this.serialManager.on('rateChange', ({ rateHz }) => {
        this.updateRateHz(rateHz);
      });
    }
  }

  /**
   * Subscribes to central AppState changes
   * @private
   */
  _initAppStateListeners() {
    if (!this.appState) return;

    // Listen for live telemetry updates (10Hz JTELEM)
    const unsubTelem = this.appState.subscribe('telemetry:updated', (livePose) => {
      this.ingestTelemetry(livePose);
    });
    this._unsubState.push(unsubTelem);

    // Listen for safety state changes
    const unsubSafety = this.appState.subscribe('stateChange:safety', (safety) => {
      if (!safety) return;
      if (safety.estop !== undefined) this.updateEstop(safety.estop);
      if (safety.homing !== undefined) this.updateHoming(safety.homing);
      if (safety.closedLoop !== undefined) this.updateClosedLoop(safety.closedLoop);
      if (safety.speedOverride !== undefined) this.updateSpeed(safety.speedOverride);
      if (safety.collision !== undefined) {
        this.updateCollision({
          j1: safety.collision.j1,
          j2: safety.collision.j2,
          j3: safety.collision.j3,
          hasCollision: safety.hasCollision
        });
      }
    });
    this._unsubState.push(unsubSafety);

    // Listen for connection / Hz rate changes
    const unsubConn = this.appState.subscribe('stateChange:connection', (conn) => {
      if (conn && conn.telemRateHz !== undefined) {
        this.updateRateHz(conn.telemRateHz);
      }
    });
    this._unsubState.push(unsubConn);

    const unsubRate = this.appState.subscribe('change:connection.telemRateHz', ({ value }) => {
      if (value !== undefined) {
        this.updateRateHz(value);
      }
    });
    this._unsubState.push(unsubRate);
  }

  /**
   * Ingests a new telemetry sample point into rolling ring buffer
   * @param {Object} point - Telemetry sample containing j1..j3, x..z, spd, estop, etc.
   */
  ingestTelemetry(point) {
    if (!point || typeof point !== 'object') return;

    const now = Number(point.timestamp || point.lastUpdated || Date.now());

    const sample = {
      timestamp: now,
      j1: Number(point.j1 ?? 0),
      j2: Number(point.j2 ?? 0),
      j3: Number(point.j3 ?? 0),
      j4: Number(point.j4 ?? 90),
      j5: Number(point.j5 ?? 90),
      grip: Number(point.grip ?? 0),
      x: Number(point.x ?? 0),
      y: Number(point.y ?? 0),
      z: Number(point.z ?? 0),
      spd: Number(point.spd ?? this.status.speedOverride),
      estop: Boolean(point.estop ?? this.status.estop),
      homing: Boolean(point.homing ?? this.status.homing),
      teach: Boolean(point.teach ?? false),
      tpts: Number(point.tpts ?? 0)
    };

    // Sanitize any NaN values
    if (isNaN(sample.j1)) sample.j1 = 0;
    if (isNaN(sample.j2)) sample.j2 = 0;
    if (isNaN(sample.j3)) sample.j3 = 0;
    if (isNaN(sample.x)) sample.x = 0;
    if (isNaN(sample.y)) sample.y = 0;
    if (isNaN(sample.z)) sample.z = 0;

    this.history.push(sample);

    // Prune buffer based on window duration and max capacity
    const cutoffTime = now - (this.windowDurationSec * 1000);
    while (this.history.length > 0 && this.history[0].timestamp < cutoffTime && this.history.length > (this.windowDurationSec * 10)) {
      this.history.shift();
    }

    if (this.history.length > this.maxBufferSize) {
      this.history.splice(0, this.history.length - this.maxBufferSize);
    }

    // Update status indicators if sample contains flags
    if (point.estop !== undefined) this.updateEstop(point.estop);
    if (point.homing !== undefined) this.updateHoming(point.homing);
    if (point.spd !== undefined) this.updateSpeed(point.spd);

    // If Chart.js active, update datasets
    if (this.useChartJs) {
      this._updateChartJs();
    }
  }

  /**
   * Returns current rolling history array
   * @returns {Array<Object>}
   */
  getHistory() {
    return this.history.slice();
  }

  /**
   * Clears the rolling history buffer
   */
  clearHistory() {
    this.history = [];
    if (this.useChartJs) {
      this._resetChartJs();
    }
    this.renderCharts();
  }

  /**
   * Sets time window duration in seconds (e.g. 10, 30, 60)
   * @param {number} seconds
   */
  setWindowDuration(seconds) {
    this.windowDurationSec = Math.max(5, Math.min(120, Number(seconds || DEFAULT_WINDOW_SEC)));
    this.maxBufferSize = Math.max(100, this.windowDurationSec * 20);
    if (this.dom.timeSelect) {
      this.dom.timeSelect.value = String(this.windowDurationSec);
    }
    this.renderCharts();
  }

  /**
   * Toggles pause / resume of chart rendering
   * @param {boolean} [pauseState]
   * @returns {boolean} New paused state
   */
  togglePause(pauseState) {
    this.isPaused = typeof pauseState === 'boolean' ? pauseState : !this.isPaused;
    if (this.dom.btnPause) {
      this.dom.btnPause.textContent = this.isPaused ? 'Resume' : 'Pause';
      this.dom.btnPause.classList.toggle('active', this.isPaused);
    }
    return this.isPaused;
  }

  /**
   * Toggles visibility of a specific series channel
   * @param {'joints'|'tcp'} chartType
   * @param {string} channelKey - 'j1'|'j2'|'j3' or 'x'|'y'|'z'
   */
  toggleChannel(chartType, channelKey) {
    if (this.channels[chartType] && channelKey in this.channels[chartType]) {
      this.channels[chartType][channelKey] = !this.channels[chartType][channelKey];
      this._updateChannelPillUI(chartType, channelKey);
      this.renderCharts();
    }
  }

  // ==========================================================================
  // STATUS INDICATORS & GAUGES
  // ==========================================================================

  /**
   * Updates ESTOP active badge
   * @param {boolean} active
   */
  updateEstop(active) {
    this.status.estop = Boolean(active);
    if (this.dom.badgeEstop) {
      this.dom.badgeEstop.className = `status-badge badge-estop ${this.status.estop ? 'active' : 'safe'}`;
      this.dom.badgeEstop.textContent = this.status.estop ? 'ESTOP ACTIVE' : 'ESTOP OK';
    }
  }

  /**
   * Updates Homing active badge
   * @param {boolean} active
   */
  updateHoming(active) {
    this.status.homing = Boolean(active);
    if (this.dom.badgeHoming) {
      this.dom.badgeHoming.className = `status-badge badge-homing ${this.status.homing ? 'active' : 'idle'}`;
      this.dom.badgeHoming.textContent = this.status.homing ? 'HOMING...' : 'HOMED / IDLE';
    }
  }

  /**
   * Updates Closed Loop mode badge (CL_ON / CL_OFF)
   * @param {boolean} active
   */
  updateClosedLoop(active) {
    this.status.closedLoop = Boolean(active);
    if (this.dom.badgeClosedLoop) {
      this.dom.badgeClosedLoop.className = `status-badge badge-closed-loop ${this.status.closedLoop ? 'cl-on' : 'cl-off'}`;
      this.dom.badgeClosedLoop.textContent = this.status.closedLoop ? 'CL_ON' : 'CL_OFF';
    }
  }

  /**
   * Toggles Closed Loop mode and sends command to Teensy
   */
  async toggleClosedLoop() {
    const nextState = !this.status.closedLoop;
    this.updateClosedLoop(nextState);
    if (this.appState) {
      this.appState.setClosedLoop(nextState);
    }
    if (this.serialManager && this.serialManager.isConnected()) {
      try {
        const cmd = nextState ? 'CL_ON\n' : 'CL_OFF\n';
        await this.serialManager.send(cmd);
      } catch (err) {
        console.error('Failed to toggle closed loop:', err);
      }
    }
  }

  /**
   * Updates Speed % Gauge
   * @param {number} percent - 5 to 100
   */
  updateSpeed(percent) {
    const val = Math.min(100, Math.max(5, Math.round(Number(percent || 100))));
    this.status.speedOverride = val;
    if (this.dom.speedValue) {
      this.dom.speedValue.textContent = `${val}%`;
    }
    if (this.dom.speedFill) {
      this.dom.speedFill.style.width = `${val}%`;
    }
  }

  /**
   * Updates Telemetry Rate Gauge in Hz
   * @param {number} rateHz
   */
  updateRateHz(rateHz) {
    const hz = Math.max(0, Number(rateHz || 0));
    this.status.telemetryRateHz = hz;
    if (this.dom.rateValue) {
      this.dom.rateValue.textContent = `${hz.toFixed(1)} Hz`;
    }
    if (this.dom.rateDot) {
      this.dom.rateDot.className = `rate-pulse-dot ${hz >= 8.0 ? 'healthy' : hz > 0 ? 'slow' : 'offline'}`;
    }
  }

  /**
   * Updates EC Collision Flag monitor
   * @param {Object} col - { j1, j2, j3, hasCollision }
   */
  updateCollision(col = {}) {
    const j1 = Boolean(col.j1);
    const j2 = Boolean(col.j2);
    const j3 = Boolean(col.j3);
    const hasCollision = Boolean(col.hasCollision || j1 || j2 || j3);

    this.status.collision = { j1, j2, j3, hasCollision };

    if (this.dom.badgeCollision) {
      this.dom.badgeCollision.className = `status-badge badge-collision ${hasCollision ? 'collision-alert' : 'no-collision'}`;
      const code = `EC${j1 ? '1' : '0'}${j2 ? '1' : '0'}${j3 ? '1' : '0'}`;
      this.dom.badgeCollision.textContent = hasCollision ? `COLLISION: ${code}` : `OK: ${code}`;
    }

    if (this.dom.colDotJ1) this.dom.colDotJ1.classList.toggle('col-active', j1);
    if (this.dom.colDotJ2) this.dom.colDotJ2.classList.toggle('col-active', j2);
    if (this.dom.colDotJ3) this.dom.colDotJ3.classList.toggle('col-active', j3);
  }

  // ==========================================================================
  // EXPORT UTILITIES (CSV / JSON)
  // ==========================================================================

  /**
   * Generates CSV format string from rolling telemetry history
   * @returns {string} CSV string
   */
  exportCSV() {
    const headers = ['Timestamp_ISO', 'Timestamp_MS', 'J1_Deg', 'J2_Deg', 'J3_Deg', 'J4_Deg', 'J5_Deg', 'Grip_Deg', 'TCP_X_mm', 'TCP_Y_mm', 'TCP_Z_mm', 'Speed_Pct', 'ESTOP', 'Homing'];
    const rows = this.history.map(s => [
      new Date(s.timestamp).toISOString(),
      s.timestamp,
      s.j1.toFixed(2),
      s.j2.toFixed(2),
      s.j3.toFixed(2),
      s.j4.toFixed(1),
      s.j5.toFixed(1),
      s.grip,
      s.x.toFixed(2),
      s.y.toFixed(2),
      s.z.toFixed(2),
      s.spd,
      s.estop ? 1 : 0,
      s.homing ? 1 : 0
    ].join(','));

    const csvContent = [headers.join(','), ...rows].join('\n');

    if (typeof document !== 'undefined' && typeof Blob !== 'undefined') {
      const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `robot_telemetry_${Date.now()}.csv`;
      a.click();
      URL.revokeObjectURL(url);
    }

    return csvContent;
  }

  /**
   * Generates JSON format string from rolling telemetry history
   * @returns {string} JSON string
   */
  exportJSON() {
    const payload = {
      exportedAt: new Date().toISOString(),
      sampleCount: this.history.length,
      windowDurationSec: this.windowDurationSec,
      telemetry: this.history
    };
    const jsonStr = JSON.stringify(payload, null, 2);

    if (typeof document !== 'undefined' && typeof Blob !== 'undefined') {
      const blob = new Blob([jsonStr], { type: 'application/json' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `robot_telemetry_${Date.now()}.json`;
      a.click();
      URL.revokeObjectURL(url);
    }

    return jsonStr;
  }

  // ==========================================================================
  // HIGH PERFORMANCE 2D CANVAS RENDERING ENGINE
  // ==========================================================================

  /**
   * Renders both Joint Angles and TCP Coordinates charts
   */
  renderCharts() {
    if (this.isPaused) return;

    if (this.dom.canvasJoints) {
      this._renderCanvasChart(
        this.dom.canvasJoints,
        [
          { key: 'j1', label: 'J1', color: CHART_COLORS.j1, active: this.channels.joints.j1, unit: '°' },
          { key: 'j2', label: 'J2', color: CHART_COLORS.j2, active: this.channels.joints.j2, unit: '°' },
          { key: 'j3', label: 'J3', color: CHART_COLORS.j3, active: this.channels.joints.j3, unit: '°' }
        ],
        { minSpan: 40, defaultMin: -100, defaultMax: 100, title: 'Joint Angles (°)' }
      );
    }

    if (this.dom.canvasTcp) {
      this._renderCanvasChart(
        this.dom.canvasTcp,
        [
          { key: 'x', label: 'X', color: CHART_COLORS.x, active: this.channels.tcp.x, unit: 'mm' },
          { key: 'y', label: 'Y', color: CHART_COLORS.y, active: this.channels.tcp.y, unit: 'mm' },
          { key: 'z', label: 'Z', color: CHART_COLORS.z, active: this.channels.tcp.z, unit: 'mm' }
        ],
        { minSpan: 100, defaultMin: -500, defaultMax: 600, title: 'TCP Coordinates (mm)' }
      );
    }
  }

  /**
   * Internal pure 2D Canvas rendering algorithm
   * @private
   */
  _renderCanvasChart(canvas, seriesList, opts = {}) {
    if (!canvas) return;
    const ctx = canvas.getContext ? canvas.getContext('2d') : null;
    if (!ctx) return;

    // Handle HiDPI / retina displays
    const dpr = typeof window !== 'undefined' ? window.devicePixelRatio || 1 : 1;
    const rect = canvas.getBoundingClientRect ? canvas.getBoundingClientRect() : { width: canvas.width || 400, height: canvas.height || 180 };
    const width = Math.max(50, Math.floor(rect.width || canvas.width || 400));
    const height = Math.max(50, Math.floor(rect.height || canvas.height || 180));

    if (canvas.width !== width * dpr || canvas.height !== height * dpr) {
      canvas.width = width * dpr;
      canvas.height = height * dpr;
    }

    ctx.save();
    ctx.scale(dpr, dpr);

    // Background fill
    ctx.fillStyle = CHART_COLORS.bg;
    ctx.fillRect(0, 0, width, height);

    const padding = { left: 46, right: 16, top: 16, bottom: 24 };
    const plotW = Math.max(10, width - padding.left - padding.right);
    const plotH = Math.max(10, height - padding.top - padding.bottom);

    const now = Date.now();
    const windowMs = this.windowDurationSec * 1000;
    const tMin = now - windowMs;
    const tMax = now;

    // Filter points inside the time window
    const activePoints = this.history.filter(p => p.timestamp >= tMin - 1000);

    if (activePoints.length === 0) {
      ctx.fillStyle = CHART_COLORS.text;
      ctx.font = '11px monospace';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText('Waiting for live telemetry stream (10Hz JTELEM)...', width / 2, height / 2);
      ctx.restore();
      return;
    }

    // Determine dynamic Y min/max
    let yMin = Infinity;
    let yMax = -Infinity;

    for (const s of seriesList) {
      if (!s.active) continue;
      for (const p of activePoints) {
        const v = p[s.key];
        if (typeof v === 'number' && !isNaN(v)) {
          yMin = Math.min(yMin, v);
          yMax = Math.max(yMax, v);
        }
      }
    }

    if (yMin === Infinity) {
      yMin = opts.defaultMin ?? -100;
      yMax = opts.defaultMax ?? 100;
    }

    // Ensure minimum span & nice margins
    const span = Math.max(opts.minSpan || 20, (yMax - yMin) * 1.15);
    const center = (yMin + yMax) / 2;
    yMin = Math.floor(center - span / 2);
    yMax = Math.ceil(center + span / 2);
    const yRange = yMax - yMin || 1;

    // Draw horizontal grid lines & Y-axis labels
    const ySteps = 4;
    ctx.strokeStyle = CHART_COLORS.grid;
    ctx.lineWidth = 1;
    ctx.fillStyle = CHART_COLORS.text;
    ctx.font = '10px monospace';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';

    for (let i = 0; i <= ySteps; i++) {
      const val = yMin + (yRange * i) / ySteps;
      const y = padding.top + plotH - (plotH * i) / ySteps;
      ctx.beginPath();
      ctx.moveTo(padding.left, y);
      ctx.lineTo(padding.left + plotW, y);
      ctx.stroke();
      ctx.fillText(val.toFixed(0), padding.left - 6, y);
    }

    // Draw vertical time grid lines & X-axis labels
    const xSteps = 4;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';

    for (let i = 0; i <= xSteps; i++) {
      const x = padding.left + (plotW * i) / xSteps;
      const secOffset = Math.round(((xSteps - i) / xSteps) * this.windowDurationSec);
      const label = secOffset === 0 ? 'Now' : `-${secOffset}s`;

      ctx.beginPath();
      ctx.moveTo(x, padding.top);
      ctx.lineTo(x, padding.top + plotH);
      ctx.stroke();
      ctx.fillText(label, x, padding.top + plotH + 6);
    }

    // Draw Series Lines
    for (const s of seriesList) {
      if (!s.active) continue;

      ctx.strokeStyle = s.color;
      ctx.lineWidth = 2;
      ctx.beginPath();

      let hasDrawn = false;
      let lastX = 0;
      let lastY = 0;

      for (let i = 0; i < activePoints.length; i++) {
        const pt = activePoints[i];
        const val = pt[s.key];
        if (typeof val !== 'number' || isNaN(val)) continue;

        const normX = Math.max(0, Math.min(1, (pt.timestamp - tMin) / windowMs));
        const normY = Math.max(0, Math.min(1, (val - yMin) / yRange));

        const x = padding.left + normX * plotW;
        const y = padding.top + plotH - normY * plotH;

        if (!hasDrawn) {
          ctx.moveTo(x, y);
          hasDrawn = true;
        } else {
          ctx.lineTo(x, y);
        }
        lastX = x;
        lastY = y;
      }

      if (hasDrawn) {
        ctx.stroke();

        // Draw live endpoint pulse dot
        ctx.fillStyle = s.color;
        ctx.beginPath();
        ctx.arc(lastX, lastY, 3.5, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    ctx.restore();
  }

  // ==========================================================================
  // CHART.JS OPTIONAL INTEGRATION ADAPTER
  // ==========================================================================

  _initChartJs() {
    if (typeof window === 'undefined' || !window.Chart) return;
    const Chart = window.Chart;

    if (this.dom.canvasJoints && !this._chartJsInstances.joints) {
      this._chartJsInstances.joints = new Chart(this.dom.canvasJoints, {
        type: 'line',
        data: {
          labels: [],
          datasets: [
            { label: 'J1', data: [], borderColor: CHART_COLORS.j1, borderWidth: 2, tension: 0.1 },
            { label: 'J2', data: [], borderColor: CHART_COLORS.j2, borderWidth: 2, tension: 0.1 },
            { label: 'J3', data: [], borderColor: CHART_COLORS.j3, borderWidth: 2, tension: 0.1 }
          ]
        },
        options: { responsive: true, animation: false }
      });
    }

    if (this.dom.canvasTcp && !this._chartJsInstances.tcp) {
      this._chartJsInstances.tcp = new Chart(this.dom.canvasTcp, {
        type: 'line',
        data: {
          labels: [],
          datasets: [
            { label: 'X', data: [], borderColor: CHART_COLORS.x, borderWidth: 2, tension: 0.1 },
            { label: 'Y', data: [], borderColor: CHART_COLORS.y, borderWidth: 2, tension: 0.1 },
            { label: 'Z', data: [], borderColor: CHART_COLORS.z, borderWidth: 2, tension: 0.1 }
          ]
        },
        options: { responsive: true, animation: false }
      });
    }
  }

  _updateChartJs() {
    if (!this._chartJsInstances.joints && !this._chartJsInstances.tcp) return;
    const pts = this.history.slice(-30);
    const labels = pts.map(p => new Date(p.timestamp).toLocaleTimeString());

    if (this._chartJsInstances.joints) {
      this._chartJsInstances.joints.data.labels = labels;
      this._chartJsInstances.joints.data.datasets[0].data = pts.map(p => p.j1);
      this._chartJsInstances.joints.data.datasets[1].data = pts.map(p => p.j2);
      this._chartJsInstances.joints.data.datasets[2].data = pts.map(p => p.j3);
      this._chartJsInstances.joints.update('none');
    }

    if (this._chartJsInstances.tcp) {
      this._chartJsInstances.tcp.data.labels = labels;
      this._chartJsInstances.tcp.data.datasets[0].data = pts.map(p => p.x);
      this._chartJsInstances.tcp.data.datasets[1].data = pts.map(p => p.y);
      this._chartJsInstances.tcp.data.datasets[2].data = pts.map(p => p.z);
      this._chartJsInstances.tcp.update('none');
    }
  }

  _resetChartJs() {
    if (this._chartJsInstances.joints) this._chartJsInstances.joints.reset();
    if (this._chartJsInstances.tcp) this._chartJsInstances.tcp.reset();
  }

  // ==========================================================================
  // DOM BINDING & LIFECYCLE
  // ==========================================================================

  /**
   * Binds controller to DOM container element
   * @param {HTMLElement} container
   */
  bindDom(container) {
    if (!container) return;
    this.unbindDom();
    this.container = container;

    // Check if container already has markup or needs generation
    if (!container.querySelector('.telemetry-panel') && !container.querySelector('.status-hud-grid')) {
      container.innerHTML = this.generateTemplateHTML();
    }

    // Query elements
    this.dom.badgeEstop = container.querySelector('.badge-estop');
    this.dom.badgeHoming = container.querySelector('.badge-homing');
    this.dom.badgeClosedLoop = container.querySelector('.badge-closed-loop');
    this.dom.speedValue = container.querySelector('.speed-value');
    this.dom.speedFill = container.querySelector('.speed-meter-fill');
    this.dom.rateValue = container.querySelector('.rate-value');
    this.dom.rateDot = container.querySelector('.rate-pulse-dot');
    this.dom.badgeCollision = container.querySelector('.badge-collision');
    this.dom.colDotJ1 = container.querySelector('.col-pill-j1');
    this.dom.colDotJ2 = container.querySelector('.col-pill-j2');
    this.dom.colDotJ3 = container.querySelector('.col-pill-j3');

    this.dom.canvasJoints = container.querySelector('.canvas-joints');
    this.dom.canvasTcp = container.querySelector('.canvas-tcp');

    this.dom.btnPause = container.querySelector('.btn-chart-pause');
    this.dom.btnClear = container.querySelector('.btn-chart-clear');
    this.dom.timeSelect = container.querySelector('.time-window-select');
    this.dom.btnExportCsv = container.querySelector('.btn-export-csv');
    this.dom.btnExportJson = container.querySelector('.btn-export-json');

    // Attach event listeners
    if (this.dom.badgeClosedLoop) {
      const onClClick = () => this.toggleClosedLoop();
      this.dom.badgeClosedLoop.addEventListener('click', onClClick);
      this._eventCleanups.push(() => this.dom.badgeClosedLoop.removeEventListener('click', onClClick));
    }

    if (this.dom.btnPause) {
      const onPauseClick = () => this.togglePause();
      this.dom.btnPause.addEventListener('click', onPauseClick);
      this._eventCleanups.push(() => this.dom.btnPause.removeEventListener('click', onPauseClick));
    }

    if (this.dom.btnClear) {
      const onClearClick = () => this.clearHistory();
      this.dom.btnClear.addEventListener('click', onClearClick);
      this._eventCleanups.push(() => this.dom.btnClear.removeEventListener('click', onClearClick));
    }

    if (this.dom.timeSelect) {
      const onTimeChange = (e) => this.setWindowDuration(Number(e.target.value));
      this.dom.timeSelect.addEventListener('change', onTimeChange);
      this._eventCleanups.push(() => this.dom.timeSelect.removeEventListener('change', onTimeChange));
    }

    if (this.dom.btnExportCsv) {
      const onCsvClick = () => this.exportCSV();
      this.dom.btnExportCsv.addEventListener('click', onCsvClick);
      this._eventCleanups.push(() => this.dom.btnExportCsv.removeEventListener('click', onCsvClick));
    }

    if (this.dom.btnExportJson) {
      const onJsonClick = () => this.exportJSON();
      this.dom.btnExportJson.addEventListener('click', onJsonClick);
      this._eventCleanups.push(() => this.dom.btnExportJson.removeEventListener('click', onJsonClick));
    }

    // Channel toggle pills
    const channelPills = container.querySelectorAll('.channel-pill');
    channelPills.forEach(pill => {
      const chartType = pill.dataset.chart;
      const chKey = pill.dataset.channel;
      const onPillClick = () => this.toggleChannel(chartType, chKey);
      pill.addEventListener('click', onPillClick);
      this._eventCleanups.push(() => pill.removeEventListener('click', onPillClick));
    });

    if (this.useChartJs) {
      this._initChartJs();
    }

    // Start 60fps render loop
    this._startRenderLoop();
  }

  /**
   * Generates default HTML template string for Telemetry UI
   * @returns {string}
   */
  generateTemplateHTML() {
    return `
      <div class="telemetry-panel">
        <div class="telemetry-header">
          <div class="telemetry-title">
            <span>Real-Time Telemetry & Status HUD</span>
          </div>
          <div class="chart-toolbar-right">
            <button class="chart-btn btn-export-csv" title="Export CSV">CSV</button>
            <button class="chart-btn btn-export-json" title="Export JSON">JSON</button>
          </div>
        </div>

        <!-- Status HUD Grid -->
        <div class="status-hud-grid">
          <div class="hud-card">
            <span class="hud-card-label">Safety State</span>
            <div class="status-badge badge-estop safe">ESTOP OK</div>
          </div>

          <div class="hud-card">
            <span class="hud-card-label">Homing Routine</span>
            <div class="status-badge badge-homing idle">HOMED / IDLE</div>
          </div>

          <div class="hud-card">
            <span class="hud-card-label">Closed Loop Mode</span>
            <div class="status-badge badge-closed-loop cl-off">CL_OFF</div>
          </div>

          <div class="hud-card">
            <span class="hud-card-label">Speed Override</span>
            <div class="hud-card-value speed-value">100%</div>
            <div class="speed-meter-bar"><div class="speed-meter-fill" style="width: 100%;"></div></div>
          </div>

          <div class="hud-card">
            <span class="hud-card-label">Telemetry Rate</span>
            <div class="hud-card-value badge-rate">
              <span class="rate-pulse-dot offline"></span>
              <span class="rate-value">0.0 Hz</span>
            </div>
          </div>

          <div class="hud-card">
            <span class="hud-card-label">Collision Flag</span>
            <div class="status-badge badge-collision no-collision">OK: EC000</div>
            <div class="collision-axis-dots">
              <span class="collision-axis-pill col-pill-j1">J1</span>
              <span class="collision-axis-pill col-pill-j2">J2</span>
              <span class="collision-axis-pill col-pill-j3">J3</span>
            </div>
          </div>
        </div>

        <!-- Charts Grid -->
        <div class="charts-grid">
          <!-- Joint Angles Chart Card -->
          <div class="chart-card">
            <div class="chart-card-header">
              <span class="chart-card-title">Joint Angles (J1, J2, J3)</span>
              <div class="chart-channel-pills">
                <span class="channel-pill j1 active" data-chart="joints" data-channel="j1">J1 (Base)</span>
                <span class="channel-pill j2 active" data-chart="joints" data-channel="j2">J2 (Shoulder)</span>
                <span class="channel-pill j3 active" data-chart="joints" data-channel="j3">J3 (Elbow)</span>
              </div>
            </div>
            <div class="chart-canvas-wrapper">
              <canvas class="chart-canvas canvas-joints" width="400" height="180"></canvas>
            </div>
          </div>

          <!-- TCP Coordinates Chart Card -->
          <div class="chart-card">
            <div class="chart-card-header">
              <span class="chart-card-title">TCP Coordinates (X, Y, Z)</span>
              <div class="chart-channel-pills">
                <span class="channel-pill x active" data-chart="tcp" data-channel="x">X (mm)</span>
                <span class="channel-pill y active" data-chart="tcp" data-channel="y">Y (mm)</span>
                <span class="channel-pill z active" data-chart="tcp" data-channel="z">Z (mm)</span>
              </div>
            </div>
            <div class="chart-canvas-wrapper">
              <canvas class="chart-canvas canvas-tcp" width="400" height="180"></canvas>
            </div>
          </div>
        </div>

        <!-- Chart Controls Toolbar -->
        <div class="chart-toolbar">
          <div class="chart-toolbar-left">
            <button class="chart-btn btn-chart-pause">Pause</button>
            <button class="chart-btn btn-chart-clear">Clear History</button>
          </div>
          <div class="chart-toolbar-right">
            <span style="font-size: 11px; color: var(--text-muted);">Window:</span>
            <select class="time-window-select">
              <option value="10">10 Seconds</option>
              <option value="30" selected>30 Seconds</option>
              <option value="60">60 Seconds</option>
            </select>
          </div>
        </div>
      </div>
    `;
  }

  _updateChannelPillUI(chartType, channelKey) {
    if (!this.container) return;
    const pill = this.container.querySelector(`.channel-pill[data-chart="${chartType}"][data-channel="${channelKey}"]`);
    if (pill) {
      const isActive = this.channels[chartType][channelKey];
      pill.classList.toggle('active', isActive);
      pill.classList.toggle('inactive', !isActive);
    }
  }

  _startRenderLoop() {
    if (typeof window === 'undefined' || typeof window.requestAnimationFrame !== 'function') return;

    // Cancel any existing loop to prevent overlapping animation frames
    if (this._animFrameId) {
      window.cancelAnimationFrame(this._animFrameId);
      this._animFrameId = null;
    }

    const loop = () => {
      this.renderCharts();
      this._animFrameId = window.requestAnimationFrame(loop);
    };
    this._animFrameId = window.requestAnimationFrame(loop);
  }

  /**
   * Releases DOM bindings, event listeners, and render loop cleanly
   */
  unbindDom() {
    if (this._animFrameId && typeof window !== 'undefined') {
      window.cancelAnimationFrame(this._animFrameId);
      this._animFrameId = null;
    }

    this._eventCleanups.forEach(fn => fn());
    this._eventCleanups = [];

    this._unsubState.forEach(fn => fn());
    this._unsubState = [];

    if (this._chartJsInstances.joints) {
      this._chartJsInstances.joints.destroy();
      this._chartJsInstances.joints = null;
    }
    if (this._chartJsInstances.tcp) {
      this._chartJsInstances.tcp.destroy();
      this._chartJsInstances.tcp = null;
    }

    this.container = null;
    this.dom = {};
  }

  /**
   * Alias for unbindDom
   */
  dispose() {
    this.unbindDom();
  }
}

export default TelemetryController;
