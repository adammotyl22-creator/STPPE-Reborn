/**
 * ConsoleController.js - Master Interactive Serial Terminal & Command Controller
 * 
 * Provides an interactive serial console with:
 * - Color-coded badges for TX (Cyan), RX (Green), JTELEM (Muted), ALERT/COLLISION (Amber/Crimson), ERROR (Red), WARN (Yellow), INFO (Slate)
 * - Filter chips: [ All ], [ TX ], [ RX ], [ Telemetry ], [ Errors ]
 * - Command prompt with Up/Down arrow history buffer navigation
 * - Quick command action chips: [ HOME ALL ], [ STATUS ], [ DIAG ], [ CL_ON ], [ CL_OFF ], [ ESTOP ], [ CAL ]
 * - Auto-scroll lock toggle, Clear logs button, and Collapsible terminal state
 */

import { appState as defaultAppState } from '../state/AppState.js';

export const CONSOLE_FILTERS = Object.freeze({
  ALL: 'all',
  TX: 'tx',
  RX: 'rx',
  TELEMETRY: 'telemetry',
  ERRORS: 'errors'
});

export const QUICK_COMMANDS = Object.freeze([
  { label: 'HOME ALL', command: 'HOME\n', title: 'Home all robot axes' },
  { label: 'STATUS', command: 'STATUS\n', title: 'Query firmware status & telemetry' },
  { label: 'DIAG', command: 'DIAG\n', title: 'Run hardware diagnostics' },
  { label: 'CL_ON', command: 'CL_ON\n', title: 'Enable closed-loop encoder feedback' },
  { label: 'CL_OFF', command: 'CL_OFF\n', title: 'Disable closed-loop encoder feedback' },
  { label: 'ESTOP', command: 'ESTOP\n', className: 'estop', title: 'Emergency stop steppers' },
  { label: 'CAL', command: 'CAL\n', title: 'Calibrate encoder zero references' }
]);

export const MAX_COMMAND_HISTORY = 100;

export class ConsoleController {
  /**
   * @param {Object} [options]
   * @param {HTMLElement|string} [options.container] - DOM element or selector
   * @param {AppState} [options.appState] - Central reactive state store
   * @param {WebSerialManager} [options.serialManager] - WebSerial communication manager
   * @param {boolean} [options.autoScroll=true] - Auto-scroll on new log entries
   * @param {boolean} [options.isCollapsed=false] - Collapsed state of terminal
   * @param {string} [options.activeFilter='all'] - Initial filter chip
   */
  constructor(options = {}) {
    this.appState = options.appState || defaultAppState;
    this.serialManager = options.serialManager || null;
    this.autoScroll = options.autoScroll !== false;
    this.isCollapsed = Boolean(options.isCollapsed);
    this.activeFilter = options.activeFilter || CONSOLE_FILTERS.ALL;

    // Command History Buffer & Navigation State
    this.commandHistory = [];
    this.historyIndex = -1;
    this.draftInput = '';

    // Logs Array (mirrors / caches AppState logs)
    this.logs = [];

    // DOM References & Event Cleanups
    this.container = null;
    this.dom = {};
    this._eventCleanups = [];
    this._unsubState = [];

    if (options.serialManager) {
      this.setSerialManager(options.serialManager);
    }

    // Initialize AppState Subscribers
    this._initAppStateListeners();

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
   * Sets serial manager and binds serial stream TX/RX events
   * @param {WebSerialManager} manager
   */
  setSerialManager(manager) {
    this.serialManager = manager;
    if (this.serialManager) {
      this.serialManager.on('tx', ({ data, timestamp }) => {
        this.addLog('TX', data.trim(), timestamp);
      });
      this.serialManager.on('rx', ({ line, timestamp }) => {
        this.addLog('RX', line.trim(), timestamp);
      });
      this.serialManager.on('collision', (pkt) => {
        this.addLog('COLLISION', pkt.raw || 'COLLISION ALERT', pkt.timestamp);
      });
      this.serialManager.on('error', ({ message, timestamp }) => {
        this.addLog('ERROR', message, timestamp);
      });
    }
  }

  /**
   * Initializes listeners on AppState
   * @private
   */
  _initAppStateListeners() {
    if (!this.appState) return;

    // Populate initial logs from AppState
    const initialLogs = this.appState.getState('history.logs') || [];
    this.logs = initialLogs.slice();

    // Listen for new log entries
    const unsubLog = this.appState.subscribe('log:added', (entry) => {
      if (entry && !this.logs.some(l => l.id === entry.id)) {
        this.logs.push(entry);
        if (this.logs.length > 300) {
          this.logs.shift();
        }
        this._appendLogToDOM(entry);
        this._updateCountBadge();
      }
    });
    this._unsubState.push(unsubLog);

    // Listen for clear events
    const unsubClear = this.appState.subscribe('log:cleared', () => {
      this.logs = [];
      if (this.dom.body) {
        this.dom.body.innerHTML = '';
      }
      this._updateCountBadge();
    });
    this._unsubState.push(unsubClear);
  }

  // ==========================================================================
  // LOG ENTRY INGESTION & FILTERING
  // ==========================================================================

  /**
   * Adds a new log entry to terminal and AppState
   * @param {string} type - 'TX'|'RX'|'JTELEM'|'COLLISION'|'ERROR'|'WARN'|'INFO'|'LOG'
   * @param {string} text
   * @param {number} [timestamp]
   * @returns {Object} Created log entry
   */
  addLog(type, text, timestamp = Date.now()) {
    if (!text && text !== '') return null;
    const cleanType = String(type || 'INFO').toUpperCase();
    const cleanText = String(text);

    if (this.appState && typeof this.appState.addConsoleLog === 'function') {
      return this.appState.addConsoleLog({ type: cleanType, text: cleanText, timestamp });
    }

    const entry = {
      id: 'log_' + Date.now() + '_' + Math.random().toString(36).substr(2, 5),
      type: cleanType,
      text: cleanText,
      timestamp: Number(timestamp || Date.now())
    };

    this.logs.push(entry);
    if (this.logs.length > 300) {
      this.logs.shift();
    }

    this._appendLogToDOM(entry);
    this._updateCountBadge();
    return entry;
  }

  /**
   * Clears terminal logs
   */
  clearLogs() {
    this.logs = [];
    if (this.appState && typeof this.appState.clearConsoleLogs === 'function') {
      this.appState.clearConsoleLogs();
    }
    if (this.dom.body) {
      this.dom.body.innerHTML = '';
    }
    this._updateCountBadge();
  }

  /**
   * Sets active filter chip ('all', 'tx', 'rx', 'telemetry', 'errors')
   * @param {string} filter
   */
  setFilter(filter) {
    this.activeFilter = filter.toLowerCase();
    this._updateFilterChipsUI();
    this.renderLogs();
  }

  /**
   * Determines if a log entry matches the current active filter
   * @param {Object} entry
   * @returns {boolean}
   */
  matchesFilter(entry) {
    if (!entry) return false;
    const f = this.activeFilter;
    if (f === CONSOLE_FILTERS.ALL) return true;

    const t = (entry.type || '').toUpperCase();
    const txt = (entry.text || '').toUpperCase();

    if (f === CONSOLE_FILTERS.TX) {
      return t === 'TX';
    }
    if (f === CONSOLE_FILTERS.RX) {
      return t === 'RX' || t === 'ACK' || t === 'PKT_ACK';
    }
    if (f === CONSOLE_FILTERS.TELEMETRY) {
      return t === 'JTELEM' || t === 'AR4_TELEM' || t === 'TELEM' || (t === 'RX' && txt.startsWith('{') && txt.includes('"J1"'));
    }
    if (f === CONSOLE_FILTERS.ERRORS) {
      return t === 'ERROR' || t === 'ERR' || t === 'WARN' || t === 'WARNING' || t === 'COLLISION' || t === 'ALERT' || t === 'COLLISION_FLAG' || txt.includes('ERR:') || txt.includes('COLLISION');
    }

    return true;
  }

  // ==========================================================================
  // COMMAND EXECUTION & HISTORY NAVIGATION
  // ==========================================================================

  /**
   * Executes a command string: sends via WebSerial, logs as TX, records in history
   * @param {string} rawCommand
   * @returns {Promise<boolean>}
   */
  async executeCommand(rawCommand) {
    if (typeof rawCommand !== 'string') return false;
    const cmd = rawCommand.trim();
    if (!cmd) return false;

    // Add to history buffer (deduping consecutive identical entries)
    if (this.commandHistory.length === 0 || this.commandHistory[this.commandHistory.length - 1] !== cmd) {
      this.commandHistory.push(cmd);
      if (this.commandHistory.length > MAX_COMMAND_HISTORY) {
        this.commandHistory.shift();
      }
    }
    this.historyIndex = this.commandHistory.length;
    this.draftInput = '';

    // Send to connected serial device
    if (this.serialManager && this.serialManager.isConnected()) {
      try {
        const payload = cmd.endsWith('\n') ? cmd : `${cmd}\n`;
        await this.serialManager.send(payload);
        return true;
      } catch (err) {
        this.addLog('ERROR', `Send failed: ${err.message}`);
        return false;
      }
    } else {
      this.addLog('TX', cmd);
      this.addLog('WARN', `[Offline] Port disconnected. Command '${cmd}' not transmitted to hardware.`);
      return true;
    }
  }

  /**
   * Sets the pending draft input command
   * @param {string} draft
   */
  setDraftCommand(draft) {
    this.draftInput = String(draft || '');
  }

  /**
   * Navigates command history in given direction ('UP' or 'DOWN')
   * @param {'UP'|'DOWN'} direction
   * @param {string} [currentDraft]
   * @returns {string} Historical command text or draft
   */
  navigateHistory(direction = 'UP', currentDraft = this.draftInput) {
    const dir = String(direction).toUpperCase();
    if (dir === 'UP') {
      return this.navigateHistoryUp(currentDraft);
    } else {
      return this.navigateHistoryDown();
    }
  }

  /**
   * Navigates up (older) in command history buffer
   * @param {string} currentDraft - Current text in input box
   * @returns {string} Historical command text
   */
  navigateHistoryUp(currentDraft = '') {
    if (this.commandHistory.length === 0) return currentDraft;

    // If at the bottom, store draft
    if (this.historyIndex === this.commandHistory.length) {
      this.draftInput = currentDraft;
    }

    if (this.historyIndex > 0) {
      this.historyIndex--;
    }

    return this.commandHistory[this.historyIndex] ?? '';
  }

  /**
   * Navigates down (newer) in command history buffer
   * @returns {string} Historical command text or draft
   */
  navigateHistoryDown() {
    if (this.commandHistory.length === 0) return this.draftInput;

    if (this.historyIndex < this.commandHistory.length - 1) {
      this.historyIndex++;
      return this.commandHistory[this.historyIndex];
    } else {
      this.historyIndex = this.commandHistory.length;
      return this.draftInput;
    }
  }

  // ==========================================================================
  // COLLAPSIBLE & AUTO-SCROLL CONTROLS
  // ==========================================================================

  /**
   * Toggles terminal collapse / expand state
   * @param {boolean} [collapseState]
   * @returns {boolean} New collapsed state
   */
  toggleCollapse(collapseState) {
    this.isCollapsed = typeof collapseState === 'boolean' ? collapseState : !this.isCollapsed;
    if (this.dom.panel) {
      this.dom.panel.classList.toggle('collapsed', this.isCollapsed);
    }
    if (this.dom.btnCollapse) {
      this.dom.btnCollapse.textContent = this.isCollapsed ? 'Expand' : 'Collapse';
    }
    return this.isCollapsed;
  }

  /**
   * Toggles auto-scroll lock
   * @param {boolean} [scrollState]
   * @returns {boolean} New autoScroll state
   */
  toggleAutoScroll(scrollState) {
    this.autoScroll = typeof scrollState === 'boolean' ? scrollState : !this.autoScroll;
    if (this.dom.chkAutoScroll) {
      this.dom.chkAutoScroll.checked = this.autoScroll;
    }
    if (this.autoScroll) {
      this._scrollToBottom();
    }
    return this.autoScroll;
  }

  /**
   * Copies all currently displayed logs to clipboard
   */
  async copyLogs() {
    const text = this.logs
      .filter(l => this.matchesFilter(l))
      .map(l => `[${this._formatTimestamp(l.timestamp)}] [${l.type}] ${l.text}`)
      .join('\n');

    if (typeof navigator !== 'undefined' && navigator.clipboard) {
      try {
        await navigator.clipboard.writeText(text);
        this.addLog('INFO', 'Console logs copied to clipboard.');
      } catch (err) {
        console.error('Clipboard copy failed:', err);
      }
    }
    return text;
  }

  // ==========================================================================
  // DOM RENDERING & BADGE FORMATTING
  // ==========================================================================

  /**
   * Full re-render of log entries matching current filter
   */
  renderLogs() {
    if (!this.dom.body) return;
    this.dom.body.innerHTML = '';

    const matching = this.logs.filter(l => this.matchesFilter(l));
    for (const entry of matching) {
      const lineEl = this._createLogElement(entry);
      this.dom.body.appendChild(lineEl);
    }

    if (this.autoScroll) {
      this._scrollToBottom();
    }
  }

  _appendLogToDOM(entry) {
    if (!this.dom.body || !this.matchesFilter(entry)) return;
    const lineEl = this._createLogElement(entry);
    this.dom.body.appendChild(lineEl);

    // Limit DOM elements to 300
    while (this.dom.body.children.length > 300) {
      this.dom.body.removeChild(this.dom.body.firstChild);
    }

    if (this.autoScroll) {
      this._scrollToBottom();
    }
  }

  _createLogElement(entry) {
    const div = document.createElement('div');
    const typeLower = (entry.type || 'info').toLowerCase();
    div.className = `log-line type-${typeLower}`;

    const badgeClass = this._getBadgeClass(entry.type);
    const badgeLabel = this._getBadgeLabel(entry.type);
    const timeStr = this._formatTimestamp(entry.timestamp);

    div.innerHTML = `
      <span class="log-timestamp">[${timeStr}]</span>
      <span class="log-badge ${badgeClass}">${badgeLabel}</span>
      <span class="log-text">${this._escapeHTML(entry.text)}</span>
    `;
    return div;
  }

  _getBadgeClass(type = '') {
    const t = type.toUpperCase();
    if (t === 'TX') return 'badge-tx';
    if (t === 'RX' || t === 'ACK' || t === 'PKT_ACK') return 'badge-rx';
    if (t === 'JTELEM' || t === 'AR4_TELEM') return 'badge-jtelem';
    if (t === 'COLLISION' || t === 'COLLISION_FLAG' || t === 'ALERT') return 'badge-alert';
    if (t === 'ERROR' || t === 'ERR') return 'badge-error';
    if (t === 'WARN' || t === 'WARNING') return 'badge-warn';
    return 'badge-info';
  }

  _getBadgeLabel(type = '') {
    const t = type.toUpperCase();
    if (t === 'TX') return 'TX';
    if (t === 'RX') return 'RX';
    if (t === 'ACK' || t === 'PKT_ACK') return 'ACK';
    if (t === 'JTELEM' || t === 'AR4_TELEM') return 'TELEM';
    if (t === 'COLLISION' || t === 'COLLISION_FLAG') return 'ALERT';
    if (t === 'ERROR' || t === 'ERR') return 'ERR';
    if (t === 'WARN' || t === 'WARNING') return 'WARN';
    return t.substring(0, 5) || 'INFO';
  }

  _formatTimestamp(ts) {
    const d = new Date(ts || Date.now());
    const hh = String(d.getHours()).padStart(2, '0');
    const mm = String(d.getMinutes()).padStart(2, '0');
    const ss = String(d.getSeconds()).padStart(2, '0');
    const ms = String(d.getMilliseconds()).padStart(3, '0');
    return `${hh}:${mm}:${ss}.${ms}`;
  }

  _escapeHTML(str = '') {
    return String(str)
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;')
      .replace(/'/g, '&#039;');
  }

  _scrollToBottom() {
    if (this.dom.body) {
      this.dom.body.scrollTop = this.dom.body.scrollHeight;
    }
  }

  _updateCountBadge() {
    if (this.dom.countBadge) {
      this.dom.countBadge.textContent = `${this.logs.length} lines`;
    }
  }

  _updateFilterChipsUI() {
    if (!this.container) return;
    const chips = this.container.querySelectorAll('.filter-chip');
    chips.forEach(chip => {
      chip.classList.toggle('active', chip.dataset.filter === this.activeFilter);
    });
  }

  // ==========================================================================
  // DOM BINDING & TEMPLATE GENERATION
  // ==========================================================================

  /**
   * Binds ConsoleController to DOM container
   * @param {HTMLElement} container
   */
  bindDom(container) {
    if (!container) return;
    this.unbindDom();
    this.container = container;

    // Check if container already has markup or needs generation
    if (!container.querySelector('.console-panel')) {
      container.innerHTML = this.generateTemplateHTML();
    }

    this.dom.panel = container.querySelector('.console-panel');
    this.dom.body = container.querySelector('.console-body');
    this.dom.input = container.querySelector('.console-input');
    this.dom.btnSend = container.querySelector('.console-send-btn');
    this.dom.btnClear = container.querySelector('.btn-console-clear');
    this.dom.btnCollapse = container.querySelector('.btn-console-collapse');
    this.dom.btnCopy = container.querySelector('.btn-console-copy');
    this.dom.chkAutoScroll = container.querySelector('.chk-autoscroll');
    this.dom.countBadge = container.querySelector('.console-count-badge');

    // Attach Event Listeners
    if (this.dom.btnCollapse) {
      const onCollapseClick = () => this.toggleCollapse();
      this.dom.btnCollapse.addEventListener('click', onCollapseClick);
      this._eventCleanups.push(() => this.dom.btnCollapse.removeEventListener('click', onCollapseClick));
    }

    if (this.dom.btnClear) {
      const onClearClick = () => this.clearLogs();
      this.dom.btnClear.addEventListener('click', onClearClick);
      this._eventCleanups.push(() => this.dom.btnClear.removeEventListener('click', onClearClick));
    }

    if (this.dom.btnCopy) {
      const onCopyClick = () => this.copyLogs();
      this.dom.btnCopy.addEventListener('click', onCopyClick);
      this._eventCleanups.push(() => this.dom.btnCopy.removeEventListener('click', onCopyClick));
    }

    if (this.dom.chkAutoScroll) {
      const onScrollToggle = (e) => this.toggleAutoScroll(e.target.checked);
      this.dom.chkAutoScroll.addEventListener('change', onScrollToggle);
      this._eventCleanups.push(() => this.dom.chkAutoScroll.removeEventListener('change', onScrollToggle));
    }

    // Input prompt submission & arrow key history navigation
    if (this.dom.input) {
      const onKeyDown = (e) => {
        if (e.key === 'Enter') {
          e.preventDefault();
          const val = this.dom.input.value;
          if (val.trim()) {
            this.executeCommand(val);
            this.dom.input.value = '';
          }
        } else if (e.key === 'ArrowUp') {
          e.preventDefault();
          this.dom.input.value = this.navigateHistoryUp(this.dom.input.value);
        } else if (e.key === 'ArrowDown') {
          e.preventDefault();
          this.dom.input.value = this.navigateHistoryDown();
        }
      };
      this.dom.input.addEventListener('keydown', onKeyDown);
      this._eventCleanups.push(() => this.dom.input.removeEventListener('keydown', onKeyDown));
    }

    if (this.dom.btnSend) {
      const onSendClick = () => {
        if (this.dom.input) {
          const val = this.dom.input.value;
          if (val.trim()) {
            this.executeCommand(val);
            this.dom.input.value = '';
          }
        }
      };
      this.dom.btnSend.addEventListener('click', onSendClick);
      this._eventCleanups.push(() => this.dom.btnSend.removeEventListener('click', onSendClick));
    }

    // Filter Chips
    const filterChips = container.querySelectorAll('.filter-chip');
    filterChips.forEach(chip => {
      const onChipClick = () => this.setFilter(chip.dataset.filter);
      chip.addEventListener('click', onChipClick);
      this._eventCleanups.push(() => chip.removeEventListener('click', onChipClick));
    });

    // Quick Command Buttons
    const quickButtons = container.querySelectorAll('.quick-cmd-btn');
    quickButtons.forEach(btn => {
      const onQuickClick = () => {
        const cmd = btn.dataset.command;
        if (cmd) {
          this.executeCommand(cmd);
        }
      };
      btn.addEventListener('click', onQuickClick);
      this._eventCleanups.push(() => btn.removeEventListener('click', onQuickClick));
    });

    // Initial render
    this.renderLogs();
    this._updateCountBadge();
  }

  /**
   * Generates default HTML template string for Console UI
   * @returns {string}
   */
  generateTemplateHTML() {
    return `
      <div class="console-panel ${this.isCollapsed ? 'collapsed' : ''}">
        <!-- Terminal Header -->
        <div class="console-header">
          <div class="console-title-area">
            <span class="console-title">Command Terminal & Protocol Monitor</span>
            <span class="console-count-badge">0 lines</span>
          </div>
          <div class="console-header-controls">
            <label class="autoscroll-toggle">
              <input type="checkbox" class="chk-autoscroll" ${this.autoScroll ? 'checked' : ''}>
              <span>Auto-scroll</span>
            </label>
            <button class="console-btn btn-console-copy" title="Copy visible logs">Copy</button>
            <button class="console-btn btn-console-clear" title="Clear terminal logs">Clear</button>
          </div>
        </div>

        <!-- Filter Chips -->
        <div class="console-filters">
          <span class="filter-chip ${this.activeFilter === 'all' ? 'active' : ''}" data-filter="all">All Logs</span>
          <span class="filter-chip ${this.activeFilter === 'tx' ? 'active' : ''}" data-filter="tx">TX (Commands)</span>
          <span class="filter-chip ${this.activeFilter === 'rx' ? 'active' : ''}" data-filter="rx">RX (Responses)</span>
          <span class="filter-chip ${this.activeFilter === 'telemetry' ? 'active' : ''}" data-filter="telemetry">Telemetry (JTELEM)</span>
          <span class="filter-chip ${this.activeFilter === 'errors' ? 'active' : ''}" data-filter="errors">Errors & Alerts</span>
        </div>

        <!-- Log Viewport -->
        <div class="console-body"></div>

        <!-- Quick Commands Bar -->
        <div class="quick-commands-bar">
          ${QUICK_COMMANDS.map(q => `
            <button class="quick-cmd-btn ${q.className || ''}" data-command="${q.command}" title="${q.title}">
              ${q.label}
            </button>
          `).join('')}
        </div>

        <!-- Interactive Prompt Area -->
        <div class="console-prompt-area">
          <span class="prompt-icon">&gt;</span>
          <input type="text" class="console-input" placeholder="Enter firmware command (e.g. MOV 0 0 200, DIAG, STATUS, SPEED 50)..." spellcheck="false" autocomplete="off">
          <button class="console-send-btn">Send</button>
        </div>
      </div>
    `;
  }

  /**
   * Releases DOM bindings and subscriptions
   */
  unbindDom() {
    this._eventCleanups.forEach(fn => fn());
    this._eventCleanups = [];

    this._unsubState.forEach(fn => fn());
    this._unsubState = [];

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

export default ConsoleController;
