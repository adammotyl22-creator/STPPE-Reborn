/**
 * WebSerialManager.js - WebSerial Connection Lifecycle & Protocol Engine Manager
 * 
 * Manages USB Serial connection to Teensy 4.1 via browser WebSerial API or MockSerialPort.
 * Integrates stream decoding, line buffering, TelemetryParser, and connection events.
 */

import { TelemetryParser, parseSerialLine } from './TelemetryParser.js';
import { MockSerialPort } from './MockSerialPort.js';

export const ConnectionState = {
  DISCONNECTED: 'DISCONNECTED',
  CONNECTING: 'CONNECTING',
  CONNECTED: 'CONNECTED',
  DISCONNECTING: 'DISCONNECTING',
  ERROR: 'ERROR',
  RECONNECTING: 'RECONNECTING'
};

/**
 * Line-splitting TransformStream transformer
 */
export class LineSplitterTransformer {
  constructor() {
    this.container = '';
  }

  transform(chunk, controller) {
    this.container += chunk;
    const lines = this.container.split(/\r?\n/);
    this.container = lines.pop() ?? '';
    for (const line of lines) {
      if (line.trim()) {
        controller.enqueue(line);
      }
    }
  }

  flush(controller) {
    if (this.container.trim()) {
      controller.enqueue(this.container);
      this.container = '';
    }
  }
}

export class WebSerialManager {
  constructor(options = {}) {
    this.options = {
      baudRate: 115200,
      bufferSize: 8192,
      autoEnableJTelem: true,
      ...options
    };

    this.state = ConnectionState.DISCONNECTED;
    this.port = null;
    this.reader = null;
    this.writer = null;
    this.readableStreamClosed = null;
    this.writableStreamClosed = null;
    this._isDisconnecting = false;
    this._sendQueue = null;

    this.parser = new TelemetryParser();
    this.mockPort = null;
    this.isMock = false;

    this.transportMode = options.transportMode || 'serial'; // 'serial' | 'websocket'
    this.wsUrl = options.wsUrl || 'ws://192.168.4.1:81/';
    this.ws = null;
    this._wsOnMessage = null;
    this._wsOnClose = null;
    this._wsOnError = null;

    this.listeners = new Map();
    this._readLoopActive = false;

    // Forward parser events
    this.parser.on('packet', (pkt) => this.emit('data', pkt));
    this.parser.on('jtelem', (pkt) => this.emit('jtelem', pkt));
    this.parser.on('ar4', (pkt) => this.emit('ar4', pkt));
    this.parser.on('collision', (pkt) => this.emit('collision', pkt));
    this.parser.on('error', (pkt) => this.emit('parserError', pkt));
    this.parser.on('remote_data', (pkt) => this.emit('remote_data', pkt));
    this.parser.on('scale_update', (pkt) => this.emit('scale_update', pkt));

    // Monitor telemetry rate
    this._rateInterval = setInterval(() => {
      if (this.isConnected()) {
        const rateHz = this.parser.getRateHz();
        this.emit('rateChange', { rateHz });
      }
    }, 1000);

    // Bind browser serial disconnect if available
    this._onSerialDisconnect = (event) => {
      if (event.port === this.port) {
        this._handleUnexpectedDisconnect('Hardware disconnected');
      }
    };
    if (typeof navigator !== 'undefined' && navigator.serial) {
      navigator.serial.addEventListener('disconnect', this._onSerialDisconnect);
    }
  }

  /**
   * Check if WebSerial API is supported in current environment
   * @returns {boolean}
   */
  static isSupported() {
    return typeof navigator !== 'undefined' && 'serial' in navigator;
  }

  /**
   * Check if WebSocket API is supported in current environment
   * @returns {boolean}
   */
  static isWebSocketSupported() {
    return typeof WebSocket !== 'undefined';
  }

  /**
   * Subscribe to manager events
   * @param {string} event - 'statusChange'|'connected'|'disconnected'|'data'|'tx'|'rx'|'error'|'rateChange'
   * @param {Function} handler
   */
  on(event, handler) {
    const norm = event.toLowerCase();
    if (!this.listeners.has(norm)) {
      this.listeners.set(norm, new Set());
    }
    this.listeners.get(norm).add(handler);
    return () => this.off(norm, handler);
  }

  /**
   * Unsubscribe from events
   * @param {string} event
   * @param {Function} handler
   */
  off(event, handler) {
    const norm = event.toLowerCase();
    if (this.listeners.has(norm)) {
      this.listeners.get(norm).delete(handler);
    }
  }

  /**
   * Emit internal event
   * @private
   */
  emit(event, data) {
    const norm = event.toLowerCase();
    if (this.listeners.has(norm)) {
      for (const fn of this.listeners.get(norm)) {
        try {
          fn(data);
        } catch (e) {
          console.error(`Error in WebSerialManager listener for '${event}':`, e);
        }
      }
    }
  }

  /**
   * Updates state and emits statusChange event
   * @private
   */
  _setState(newState, details = null) {
    if (this.state === newState) return;
    const oldState = this.state;
    this.state = newState;
    this.emit('statusChange', { oldState, newState, details });
  }

  /**
   * Check if currently connected
   * @returns {boolean}
   */
  isConnected() {
    return this.state === ConnectionState.CONNECTED;
  }

  /**
   * Check if serial port is open
   * @returns {boolean}
   */
  get isOpen() {
    return this.isConnected() || Boolean(this.port && this.port.isOpen) || Boolean(this.ws && (this.ws.readyState === 1 || (typeof WebSocket !== 'undefined' && this.ws.readyState === WebSocket.OPEN)));
  }

  /**
   * Sets transport mode ('serial' or 'websocket')
   * @param {'serial'|'websocket'} mode
   * @param {string} [wsUrl] - Optional WebSocket endpoint (e.g. 'ws://192.168.4.1:81/')
   */
  setTransportMode(mode, wsUrl = null) {
    if (this.isConnected()) {
      throw new Error('Cannot change transport mode while connected. Disconnect first.');
    }
    const norm = String(mode).toLowerCase();
    if (norm !== 'serial' && norm !== 'websocket') {
      throw new Error(`Invalid transport mode: ${mode}. Expected 'serial' or 'websocket'.`);
    }
    this.transportMode = norm;
    if (wsUrl) {
      this.setWsUrl(wsUrl);
    }
    this.emit('transportChange', { transportMode: this.transportMode, wsUrl: this.wsUrl });
  }

  /**
   * Sets WebSocket URL endpoint
   * @param {string} url - e.g. '192.168.4.1:81' or 'ws://192.168.4.1:81/'
   */
  setWsUrl(url) {
    if (!url) return;
    let target = String(url).trim();
    if (!target.startsWith('ws://') && !target.startsWith('wss://')) {
      target = `ws://${target}`;
    }
    if (!target.endsWith('/')) {
      target = `${target}/`;
    }
    this.wsUrl = target;
  }

  /**
   * Requests a serial port via browser prompt
   * @returns {Promise<SerialPort>}
   */
  async requestPort() {
    if (!WebSerialManager.isSupported()) {
      throw new Error('WebSerial API is not supported in this browser. Use Mock Mode, Wi-Fi Bridge, or Chrome/Edge.');
    }
    return await navigator.serial.requestPort();
  }

  /**
   * Connect to WebSerial port, WebSocket bridge, or MockSerialPort
   * @param {Object} [customPort] - Explicit SerialPort, WebSocket, or MockSerialPort
   * @param {Object} [options] - Connection options (baudRate, wsUrl, etc.)
   */
  async connect(customPort = null, options = {}) {
    if (this.isConnected()) {
      return { success: true, port: this.port, ws: this.ws, transportMode: this.transportMode, isMock: this.isMock };
    }

    this._isDisconnecting = false;
    this._sendQueue = null;
    const opts = { ...this.options, ...options };
    this._setState(ConnectionState.CONNECTING);

    try {
      if (this.isMock || customPort instanceof MockSerialPort) {
        await this._connectMock(customPort, opts);
      } else if (this.transportMode === 'websocket' || opts.transport === 'websocket' || opts.transportMode === 'websocket') {
        const url = opts.wsUrl || this.wsUrl;
        await this._connectWebSocket(url, opts, customPort);
      } else {
        await this._connectHardware(customPort, opts);
      }

      this._setState(ConnectionState.CONNECTED);
      this.emit('connected', {
        port: this.port,
        ws: this.ws,
        transportMode: this.transportMode,
        baudRate: opts.baudRate,
        isMock: this.isMock
      });

      // Enable 10Hz JSON telemetry & set CL_OFF by default upon connection
      if (opts.autoEnableJTelem) {
        await this.send('JTELEM 1\n');
        await this.send('CL_OFF\n');
      }

      return {
        success: true,
        port: this.port,
        ws: this.ws,
        transportMode: this.transportMode,
        baudRate: opts.baudRate,
        isMock: this.isMock
      };
    } catch (err) {
      this._setState(ConnectionState.ERROR, err.message);
      this.emit('error', { error: err, message: err.message });
      await this.disconnect();
      throw err;
    }
  }

  /**
   * Connects to WebSocket bridge (e.g. ESP32-C3 Master Bridge)
   * @private
   */
  async _connectWebSocket(url, opts, customSocket = null) {
    let targetUrl = url || this.wsUrl;
    if (!targetUrl.startsWith('ws://') && !targetUrl.startsWith('wss://')) {
      targetUrl = `ws://${targetUrl}`;
    }

    let socket = customSocket;
    if (!socket) {
      if (typeof WebSocket === 'undefined') {
        throw new Error('WebSocket API is not supported in this environment');
      }
      socket = new WebSocket(targetUrl);
    }

    // Wait for connection open or error
    await new Promise((resolve, reject) => {
      let resolved = false;
      const timeoutMs = opts.timeout || 6000;
      const timer = setTimeout(() => {
        if (!resolved) {
          resolved = true;
          cleanup();
          reject(new Error(`WebSocket connection timed out to ${targetUrl}`));
        }
      }, timeoutMs);

      const onOpen = () => {
        if (!resolved) {
          resolved = true;
          cleanup();
          resolve();
        }
      };

      const onError = (err) => {
        if (!resolved) {
          resolved = true;
          cleanup();
          reject(err instanceof Error ? err : new Error(`WebSocket failed to connect to ${targetUrl}`));
        }
      };

      const cleanup = () => {
        clearTimeout(timer);
        if (socket.removeEventListener) {
          socket.removeEventListener('open', onOpen);
          socket.removeEventListener('error', onError);
        }
      };

      if (socket.readyState === 1 || socket.readyState === (typeof WebSocket !== 'undefined' ? WebSocket.OPEN : 1)) {
        cleanup();
        resolve();
      } else {
        if (socket.addEventListener) {
          socket.addEventListener('open', onOpen, { once: true });
          socket.addEventListener('error', onError, { once: true });
        } else {
          socket.onopen = onOpen;
          socket.onerror = onError;
        }
      }
    });

    this.ws = socket;
    this.parser.reset();

    this._wsOnMessage = (event) => {
      const data = event.data;
      if (typeof data === 'string') {
        this._handleIncomingChunk(data);
      } else if (data instanceof ArrayBuffer) {
        const text = new TextDecoder().decode(data);
        this._handleIncomingChunk(text);
      }
    };

    this._wsOnClose = (event) => {
      if (!this._isDisconnecting && this.isConnected()) {
        const reason = event && event.reason ? event.reason : 'WebSocket closed by remote host';
        this._handleUnexpectedDisconnect(reason);
      }
    };

    this._wsOnError = (err) => {
      if (this.isConnected()) {
        this.emit('error', { error: err, message: 'WebSocket communication error' });
      }
    };

    if (socket.addEventListener) {
      socket.addEventListener('message', this._wsOnMessage);
      socket.addEventListener('close', this._wsOnClose);
      socket.addEventListener('error', this._wsOnError);
    } else {
      socket.onmessage = this._wsOnMessage;
      socket.onclose = this._wsOnClose;
      socket.onerror = this._wsOnError;
    }
  }

  /**
   * Connects to hardware WebSerial port
   * @private
   */
  async _connectHardware(customPort, opts) {
    if (!WebSerialManager.isSupported()) {
      throw new Error('WebSerial API not supported in this environment');
    }

    let targetPort = customPort;
    if (!targetPort) {
      targetPort = await this.requestPort();
    }
    this.port = targetPort;

    await this.port.open({
      baudRate: opts.baudRate,
      bufferSize: opts.bufferSize,
      dataBits: 8,
      stopBits: 1,
      parity: 'none'
    });

    this.parser.reset();
    this._startHardwareReader();
    if (this.port && this.port.writable) {
      this.writer = this.port.writable.getWriter();
    }
  }

  /**
   * Starts reading from hardware WebSerial port stream
   * @private
   */
  _startHardwareReader() {
    this._readLoopActive = true;

    // Use pipeThrough or manual loop if TextDecoderStream available
    const textDecoder = new TextDecoder();
    const reader = this.port.readable.getReader();
    this.reader = reader;

    (async () => {
      while (this._readLoopActive) {
        try {
          const { value, done } = await reader.read();
          if (done) {
            break;
          }
          if (value) {
            const chunkText = typeof value === 'string' ? value : textDecoder.decode(value, { stream: true });
            this._handleIncomingChunk(chunkText);
          }
        } catch (err) {
          if (this._readLoopActive) {
            console.error('Serial read stream error:', err);
            this._handleUnexpectedDisconnect(err.message);
          }
          break;
        }
      }
      try {
        reader.releaseLock();
      } catch (_) {}
    })();
  }

  /**
   * Connects to MockSerialPort
   * @private
   */
  async _connectMock(customMockPort, opts) {
    this.isMock = true;
    this.mockPort = customMockPort || new MockSerialPort();
    await this.mockPort.open({ baudRate: opts.baudRate });
    this.port = this.mockPort;

    this.parser.reset();

    // Attach output listener
    this._mockUnsub = this.mockPort.onOutput((text) => {
      this._handleIncomingChunk(text);
    });
  }

  /**
   * Handles incoming text chunk from hardware or mock port
   * @private
   */
  _handleIncomingChunk(chunkText) {
    // Process through line-buffered telemetry parser
    const packets = this.parser.push(chunkText);
    for (const pkt of packets) {
      if (pkt.raw) {
        this.emit('rx', { line: pkt.raw, timestamp: pkt.timestamp });
      }
    }
  }

  /**
   * Transmits raw string command or framed packet to connected device
   * @param {string|Uint8Array} data
   * @returns {Promise<void>}
   */
  async send(data) {
    if (!this.isConnected() || this._isDisconnecting) {
      throw new Error('Cannot send: Serial port is not connected');
    }

    const text = typeof data === 'string' ? data : new TextDecoder().decode(data);
    const now = Date.now();

    const prevSend = this._sendQueue || Promise.resolve();
    this._sendQueue = (async () => {
      await prevSend.catch(() => {});
      if (this.state !== ConnectionState.CONNECTED || this._isDisconnecting || !this.isConnected()) {
        return;
      }
      try {
        if (this.isMock && this.mockPort) {
          this.mockPort.receiveFromClient(text);
        } else if (this.transportMode === 'websocket' && this.ws) {
          if (this.ws.readyState !== 1 && this.ws.readyState !== (typeof WebSocket !== 'undefined' ? WebSocket.OPEN : 1)) {
            throw new Error('WebSocket connection is not open');
          }
          this.ws.send(text);
        } else if (this.writer) {
          const encoder = new TextEncoder();
          const encoded = encoder.encode(text);
          await this.writer.write(encoded);
        } else if (this.port && this.port.writable && this.state === ConnectionState.CONNECTED && !this._isDisconnecting) {
          this.writer = this.port.writable.getWriter();
          const encoder = new TextEncoder();
          const encoded = encoder.encode(text);
          await this.writer.write(encoded);
        }
        this.emit('tx', { data: text, timestamp: now });
      } catch (err) {
        this.emit('error', { error: err, message: `Send failed: ${err.message}` });
        throw err;
      }
    })();

    return this._sendQueue;
  }

  /**
   * Alias for send()
   * @param {string|Uint8Array} data
   * @returns {Promise<void>}
   */
  async write(data) {
    return this.send(data);
  }

  /**
   * Disconnects and releases all resources cleanly
   * @param {string} [reason='User requested disconnect']
   * @returns {Promise<void>}
   */
  async disconnect(reason = 'User requested disconnect') {
    if (this.state === ConnectionState.DISCONNECTED) {
      return;
    }

    this._isDisconnecting = true;
    this._setState(ConnectionState.DISCONNECTING);
    this._readLoopActive = false;

    // Reset and detach send queue
    this._sendQueue = null;

    // Clean up WebSocket
    if (this.ws) {
      if (this._wsOnMessage && this.ws.removeEventListener) {
        this.ws.removeEventListener('message', this._wsOnMessage);
        this.ws.removeEventListener('close', this._wsOnClose);
        this.ws.removeEventListener('error', this._wsOnError);
      }
      try {
        this.ws.close();
      } catch (_) {}
      this.ws = null;
      this._wsOnMessage = null;
      this._wsOnClose = null;
      this._wsOnError = null;
    }

    // Clean up mock port
    if (this.mockPort) {
      if (this._mockUnsub) {
        this._mockUnsub();
        this._mockUnsub = null;
      }
      try {
        await this.mockPort.close();
      } catch (_) {}
      this.mockPort = null;
    }

    // Clean up hardware reader
    if (this.reader) {
      try {
        await this.reader.cancel();
      } catch (_) {}
      try {
        this.reader.releaseLock();
      } catch (_) {}
      this.reader = null;
    }

    // Clean up hardware writer
    const writerToRelease = this.writer;
    this.writer = null;
    if (writerToRelease) {
      try {
        await writerToRelease.close();
      } catch (_) {}
      try {
        writerToRelease.releaseLock();
      } catch (_) {}
    }

    // Close hardware port
    if (this.port && !this.isMock) {
      try {
        await this.port.close();
      } catch (err) {
        console.warn('Error closing serial port:', err);
      }
    }

    this.port = null;
    this.isMock = false;
    this.parser.reset();
    this._isDisconnecting = false;

    this._setState(ConnectionState.DISCONNECTED);
    this.emit('disconnected', { reason });
  }

  /**
   * Handles unexpected port disconnection (e.g. cable pull)
   * @private
   */
  async _handleUnexpectedDisconnect(reason) {
    this._readLoopActive = false;
    this._setState(ConnectionState.ERROR, reason);
    await this.disconnect(reason);
  }

  /**
   * Sets mock mode state
   * @param {boolean} enable
   * @param {MockSerialPort} [customMock]
   */
  setMockMode(enable, customMock = null) {
    this.isMock = Boolean(enable);
    if (customMock) {
      this.mockPort = customMock;
    }
  }

  /**
   * Destroys manager and clears all timers
   */
  destroy() {
    if (this._rateInterval) {
      clearInterval(this._rateInterval);
      this._rateInterval = null;
    }
    if (typeof navigator !== 'undefined' && navigator.serial && this._onSerialDisconnect) {
      navigator.serial.removeEventListener('disconnect', this._onSerialDisconnect);
    }
    this.disconnect().catch(() => {});
    this.listeners.clear();
  }
}

export default WebSerialManager;
