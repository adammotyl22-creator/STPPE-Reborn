/**
 * TelemetryParser.js - Stream-buffered Telemetry & Serial Protocol Parser
 * 
 * Handles incoming Teensy 4.1 serial streams:
 * - 10Hz JSON Telemetry (`JTELEM` stream & `STATUS` query)
 * - AR4 Telemetry format (`A...B...C...D...E...F...G...H...I...`)
 * - Collision flags (`EC000`, `EC100`, etc.)
 * - Framed Packet Acknowledgment (`PKT (CRC OK): <...>`)
 * - Acknowledgment (`OK`), Errors (`ERR:...`), Warnings (`WARN:...`)
 * - General firmware log and status messages
 */

/**
 * Parses a single complete serial line
 * @param {string} rawLine - Single line of text from serial stream
 * @returns {Object|null} Parsed packet structure or null if empty
 */
export function parseSerialLine(rawLine) {
  if (typeof rawLine !== 'string') return null;
  const line = rawLine.trim();
  if (!line) return null;

  const now = Date.now();

  // 1. JSON Telemetry (JTELEM 10Hz stream or STATUS query)
  if (line.startsWith('{') && line.endsWith('}')) {
    try {
      const data = JSON.parse(line);
      return {
        type: 'JTELEM',
        timestamp: now,
        j1: Number(data.j1 ?? 0),
        j2: Number(data.j2 ?? 0),
        j3: Number(data.j3 ?? 0),
        j4: Number(data.j4 ?? 90),
        j5: Number(data.j5 ?? 90),
        grip: Number(data.grip ?? 0),
        x: Number(data.x ?? 0),
        y: Number(data.y ?? 0),
        z: Number(data.z ?? 0),
        spd: Number(data.spd ?? 100),
        estop: Boolean(data.estop),
        homing: Boolean(data.homing),
        teach: Boolean(data.teach),
        tpts: Number(data.tpts ?? 0),
        raw: line
      };
    } catch (e) {
      return {
        type: 'ERROR',
        code: 'MALFORMED_JSON',
        message: e.message,
        timestamp: now,
        raw: line
      };
    }
  }

  // 2. AR4 Telemetry format (A...B...C...D...E...F...G...H...I...)
  if (line.startsWith('A') && line.includes('B') && line.includes('C') && line.includes('G')) {
    const regex = /^A([-\d.]+)B([-\d.]+)C([-\d.]+)D([-\d.]+)E([-\d.]+)F([-\d.]+)G([-\d.]+)H([-\d.]+)I([-\d.]+)/;
    const m = line.match(regex);
    if (m) {
      return {
        type: 'AR4_TELEM',
        timestamp: now,
        j1: parseFloat(m[1]),
        j2: parseFloat(m[2]),
        j3: parseFloat(m[3]),
        j4: parseFloat(m[4]),
        j5: parseFloat(m[5]),
        j6: parseFloat(m[6]),
        x: parseFloat(m[7]),
        y: parseFloat(m[8]),
        z: parseFloat(m[9]),
        raw: line
      };
    }
  }

  // 3. Collision flag (EC000 .. EC111)
  if (line.startsWith('EC') && line.length === 5) {
    const j1Col = line[2] === '1';
    const j2Col = line[3] === '1';
    const j3Col = line[4] === '1';
    return {
      type: 'COLLISION_FLAG',
      timestamp: now,
      j1: j1Col,
      j2: j2Col,
      j3: j3Col,
      hasCollision: j1Col || j2Col || j3Col,
      raw: line
    };
  }

  // 4. Framed Packet CRC Acknowledgment
  if (line.startsWith('PKT (CRC OK):')) {
    return {
      type: 'PKT_ACK',
      timestamp: now,
      payload: line.substring(13).trim(),
      raw: line
    };
  }

  // 5. Standard Acknowledgment
  if (line === 'OK') {
    return {
      type: 'ACK',
      status: 'OK',
      timestamp: now,
      raw: line
    };
  }

  // 6. Emergency Stop Alert or Error lines
  if (line.startsWith('ERR:') || line.includes('EMERGENCY STOP')) {
    const code = line.startsWith('ERR:') ? line.substring(4).trim() : line;
    return {
      type: 'ERROR',
      code: code,
      isEstop: line.includes('EMERGENCY STOP') || code.includes('ESTOP'),
      timestamp: now,
      raw: line
    };
  }

  // 7. Warning lines
  if (line.startsWith('WARN:') || line.startsWith('WARNING:')) {
    const msg = line.replace(/^WARN(ING)?:\s*/i, '').trim();
    return {
      type: 'WARN',
      message: msg,
      timestamp: now,
      raw: line
    };
  }

  // 8. ESP32 Master Bridge Handheld Telemetry (REMOTE_DATA:{...})
  if (line.startsWith('REMOTE_DATA:')) {
    try {
      const jsonStr = line.substring(12).trim();
      const data = JSON.parse(jsonStr);
      return {
        type: 'REMOTE_DATA',
        timestamp: now,
        pitch: Number(data.pitch ?? 0),
        roll: Number(data.roll ?? 0),
        clutch: Boolean(data.clutch),
        scale: Number(data.scale ?? 1.0),
        speed: Number(data.speed ?? 100),
        battery: Number(data.bat ?? 100),
        online: Boolean(data.online),
        data,
        raw: line
      };
    } catch (_) {
      // Fall through to LOG if not valid JSON
    }
  }

  // 9. ESP32 Master Bridge Scale / Speed Update (SCALE_UPDATE {...})
  if (line.startsWith('SCALE_UPDATE')) {
    try {
      const jsonStr = line.substring(12).trim();
      const data = JSON.parse(jsonStr);
      return {
        type: 'SCALE_UPDATE',
        timestamp: now,
        scale: Number(data.scale ?? 1.0),
        speed: Number(data.speed ?? 100),
        data,
        raw: line
      };
    } catch (_) {
      // Fall through to LOG if not valid JSON
    }
  }

  // 10. General Log / Status message
  return {
    type: 'LOG',
    text: line,
    timestamp: now,
    raw: line
  };
}

/**
 * Line-buffered stream telemetry parser with rate calculation and event dispatch
 */
export class TelemetryParser {
  constructor() {
    this.buffer = '';
    this.listeners = new Map();
    this.rateTimestamps = [];
    this.rateWindowMs = 2000;
  }

  /**
   * Subscribe to parsed packet events
   * @param {string} event - 'packet', 'jtelem', 'ar4', 'collision', 'ack', 'error', 'warn', 'log'
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
   * Emit event to listeners
   * @private
   */
  emit(event, data) {
    const norm = event.toLowerCase();
    if (this.listeners.has(norm)) {
      for (const fn of this.listeners.get(norm)) {
        try {
          fn(data);
        } catch (e) {
          console.error(`Error in TelemetryParser listener for '${event}':`, e);
        }
      }
    }
  }

  /**
   * Feed raw text chunk into line buffer and return parsed packets
   * @param {string} chunk - Incoming serial chunk
   * @returns {Array<Object>} List of parsed packets from complete lines
   */
  push(chunk) {
    if (typeof chunk !== 'string' || !chunk) return [];
    
    this.buffer += chunk;
    const lines = this.buffer.split(/\r?\n/);
    
    // The last element is either empty (if chunk ended with newline) or incomplete line fragment
    this.buffer = lines.pop() ?? '';

    const packets = [];
    for (const rawLine of lines) {
      const parsed = parseSerialLine(rawLine);
      if (parsed) {
        this.trackPacket(parsed);
        packets.push(parsed);
        this.dispatchPacket(parsed);
      }
    }
    return packets;
  }

  /**
   * Process and parse any remaining buffer content
   * @returns {Array<Object>}
   */
  flush() {
    if (!this.buffer.trim()) {
      this.buffer = '';
      return [];
    }
    const rawLine = this.buffer;
    this.buffer = '';
    const parsed = parseSerialLine(rawLine);
    if (parsed) {
      this.trackPacket(parsed);
      this.dispatchPacket(parsed);
      return [parsed];
    }
    return [];
  }

  /**
   * Clears internal line buffer
   */
  reset() {
    this.buffer = '';
    this.rateTimestamps = [];
  }

  /**
   * Internal packet tracking for Hz calculation
   * @private
   */
  trackPacket(packet) {
    if (packet.type === 'JTELEM') {
      const now = packet.timestamp || Date.now();
      this.rateTimestamps.push(now);
      const cutoff = now - this.rateWindowMs;
      while (this.rateTimestamps.length > 0 && this.rateTimestamps[0] < cutoff) {
        this.rateTimestamps.shift();
      }
    }
  }

  /**
   * Dispatches packet to specific event listeners
   * @private
   */
  dispatchPacket(packet) {
    // Global packet event
    this.emit('packet', packet);

    // Specific typed events
    switch (packet.type) {
      case 'JTELEM':
        this.emit('jtelem', packet);
        break;
      case 'AR4_TELEM':
        this.emit('ar4', packet);
        break;
      case 'COLLISION_FLAG':
        this.emit('collision', packet);
        break;
      case 'ACK':
        this.emit('ack', packet);
        break;
      case 'PKT_ACK':
        this.emit('pkt_ack', packet);
        break;
      case 'ERROR':
        this.emit('error', packet);
        break;
      case 'WARN':
        this.emit('warn', packet);
        break;
      case 'LOG':
        this.emit('log', packet);
        break;
      case 'REMOTE_DATA':
        this.emit('remote_data', packet);
        break;
      case 'SCALE_UPDATE':
        this.emit('scale_update', packet);
        break;
    }
  }

  /**
   * Returns current telemetry packet rate in Hz
   * @returns {number} Frequency in Hz (e.g. 10.0)
   */
  getRateHz() {
    const now = Date.now();
    const cutoff = now - this.rateWindowMs;
    while (this.rateTimestamps.length > 0 && this.rateTimestamps[0] < cutoff) {
      this.rateTimestamps.shift();
    }
    if (this.rateTimestamps.length < 2) return 0.0;

    const spanSec = (this.rateTimestamps[this.rateTimestamps.length - 1] - this.rateTimestamps[0]) / 1000;
    if (spanSec <= 0) return 0.0;

    const hz = (this.rateTimestamps.length - 1) / spanSec;
    return Math.round(hz * 10) / 10;
  }
}

export default TelemetryParser;
