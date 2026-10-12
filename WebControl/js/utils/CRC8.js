/**
 * CRC8.js - Bit-Exact CRC-8 Dallas/Maxim Implementation
 * 
 * Polynomial: 0x31 (x^8 + x^5 + x^4 + 1)
 * Reflected / Reversed Polynomial: 0x8C (LSB first)
 * Initial Value: 0x00
 * Final XOR: 0x00
 * 
 * Matches Teensy 4.1 firmware crc8_maxim() in IK_Control.ino bit-for-bit:
 * uint8_t crc8_maxim(const uint8_t *data, size_t len)
 */

/**
 * Precomputed CRC-8 Dallas/Maxim Lookup Table (256 entries)
 * Polynomial 0x31 reflected (0x8C), Initial 0x00
 */
export const CRC8_MAXIM_TABLE = new Uint8Array([
  0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
  0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
  0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
  0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
  0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
  0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
  0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
  0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
  0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
  0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
  0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
  0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
  0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
  0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
  0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
  0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
  0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
  0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
  0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
  0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
  0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
  0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
  0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
  0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
  0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
  0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
  0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
  0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
  0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
  0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
  0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
  0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
]);

/**
 * Normalizes various input types into a Uint8Array byte sequence
 * @param {string|Uint8Array|ArrayBuffer|Array<number>|DataView} input
 * @returns {Uint8Array}
 */
function toUint8Array(input) {
  if (typeof input === 'string') {
    return new TextEncoder().encode(input);
  }
  if (input instanceof Uint8Array) {
    return input;
  }
  if (input instanceof ArrayBuffer) {
    return new Uint8Array(input);
  }
  if (ArrayBuffer.isView(input)) {
    return new Uint8Array(input.buffer, input.byteOffset, input.byteLength);
  }
  if (Array.isArray(input)) {
    return new Uint8Array(input);
  }
  throw new TypeError('Unsupported input type for CRC calculation: ' + typeof input);
}

/**
 * Computes CRC-8 Maxim using bitwise calculation matching firmware loop
 * @param {string|Uint8Array|ArrayBuffer|Array<number>} data
 * @returns {number} 8-bit unsigned integer (0-255)
 */
export function crc8MaximBitwise(data) {
  const bytes = toUint8Array(data);
  let crc = 0;
  for (let i = 0; i < bytes.length; i++) {
    let inbyte = bytes[i];
    for (let j = 0; j < 8; j++) {
      const mix = (crc ^ inbyte) & 0x01;
      crc >>>= 1;
      if (mix) {
        crc ^= 0x8C;
      }
      inbyte >>>= 1;
    }
  }
  return crc & 0xFF;
}

/**
 * Computes CRC-8 Maxim using high-performance 256-entry lookup table
 * @param {string|Uint8Array|ArrayBuffer|Array<number>} data
 * @returns {number} 8-bit unsigned integer (0-255)
 */
export function crc8MaximTable(data) {
  const bytes = toUint8Array(data);
  let crc = 0;
  for (let i = 0; i < bytes.length; i++) {
    crc = CRC8_MAXIM_TABLE[(crc ^ bytes[i]) & 0xFF];
  }
  return crc & 0xFF;
}

/**
 * Default fast CRC-8 Maxim computation
 * @param {string|Uint8Array|ArrayBuffer|Array<number>} data
 * @returns {number} 8-bit unsigned integer (0-255)
 */
export function crc8Maxim(data) {
  return crc8MaximTable(data);
}

/**
 * Class-based interface for CRC-8 Dallas/Maxim operations
 */
export class CRC8Maxim {
  /**
   * Compute CRC-8 Maxim (fast lookup table)
   * @param {string|Uint8Array|ArrayBuffer|Array<number>} data
   * @returns {number} 8-bit integer [0, 255]
   */
  static compute(data) {
    return crc8MaximTable(data);
  }

  /**
   * Compute CRC-8 Maxim (bitwise algorithm)
   * @param {string|Uint8Array|ArrayBuffer|Array<number>} data
   * @returns {number} 8-bit integer [0, 255]
   */
  static computeBitwise(data) {
    return crc8MaximBitwise(data);
  }

  /**
   * Compute CRC-8 Maxim with lookup table
   * @param {string|Uint8Array|ArrayBuffer|Array<number>} data
   * @returns {number} 8-bit integer [0, 255]
   */
  static computeTable(data) {
    return crc8MaximTable(data);
  }

  /**
   * Verify if data matches expected CRC-8 value
   * @param {string|Uint8Array|ArrayBuffer|Array<number>} data
   * @param {number} expectedCrc
   * @returns {boolean}
   */
  static verify(data, expectedCrc) {
    return crc8MaximTable(data) === (expectedCrc & 0xFF);
  }

  /**
   * Generates lookup table dynamically (for validation / inspection)
   * @returns {Uint8Array}
   */
  static generateTable() {
    const table = new Uint8Array(256);
    for (let i = 0; i < 256; i++) {
      let curr = i;
      for (let j = 0; j < 8; j++) {
        if ((curr & 0x01) !== 0) {
          curr = (curr >>> 1) ^ 0x8C;
        } else {
          curr = curr >>> 1;
        }
      }
      table[i] = curr & 0xFF;
    }
    return table;
  }

  /**
   * Returns precomputed lookup table
   * @returns {Uint8Array}
   */
  static getTable() {
    return CRC8_MAXIM_TABLE;
  }
}

export default CRC8Maxim;
