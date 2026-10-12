/**
 * I18nManager.js - Language Manager
 * Manages active language, translation lookups, and subscriber notifications.
 * Persists user's language choice in localStorage.
 */

import { translations, LANGUAGES } from './translations.js';

const STORAGE_KEY = 'robot_arm_language';
const DEFAULT_LANG = 'en';

export class I18nManager {
  constructor() {
    /** @type {'en'|'pl'} */
    this._lang = DEFAULT_LANG;
    /** @type {Set<Function>} */
    this._subscribers = new Set();

    // Restore persisted language preference
    try {
      const saved = (typeof localStorage !== 'undefined') && localStorage.getItem(STORAGE_KEY);
      if (saved && LANGUAGES.includes(saved)) {
        this._lang = saved;
      }
    } catch (_) { /* storage unavailable */ }
  }

  /**
   * Current active language code ('en' or 'pl')
   * @returns {'en'|'pl'}
   */
  get lang() {
    return this._lang;
  }

  /**
   * Set active language and notify all subscribers
   * @param {'en'|'pl'} lang
   */
  setLang(lang) {
    if (!LANGUAGES.includes(lang)) return;
    this._lang = lang;
    try {
      if (typeof localStorage !== 'undefined') {
        localStorage.setItem(STORAGE_KEY, lang);
      }
    } catch (_) { /* storage unavailable */ }
    this._notify();
  }

  /**
   * Toggle between 'en' and 'pl'
   */
  toggle() {
    this.setLang(this._lang === 'en' ? 'pl' : 'en');
  }

  /**
   * Get translated string for key in current language.
   * Falls back to English, then to the key itself.
   * @param {string} key
   * @returns {string}
   */
  t(key) {
    const entry = translations[key];
    if (!entry) return key;
    return entry[this._lang] ?? entry['en'] ?? key;
  }

  /**
   * Subscribe to language change events.
   * @param {Function} callback - Called with no args whenever language changes.
   * @returns {Function} Unsubscribe function
   */
  subscribe(callback) {
    this._subscribers.add(callback);
    return () => this._subscribers.delete(callback);
  }

  /** @private */
  _notify() {
    for (const cb of this._subscribers) {
      try { cb(this._lang); } catch (e) { console.warn('I18nManager subscriber error:', e); }
    }
  }
}

// Singleton instance shared across the application
export const i18n = new I18nManager();
export default i18n;
