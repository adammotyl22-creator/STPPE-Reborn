/**
 * HorizontalTimelineController.js - Interactive Horizontal Keyframe Track & Playhead Scrubber
 * 
 * Provides a professional horizontal DAW/CAD-style timeline for robot trajectory sequences:
 *   - Proportional waypoint blocks based on duration (duration_ms).
 *   - Color-coded keyframe segments:
 *       • Green / Emerald (#10b981): Standard cubic spline moves
 *       • Titanium / White (#e2e8f0): Linear Cartesian MOVL moves
 *       • Warm Amber (#f59e0b): Gripper actuation and dwell pauses
 *   - Draggable playhead cursor for real-time 3D Ghost Arm scrubbing.
 *   - Drag-and-drop waypoint reordering.
 *   - Integrated transport controls (Play, Pause, Stop, Step, +Capture).
 */

import { appState as defaultAppState } from '../state/AppState.js';

export class HorizontalTimelineController {
  /**
   * @param {Object} options
   * @param {HTMLElement|string} options.container - DOM element to render into
   * @param {import('../state/AppState.js').AppState} [options.appState]
   * @param {import('./TrajectoryController.js').TrajectoryController} [options.trajectoryController]
   */
  constructor(options = {}) {
    this.appState = options.appState || defaultAppState;
    this.trajectoryController = options.trajectoryController || null;
    this.container = typeof options.container === 'string'
      ? document.querySelector(options.container)
      : options.container;

    // Timeline configuration
    this.scale = 0.075; // pixels per millisecond (1000ms = 75px)
    this.minBlockWidth = 70; // px
    this.currentTime = 0; // ms
    this.totalDuration = 0; // ms
    this.isScrubbing = false;
    this.selectedIndex = -1;
    this._dragSourceIndex = -1;

    // Subscriptions cleanup
    this._unsubs = [];

    // Bound window pointer listeners for scrubber drag
    this._handleWindowPointerMove = (e) => {
      if (this.isScrubbing) {
        this._handleScrub(e.clientX);
      }
    };
    this._handleWindowPointerUp = () => {
      if (this.isScrubbing) {
        this.isScrubbing = false;
        if (typeof document !== 'undefined' && document.body) {
          document.body.style.userSelect = '';
        }
      }
    };

    if (typeof window !== 'undefined') {
      window.addEventListener('pointermove', this._handleWindowPointerMove);
      window.addEventListener('pointerup', this._handleWindowPointerUp);
    }

    this._initAppStateListeners();
    if (this.container) {
      this.render();
    }
  }

  _initAppStateListeners() {
    if (!this.appState) return;

    const onSeqUpdate = () => {
      const seq = this.getSequence();
      this.totalDuration = this.calculateTotalDuration(seq.waypoints || []);
      this.render();
    };

    const unsubSeq = this.appState.subscribe('sequence:updated', onSeqUpdate);
    const unsubStateSeq = this.appState.subscribe('stateChange:sequence', onSeqUpdate);
    this._unsubs.push(unsubSeq, unsubStateSeq);

    const unsubPlayback = this.appState.subscribe('trajectory:playbackState', (state) => {
      this._updatePlaybackButtons(state);
    });
    this._unsubs.push(unsubPlayback);

    const unsubProgress = this.appState.subscribe('trajectory:progress', (progress) => {
      if (!this.isScrubbing && typeof progress === 'object' && progress.elapsedMs !== undefined) {
        this.setPlayheadTime(progress.elapsedMs, false);
      }
    });
    this._unsubs.push(unsubProgress);
  }

  getSequence() {
    if (this.trajectoryController && typeof this.trajectoryController.getSequence === 'function') {
      return this.trajectoryController.getSequence();
    }
    const stateSeq = this.appState ? this.appState.getState('sequence') : null;
    return stateSeq || { waypoints: [] };
  }

  /**
   * Calculates total duration of sequence in milliseconds
   */
  calculateTotalDuration(waypoints) {
    if (!Array.isArray(waypoints) || waypoints.length === 0) return 0;
    return waypoints.reduce((sum, wp) => sum + (Number(wp.dur || wp.duration || wp.duration_ms) || 1500), 0);
  }

  render() {
    if (!this.container || typeof document === 'undefined') return;

    const seq = this.getSequence();
    const waypoints = seq.waypoints || [];
    this.totalDuration = this.calculateTotalDuration(waypoints);

    this.container.innerHTML = `
      <div class="horizontal-timeline-wrap">
        <!-- Top Timeline Toolbar -->
        <div class="timeline-toolbar-bar">
          <div class="timeline-transport-group">
            <button type="button" class="btn btn-xs btn-primary btn-tl-play" id="tl-btn-play" title="Play Sequence"><svg class="ico" viewBox="0 0 16 16" fill="currentColor" style="width:10px;height:10px;"><polygon points="4,2 14,8 4,14"/></svg> Play</button>
            <button type="button" class="btn btn-xs btn-secondary btn-tl-pause" id="tl-btn-pause" title="Pause Sequence"><svg class="ico" viewBox="0 0 16 16" fill="currentColor" style="width:10px;height:10px;"><rect x="3" y="2" width="3.5" height="12" rx="0.5"/><rect x="9.5" y="2" width="3.5" height="12" rx="0.5"/></svg> Pause</button>
            <button type="button" class="btn btn-xs btn-secondary btn-tl-stop" id="tl-btn-stop" title="Stop Sequence"><svg class="ico" viewBox="0 0 16 16" fill="currentColor" style="width:10px;height:10px;"><rect x="3" y="3" width="10" height="10" rx="1"/></svg> Stop</button>
            <div class="tl-divider"></div>
            <button type="button" class="btn btn-xs btn-secondary" id="tl-btn-prev" title="Previous Waypoint"><svg class="ico" viewBox="0 0 16 16" fill="currentColor" style="width:10px;height:10px;"><rect x="2" y="3" width="2" height="10" rx="0.5"/><polygon points="14,3 6,8 14,13"/></svg></button>
            <button type="button" class="btn btn-xs btn-secondary" id="tl-btn-next" title="Next Waypoint"><svg class="ico" viewBox="0 0 16 16" fill="currentColor" style="width:10px;height:10px;"><polygon points="2,3 10,8 2,13"/><rect x="12" y="3" width="2" height="10" rx="0.5"/></svg></button>
            <div class="tl-divider"></div>
            <button type="button" class="btn btn-xs btn-action" id="tl-btn-capture" title="Capture current pose into timeline">+ Capture Point</button>
          </div>

          <div class="timeline-meta-group">
            <span class="timeline-meta-time" id="tl-time-readout">0.00s / ${(this.totalDuration / 1000).toFixed(2)}s</span>
            <span class="badge badge-slate" id="tl-wpt-counter">${waypoints.length} waypoints</span>
            <label class="timeline-loop-toggle">
              <input type="checkbox" id="tl-chk-loop" ${seq.loop ? 'checked' : ''}>
              <span>Loop</span>
            </label>
          </div>
        </div>

        <!-- Scrollable Track & Ruler Body -->
        <div class="timeline-scroll-track" id="tl-scroll-container">
          <!-- Time Ruler -->
          <div class="timeline-ruler" id="tl-ruler" style="width: ${Math.max(600, Math.round(this.totalDuration * this.scale) + 120)}px;">
            ${this._buildRulerMarkup(this.totalDuration)}
          </div>

          <!-- Keyframe Track -->
          <div class="timeline-track-canvas" id="tl-track" style="width: ${Math.max(600, Math.round(this.totalDuration * this.scale) + 120)}px;">
            ${this._buildBlocksMarkup(waypoints)}
            
            <!-- Draggable Playhead Needle -->
            <div class="timeline-playhead-cursor" id="tl-playhead" style="left: ${Math.round(this.currentTime * this.scale)}px;">
              <div class="playhead-head">▼</div>
              <div class="playhead-needle"></div>
            </div>
          </div>
        </div>
      </div>
    `;

    this._bindEvents();
  }

  _buildRulerMarkup(totalMs) {
    const stepMs = totalMs > 15000 ? 2000 : (totalMs > 6000 ? 1000 : 500);
    const count = Math.ceil(totalMs / stepMs) + 1;
    let html = '';
    for (let i = 0; i < count; i++) {
      const ms = i * stepMs;
      const x = Math.round(ms * this.scale);
      const sec = (ms / 1000).toFixed(1);
      html += `<div class="ruler-tick" style="left: ${x}px;"><span class="ruler-label">${sec}s</span></div>`;
    }
    return html;
  }

  _buildBlocksMarkup(waypoints) {
    if (waypoints.length === 0) {
      return `<div class="timeline-empty-hint">No waypoints in timeline. Click "+ Capture Point" or load a preset to begin.</div>`;
    }

    return waypoints.map((wp, i) => {
      const dur = Number(wp.dur || wp.duration || wp.duration_ms) || 1500;
      const blockWidth = Math.max(this.minBlockWidth, Math.round(dur * this.scale));
      const moveType = (wp.moveType || (wp.type === 'MOVL' ? 'MOVL' : 'SPLINE')).toUpperCase();
      
      let typeClass = 'block-spline';
      let typeLabel = 'SPLINE';
      if (moveType === 'MOVL') {
        typeClass = 'block-movl';
        typeLabel = 'MOVL';
      } else if (wp.action === 'GRIPPER' || dur <= 300) {
        typeClass = 'block-dwell';
        typeLabel = 'DWELL';
      }

      const isSelected = i === this.selectedIndex ? ' selected' : '';

      return `
        <div class="timeline-block ${typeClass}${isSelected}" 
             data-wpt-index="${i}" 
             draggable="true" 
             style="width: ${blockWidth}px;"
             title="Waypoint #${i + 1}: ${wp.label || 'Point'} (${dur}ms, ${typeLabel})">
          <div class="block-drag-handle">⠿</div>
          <div class="block-content">
            <div class="block-title">#${i + 1} ${wp.label || 'Point'}</div>
            <div class="block-sub">${dur}ms • ${typeLabel}</div>
          </div>
          <button type="button" class="btn-block-del" data-del-index="${i}" title="Delete Point">✕</button>
        </div>
      `;
    }).join('');
  }

  _bindEvents() {
    if (!this.container) return;

    // Transport buttons
    const btnPlay = this.container.querySelector('#tl-btn-play');
    if (btnPlay) {
      btnPlay.addEventListener('click', () => {
        if (this.trajectoryController) this.trajectoryController.play();
      });
    }

    const btnPause = this.container.querySelector('#tl-btn-pause');
    if (btnPause) {
      btnPause.addEventListener('click', () => {
        if (this.trajectoryController) this.trajectoryController.pause();
      });
    }

    const btnStop = this.container.querySelector('#tl-btn-stop');
    if (btnStop) {
      btnStop.addEventListener('click', () => {
        if (this.trajectoryController) this.trajectoryController.stop();
        this.setPlayheadTime(0, true);
      });
    }

    const btnPrev = this.container.querySelector('#tl-btn-prev');
    if (btnPrev) {
      btnPrev.addEventListener('click', () => {
        if (this.trajectoryController) this.trajectoryController.stepBackward();
      });
    }

    const btnNext = this.container.querySelector('#tl-btn-next');
    if (btnNext) {
      btnNext.addEventListener('click', () => {
        if (this.trajectoryController) this.trajectoryController.stepForward();
      });
    }

    const btnCapture = this.container.querySelector('#tl-btn-capture');
    if (btnCapture) {
      btnCapture.addEventListener('click', () => {
        if (this.trajectoryController) {
          this.trajectoryController.captureGhostPose();
        }
      });
    }

    const chkLoop = this.container.querySelector('#tl-chk-loop');
    if (chkLoop) {
      chkLoop.addEventListener('change', (e) => {
        if (this.trajectoryController) {
          if (typeof this.trajectoryController.setLoop === 'function') {
            this.trajectoryController.setLoop(e.target.checked);
          } else {
            this.trajectoryController.loop = e.target.checked;
          }
        }
      });
    }

    // Waypoint Block Selection & Deletion
    const blocks = this.container.querySelectorAll('.timeline-block[data-wpt-index]');
    blocks.forEach((block) => {
      const idx = parseInt(block.getAttribute('data-wpt-index'), 10);
      block.addEventListener('click', (e) => {
        if (e.target.closest('.btn-block-del')) return;
        this.selectWaypoint(idx);
      });

      // Drag and drop reordering
      block.addEventListener('dragstart', (e) => {
        this._dragSourceIndex = idx;
        e.dataTransfer.effectAllowed = 'move';
        block.classList.add('dragging');
      });

      block.addEventListener('dragend', () => {
        block.classList.remove('dragging');
      });

      block.addEventListener('dragover', (e) => {
        e.preventDefault();
        e.dataTransfer.dropEffect = 'move';
        block.classList.add('drag-over');
      });

      block.addEventListener('dragleave', () => {
        block.classList.remove('drag-over');
      });

      block.addEventListener('drop', (e) => {
        e.preventDefault();
        block.classList.remove('drag-over');
        if (this._dragSourceIndex >= 0 && this._dragSourceIndex !== idx) {
          this.reorderWaypoints(this._dragSourceIndex, idx);
        }
        this._dragSourceIndex = -1;
      });
    });

    const delBtns = this.container.querySelectorAll('[data-del-index]');
    delBtns.forEach((btn) => {
      btn.addEventListener('click', (e) => {
        e.stopPropagation();
        const delIdx = parseInt(btn.getAttribute('data-del-index'), 10);
        this.removeWaypoint(delIdx);
      });
    });

    // Playhead Drag Scrubbing
    const track = this.container.querySelector('#tl-track');
    const ruler = this.container.querySelector('#tl-ruler');
    const playhead = this.container.querySelector('#tl-playhead');

    if (track) {
      track.addEventListener('pointerdown', (e) => {
        if (e.target.closest('.timeline-block')) return;
        this.isScrubbing = true;
        this._handleScrub(e.clientX);
        document.body.style.userSelect = 'none';
      });
    }

    if (ruler) {
      ruler.addEventListener('pointerdown', (e) => {
        this.isScrubbing = true;
        this._handleScrub(e.clientX);
        document.body.style.userSelect = 'none';
      });
    }

    if (playhead) {
      playhead.addEventListener('pointerdown', (e) => {
        this.isScrubbing = true;
        e.stopPropagation();
        document.body.style.userSelect = 'none';
      });
    }
  }

  /**
   * Helper to scrub timeline to client coordinate
   * @private
   */
  _handleScrub(clientX) {
    if (!this.container) return;
    const track = this.container.querySelector('#tl-track');
    if (!track) return;
    const rect = track.getBoundingClientRect();
    const clickX = Math.max(0, Math.min(clientX - rect.left, rect.width));
    const targetTime = Math.max(0, Math.min(clickX / this.scale, this.totalDuration));
    this.setPlayheadTime(targetTime, true);
  }

  /**
   * Sets playhead time and optionally updates 3D Ghost Arm target pose
   * @param {number} timeMs
   * @param {boolean} updateGhost
   */
  setPlayheadTime(timeMs, updateGhost = false) {
    this.currentTime = Math.max(0, Math.min(timeMs, this.totalDuration));

    const playhead = this.container ? this.container.querySelector('#tl-playhead') : null;
    if (playhead) {
      playhead.style.left = `${Math.round(this.currentTime * this.scale)}px`;
    }

    const readout = this.container ? this.container.querySelector('#tl-time-readout') : null;
    if (readout) {
      readout.textContent = `${(this.currentTime / 1000).toFixed(2)}s / ${(this.totalDuration / 1000).toFixed(2)}s`;
    }

    if (updateGhost) {
      this._interpolatePoseAtTime(this.currentTime);
    }
  }

  /**
   * Calculates interpolated joint pose at specified timestamp and updates ghost arm
   * @private
   */
  _interpolatePoseAtTime(timeMs) {
    const seq = this.getSequence();
    const waypoints = seq.waypoints || [];
    if (waypoints.length === 0) return;

    if (waypoints.length === 1 || timeMs <= 0) {
      const wp = waypoints[0];
      this._applyPoseToGhost(wp);
      return;
    }

    // Find which waypoint segment timeMs falls into
    let accumulated = 0;
    for (let i = 0; i < waypoints.length; i++) {
      const wp = waypoints[i];
      const dur = Number(wp.dur || wp.duration || wp.duration_ms) || 1500;
      const nextAccumulated = accumulated + dur;

      if (timeMs <= nextAccumulated || i === waypoints.length - 1) {
        const prevWp = i > 0 ? waypoints[i - 1] : wp;
        const segmentProgress = dur > 0 ? (timeMs - accumulated) / dur : 1.0;
        const alpha = Math.max(0, Math.min(1, segmentProgress));

        const interp = {
          j1: (prevWp.j1 ?? 0) + alpha * ((wp.j1 ?? 0) - (prevWp.j1 ?? 0)),
          j2: (prevWp.j2 ?? 0) + alpha * ((wp.j2 ?? 0) - (prevWp.j2 ?? 0)),
          j3: (prevWp.j3 ?? 0) + alpha * ((wp.j3 ?? 0) - (prevWp.j3 ?? 0)),
          j4: (prevWp.j4 ?? 0) + alpha * ((wp.j4 ?? 0) - (prevWp.j4 ?? 0)),
          j5: (prevWp.j5 ?? 0) + alpha * ((wp.j5 ?? 0) - (prevWp.j5 ?? 0)),
          j6: (prevWp.j6 ?? 0) + alpha * ((wp.j6 ?? 0) - (prevWp.j6 ?? 0)),
          grip: (prevWp.grip ?? 90) + alpha * ((wp.grip ?? 90) - (prevWp.grip ?? 90))
        };

        for (const axis of ['x', 'y', 'z']) {
          const start = Number(prevWp[axis]);
          const end = Number(wp[axis]);
          if (prevWp[axis] != null && wp[axis] != null && Number.isFinite(start) && Number.isFinite(end)) {
            interp[axis] = start + alpha * (end - start);
          }
        }
        for (const angle of ['roll', 'pitch', 'yaw']) {
          const start = Number(prevWp[angle]);
          const end = Number(wp[angle]);
          if (prevWp[angle] != null && wp[angle] != null && Number.isFinite(start) && Number.isFinite(end)) {
            const shortest = ((end - start + 540) % 360) - 180;
            interp[angle] = start + alpha * shortest;
          }
        }
        const modeSource = alpha < 0.5 ? prevWp : wp;
        interp.orientationMode = modeSource.orientationMode;
        interp.w = modeSource.w;
        interp.frame = modeSource.frame;

        this._applyPoseToGhost(interp);
        break;
      }
      accumulated = nextAccumulated;
    }
  }

  _applyPoseToGhost(pose) {
    if (this.appState && typeof this.appState.updateGhostPose === 'function') {
      this.appState.updateGhostPose(pose);
    }
  }

  selectWaypoint(index) {
    this.selectedIndex = index;
    const blocks = this.container ? this.container.querySelectorAll('.timeline-block[data-wpt-index]') : [];
    blocks.forEach((b) => {
      const i = parseInt(b.getAttribute('data-wpt-index'), 10);
      b.classList.toggle('selected', i === index);
    });

    const seq = this.getSequence();
    const wp = seq.waypoints && seq.waypoints[index];
    if (wp) {
      this._applyPoseToGhost(wp);
    }
  }

  reorderWaypoints(fromIndex, toIndex) {
    if (this.trajectoryController && typeof this.trajectoryController.reorderWaypoints === 'function') {
      this.trajectoryController.reorderWaypoints(fromIndex, toIndex);
    } else {
      const seq = this.getSequence();
      const waypoints = [...(seq.waypoints || [])];
      if (fromIndex >= 0 && fromIndex < waypoints.length && toIndex >= 0 && toIndex < waypoints.length) {
        const [moved] = waypoints.splice(fromIndex, 1);
        waypoints.splice(toIndex, 0, moved);
        seq.waypoints = waypoints;
        if (this.appState && typeof this.appState.setSequence === 'function') {
          this.appState.setSequence(seq);
        }
      }
    }
    this.render();
  }

  removeWaypoint(index) {
    if (this.trajectoryController) {
      if (typeof this.trajectoryController.removeWaypoint === 'function') {
        this.trajectoryController.removeWaypoint(index);
      } else if (typeof this.trajectoryController.deleteWaypoint === 'function') {
        this.trajectoryController.deleteWaypoint(index);
      }
    } else {
      const seq = this.getSequence();
      const waypoints = [...(seq.waypoints || [])];
      if (index >= 0 && index < waypoints.length) {
        waypoints.splice(index, 1);
        seq.waypoints = waypoints;
        if (this.appState && typeof this.appState.setSequence === 'function') {
          this.appState.setSequence(seq);
        }
      }
    }
    this.render();
  }

  _updatePlaybackButtons(playbackState) {
    if (!this.container) return;
    const btnPlay = this.container.querySelector('#tl-btn-play');
    const btnPause = this.container.querySelector('#tl-btn-pause');
    if (!btnPlay || !btnPause) return;

    if (playbackState === 'PLAYING') {
      btnPlay.classList.add('active');
      btnPause.classList.remove('active');
    } else if (playbackState === 'PAUSED') {
      btnPlay.classList.remove('active');
      btnPause.classList.add('active');
    } else {
      btnPlay.classList.remove('active');
      btnPause.classList.remove('active');
    }
  }

  dispose() {
    if (typeof window !== 'undefined' && this._handleWindowPointerMove && this._handleWindowPointerUp) {
      window.removeEventListener('pointermove', this._handleWindowPointerMove);
      window.removeEventListener('pointerup', this._handleWindowPointerUp);
    }
    for (const unsub of this._unsubs) {
      if (typeof unsub === 'function') unsub();
    }
    this._unsubs = [];
    this.container = null;
  }
}

export default HorizontalTimelineController;
