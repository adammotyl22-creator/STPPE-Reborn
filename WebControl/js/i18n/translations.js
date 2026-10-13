/**
 * translations.js - Internationalization Dictionary
 * English (EN) and Polish (PL) translations for the Control Studio UI
 *
 * Polish translation guidelines:
 * - Technical terms kept in English where standard (e.g. "TCP", "IK", "FK", "Hz")
 * - Natural, professional Polish phrasing - not literal word-for-word translation
 * - Robot/automation domain vocabulary used correctly
 */

export const LANGUAGES = ['en', 'pl'];

/** @type {Record<string, Record<string, string>>} */
export const translations = {

  // ── HEADER ───────────────────────────────────────────────────────────────────
  'header.brand.title': { en: 'CONTROL STUDIO',  pl: 'CONTROL STUDIO' },
  'header.brand.sub':   { en: 'CONTROL',         pl: 'STEROWANIE' },
  'header.connect':     { en: 'Connect',    pl: 'Połącz' },
  'header.disconnect':  { en: 'Disconnect', pl: 'Rozłącz' },
  'header.transport.usb':       { en: 'USB',              pl: 'USB' },
  'header.transport.wifi':      { en: 'Wi-Fi',            pl: 'Wi-Fi' },
  'header.transport.ws_ip':     { en: 'Bridge Host / IP', pl: 'Host / IP Mostka' },
  'header.pwa.fullscreen':      { en: 'Teach Pendant',    pl: 'Pendant sterujący' },
  'header.mock.label':  { en: 'Mock',       pl: 'Symul.' },
  'header.auto.label':  { en: 'Auto',       pl: 'Auto' },

  'header.status.disconnected': { en: 'Disconnected',  pl: 'Rozłączono' },
  'header.status.connecting':   { en: 'Connecting...', pl: 'Łączenie...' },
  'header.status.connected':    { en: 'Connected',     pl: 'Połączono' },
  'header.status.streaming':    { en: 'Streaming',     pl: 'Transmisja' },
  'header.status.error':        { en: 'Error',         pl: 'Błąd' },
  'header.status.mock':         { en: 'Mock',          pl: 'Symulacja' },

  'header.home.label':  { en: 'HOME:',  pl: 'HOME:' },
  'header.home.all':    { en: 'ALL',    pl: 'WSZ.' },
  'header.home.j1':     { en: 'J1',     pl: 'J1' },
  'header.home.j2':     { en: 'J2',     pl: 'J2' },
  'header.home.j3':     { en: 'J3',     pl: 'J3' },
  'header.home.manual': { en: 'MANUAL', pl: 'RĘCZ.' },
  'header.home.gantry': { en: 'GANTRY', pl: 'GANTRY' },
  'header.home.status.idle':   { en: 'IDLE',   pl: 'BEZCZ.' },
  'header.home.status.homing': { en: 'HOMING', pl: 'ZEROWANIE' },
  'header.home.status.done':   { en: 'DONE',   pl: 'GOTOWE' },
  'header.home.status.error':  { en: 'ERROR',  pl: 'BŁĄD' },

  'header.speed.label':        { en: 'SPEED:',           pl: 'PRĘDKOŚĆ:' },
  'header.estop':               { en: 'EMERGENCY STOP',  pl: 'AWARYJNE ZATRZYMANIE' },
  'header.reenable':            { en: 'REENABLE',        pl: 'ODBLOKUJ' },
  'header.lang.toggle.title':  { en: 'Switch to Polish', pl: 'Przełącz na angielski' },

  // ── LEFT PANEL ────────────────────────────────────────────────────────────────
  'panel.left.title': { en: 'Motion control', pl: 'Sterowanie ruchem' },
  'panel.left.badge': { en: 'Local',           pl: 'Lokalnie' },

  // ── MODE TABS ─────────────────────────────────────────────────────────────────
  'tab.joints':     { en: 'Joint Control (FK)',  pl: 'Sterowanie osiami (FK)' },
  'tab.ik':         { en: 'Target (IK)', pl: 'Cel (IK)' },
  'tab.trajectory': { en: 'Sequencer',   pl: 'Sekwencer' },

  // ── IK ACCORDION ──────────────────────────────────────────────────────────────
  'accordion.ik.title':          { en: 'Inverse Kinematics (IK & TCP)', pl: 'Kinematyka odwrotna (IK i TCP)' },
  'jog.step.label':              { en: 'JOG STEP:',          pl: 'KROK JOG:' },
  'jog.step.joint.label':        { en: 'JOG STEP:',          pl: 'KROK JOG:' },
  'ik.coord.mode':               { en: 'COORDINATE MODE',    pl: 'TRYB WSPÓŁRZĘDNYCH' },
  'ik.target.x':                 { en: 'Target X (mm):',     pl: 'Cel X (mm):' },
  'ik.target.y':                 { en: 'Target Y (mm):',     pl: 'Cel Y (mm):' },
  'ik.target.z':                 { en: 'Target Z (mm):',     pl: 'Cel Z (mm):' },
  'ik.target.r':                 { en: 'Radius R (mm):',     pl: 'Promień R (mm):' },
  'ik.target.theta':             { en: 'Angle θ (deg):',     pl: 'Kąt θ (stopni):' },
  'ik.target.roll':              { en: 'Target Roll (°):',   pl: 'Cel Roll (°):' },
  'ik.target.pitch':             { en: 'Target Pitch (°):',  pl: 'Cel Pitch (°):' },
  'ik.target.yaw':               { en: 'Target Yaw (°):',    pl: 'Cel Yaw (°):' },
  'accordion.backlash.title':    { en: 'Motion Engine & Backlash', pl: 'Silnik Ruchu i Luzy' },
  'ik.wrist.mode.label':         { en: 'Wrist Mode (W0-W6):', pl: 'Tryb nadgarstka (W0-W6):' },
  'ik.wrist.w0': { en: 'W0: Level (Parallel to Floor)',    pl: 'W0: Poziomo (równoległy do podłoża)' },
  'ik.wrist.w1': { en: 'W1: Down_0 (Vertical Gripper)',    pl: 'W1: W dół_0 (chwytnik pionowy)' },
  'ik.wrist.w2': { en: 'W2: Down_90 (Vertical Roll 90°)', pl: 'W2: W dół_90 (obrót pionowy 90°)' },
  'ik.wrist.w3': { en: 'W3: Pitch_0 (Aligned Roll 0°)',   pl: 'W3: Pochylenie_0 (zbieżny 0°)' },
  'ik.wrist.w4': { en: 'W4: Pitch_90 (Aligned Roll 90°)', pl: 'W4: Pochylenie_90 (zbieżny 90°)' },
  'ik.wrist.w5': { en: 'W5: Pitch_180 (Aligned Roll 180°)', pl: 'W5: Pochylenie_180 (zbieżny 180°)' },
  'ik.wrist.w6': { en: 'W6: Manual (Direct Angles)',       pl: 'W6: Ręczny (kąty bezpośrednie)' },
  'ik.duration.label':           { en: 'Duration (ms):',    pl: 'Czas ruchu (ms):' },
  'ik.reachability.label':       { en: 'Kinematics Check:', pl: 'Sprawdzenie kinematyki:' },
  'ik.reachability.valid':       { en: 'VALID (Reachable)', pl: 'POPRAWNE (osiągalne)' },
  'ik.reachability.invalid':     { en: 'OUT OF RANGE',      pl: 'POZA ZASIĘGIEM' },
  'ik.reachability.singularity': { en: 'SINGULARITY',       pl: 'OSOBLIWOŚĆ' },
  'ik.btn.mov':                  { en: 'MOV (Spline)',      pl: 'RUCH (Splajn)' },
  'ik.btn.movl':                 { en: 'MOVL (Linear)',     pl: 'RUCH.L (Liniowy)' },
  'ik.btn.ghost.from.live':      { en: 'Ghost <- Live',     pl: 'Cel <- Aktualny' },
  'ik.btn.ghost.to.robot':       { en: 'Ghost -> Robot',    pl: 'Cel -> Robot' },

  // ── FK / JOINTS ACCORDION ─────────────────────────────────────────────────────
  'accordion.joints.title': { en: 'Joint Angle Controls (FK & Gripper)', pl: 'Sterowanie kątami osi (FK i chwytnik)' },
  'joint.j1.name':  { en: 'J1 Base Rotation',      pl: 'J1 Obrót podstawy' },
  'joint.j2.name':  { en: 'J2 Shoulder Elevation',  pl: 'J2 Uniesienie ramienia' },
  'joint.j3.name':  { en: 'J3 Elbow Flexion',       pl: 'J3 Zgięcie łokcia' },
  'joint.j4.name':  { en: 'J4 Forearm Roll',        pl: 'J4 Obrót przedramienia' },
  'joint.j5.name':  { en: 'J5 Wrist Pitch',         pl: 'J5 Pochylenie nadgarstka' },
  'joint.j6.name':  { en: 'J6 Tool Roll',           pl: 'J6 Obrót narzędzia' },
  'joint.grip.name':  { en: 'Gripper Servo',         pl: 'Serwomechanizm chwytnika' },
  'joint.grip.open':  { en: 'Open (180°)',           pl: 'Otwarty (180°)' },
  'joint.grip.half':  { en: 'Half (90°)',            pl: 'Połowa (90°)' },
  'joint.grip.close': { en: 'Close (0°)',            pl: 'Zamknięty (0°)' },
  'joint.live.control': { en: 'Live Control',        pl: 'Sterowanie na żywo' },
  'joint.btn.get.live': { en: 'Get Live',            pl: 'Pobierz aktualne' },
  'joint.btn.send':     { en: 'Send to Robot',       pl: 'Wyślij do robota' },

  // ── TRAJECTORY ACCORDION ──────────────────────────────────────────────────────
  'accordion.trajectory.title': { en: 'Trajectory & Sequencer', pl: 'Trajektoria i sekwencer' },

  // ── 3D HUD ────────────────────────────────────────────────────────────────────
  'hud.tcp.pose':     { en: 'TCP POSE',                           pl: 'POZYCJA TCP' },
  'hud.joints':       { en: 'JOINTS',                             pl: 'OSIE' },
  'hud.floor.ok':     { en: 'FLOOR CLEARANCE: OK (>20mm)',        pl: 'PRZEŚWIT OD PODŁOGI: OK (>20mm)' },
  'hud.floor.breach': { en: 'FLOOR CLEARANCE BREACH (Z <= 20mm)', pl: 'NARUSZENIE PRZEŚWITU (Z ≤ 20mm)' },

  // ── CAMERA PRESETS ────────────────────────────────────────────────────────────
  'camera.isometric': { en: 'Iso',   pl: 'Iso' },
  'camera.top':       { en: 'Top',   pl: 'Góra' },
  'camera.front':     { en: 'Front', pl: 'Przód' },
  'camera.side':      { en: 'Side',  pl: 'Bok' },
  'camera.reset':     { en: 'Reset', pl: 'Reset' },

  // ── LAYER TOGGLES & VISION POPOVER ───────────────────────────────────────────
  'viewport.vision.btn':   { en: 'Vision',          pl: 'Wizja' },
  'viewport.vision.title': { en: 'Vision & Layers', pl: 'Wizja i warstwy' },
  'tooltip.viewport.vision': { en: 'Manage Viewport Vision & Layer Toggles', pl: 'Zarządzaj widocznością warstw i HUD wizji' },
  'layer.ghost':     { en: 'Ghost',       pl: 'Cień' },
  'layer.path':      { en: 'Path',        pl: 'Ścieżka' },
  'layer.reach':     { en: 'Reach Envelope', pl: 'Zasięg robota' },
  'layer.floor':     { en: 'Floor Plane', pl: 'Płaszcz.' },
  'layer.exclusion': { en: 'Base Cyl',    pl: 'Cyl. bazy' },
  'layer.grid':      { en: 'Grid',        pl: 'Siatka' },

  // ── DUAL ARM LEGEND ───────────────────────────────────────────────────────────
  'legend.live':  { en: 'Live Arm (Titanium)',  pl: 'Ramię aktualne (tytanowe)' },
  'legend.ghost': { en: 'Ghost Target (Amber)', pl: 'Ramię docelowe (bursztynowe)' },

  // ── BOTTOM DOCK ───────────────────────────────────────────────────────────────
  'dock.tab.split':     { en: 'Split View',              pl: 'Widok podzielony' },
  'dock.tab.telemetry': { en: 'Telemetry Charts & HUD',  pl: 'Wykresy telemetrii i HUD' },
  'dock.tab.console':   { en: 'Command Terminal & Logs', pl: 'Terminal i logi' },
  'dock.tab.timeline':  { en: 'Timeline',                pl: 'Oś czasu' },
  'dock.collapse':      { en: 'Collapse Dock',           pl: 'Zwiń dok' },
  'dock.expand':        { en: 'Expand Dock',             pl: 'Rozwiń dok' },

  // ── PANEL RAIL ────────────────────────────────────────────────────────────────
  'panel.expand.title':  { en: 'Expand Control Panel (Alt+[)', pl: 'Rozwiń panel sterowania (Alt+[)' },
  'panel.controls.rail': { en: 'CONTROLS',                     pl: 'STEROWANIE' },

  // ── CONSOLE ───────────────────────────────────────────────────────────────────
  'console.ready': {
    en: 'Teensy 4.1 Robot Arm CAD Workspace ready. Connect USB or enable Mock Mode.',
    pl: 'Środowisko CAD ramienia robota Teensy 4.1 gotowe. Podłącz USB lub włącz tryb symulacji.'
  },

  // ── TOOLTIPS ─────────────────────────────────────────────────────────────────
  'tooltip.connect':       { en: 'Connect to Teensy 4.1 USB Serial Port',            pl: 'Połącz z portem szeregowym USB Teensy 4.1' },
  'tooltip.mock':          { en: 'Simulate Teensy 4.1 Virtual Hardware without USB', pl: 'Symuluj wirtualny sprzęt Teensy 4.1 bez USB' },
  'tooltip.auto':          { en: 'Auto Reconnect if USB port disconnects',            pl: 'Automatyczne ponowne połączenie po odłączeniu USB' },
  'tooltip.home.all':      { en: 'Home all axes (J1, J2, J3)',    pl: 'Zeruj wszystkie osie (J1, J2, J3)' },
  'tooltip.home.j1':       { en: 'Home Joint 1 (Base)',           pl: 'Zeruj oś 1 (podstawa)' },
  'tooltip.home.j2':       { en: 'Home Joint 2 (Shoulder)',       pl: 'Zeruj oś 2 (ramię)' },
  'tooltip.home.j3':       { en: 'Home Joint 3 (Elbow)',          pl: 'Zeruj oś 3 (łokieć)' },
  'tooltip.home.manual':   { en: 'Set current pose as Home Zero', pl: 'Ustaw bieżącą pozycję jako zero referencyjne' },
  'tooltip.estop':         { en: 'Emergency Stop [Spacebar / Esc]', pl: 'Awaryjne zatrzymanie [Spacja / Esc]' },
  'tooltip.reenable':      { en: 'Re-enable stepper motors after Emergency Stop', pl: 'Odblokuj silniki krokowe po awaryjnym zatrzymaniu' },
  'tooltip.ghost.visible': { en: 'Toggle Ghost Arm',                              pl: 'Przełącz widoczność ramienia cienia' },
  'tooltip.path.visible':  { en: 'Toggle 3D Trajectory Toolpath Spline & Waypoints', pl: 'Przełącz widoczność ścieżki trajektorii 3D i punktów' },
  'tooltip.floor.alert':   { en: 'Toggle Floor Clearance Alert Plane (Z=20mm)',   pl: 'Przełącz płaszczyznę ostrzeżenia (Z=20mm)' },
  'tooltip.exclusion':     { en: 'Toggle Base Exclusion Warning Cylinder (R=80mm)', pl: 'Przełącz cylinder strefy wykluczenia (R=80mm)' },
  'tooltip.grid':          { en: 'Toggle Floor Grid',                              pl: 'Przełącz siatkę podłogi' },
  'tooltip.mov.spline':    { en: 'Execute Spline Move via CRC-8 Maxim Packet',     pl: 'Wykonaj ruch po splajnie (pakiet CRC-8 Maxim)' },
  'tooltip.mov.linear':    { en: 'Execute Cartesian Linear Interpolation',          pl: 'Wykonaj interpolację liniową kartezjańską' },
  'tooltip.ghost.from.live': { en: 'Copy Live Robot Pose to Ghost Target', pl: 'Kopiuj bieżącą pozycję robota jako cel' },
  'tooltip.ghost.to.robot':  { en: 'Send Ghost Target Pose to Robot',     pl: 'Wyślij pozycję docelową do robota' },
  'tooltip.live.control':    { en: 'Live Control: Dragging joint sliders moves real robot in real-time', pl: 'Sterowanie na żywo: suwaki poruszają robotem w czasie rzeczywistym' },
  'tooltip.get.live':        { en: 'Copy Live Robot Pose to Sliders',     pl: 'Kopiuj aktualną pozycję robota do suwaków' },
  'tooltip.send.joints':     { en: 'Send Joint Sliders Target to Robot',  pl: 'Wyślij wartości suwaków do robota' },
  'tooltip.dock.toggle':     { en: 'Toggle Dock Collapse / Expand (\\)', pl: 'Zwiń / rozwiń dok (\\)' },
  'tooltip.camera.isometric': { en: 'Isometric View [1]', pl: 'Widok izometryczny [1]' },
  'tooltip.camera.top':       { en: 'Top View XY [2]',    pl: 'Widok z góry XY [2]' },
  'tooltip.camera.front':     { en: 'Front View XZ [3]',  pl: 'Widok z przodu XZ [3]' },
  'tooltip.camera.side':      { en: 'Side View YZ [4]',   pl: 'Widok z boku YZ [4]' },
  'tooltip.camera.reset':     { en: 'Reset Camera [5]',   pl: 'Resetuj kamerę [5]' },
  'tooltip.ik.cylindrical':   { en: 'Switch between Cartesian (X/Y/Z) and Cylindrical (R/th/Z) modes', pl: 'Przełącz między trybem kartezjańskim (X/Y/Z) a cylindrycznym (R/θ/Z)' },

  // ── JSON & G-CODE TRANSFER ───────────────────────────────────────────────────
  'sequence.btn.import':      { en: 'Import Path',                     pl: 'Importuj ścieżkę' },
  'sequence.btn.export':      { en: 'Export Path',                     pl: 'Eksportuj ścieżkę' },
  'tooltip.sequence.import':  { en: 'Import Robot Sequence or CNC Toolpath (JSON / G-Code)', pl: 'Importuj sekwencję robota lub ścieżkę CNC (JSON / G-Code)' },
  'tooltip.sequence.export':  { en: 'Export Robot Sequence or CNC Toolpath (JSON / G-Code)', pl: 'Eksportuj sekwencję robota lub ścieżkę CNC (JSON / G-Code)' },
  'format.modal.import.title': { en: 'Import Trajectory / Toolpath',   pl: 'Importuj trajektorię / ścieżkę' },
  'format.modal.import.desc': { en: 'Select the file format you would like to import:', pl: 'Wybierz format pliku, który chcesz zaimportować:' },
  'format.modal.export.title': { en: 'Export Trajectory / Toolpath',   pl: 'Eksportuj trajektorię / ścieżkę' },
  'format.modal.export.desc': { en: 'Choose export format for current sequence:', pl: 'Wybierz format eksportu bieżącej sekwencji:' },
  'format.card.json.title':   { en: 'JSON Robotic Sequence (.json)',   pl: 'Sekwencja robota JSON (.json)' },
  'format.card.json.desc':    { en: 'Native robot sequence containing joint angles, durations, and gripper states', pl: 'Natywna sekwencja robota z kątami osi, czasami trwania i stanem chwytaka' },
  'format.card.gcode.title':  { en: 'CNC G-Code Toolpath (.gcode, .nc, .tap)', pl: 'Ścieżka CNC G-Code (.gcode, .nc, .tap)' },
  'format.card.gcode.desc':   { en: 'Cartesian toolpath from CNC/CAD software with auto-centering & scaling', pl: 'Ścieżka kartezjańska z oprogramowania CNC/CAD z auto-centrowaniem i skalowaniem' },
  'format.btn.browse':        { en: 'Browse Any File (.json, .gcode)...', pl: 'Przeglądaj dowolny plik (.json, .gcode)...' },
  'format.modal.download':    { en: 'Download File',                   pl: 'Pobierz plik' },
  'json.btn.import':          { en: 'Import JSON',                     pl: 'Importuj JSON' },
  'json.btn.export':          { en: 'Export JSON',                     pl: 'Eksportuj JSON' },
  'tooltip.json.import':      { en: 'Import Sequence from JSON File', pl: 'Importuj sekwencję z pliku JSON' },
  'tooltip.json.export':      { en: 'Export Sequence to JSON File',   pl: 'Eksportuj sekwencję do pliku JSON' },
  'gcode.btn.import':         { en: 'Import G-Code',                   pl: 'Importuj G-Code' },
  'gcode.btn.export':         { en: 'Export G-Code',                   pl: 'Eksportuj G-Code' },
  'tooltip.gcode.import':     { en: 'Import CNC/3D Toolpath from G-Code (.gcode / .nc / .tap)', pl: 'Importuj ścieżkę CNC/3D z G-Code (.gcode / .nc / .tap)' },
  'tooltip.gcode.export':     { en: 'Export Sequence to Standard CNC G-Code (.gcode)', pl: 'Eksportuj sekwencję do standardowego formatu CNC G-Code (.gcode)' },
  'gcode.modal.title':        { en: 'G-Code Toolpath Engine',          pl: 'Silnik ścieżek CNC G-Code' },
  'gcode.modal.label':        { en: 'Paste G-Code or Select File:',    pl: 'Wklej G-Code lub wybierz plik:' },
  'gcode.modal.browse':       { en: 'Browse File (.gcode)',            pl: 'Przeglądaj plik (.gcode)' },
  'gcode.modal.copy':         { en: 'Copy to Clipboard',               pl: 'Kopiuj do schowka' },
  'gcode.modal.apply':        { en: 'Apply Toolpath',                  pl: 'Zastosuj ścieżkę' },
  'gcode.opt.centerx':        { en: 'Center X (mm):',                  pl: 'Środek X (mm):' },
  'gcode.opt.centery':        { en: 'Center Y (mm):',                  pl: 'Środek Y (mm):' },
  'gcode.opt.centerz':        { en: 'Base Z (mm):',                    pl: 'Baza Z (mm):' },
  'gcode.opt.scale':          { en: 'Scale Factor:',                   pl: 'Współczynnik skali:' },
  'gcode.opt.feedrate':       { en: 'Default Feed (mm/min):',          pl: 'Domyślny posuw (mm/min):' },
  'gcode.opt.wrist':          { en: 'Wrist Mode:',                     pl: 'Tryb nadgarstka:' },

  // ── J5 EYE-IN-HAND CAMERA & VISION ──────────────────────────────────────────
  'camera.hud.title':         { en: 'J5 Eye-in-Hand Vision',            pl: 'Wizja J5 Eye-in-Hand' },
  'camera.tab.title':         { en: 'Vision & Camera',                 pl: 'Wizja i Kamera' },
  'camera.btn.start':         { en: 'Start Stream',                    pl: 'Uruchom kamerę' },
  'camera.btn.stop':          { en: 'Stop Stream',                     pl: 'Zatrzymaj kamerę' },
  'camera.btn.snap':          { en: 'Snap Photo',                      pl: 'Zrób zdjęcie' },
  'camera.btn.rotate':        { en: 'Rotate 90°',                      pl: 'Obróć 90°' },
  'camera.btn.fliph':         { en: 'Flip H',                          pl: 'Odbij poziom.' },
  'camera.btn.flipv':         { en: 'Flip V',                          pl: 'Odbij pion.' },
  'camera.reticle.label':     { en: 'TCP Crosshair Reticle',           pl: 'Siatka celownicza TCP' },
  'camera.gripper.label':     { en: 'Gripper Jaws Guides',             pl: 'Prowadnice szczęk chwytaka' },
  'camera.rings.label':       { en: 'Millimeter Scale Rings',          pl: 'Pierścienie skali (mm)' },
  'camera.colortrack.label':  { en: 'Color Tracking Lock',             pl: 'Śledzenie koloru' },
  'camera.layer.pip':         { en: 'Camera HUD',                      pl: 'Kamera HUD' },
  'tooltip.camera.pip':       { en: 'Toggle Floating J5 Eye-in-Hand Camera HUD', pl: 'Przełącz pływające okno kamery J5' },

  // ── MILESTONE 1: MOTION PANEL OVERHAUL (R1–R4) ──────────────────────────────
  // Frame Selector & IK Keypad
  'ik.frame.label':             { en: 'FRAME:',                   pl: 'UKŁAD:' },
  'ik.frame.world':             { en: 'World',                    pl: 'Świat' },
  'ik.frame.base':              { en: 'Base',                     pl: 'Baza' },
  'ik.frame.tool':              { en: 'Tool',                     pl: 'Narzędzie' },
  'ik.keypad.title':            { en: 'DIRECT STEP JOG PAD',      pl: 'KLAWIATURA KROKOWA JOG' },

  // Quick Pose Presets
  'preset.quick.title':         { en: 'QUICK POSE PRESETS',       pl: 'SZYBKIE POZYCJE BAZOWE' },
  'preset.quick.hint':          { en: '1-CLICK TARGET',           pl: '1-KLIKNIĘCIE CEL' },
  'preset.pose.home':           { en: 'Home Pose',                pl: 'Pozycja zerowa' },
  'preset.pose.zero':           { en: 'Zero All',                 pl: 'Wyzeruj osie' },
  'preset.pose.ready':          { en: 'Ready',                    pl: 'Gotowość do pracy' },
  'preset.pose.stow':           { en: 'Stow',                     pl: 'Pozycja złożona' },
  'joint.preset.home':          { en: 'Home Pose',                pl: 'Pozycja bazowa' },
  'joint.preset.zero':          { en: 'Zero All',                 pl: 'Wszystkie zero' },
  'joint.preset.ready':         { en: 'Ready Pose',               pl: 'Pozycja gotowości' },
  'joint.preset.stow':          { en: 'Stow Pose',                pl: 'Pozycja transportowa' },

  // Two-State Safety UI & Direct Drive
  'safety.execution.title':     { en: 'Safety Mode',              pl: 'Tryb bezpieczeństwa' },
  'safety.mode.preview':        { en: 'PREVIEW (GHOST)',          pl: 'PODGLĄD (CIEŃ)' },
  'safety.mode.live':           { en: 'LIVE FOLLOW',              pl: 'ŚLEDZENIE NA ŻYWO' },
  'safety.direct_drive.toggle': { en: 'Direct Drive',             pl: 'Napęd bezpośredni' },
  'safety.direct_drive.active': { en: 'DIRECT DRIVE ACTIVE',      pl: 'BEZPOŚREDNI NAPĘD AKTYWNY' },
  'safety.direct_drive.banner': { en: 'DIRECT DRIVE ACTIVE',      pl: 'BEZPOŚREDNI NAPĘD AKTYWNY' },
  'safety.direct_drive.sub':    { en: '— PHYSICAL ARM FOLLOW',    pl: '— FIZYCZNE ŚLEDZENIE RAMIENIA' },
  'safety.btn.ptp':             { en: 'Send to Target (PTP)',     pl: 'Wyślij do celu (PTP)' },
  'safety.btn.plan':            { en: 'Plan Trajectory',          pl: 'Planuj trajektorię' },
  'action.send_ptp':            { en: 'Send to Target (PTP)',     pl: 'Wyślij do celu (PTP)' },
  'action.plan_traj':           { en: 'Plan Trajectory',          pl: 'Planuj trajektorię' },

  // Direct Drive Confirmation Modal
  'safety.modal.title':         { en: 'ARM DIRECT DRIVE (LIVE PHYSICAL FOLLOW)?', pl: 'UZBROIĆ BEZPOŚREDNI NAPĘD (RUCH FIZYCZNY)?' },
  'safety.direct_drive.modal_title': { en: 'Arm Direct Drive Mode?', pl: 'Uzbroić tryb bezpośredniego napędu?' },
  'safety.modal.warn':          { en: 'CAUTION: Physical robot arm will immediately mirror target inputs in real time!', pl: 'UWAGA: Fizyczne ramię robota natychmiast odzwierciedli ruchy w czasie rzeczywistym!' },
  'safety.direct_drive.modal_body': { en: 'Live jogging immediately moves physical hardware servos.', pl: 'Ruchy natychmiast przestawiają serwomechanizmy robota.' },
  'safety.modal.check1':        { en: 'Robot workspace is clear of obstacles and personnel', pl: 'Przestrzeń robocza robota jest wolna od przeszkód i ludzi' },
  'safety.modal.check2':        { en: 'Emergency Stop (ESTOP) button is readily accessible', pl: 'Przycisk wyłącznika awaryjnego (ESTOP) jest pod ręką' },
  'safety.modal.check3':        { en: 'End-effector payload is securely mounted', pl: 'Chwytak i detal są poprawnie zamocowane' },
  'safety.modal.cancel':        { en: 'Cancel (Stay Safe)',       pl: 'Anuluj (Pozostań w podglądzie)' },
  'safety.direct_drive.cancel': { en: 'Cancel (Safe Preview)',    pl: 'Anuluj (Bezpieczny podgląd)' },
  'safety.modal.confirm':       { en: 'Arm Direct Drive',         pl: 'Uzbrój napęd bezpośredni' },
  'safety.direct_drive.confirm':{ en: 'Arm Live Drive',           pl: 'Uzbrój napęd' },

  // Persistent Gripper Card
  'gripper.card.title':         { en: 'END-EFFECTOR (GRIPPER)',   pl: 'EFEKTOR KOŃCOWY (CHWYTAK)' },
  'gripper.span.label':         { en: 'SPAN:',                    pl: 'ROZWARCIE:' },
  'gripper.telem.state':        { en: 'STATE:',                   pl: 'STAN:' },
  'gripper.telem.effort':       { en: 'EFFORT:',                  pl: 'OBCIĄŻENIE:' },
  'gripper.effort.label':       { en: 'Effort / Load:',           pl: 'Obciążenie:' },
  'gripper.telem.object':       { en: 'OBJECT:',                  pl: 'OBIEKT:' },
  'gripper.status.ready':       { en: 'READY',                    pl: 'GOTOWY' },
  'gripper.status.clear':       { en: 'CLEAR',                    pl: 'BRAK' },
  'gripper.status.holding':     { en: 'HOLDING',                  pl: 'UCHWYCONY' },
  'gripper.object.detected':    { en: 'OBJECT GRIPPED',           pl: 'OBIEKT CHWYCONY' },
  'gripper.object.none':        { en: 'NO OBJECT',                pl: 'BRAK OBIEKTU' },

  // Tooltips
  'tooltip.btn.ptp':            { en: 'Execute Point-to-Point trajectory to Ghost Target', pl: 'Wykonaj ruch punkt-do-punktu do pozycji celu' },
  'tooltip.btn.plan':           { en: 'Plan collision-free trajectory to target', pl: 'Zaplanuj bezkolizyjną trajektorię do celu' },
  'tooltip.direct_drive.toggle':{ en: 'Toggle Physical Direct Drive Arm Follow (Confirmation Required)', pl: 'Przełącz bezpośrednie sterowanie ramieniem fizycznym (wymaga potwierdzenia)' },
  'tooltip.preset.home':        { en: 'Home: J1..J6 = [0, 25, 140, 0, 0, 0]', pl: 'Pozycja zerowa: J1..J6 = [0, 25, 140, 0, 0, 0]' },
  'tooltip.preset.zero':        { en: 'Zero All: All joint encoders 0.0°', pl: 'Wyzeruj wszystkie osie na 0.0°' },
  'tooltip.preset.ready':       { en: 'Ready: Elevated operational work pose', pl: 'Gotowość: podniesiona pozycja robocza' },
  'tooltip.preset.stow':        { en: 'Stow: Folded safe parking pose', pl: 'Złożenie: bezpieczna pozycja parkowania' },
};

export default translations;
