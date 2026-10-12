/**
 * sw.js - Service Worker for STPE Robot Arm Studio PWA
 * 
 * Provides offline caching for standalone robot operation
 * when connected to the local ESP32 Access Point (STPE_Robot).
 */

const CACHE_NAME = 'stpe-robot-arm-v2';

const STATIC_ASSETS = [
  './',
  './index.html',
  './manifest.json',
  './sw.js',
  './css/main.css',
  './css/theme/cyberpunk-dark.css',
  './css/components/header.css',
  './css/components/left-rail.css',
  './css/components/viewport.css',
  './css/components/motion-panel.css',
  './css/components/trajectory-studio.css',
  './css/components/telemetry-charts.css',
  './css/components/console.css',
  './css/components/camera-hud.css',
  './css/components/timeline.css',
  './js/app.js',
  './js/comms/WebSerialManager.js',
  './js/comms/TelemetryParser.js',
  './js/comms/PacketBuilder.js',
  './js/comms/MockSerialPort.js',
  './js/comms/GamepadController.js',
  './js/kinematics/RobotKinematics.js',
  './js/kinematics/WorkspaceValidator.js',
  './js/kinematics/GCodeInterpreter.js',
  './js/state/AppState.js',
  './js/state/PresetManager.js',
  './js/ui/HeaderController.js',
  './js/ui/MotionController.js',
  './js/ui/TrajectoryController.js',
  './js/ui/TelemetryController.js',
  './js/ui/ConsoleController.js',
  './js/ui/HorizontalTimelineController.js',
  './js/camera/CameraController.js',
  './js/i18n/I18nManager.js',
  './js/i18n/translations.js',
  './js/utils/CRC8.js',
  './js/visualizer/RobotVisualizer3D.js',
  './js/visualizer/ThreeScene.js',
  './js/visualizer/RobotModel.js',
  './js/visualizer/WorkspaceVisualizer.js',
  './js/visualizer/TrajectoryVisualizer3D.js',
  './js/visualizer/TcpDragController.js',
  './vendor/three/three.module.js',
  './vendor/three/OrbitControls.js',
  './vendor/three/STLLoader.js',
  './meshes/base_link.stl',
  './meshes/link_1.stl',
  './meshes/link_2.stl',
  './meshes/link_3.stl',
  './meshes/link_4.stl',
  './meshes/link_5.stl',
  './meshes/link_6.stl',
  './assets/icons/icon-192.png',
  './assets/icons/icon-512.png'
];

// Install: pre-cache application shell
self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(CACHE_NAME).then((cache) => {
      return cache.addAll(STATIC_ASSETS);
    }).then(() => self.skipWaiting())
  );
});

// Activate: clean up outdated caches
self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys().then((cacheNames) => {
      return Promise.all(
        cacheNames.map((name) => {
          if (name !== CACHE_NAME) {
            return caches.delete(name);
          }
        })
      );
    }).then(() => self.clients.claim())
  );
});

// Fetch: Cache-first for local static resources, network fallback
self.addEventListener('fetch', (event) => {
  const request = event.request;
  
  // Skip non-GET requests and WebSocket connections
  if (request.method !== 'GET' || request.url.startsWith('ws://') || request.url.startsWith('wss://')) {
    return;
  }

  event.respondWith(
    caches.match(request).then((cachedResponse) => {
      if (cachedResponse) {
        return cachedResponse;
      }

      return fetch(request).then((networkResponse) => {
        if (!networkResponse || networkResponse.status !== 200 || networkResponse.type !== 'basic') {
          return networkResponse;
        }
        const responseToCache = networkResponse.clone();
        caches.open(CACHE_NAME).then((cache) => {
          cache.put(request, responseToCache);
        });
        return networkResponse;
      }).catch(() => {
        // Offline fallback for navigation
        if (request.mode === 'navigate') {
          return caches.match('./index.html');
        }
      });
    })
  );
});