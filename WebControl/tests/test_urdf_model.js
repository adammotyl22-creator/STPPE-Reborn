/**
 * test_urdf_model.js - Verification Suite for URDF STL Robot Model
 */

import * as THREE from '../vendor/three/three.module.js';
import { STLLoader } from '../vendor/three/STLLoader.js';
import { RobotModel } from '../js/visualizer/RobotModel.js';
import * as fs from 'fs';
import * as path from 'path';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const rootDir = path.resolve(__dirname, '..');

console.log('=== [SUITE] URDF STL Robot Model Verification ===');

function assert(cond, msg) {
  if (!cond) {
    console.error('FAIL:', msg);
    process.exit(1);
  }
  console.log('  [PASS]', msg);
}

// 1. Verify STL files exist
const meshFiles = [
  'base_link.stl',
  'link_1.stl',
  'link_2.stl',
  'link_3.stl',
  'link_4.stl',
  'link_5.stl',
  'link_6.stl'
];

for (const file of meshFiles) {
  const filePath = path.resolve(rootDir, 'meshes', file);
  assert(fs.existsSync(filePath), `Mesh file exists: meshes/${file}`);
}

// 2. Parse STL files using STLLoader
const loader = new STLLoader();
const geometries = {};

for (const file of meshFiles) {
  const filePath = path.resolve(rootDir, 'meshes', file);
  const buf = fs.readFileSync(filePath);
  const arrayBuf = buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength);
  const geom = loader.parse(arrayBuf);
  const linkName = file.replace('.stl', '');
  geometries[linkName] = geom;
  assert(geom.attributes.position && geom.attributes.position.count > 0, `Parsed ${file}: ${geom.attributes.position.count} vertices`);
}

// 3. Test RobotModel Assembly Construction
const model = new RobotModel({ showGhost: true, shadows: true });
assert(model.root instanceof THREE.Group, 'RobotModel has root Three.js Group');
assert(model.isGhostVisible === true, 'RobotModel isGhostVisible returns true by default');
assert(model.isLiveVisible === true, 'RobotModel isLiveVisible returns true by default');

model._cachedGeometries = geometries;
model.liveArm = model._buildArmAssembly(geometries, false);
model.root.add(model.liveArm.root);

model.ghostArm = model._buildArmAssembly(geometries, true);
model.root.add(model.ghostArm.root);
model._meshesLoaded = true;

assert(model.liveArm.root !== null, 'liveArm root constructed');
assert(model.ghostArm.root !== null, 'ghostArm root constructed');
assert(model.liveArm.j1 instanceof THREE.Group, 'liveArm has j1 group');
assert(model.liveArm.j2 instanceof THREE.Group, 'liveArm has j2 group');
assert(model.liveArm.j3 instanceof THREE.Group, 'liveArm has j3 group');
assert(model.liveArm.j4 instanceof THREE.Group, 'liveArm has j4 group');
assert(model.liveArm.j5 instanceof THREE.Group, 'liveArm has j5 group');
assert(model.liveArm.j6 instanceof THREE.Group, 'liveArm has j6 group');
assert(model.liveArm.tcp instanceof THREE.Group, 'liveArm has tcp group');

// 4. Test Zero Pose Alignment & Floor Clearance
model.updateLivePose(0, 0, 0, 0, 0, 0);
model.updateGhostPose(0, 0, 0, 0, 0, 0);
model.root.updateMatrixWorld(true);

const liveBox = new THREE.Box3().setFromObject(model.liveArm.root);
assert(liveBox.min.y >= -1.0, `Live arm base rests on floor: min Y = ${liveBox.min.y.toFixed(2)}mm`);
assert(liveBox.max.y <= 400.0, `Live arm height is within reasonable envelope: max Y = ${liveBox.max.y.toFixed(2)}mm`);

const tcpPos = new THREE.Vector3();
model.getLiveTcpWorldPosition(tcpPos);
assert(tcpPos.y > 50.0, `Live TCP is above floor plane: Y = ${tcpPos.y.toFixed(2)}mm`);

const elbowPos = new THREE.Vector3();
model.getLiveElbowWorldPosition(elbowPos);
assert(elbowPos.y > 200.0, `Live Elbow is elevated: Y = ${elbowPos.y.toFixed(2)}mm`);

// 5. Test Joint Motion Displacements
const tcpBefore = tcpPos.clone();
model.updateLivePose(45, -30, 20, 10, -15, 30);
model.root.updateMatrixWorld(true);
model.getLiveTcpWorldPosition(tcpPos);
const motionDistance = tcpPos.distanceTo(tcpBefore);
assert(motionDistance > 50.0, `Joint motion articulates arm: TCP moved ${motionDistance.toFixed(2)}mm`);

// 6. Test Ghost & Live Visibility
model.setGhostVisible(false);
assert(model.ghostArm.root.visible === false, 'setGhostVisible(false) hides ghost arm');
assert(model.isGhostVisible === false, 'isGhostVisible reflects hidden status');

model.setGhostVisible(true);
assert(model.ghostArm.root.visible === true, 'setGhostVisible(true) shows ghost arm');
assert(model.isGhostVisible === true, 'isGhostVisible reflects shown status');

model.setLiveVisible(false);
assert(model.liveArm.root.visible === false, 'setLiveVisible(false) hides live arm');
assert(model.isLiveVisible === false, 'isLiveVisible reflects hidden status');
assert(model.ghostArm.baseLink.visible === true, 'ghostArm baseLink remains visible when live arm is hidden');

model.setLiveVisible(true);
assert(model.liveArm.root.visible === true, 'setLiveVisible(true) shows live arm');
assert(model.isLiveVisible === true, 'isLiveVisible reflects shown status');
assert(model.ghostArm.baseLink.visible === true, 'ghostArm baseLink is visible when live arm is shown');

// 7. Test Dispose
model.dispose();
console.log('  [PASS] dispose() completed without errors');

console.log('\nALL URDF MODEL VERIFICATION TESTS PASSED SUCCESSFULLY!');
