import { el } from "./common.js";
import * as THREE from "three";

let _scene = null;
let _camera = null;
let _renderer = null;
let _controls = null;
let _voxelGroup = null;
let _pointCloud = null;
let _gridHelper = null;
let _droneBody = null;
let _enabled = false;
let _fpvMode = true;
let _animFrame = 0;
let _lastHeading = 0;

// --- helpers ---------------------------------------------------------

function worldToThree(x, y, z) {
  return [x, -z, y];
}

function heatColor(t) {
  // t: 0..1, returns [r,g,b]
  if (t < 0.33) {
    const s = t / 0.33;
    return [0.47 + (0.85 - 0.47) * s, 0.67 + (0.66 - 0.67) * s, 1.0 + (0.25 - 1.0) * s];
  } else if (t < 0.66) {
    const s = (t - 0.33) / 0.33;
    return [0.85 + (0.88 - 0.85) * s, 0.66 + (0.36 - 0.66) * s, 0.25 + (0.36 - 0.25) * s];
  } else {
    const s = (t - 0.66) / 0.34;
    return [0.88 + (1.0 - 0.88) * s, 0.36 + (0.15 - 0.36) * s, 0.36 + (0.10 - 0.36) * s];
  }
}

function depthColor(dist, maxDist) {
  const t = Math.max(0, Math.min(1, dist / maxDist));
  // near = green-cyan, far = red
  const r = t < 0.5 ? 0.26 + t * 0.74 * 2 : 1.0;
  const g = t < 0.5 ? 1.0 : 1.0 - (t - 0.5) * 1.6;
  const b = t < 0.5 ? 0.26 + t * 0.74 * 2 : 0.1;
  return [Math.max(0, r), Math.max(0, g), Math.max(0, b)];
}

// --- init ------------------------------------------------------------

let _OrbitControls = null;

async function _loadControls() {
  if (_OrbitControls) return _OrbitControls;
  const mod = await import("three/addons/controls/OrbitControls.js");
  _OrbitControls = mod.OrbitControls;
  return _OrbitControls;
}

export function init3dMap() {
  if (_scene) return;
  const canvas = el("map3dCanvas");
  if (!canvas) return;

  _scene = new THREE.Scene();
  _scene.background = new THREE.Color(0x0d141a);
  _scene.fog = new THREE.Fog(0x0d141a, 15, 80);

  _camera = new THREE.PerspectiveCamera(70, 1, 0.3, 120);
  _camera.position.set(0, 5, 0);
  _camera.lookAt(10, 5, 0);

  _renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  _renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

  // Soft ambient + directional for voxel depth
  _scene.add(new THREE.AmbientLight(0x334455, 1.8));
  const dl = new THREE.DirectionalLight(0xffffff, 1.5);
  dl.position.set(20, 30, 10);
  _scene.add(dl);

  // Ground reference grid
  _gridHelper = new THREE.GridHelper(50, 25, 0x2a4048, 0x162028);
  _gridHelper.position.y = 0.02;
  _scene.add(_gridHelper);

  // Confirmed voxels group
  _voxelGroup = new THREE.Group();
  _scene.add(_voxelGroup);

  // Raw LiDAR point cloud
  _pointCloud = new THREE.Points(
    new THREE.BufferGeometry(),
    new THREE.PointsMaterial({ size: 0.22, vertexColors: true, transparent: true, opacity: 0.85, depthWrite: true })
  );
  _scene.add(_pointCloud);

  // Drone body indicator (small cross)
  const dg = new THREE.SphereGeometry(0.3, 8, 6);
  const dm = new THREE.MeshBasicMaterial({ color: 0x42c486 });
  _droneBody = new THREE.Mesh(dg, dm);
  _scene.add(_droneBody);

  _enabled = true;
  resize3dMap();
  _animFrame = requestAnimationFrame(renderLoop);

  _loadControls().then((OrbitControls) => {
    if (!_camera || !_renderer) return;
    _controls = new OrbitControls(_camera, _renderer.domElement);
    _controls.target.set(0, 4, 0);
    _controls.enableDamping = true;
    _controls.dampingFactor = 0.12;
    _controls.minDistance = 5;
    _controls.maxDistance = 80;
    _controls.maxPolarAngle = Math.PI * 0.48;
    _controls.enabled = !_fpvMode;
    _controls.update();
  }).catch((e) => { console.warn("3d: OrbitControls load failed", e); });
}

// --- public API -------------------------------------------------------

export function set3dEnabled(on) {
  _enabled = !!on;
  const container = el("map3dContainer");
  const btn = el("toggle3dBtn");
  if (container) container.style.display = _enabled ? "block" : "none";
  if (btn) btn.textContent = _enabled ? "3D ON" : "3D OFF";
  if (_enabled) {
    init3dMap();
    if (_animFrame) cancelAnimationFrame(_animFrame);
    _animFrame = requestAnimationFrame(renderLoop);
  }
}

export function toggleViewMode() {
  _fpvMode = !_fpvMode;
  if (_controls) _controls.enabled = !_fpvMode;
  const btn = el("toggle3dViewBtn");
  if (btn) btn.textContent = _fpvMode ? "FPV" : "ORBIT";
}

export function resize3dMap() {
  if (!_renderer) return;
  const container = el("map3dContainer");
  if (!container) return;
  const w = container.clientWidth;
  const h = container.clientHeight;
  if (w <= 0 || h <= 0) return;
  _renderer.setSize(w, h, false);
  if (_camera) {
    _camera.aspect = w / Math.max(1, h);
    _camera.updateProjectionMatrix();
  }
}

export function update3dData(mapData) {
  if (!_scene) return;
  const uav = (mapData && mapData.uav) || null;
  const voxels = (mapData && mapData.voxel_grid_3d) || [];
  const rawLidar = (mapData && mapData.raw_lidar_3d) || [];
  const cmdVx = (mapData && mapData.cmd_vx) || 0;
  const cmdVy = (mapData && mapData.cmd_vy) || 0;

  updateVoxels(voxels, uav);
  updatePointCloud(rawLidar);
  updateCamera(uav, cmdVx, cmdVy);
}

// --- internal update functions ----------------------------------------

function updatePointCloud(points) {
  if (!_pointCloud) return;
  const geo = new THREE.BufferGeometry();
  if (!points || !points.length) {
    _pointCloud.geometry.dispose();
    _pointCloud.geometry = geo;
    return;
  }

  // Compute distance range for coloring
  let maxDist = 1;
  const positions = new Float32Array(points.length * 3);
  const colors = new Float32Array(points.length * 3);

  for (let i = 0; i < points.length; i++) {
    const [wx, wy, wz] = worldToThree(points[i].x, points[i].y, points[i].z);
    positions[i * 3] = wx;
    positions[i * 3 + 1] = wy;
    positions[i * 3 + 2] = wz;
    // We'll compute distance from origin for coloring; update after
  }

  // Compute max distance from drone for each point
  // Use a fixed range: near=green (<5m), mid=yellow (15m), far=red (35m+)
  const nearDist = 5, farDist = 35;

  for (let i = 0; i < points.length; i++) {
    const p = points[i];
    const dist = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    // Use distance from current LiDAR origin approximation
    const t = (dist - nearDist) / (farDist - nearDist);
    const [r, g, b] = depthColor(dist, farDist);
    colors[i * 3] = r;
    colors[i * 3 + 1] = g;
    colors[i * 3 + 2] = b;
  }

  geo.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geo.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  _pointCloud.geometry.dispose();
  _pointCloud.geometry = geo;
}

function updateVoxels(voxels, uav) {
  if (!_voxelGroup) return;
  while (_voxelGroup.children.length > 0) {
    const child = _voxelGroup.children[0];
    if (child.material) child.material.dispose();
    if (child.geometry) child.geometry.dispose();
    _voxelGroup.remove(child);
  }
  if (!voxels || !voxels.length) return;

  let zMin = Infinity, zMax = -Infinity;
  for (const v of voxels) {
    const h = -v.z;
    if (h < zMin) zMin = h;
    if (h > zMax) zMax = h;
  }
  const zSpan = Math.max(0.1, zMax - zMin);
  const geo = new THREE.BoxGeometry(0.45, 0.45, 0.45);

  for (const v of voxels) {
    const h = -v.z;
    const t = Math.max(0, Math.min(1, (h - zMin) / zSpan));
    const [r, g, b] = heatColor(t);
    const mat = new THREE.MeshStandardMaterial({
      color: new THREE.Color(r, g, b),
      roughness: 0.6,
      metalness: 0.05,
      transparent: true,
      opacity: 0.55,
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(v.x, h, v.y);
    _voxelGroup.add(mesh);
  }

  if (_droneBody && uav) {
    const [dx, dy, dz] = worldToThree(uav.x, uav.y, uav.z);
    _droneBody.position.set(dx, dy, dz);
  }
}

function updateCamera(uav, cmdVx, cmdVy) {
  if (!_camera) return;
  if (!_fpvMode) return; // orbit controls active — leave camera alone

  if (!uav) return;

  const [px, py, pz] = worldToThree(uav.x, uav.y, uav.z);

  // Compute heading from commanded velocity
  const speed = Math.sqrt(cmdVx * cmdVx + cmdVy * cmdVy);
  if (speed > 0.15) {
    // cmdVx = north (world X → Three X), cmdVy = east (world Y → Three Z)
    _lastHeading = Math.atan2(cmdVy, cmdVx);
  }

  const lookDist = 18;
  const tx = px + Math.cos(_lastHeading) * lookDist;
  const tz = pz + Math.sin(_lastHeading) * lookDist;
  const ty = py + 0.8; // look slightly above horizon

  _camera.position.set(px, py + 1.2, pz);
  _camera.lookAt(tx, ty, tz);

  // Move grid to be at ground level below drone
  if (_gridHelper) {
    _gridHelper.position.set(px, 0.02, pz);
  }
}

// --- render loop ------------------------------------------------------

function renderLoop() {
  if (!_enabled) return;
  _animFrame = requestAnimationFrame(renderLoop);

  // Gradually move orbit target toward drone in FPV mode
  if (_fpvMode && _controls && _droneBody) {
    const dp = _droneBody.position;
    _controls.target.lerp(dp, 0.2);
  }

  if (_controls && _controls.enabled) _controls.update();
  if (_renderer && _scene && _camera) {
    _renderer.render(_scene, _camera);
  }
}

export function dispose3dMap() {
  _enabled = false;
  if (_animFrame) { cancelAnimationFrame(_animFrame); _animFrame = 0; }
  if (_controls) { _controls.dispose(); _controls = null; }
  if (_renderer) { _renderer.dispose(); _renderer = null; }
  if (_scene) { _scene.clear(); _scene = null; }
  _voxelGroup = null;
  _pointCloud = null;
  _droneBody = null;
  _gridHelper = null;
  _camera = null;
}
