const els = {
  systemLine: document.getElementById("systemLine"),
  armedBadge: document.getElementById("armedBadge"),
  modeBadge: document.getElementById("modeBadge"),
  ekfBadge: document.getElementById("ekfBadge"),
  serviceBadge: document.getElementById("serviceBadge"),
  agentBadge: document.getElementById("agentBadge"),
  cameraMeta: document.getElementById("cameraMeta"),
  cameraCanvas: document.getElementById("cameraCanvas"),
  cameraEmpty: document.getElementById("cameraEmpty"),
  armedValue: document.getElementById("armedValue"),
  modeValue: document.getElementById("modeValue"),
  batteryValue: document.getElementById("batteryValue"),
  gpsValue: document.getElementById("gpsValue"),
  ekfValue: document.getElementById("ekfValue"),
  frameValue: document.getElementById("frameValue"),
  posX: document.getElementById("posX"),
  posY: document.getElementById("posY"),
  posZ: document.getElementById("posZ"),
  velX: document.getElementById("velX"),
  velY: document.getElementById("velY"),
  velZ: document.getElementById("velZ"),
  depthMeta: document.getElementById("depthMeta"),
  depthCenter: document.getElementById("depthCenter"),
  depthMin: document.getElementById("depthMin"),
  depthMax: document.getElementById("depthMax"),
  depthSamples: document.getElementById("depthSamples"),
  pointcloudMeta: document.getElementById("pointcloudMeta"),
  pointcloudCanvas: document.getElementById("pointcloudCanvas"),
  pointcloudEmpty: document.getElementById("pointcloudEmpty"),
  altitudeInput: document.getElementById("altitudeInput"),
  taskForm: document.getElementById("taskForm"),
  taskInput: document.getElementById("taskInput"),
  taskResult: document.getElementById("taskResult"),
  eventLog: document.getElementById("eventLog"),
};

let localEvents = [];

function fmt(value, digits = 2) {
  if (value === null || value === undefined || Number.isNaN(Number(value))) return "--";
  return Number(value).toFixed(digits);
}

async function api(path, options = {}) {
  const res = await fetch(path, {
    ...options,
    headers: {
      "Content-Type": "application/json",
      ...(options.headers || {}),
    },
  });
  const data = await res.json();
  if (!res.ok) {
    throw new Error(data.message || `HTTP ${res.status}`);
  }
  return data;
}

async function post(path, payload = {}) {
  return api(path, {
    method: "POST",
    body: JSON.stringify(payload),
  });
}

function pushEvent(kind, message) {
  localEvents.push({ kind, message });
  localEvents = localEvents.slice(-60);
  renderEvents([]);
}

function renderEvents(remoteEvents) {
  const rows = [...(remoteEvents || []), ...localEvents].slice(-80);
  els.eventLog.innerHTML = rows
    .map((event) => `<div class="${event.kind || ""}">${escapeHtml(event.message || "")}</div>`)
    .join("");
  els.eventLog.scrollTop = els.eventLog.scrollHeight;
}

function escapeHtml(value) {
  return String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;");
}

function updateBadge(el, text, className = "badge neutral") {
  el.textContent = text;
  el.className = className;
}

async function refreshStatus() {
  try {
    const data = await api("/api/status");
    const state = data.state;
    const odom = data.odom;
    const depth = data.depth;
    const pointcloud = data.pointcloud;

    els.systemLine.textContent = "ROS 已连接";
    if (state) {
      updateBadge(els.armedBadge, state.armed ? "已解锁" : "未解锁", state.armed ? "badge" : "badge bad");
      updateBadge(els.modeBadge, state.mode || "未知模式", state.mode === "OFFBOARD" ? "badge" : "badge neutral");
      updateBadge(els.ekfBadge, state.ekf_healthy ? "EKF 正常" : "EKF 异常", state.ekf_healthy ? "badge" : "badge bad");

      els.armedValue.textContent = state.armed ? "是" : "否";
      els.modeValue.textContent = state.mode || "--";
      els.batteryValue.textContent = `${fmt(state.battery, 1)} V`;
      els.gpsValue.textContent = String(state.gps_fix ?? "--");
      els.ekfValue.textContent = state.ekf_healthy ? "正常" : "异常";
    }

    if (odom) {
      els.frameValue.textContent = odom.frame_id || "--";
      els.posX.textContent = fmt(odom.position.x);
      els.posY.textContent = fmt(odom.position.y);
      els.posZ.textContent = fmt(odom.position.z);
      els.velX.textContent = fmt(odom.velocity.x);
      els.velY.textContent = fmt(odom.velocity.y);
      els.velZ.textContent = fmt(odom.velocity.z);
    }

    if (depth) {
      els.depthMeta.textContent = `${depth.width}x${depth.height} ${depth.encoding}`;
      els.depthCenter.textContent = meters(depth.center_m);
      els.depthMin.textContent = meters(depth.min_m);
      els.depthMax.textContent = meters(depth.max_m);
      els.depthSamples.textContent = String(depth.valid_samples ?? "--");
    }

    if (pointcloud) {
      els.pointcloudMeta.textContent = `${pointcloud.total_points || 0} 点 / ${pointcloud.frame_id || "--"}`;
    }

    const services = data.services || {};
    const ready = Object.values(services).filter(Boolean).length;
    els.serviceBadge.textContent = `${ready}/${Object.keys(services).length} 就绪`;
    els.agentBadge.textContent = services.agent ? "已就绪" : "离线";
    renderEvents(data.events || []);
  } catch (err) {
    els.systemLine.textContent = "ROS 连接断开";
    pushEvent("error", err.message);
  }
}

function meters(value) {
  if (value === null || value === undefined || Number.isNaN(Number(value))) return "--";
  return `${Number(value).toFixed(2)} m`;
}

async function refreshCamera() {
  try {
    const image = await api("/api/camera");
    drawImage(image);
  } catch (err) {
    els.cameraEmpty.style.display = "grid";
    els.cameraMeta.textContent = "暂无画面";
  }
}

function drawImage(image) {
  const canvas = els.cameraCanvas;
  const ctx = canvas.getContext("2d");
  const width = image.width;
  const height = image.height;
  const source = base64ToBytes(image.data);
  const rgba = new Uint8ClampedArray(width * height * 4);
  const encoding = String(image.encoding || "").toLowerCase();
  const channels = encoding.includes("rgba") || encoding.includes("bgra") ? 4 : 3;
  const step = image.step || width * channels;

  for (let y = 0; y < height; y += 1) {
    for (let x = 0; x < width; x += 1) {
      const src = y * step + x * channels;
      const dst = (y * width + x) * 4;
      if (encoding.includes("bgr")) {
        rgba[dst] = source[src + 2] || 0;
        rgba[dst + 1] = source[src + 1] || 0;
        rgba[dst + 2] = source[src] || 0;
      } else {
        rgba[dst] = source[src] || 0;
        rgba[dst + 1] = source[src + 1] || 0;
        rgba[dst + 2] = source[src + 2] || 0;
      }
      rgba[dst + 3] = channels === 4 ? source[src + 3] || 255 : 255;
    }
  }

  canvas.width = width;
  canvas.height = height;
  ctx.putImageData(new ImageData(rgba, width, height), 0, 0);
  els.cameraEmpty.style.display = "none";
  els.cameraMeta.textContent = `${width}x${height} ${image.encoding}`;
}

async function refreshPointcloud() {
  try {
    const pointcloud = await api("/api/pointcloud");
    drawPointcloud(pointcloud);
  } catch (err) {
    els.pointcloudEmpty.style.display = "grid";
    els.pointcloudMeta.textContent = "暂无数据";
  }
}

function drawPointcloud(pointcloud) {
  const canvas = els.pointcloudCanvas;
  const ctx = canvas.getContext("2d");
  const width = canvas.width;
  const height = canvas.height;
  const points = pointcloud.points || [];

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#071013";
  ctx.fillRect(0, 0, width, height);

  drawGrid(ctx, width, height);

  if (!points.length) {
    els.pointcloudEmpty.style.display = "grid";
    els.pointcloudMeta.textContent = "点云为空";
    return;
  }

  let maxRange = 1;
  for (const point of points) {
    maxRange = Math.max(maxRange, Math.abs(point[0]), Math.abs(point[1]));
  }
  maxRange = Math.min(Math.max(maxRange, 8), 80);
  const scale = Math.min(width, height) * 0.44 / maxRange;
  const cx = width / 2;
  const cy = height / 2;

  ctx.fillStyle = "#67e8f9";
  for (const point of points) {
    const x = cx + point[0] * scale;
    const y = cy - point[1] * scale;
    if (x < 0 || x > width || y < 0 || y > height) continue;
    const z = point[2] || 0;
    const radius = Math.max(1, Math.min(3, 1.4 + Math.abs(z) * 0.08));
    ctx.globalAlpha = Math.max(0.35, Math.min(0.95, 0.75 - Math.abs(z) * 0.01));
    ctx.beginPath();
    ctx.arc(x, y, radius, 0, Math.PI * 2);
    ctx.fill();
  }
  ctx.globalAlpha = 1;

  ctx.fillStyle = "#f6c85f";
  ctx.beginPath();
  ctx.moveTo(cx, cy - 9);
  ctx.lineTo(cx - 7, cy + 8);
  ctx.lineTo(cx + 7, cy + 8);
  ctx.closePath();
  ctx.fill();

  els.pointcloudEmpty.style.display = "none";
  els.pointcloudMeta.textContent = `${pointcloud.sampled_points} / ${pointcloud.total_points} 点`;
}

function drawGrid(ctx, width, height) {
  ctx.strokeStyle = "#1f3036";
  ctx.lineWidth = 1;
  for (let x = 0; x <= width; x += 40) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
    ctx.stroke();
  }
  for (let y = 0; y <= height; y += 40) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }
  ctx.strokeStyle = "#3b4a50";
  ctx.beginPath();
  ctx.moveTo(width / 2, 0);
  ctx.lineTo(width / 2, height);
  ctx.moveTo(0, height / 2);
  ctx.lineTo(width, height / 2);
  ctx.stroke();
}

function base64ToBytes(value) {
  const binary = atob(value);
  const bytes = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i += 1) {
    bytes[i] = binary.charCodeAt(i);
  }
  return bytes;
}

async function runControl(label, path, payload) {
  try {
    const result = await post(path, payload);
    pushEvent("service", `${label}: ${result.message || "完成"}`);
  } catch (err) {
    pushEvent("error", `${label}: ${err.message}`);
  }
}

function commandPayload(name) {
  const speed = 5;
  const climb = 2;
  const yaw = 0.4;
  const zero = { linear: { x: 0, y: 0, z: 0 }, angular: { z: 0 } };
  const map = {
    forward: { linear: { x: 0, y: speed, z: 0 }, angular: { z: 0 } },
    back: { linear: { x: 0, y: -speed, z: 0 }, angular: { z: 0 } },
    left: { linear: { x: -speed, y: 0, z: 0 }, angular: { z: 0 } },
    right: { linear: { x: speed, y: 0, z: 0 }, angular: { z: 0 } },
    up: { linear: { x: 0, y: 0, z: climb }, angular: { z: 0 } },
    down: { linear: { x: 0, y: 0, z: -climb }, angular: { z: 0 } },
    "yaw-left": { linear: { x: 0, y: 0, z: 0 }, angular: { z: -yaw } },
    "yaw-right": { linear: { x: 0, y: 0, z: 0 }, angular: { z: yaw } },
    stop: zero,
  };
  return map[name] || zero;
}

document.getElementById("armBtn").addEventListener("click", () => {
  runControl("解锁", "/api/control/arm", { arm: true });
});

document.getElementById("disarmBtn").addEventListener("click", () => {
  runControl("加锁", "/api/control/arm", { arm: false });
});

document.getElementById("takeoffBtn").addEventListener("click", () => {
  const altitude = Number(els.altitudeInput.value || 10);
  runControl("起飞", "/api/control/takeoff", { altitude });
});

document.getElementById("landBtn").addEventListener("click", () => {
  runControl("降落", "/api/control/land", {});
});

document.querySelectorAll("[data-cmd]").forEach((button) => {
  button.addEventListener("click", () => {
    const name = button.getAttribute("data-cmd");
    runControl(`控制 ${button.textContent}`, "/api/cmd_vel", commandPayload(name));
  });
});

els.taskForm.addEventListener("submit", async (event) => {
  event.preventDefault();
  try {
    const result = await post("/api/agent/task", { task: els.taskInput.value });
    els.taskResult.textContent = JSON.stringify(result, null, 2);
    pushEvent("service", `自然语言: ${result.message || result.result || "完成"}`);
  } catch (err) {
    els.taskResult.textContent = err.message;
    pushEvent("error", `自然语言: ${err.message}`);
  }
});

document.getElementById("clearLogBtn").addEventListener("click", () => {
  localEvents = [];
  els.eventLog.innerHTML = "";
});

refreshStatus();
refreshCamera();
refreshPointcloud();
setInterval(refreshStatus, 1000);
setInterval(refreshCamera, 1600);
setInterval(refreshPointcloud, 2200);
