const state = {
  data: null,
  mapMode: "goto",
  navMode: "expert",
  viewX: 0,
  viewY: 0,
  scaleMetersPerGrid: 10,
  dragging: false,
  moved: false,
  lastX: 0,
  lastY: 0,
};

const canvas = document.getElementById("mapCanvas");
const ctx = canvas.getContext("2d");
const targetSelect = document.getElementById("targetSelect");
const altitudeInput = document.getElementById("altitudeInput");
const connPill = document.getElementById("connPill");

function qs(id) { return document.getElementById(id); }

async function api(path, body = null) {
  const options = body === null ? {} : {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  };
  const response = await fetch(path, options);
  const text = await response.text();
  let payload = {};
  try { payload = text ? JSON.parse(text) : {}; } catch (_) { payload = { detail: text }; }
  if (!response.ok) {
    throw new Error(payload.detail || response.statusText);
  }
  return payload;
}

function fmt(value, digits = 1) {
  const n = Number(value);
  return Number.isFinite(n) ? n.toFixed(digits) : "-";
}

function selectedVehicle(data) {
  return (data.vehicles || []).find((v) => Number(v.sysid) === Number(data.selected_uav));
}

function updateState(data) {
  state.data = data;
  connPill.textContent = data.connected ? "已连接" : "未连接";
  connPill.className = `statusPill ${data.connected ? "good" : "bad"}`;
  qs("subtitle").textContent = `AirSim ${data.config.airsim_host}:${data.config.airsim_port} | 指令速度 ${fmt(data.command_speed_mps)} m/s`;

  const ids = data.vehicles.map((v) => v.sysid);
  const currentOptions = Array.from(targetSelect.options).map((o) => Number(o.value));
  if (ids.join(",") !== currentOptions.join(",")) {
    targetSelect.innerHTML = "";
    for (const id of ids.length ? ids : [data.selected_uav || 1]) {
      const option = document.createElement("option");
      option.value = String(id);
      option.textContent = `UAV${id}`;
      targetSelect.appendChild(option);
    }
  }
  targetSelect.value = String(data.selected_uav || 1);
  if (document.activeElement !== altitudeInput) {
    altitudeInput.value = fmt(data.target_altitude_m);
  }

  renderMetrics(data);
  renderFleet(data.vehicles);
  renderCommands(data.commands);
  renderLogs(data.logs);
  renderSelectedCard(data);
  draw();
}

function renderSelectedCard(data) {
  const vehicle = selectedVehicle(data);
  const card = qs("selectedCard");
  if (!vehicle) {
    card.innerHTML = `<span>当前飞机</span><strong>UAV${data.selected_uav || 1}</strong><em>等待遥测</em>`;
    return;
  }
  card.innerHTML = `
    <span>当前飞机</span>
    <strong>UAV${vehicle.sysid}</strong>
    <em>${vehicle.online ? "在线" : "离线"} | X ${fmt(vehicle.local_x)} / Y ${fmt(vehicle.local_y)} / Z ${fmt(vehicle.local_z)}</em>
  `;
}

function renderMetrics(data) {
  const items = [
    ["在线飞机", data.overview.online],
    ["已解锁", data.overview.armed],
    ["平均速度", `${fmt(data.overview.avg_speed)} m/s`],
    ["航点数", data.overview.waypoints],
  ];
  qs("metrics").innerHTML = items.map(([k, v]) => `<div class="metric"><span>${k}</span><strong>${v}</strong></div>`).join("");
}

function renderFleet(vehicles) {
  qs("fleetCount").textContent = `${vehicles.length} 架`;
  qs("fleetRows").innerHTML = vehicles.map((v) => `
    <tr>
      <td>UAV${v.sysid}</td>
      <td>${v.display_name || "-"}</td>
      <td class="${v.online ? "" : "offline"}">${v.online ? "在线" : "离线"}</td>
      <td>${v.mode}</td>
      <td>${v.armed ? "是" : "否"}</td>
      <td>${fmt(v.alt)}</td>
      <td>${fmt(v.speed)}</td>
      <td>${fmt(v.local_x)}</td>
      <td>${fmt(v.local_y)}</td>
      <td>${fmt(v.local_z)}</td>
    </tr>
  `).join("");
}

function renderCommands(commands) {
  qs("commandRows").innerHTML = commands.map((c) => `
    <tr>
      <td>${c.index}</td>
      <td>UAV${c.target_id}</td>
      <td>${c.label}</td>
      <td class="${c.status === "FAILED" ? "failed" : ""}">${c.status}</td>
      <td>${c.issued_at}</td>
    </tr>
  `).join("");
}

function renderLogs(logs) {
  qs("logRows").innerHTML = logs.map((row) => `
    <div class="logItem">[${row.time}] <b>${row.level}</b> | ${row.message}</div>
  `).join("");
}

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.floor(rect.width * ratio));
  canvas.height = Math.max(1, Math.floor(rect.height * ratio));
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  draw();
}

function canvasSize() {
  const rect = canvas.getBoundingClientRect();
  return { width: rect.width, height: rect.height };
}

function metersToPixels(m) { return m * (40 / state.scaleMetersPerGrid); }
function pixelsToMeters(px) { return px / (40 / state.scaleMetersPerGrid); }
function localToCanvas(x, y) {
  const size = canvasSize();
  return [size.width / 2 + metersToPixels(y - state.viewY), size.height / 2 - metersToPixels(x - state.viewX)];
}
function canvasToLocal(x, y) {
  const size = canvasSize();
  return [state.viewX + pixelsToMeters(size.height / 2 - y), state.viewY + pixelsToMeters(x - size.width / 2)];
}

function drawGrid() {
  const size = canvasSize();
  ctx.clearRect(0, 0, size.width, size.height);
  ctx.fillStyle = "#f7f9f7";
  ctx.fillRect(0, 0, size.width, size.height);
  const s = 40;
  const origin = localToCanvas(0, 0);
  ctx.strokeStyle = "rgba(54, 73, 64, 0.12)";
  ctx.lineWidth = 1;
  for (let x = origin[0] % s; x < size.width; x += s) {
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, size.height); ctx.stroke();
  }
  for (let y = origin[1] % s; y < size.height; y += s) {
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(size.width, y); ctx.stroke();
  }
  ctx.strokeStyle = "rgba(36, 54, 45, 0.34)";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(origin[0], 0); ctx.lineTo(origin[0], size.height);
  ctx.moveTo(0, origin[1]); ctx.lineTo(size.width, origin[1]);
  ctx.stroke();
}

function drawTrail(trail) {
  if (!trail || trail.length < 2) return;
  ctx.strokeStyle = "rgba(19, 122, 114, 0.55)";
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  trail.forEach((p, i) => {
    const [cx, cy] = localToCanvas(p.x || 0, p.y || 0);
    if (i === 0) ctx.moveTo(cx, cy); else ctx.lineTo(cx, cy);
  });
  ctx.stroke();
}

function drawWaypoints(waypoints) {
  if (waypoints.length > 1) {
    ctx.strokeStyle = "#a56616";
    ctx.lineWidth = 2;
    ctx.setLineDash([7, 7]);
    ctx.beginPath();
    waypoints.forEach((wp, i) => {
      const [cx, cy] = localToCanvas(wp.x || 0, wp.y || 0);
      if (i === 0) ctx.moveTo(cx, cy); else ctx.lineTo(cx, cy);
    });
    ctx.stroke();
    ctx.setLineDash([]);
  }
  waypoints.forEach((wp) => {
    const [cx, cy] = localToCanvas(wp.x || 0, wp.y || 0);
    ctx.fillStyle = "#a56616";
    ctx.beginPath(); ctx.arc(cx, cy, 7, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = "#fff"; ctx.lineWidth = 2; ctx.stroke();
    ctx.fillStyle = "#5f3a0e"; ctx.font = "700 12px Arial";
    ctx.fillText(String(wp.idx), cx + 10, cy - 8);
  });
}

function drawUavs(vehicles) {
  const selectedId = state.data ? Number(state.data.selected_uav) : 0;
  for (const uav of vehicles) {
    drawTrail(uav.trail || []);
    const [cx, cy] = localToCanvas(uav.local_x || 0, uav.local_y || 0);
    ctx.fillStyle = uav.online ? "#137a72" : "#9aa7ad";
    ctx.beginPath(); ctx.arc(cx, cy, Number(uav.sysid) === selectedId ? 11 : 8, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = Number(uav.sysid) === selectedId ? "#20272d" : "#fff";
    ctx.lineWidth = Number(uav.sysid) === selectedId ? 3 : 2.5;
    ctx.stroke();
    ctx.fillStyle = "#1f2c31"; ctx.font = "700 12px Arial";
    ctx.fillText(`UAV${uav.sysid}`, cx + 13, cy - 10);
  }
}

function draw() {
  drawGrid();
  const data = state.data || { vehicles: [], waypoints: [] };
  qs("scaleBadge").textContent = `比例 ${state.scaleMetersPerGrid.toFixed(0)}m`;
  qs("mapBadge").textContent = `飞机 ${data.vehicles.length} 架`;
  drawWaypoints(data.waypoints || []);
  drawUavs(data.vehicles || []);
}

async function selectTarget() {
  const data = await api("/api/select", {
    target_id: Number(targetSelect.value || 1),
    altitude_m: Number(altitudeInput.value || 8),
  });
  updateState(data);
}

async function sendCommand(command) {
  const critical = new Set(["arm", "disarm", "takeoff", "land", "rtl"]);
  const confirmed = !critical.has(command) || window.confirm(`向 UAV${targetSelect.value} 发送 ${command} 命令？`);
  if (!confirmed) return;
  const data = await api("/api/command", {
    target_id: Number(targetSelect.value || 1),
    command,
    confirmed,
  });
  updateState(data);
}

async function handleMapClick(x, y) {
  const altitude = Number(altitudeInput.value || 8);
  if (state.mapMode === "waypoint") {
    const data = await api("/api/waypoints/add", { x, y, altitude_m: altitude });
    updateState(data);
    return;
  }
  const msg = `让 UAV${targetSelect.value} 飞到这个 AirSim 坐标点？\n\nX=${fmt(x)}\nY=${fmt(y)}\n高度=${fmt(altitude)} m`;
  if (!window.confirm(msg)) return;
  const data = await api("/api/goto", {
    target_id: Number(targetSelect.value || 1),
    x,
    y,
    altitude_m: altitude,
    mode: state.navMode,
    confirmed: true,
  });
  updateState(data);
}

let pollingStarted = false;
function startPolling() {
  if (pollingStarted) return;
  pollingStarted = true;
  setInterval(() => {
    api("/api/state").then(updateState).catch((err) => console.warn(err));
  }, 500);
}

document.querySelectorAll("[data-command]").forEach((button) => {
  button.addEventListener("click", async () => {
    try { await sendCommand(button.dataset.command); } catch (err) { alert(err.message); }
  });
});

document.querySelectorAll("[data-map-mode]").forEach((button) => {
  button.addEventListener("click", () => {
    state.mapMode = button.dataset.mapMode;
    document.querySelectorAll("[data-map-mode]").forEach((item) => item.classList.toggle("active", item === button));
  });
});

document.querySelectorAll("[data-nav-mode]").forEach((button) => {
  button.addEventListener("click", () => {
    state.navMode = button.dataset.navMode;
    document.querySelectorAll("[data-nav-mode]").forEach((item) => item.classList.toggle("active", item === button));
  });
});

targetSelect.addEventListener("change", () => selectTarget().catch((err) => alert(err.message)));
altitudeInput.addEventListener("change", () => selectTarget().catch((err) => alert(err.message)));
qs("reconnectBtn").addEventListener("click", async () => {
  try { updateState(await api("/api/reconnect", {})); } catch (err) { alert(err.message); }
});
qs("clearWpBtn").addEventListener("click", async () => {
  try { updateState(await api("/api/waypoints/clear", {})); } catch (err) { alert(err.message); }
});
qs("runWpBtn").addEventListener("click", async () => {
  try { updateState(await api("/api/waypoints/run", { target_id: Number(targetSelect.value || 1), points: [] })); } catch (err) { alert(err.message); }
});
qs("previewBtn").addEventListener("click", async () => {
  try {
    const plan = await api("/api/llm/preview", { target_id: Number(targetSelect.value || 1), text: qs("llmText").value });
    qs("llmPreview").textContent = JSON.stringify(plan, null, 2);
  } catch (err) { alert(err.message); }
});
qs("runLlmBtn").addEventListener("click", async () => {
  try {
    const text = qs("llmText").value;
    const confirmed = window.confirm("执行自然语言解析出的命令？");
    if (!confirmed) return;
    const data = await api("/api/llm/run", { target_id: Number(targetSelect.value || 1), text, confirmed });
    updateState(data);
  } catch (err) { alert(err.message); }
});

canvas.addEventListener("click", (event) => {
  if (state.moved) { state.moved = false; return; }
  const rect = canvas.getBoundingClientRect();
  const [x, y] = canvasToLocal(event.clientX - rect.left, event.clientY - rect.top);
  handleMapClick(x, y).catch((err) => alert(err.message));
});
canvas.addEventListener("mousemove", (event) => {
  const rect = canvas.getBoundingClientRect();
  const [x, y] = canvasToLocal(event.clientX - rect.left, event.clientY - rect.top);
  qs("cursorBadge").textContent = `X ${fmt(x)} / Y ${fmt(y)}`;
});
canvas.addEventListener("mousedown", (event) => {
  if (event.button !== 0) return;
  state.dragging = true;
  state.moved = false;
  state.lastX = event.clientX;
  state.lastY = event.clientY;
  canvas.classList.add("dragging");
});
window.addEventListener("mouseup", () => {
  state.dragging = false;
  canvas.classList.remove("dragging");
});
window.addEventListener("mousemove", (event) => {
  if (!state.dragging) return;
  const dx = event.clientX - state.lastX;
  const dy = event.clientY - state.lastY;
  if (Math.abs(dx) + Math.abs(dy) > 2) state.moved = true;
  state.viewY -= pixelsToMeters(dx);
  state.viewX += pixelsToMeters(dy);
  state.lastX = event.clientX;
  state.lastY = event.clientY;
  draw();
});
canvas.addEventListener("wheel", (event) => {
  event.preventDefault();
  const rect = canvas.getBoundingClientRect();
  const cx = event.clientX - rect.left;
  const cy = event.clientY - rect.top;
  const before = canvasToLocal(cx, cy);
  const factor = event.deltaY < 0 ? 0.85 : 1.18;
  state.scaleMetersPerGrid = Math.max(1, Math.min(80, state.scaleMetersPerGrid * factor));
  const after = canvasToLocal(cx, cy);
  state.viewX += before[0] - after[0];
  state.viewY += before[1] - after[1];
  draw();
}, { passive: false });

window.addEventListener("resize", resizeCanvas);
resizeCanvas();
api("/api/state").then(updateState).catch((err) => console.warn(err));
startPolling();
