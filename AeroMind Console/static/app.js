const state = {
  lastPlan: [],
  currentStatus: "idle",
  isListening: false,
  map: null,
  mapPxPerMeter: 4,
  mapViewX: 25,
  mapViewY: 0,
  mapFollow: true,
  mapDragging: false,
  mapLastX: 0,
  mapLastY: 0,
  safety: null,
  agentProgress: null,
  selectedCamera: "",
  lastPreview: null,
  previewMapActive: false,
  missionLogs: [],
  pageStartedAt: Date.now() / 1000,
  serverStartedAt: null,
  selectedVehicle: "",
};

const MAP_ZOOM_MIN = 0.6;
const MAP_ZOOM_MAX = 36;
const MAP_ZOOM_STEP = 1.18;

const text = {
  simConnected: "\u4eff\u771f\u5df2\u8fde\u63a5",
  simOffline: "\u4eff\u771f\u672a\u8fde\u63a5",
  visionPending: "\u89c6\u89c9\u5f85\u63a5\u5165",
  cameraOnline: "\u76f8\u673a\u5728\u7ebf",
  idle: "\u7a7a\u95f2",
  planning: "\u89c4\u5212\u4e2d",
  holding: "\u60ac\u505c\u4fdd\u6301",
  paused: "\u5df2\u6682\u505c",
  stopped: "\u5df2\u7ec8\u6b62",
  returning: "\u8fd4\u822a\u4e2d",
  waitTask: "\u7b49\u5f85\u4efb\u52a1",
  waitDetail: "\u8f93\u5165\u81ea\u7136\u8bed\u8a00\u4efb\u52a1\u540e\uff0c\u8fd9\u91cc\u4f1a\u5c55\u793a\u5927\u6a21\u578b\u62c6\u89e3\u7684\u6267\u884c\u6b65\u9aa4\u3002",
};

const riskLabels = {
  clear: "\u6e05\u7a7a",
  low: "\u4f4e",
  medium: "\u4e2d",
  high: "\u9ad8",
  critical: "\u5371\u6025",
  unknown: "\u672a\u77e5",
  "-": "-",
};

const directionLabels = {
  none: "\u65e0",
  front: "\u524d\u65b9",
  left: "\u5de6\u4fa7",
  right: "\u53f3\u4fa7",
  hard_left: "\u5de6\u540e",
  hard_right: "\u53f3\u540e",
};

const plannerLabels = {
  ok: "\u5df2\u751f\u6210",
  blocked: "\u53d7\u963b",
  wait: "\u7b49\u5f85",
};

const templates = {
  search: "\u641c\u7d22\u524d\u65b9 60 \u7c73\u533a\u57df\uff0c\u53d1\u73b0\u8f66\u8f86\u540e\u60ac\u505c\u5e76\u62a5\u544a\u5750\u6807",
  scan: "\u7ed5\u76ee\u6807\u5efa\u7b51\u626b\u63cf\u4e00\u5708\uff0c\u4fdd\u6301 8 \u7c73\u9ad8\u5ea6\u5e76\u4fdd\u5b58\u5173\u952e\u5e27",
  formation: "\u4e09\u67b6\u65e0\u4eba\u673a\u7ec4\u6210 V \u5b57\u7f16\u961f\uff0c\u6cbf X \u8f74\u98de\u884c 40 \u7c73\u5e76\u4fdd\u6301\u907f\u969c",
};

const templateParams = {
  search: { target: "vehicle", forward_m: 60, half_width_m: 20, altitude_m: 8, speed_mps: 2, spacing_m: 10, count: 3, shape: "v", pre_scan: true, planned_avoidance: true, scan_margin_m: 4, obstacle_distance_m: 8, avoidance_offset_m: 6, pre_scan_stop_on_high_risk: true },
  scan: { target: "building", forward_m: 35, half_width_m: 12, altitude_m: 8, speed_mps: 1.5, spacing_m: 8, count: 3, shape: "v", pre_scan: true, planned_avoidance: true, scan_margin_m: 4, obstacle_distance_m: 8, avoidance_offset_m: 6, pre_scan_stop_on_high_risk: true },
  formation: { target: "object", forward_m: 40, half_width_m: 15, altitude_m: 10, speed_mps: 2, spacing_m: 10, count: 3, shape: "v", pre_scan: false, planned_avoidance: true, scan_margin_m: 4, obstacle_distance_m: 8, avoidance_offset_m: 6, pre_scan_stop_on_high_risk: true },
};

function el(id) {
  return document.getElementById(id);
}

async function api(path, body = null) {
  const options = body === null ? {} : {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  };
  const response = await fetch(path, options);
  const payload = await response.json();
  if (!response.ok) {
    throw new Error(payload.detail || response.statusText);
  }
  return payload;
}

function fmt(value, digits = 1) {
  const n = Number(value);
  return Number.isFinite(n) ? n.toFixed(digits) : "-";
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function mapZoomLevel() {
  const range = Math.log(MAP_ZOOM_MAX / MAP_ZOOM_MIN);
  const value = Math.log(clamp(state.mapPxPerMeter, MAP_ZOOM_MIN, MAP_ZOOM_MAX) / MAP_ZOOM_MIN) / range;
  return clamp(Math.round(1 + value * 9), 1, 10);
}

function updateMapZoomLevel() {
  const badge = el("mapZoomLevel");
  if (!badge) return;
  badge.textContent = `L${mapZoomLevel()}`;
  badge.title = `${fmt(state.mapPxPerMeter)} px/m`;
}

function statusText(status) {
  return text[status] || status;
}

function riskText(value) {
  const key = String(value || "unknown").toLowerCase();
  return riskLabels[key] || value || "\u672a\u77e5";
}

function directionText(value) {
  const key = String(value || "none").toLowerCase();
  return directionLabels[key] || value || "-";
}

function plannerText(value) {
  return plannerLabels[value] || value;
}

function numInput(id, fallback) {
  const value = Number(el(id).value);
  return Number.isFinite(value) ? value : fallback;
}

function collectTaskParams() {
  const forward = numInput("forwardInput", 60);
  const halfWidth = numInput("widthInput", 20);
  return {
    target: el("targetSelect").value || "object",
    altitude_m: numInput("altitudeInput", 8),
    speed_mps: numInput("speedInput", 2),
    spacing_m: numInput("spacingInput", 10),
    count: Math.max(2, Math.min(6, Math.round(numInput("formationCountInput", 3)))),
    shape: el("formationShapeSelect").value || "v",
    formation_spacing_m: numInput("spacingInput", 10),
    avoidance: el("avoidanceInput").checked,
    planned_avoidance: el("avoidanceInput").checked,
    pre_scan: el("preScanInput").checked,
    pre_scan_stop_on_high_risk: el("stopOnRiskInput").checked,
    obstacle_distance_m: numInput("obstacleDistanceInput", 8),
    avoidance_offset_m: numInput("avoidanceOffsetInput", 6),
    scan_margin_m: numInput("scanMarginInput", 4),
    area: {
      x_min: 0,
      x_max: Math.max(5, forward),
      y_min: -Math.max(2, halfWidth),
      y_max: Math.max(2, halfWidth),
    },
  };
}

function applyTemplateParams(name) {
  const params = templateParams[name];
  if (!params) return;
  el("targetSelect").value = params.target;
  el("forwardInput").value = params.forward_m;
  el("widthInput").value = params.half_width_m;
  el("altitudeInput").value = params.altitude_m;
  el("speedInput").value = params.speed_mps;
  el("spacingInput").value = params.spacing_m;
  el("formationCountInput").value = params.count;
  el("formationShapeSelect").value = params.shape;
  el("preScanInput").checked = params.pre_scan;
  el("avoidanceInput").checked = params.planned_avoidance;
  el("scanMarginInput").value = params.scan_margin_m;
  el("obstacleDistanceInput").value = params.obstacle_distance_m;
  el("avoidanceOffsetInput").value = params.avoidance_offset_m;
  el("stopOnRiskInput").checked = params.pre_scan_stop_on_high_risk;
}

function buildTaskTextFromParams() {
  const params = collectTaskParams();
  const targetNames = {
    vehicle: "\u8f66\u8f86",
    person: "\u884c\u4eba",
    building: "\u5efa\u7b51",
    obstacle: "\u969c\u788d\u7269",
    object: "\u76ee\u6807",
  };
  return `\u641c\u7d22\u524d\u65b9 ${fmt(params.area.x_max, 0)} \u7c73\u5de6\u53f3 ${fmt(params.area.y_max, 0)} \u7c73\u8303\u56f4\u5185\u7684${targetNames[params.target] || "\u76ee\u6807"}\uff0c\u9ad8\u5ea6 ${fmt(params.altitude_m, 0)} \u7c73`;
}

function render(data) {
  if (data.server && state.serverStartedAt !== data.server.started_at) {
    state.serverStartedAt = data.server.started_at;
    state.missionLogs = [];
    renderMissionLog();
  }
  const taskStatus = data.task.status;
  const inactiveTask = ["idle", "completed", "stopped"].includes(taskStatus);
  const staleTaskView = inactiveTask && !state.previewMapActive;
  state.currentStatus = taskStatus;
  el("connectionPill").textContent = data.connected ? text.simConnected : text.simOffline;
  el("connectionPill").className = `pill ${data.connected ? "online" : "offline"}`;
  el("visionPill").textContent = data.video.mode === "placeholder" ? text.visionPending : text.cameraOnline;
  el("visionPill").className = `pill ${data.video.mode === "placeholder" ? "neutral" : "online"}`;
  renderVehicleSelector(data.vehicle);
  renderVideo(data.video);
  el("taskPill").textContent = staleTaskView ? statusText("idle") : statusText(taskStatus);
  el("taskPill").className = `pill ${staleTaskView ? "neutral" : "active"}`;
  el("videoFrame").classList.toggle("isPlanning", !inactiveTask);
  el("uavName").textContent = data.uav.name;
  el("uavMode").textContent = data.uav.mode;
  el("telemetryLine").textContent = `高度 ${fmt(data.uav.altitude_m)}m / 速度 ${fmt(data.uav.speed_mps)}m/s / X ${fmt(data.uav.x)} Y ${fmt(data.uav.y)}`;
  el("systemLine").textContent = data.video.mode === "placeholder"
    ? (data.airsim_error || data.video.message || "\u7b49\u5f85\u6444\u50cf\u5934\u6d41")
    : `相机 ${data.video.camera_name || data.video.selected_camera_name || "front"} 在线`;
  const visiblePlan = staleTaskView ? [] : (taskStatus === "idle" ? state.lastPlan : (data.task.plan || []));
  el("taskTitle").textContent = staleTaskView ? text.waitTask : data.task.title;
  const currentWp = Number(state.agentProgress && state.agentProgress.current_waypoint_index);
  const totalWp = Number(state.agentProgress && state.agentProgress.total_waypoints);
  const wpText = Number.isFinite(currentWp) && Number.isFinite(totalWp) && totalWp > 0
    ? ` / 航点 ${Math.max(0, Math.round(currentWp))}/${Math.max(0, Math.round(totalWp))}`
    : "";
  el("taskSubline").textContent = staleTaskView ? "\u7b49\u5f85\u63d0\u4ea4\u4efb\u52a1" : `\u4efb\u52a1\u72b6\u6001\uff1a${statusText(taskStatus)}${wpText}`;
  el("positionText").textContent = `X ${fmt(data.uav.x)} / Y ${fmt(data.uav.y)} / Z ${fmt(data.uav.z)}`;
  el("altitudeText").textContent = `高度 ${fmt(data.uav.altitude_m)}m / 速度 ${fmt(data.uav.speed_mps)}m/s`;
  state.safety = data.safety || null;
  state.agentProgress = staleTaskView ? null : ((data.task && data.task.agent_progress) || null);
  renderOpsOverview(data);
  renderPerception(data);
  renderPlan(visiblePlan, data.task.status, state.agentProgress);
  renderRoute(visiblePlan, data.task.status, state.agentProgress);
  renderEvents(staleTaskView ? [] : (data.events || []));
  state.map = staleTaskView ? clearedMissionMap(data.map) : (data.map || null);
  if (state.mapFollow && state.map && state.map.uav) {
    state.mapViewX = Number(state.map.uav.x) || 0;
    state.mapViewY = Number(state.map.uav.y) || 0;
  }
  renderMapStatus();
  drawMissionMap();
  updateClock();
  updateMissionLog(data);
}

function renderOpsOverview(data) {
  const progress = (data.task && data.task.agent_progress) || {};
  const safety = data.safety || {};
  const obstacle = safety.obstacle || {};
  const distances = safety.distance_sensors || {};
  const planner = progress.planner || {};
  const status = String(progress.status || data.task.status || "idle").toUpperCase();
  const current = Number(progress.current_waypoint_index || 0);
  const total = Number(progress.total_waypoints || 0);
  const distance = Number(progress.distance_to_waypoint_m);
  const risk = String((progress.area_assessment && progress.area_assessment.risk) || obstacle.risk_level || "-");
  el("phaseBadge").textContent = status;
  el("waypointBadge").textContent = total > 0 ? `${Math.max(0, current)}/${Math.max(0, total)}` : "-";
  el("distanceBadge").textContent = Number.isFinite(distance) ? `${fmt(distance)} m` : "-";
  el("riskBadge").textContent = riskText(risk);

  const hasError = ["failed", "blocked", "collision", "timeout", "stuck", "stuck_recovered"].includes(String(progress.status || ""));
  const hasWarn = ["recovering", "pre_scanning", "avoiding", "recovered"].includes(String(progress.status || ""));
  let title = "No Immediate Issue";
  let detail = String(progress.message || "Mission is running. Diagnostics will be highlighted here if anything degrades.");

  if (hasError) {
    title = "Execution Blocked";
  } else if (hasWarn) {
    title = "Risk Handling Active";
  } else if (distances && distances.emergency) {
    title = "Emergency Bubble Triggered";
    detail = `Obstacle near ${directionText(distances.emergency_direction)}; command stream is constrained.`;
  } else if (planner && planner.ok === false) {
    title = "Planner Not Ready";
    detail = String(planner.reason || detail);
  } else if (risk === "high" || risk === "critical") {
    title = "High Risk Environment";
  }

  const banner = el("issueBanner");
  banner.className = `issueBanner${hasError ? " error" : hasWarn || risk === "high" || risk === "critical" ? " warn" : ""}`;
  el("issueTitle").textContent = title;
  el("issueDetail").textContent = detail;
}

function renderTelemetry(data) {
  if (!data || !data.uav) return;
  renderVehicleSelector(data.vehicle);
  el("uavName").textContent = data.uav.name;
  el("uavMode").textContent = data.uav.mode;
  el("telemetryLine").textContent = `高度 ${fmt(data.uav.altitude_m)}m / 速度 ${fmt(data.uav.speed_mps)}m/s / X ${fmt(data.uav.x)} Y ${fmt(data.uav.y)}`;
  el("positionText").textContent = `X ${fmt(data.uav.x)} / Y ${fmt(data.uav.y)} / Z ${fmt(data.uav.z)}`;
  el("altitudeText").textContent = `高度 ${fmt(data.uav.altitude_m)}m / 速度 ${fmt(data.uav.speed_mps)}m/s`;
  if (!state.map) {
    state.map = { home: { x: 0, y: 0 } };
  }
  state.map = {
    ...state.map,
    uav: data.map && data.map.uav ? data.map.uav : { x: data.uav.x, y: data.uav.y, z: data.uav.z, name: data.uav.name },
    uavs: data.map && Array.isArray(data.map.uavs) ? data.map.uavs : [],
    trail: data.map && Array.isArray(data.map.trail) ? data.map.trail : (state.map.trail || []),
  };
  if (state.mapFollow && state.map.uav) {
    state.mapViewX = Number(state.map.uav.x) || 0;
    state.mapViewY = Number(state.map.uav.y) || 0;
  }
  renderMapStatus();
  drawMissionMap();
}

function renderVideo(video) {
  const feed = el("cameraFeed");
  const fallback = el("videoFallback");
  renderCameraSwitcher(video);
  if (video.mode === "placeholder") {
    feed.style.display = "none";
    fallback.style.display = "grid";
    return;
  }
  const selected = video.selected_camera_name || video.camera_name || "front_center";
  if (!feed.src || !feed.src.includes(video.front_camera_url) || state.selectedCamera !== selected) {
    state.selectedCamera = selected;
    feed.src = `${video.front_camera_url}?camera=${encodeURIComponent(selected)}&live=${Date.now()}`;
  }
  feed.style.display = "block";
  fallback.style.display = "none";
}

function clearedMissionMap(map) {
  if (!map) return null;
  return {
    ...map,
    trail: [],
    route: [],
    search_area: null,
    targets: [],
    blocked_zones: [],
  };
}

function renderCameraSwitcher(video) {
  const selected = video.selected_camera_name || video.camera_name || "front_center";
  document.querySelectorAll(".cameraChip").forEach((button) => {
    const camera = button.dataset.camera || "";
    button.classList.toggle("active", camera === selected);
  });
}

function renderVehicleSelector(vehicle) {
  const select = el("vehicleSelect");
  if (!select || !vehicle) return;
  const vehicles = Array.isArray(vehicle.vehicles) && vehicle.vehicles.length ? vehicle.vehicles : ["Drone1"];
  const selected = vehicle.selected || vehicles[0];
  const currentOptions = Array.from(select.options).map((option) => option.value);
  if (currentOptions.join("|") !== vehicles.join("|")) {
    select.innerHTML = vehicles.map((name) => `<option value="${name}">${name}</option>`).join("");
  }
  select.value = selected;
  state.selectedVehicle = selected;
}

function renderPlan(plan, status = state.currentStatus, agentProgress = null) {
  state.lastPlan = plan || [];
  const list = el("planList");
  if (!state.lastPlan.length) {
    list.innerHTML = `<div class="planStep" data-index="-"><strong>${text.waitTask}</strong><span>${text.waitDetail}</span></div>`;
    setConfidence(0);
    return;
  }
  let activeIndex = status === "planning" || status === "returning" ? 0 : -1;
  if (agentProgress && Number.isFinite(Number(agentProgress.current_waypoint_index)) && Number.isFinite(Number(agentProgress.total_waypoints))) {
    const current = Math.max(0, Number(agentProgress.current_waypoint_index));
    const total = Math.max(0, Number(agentProgress.total_waypoints));
    if (total > 0) {
      const ratio = current / total;
      activeIndex = Math.min(state.lastPlan.length - 1, Math.max(0, Math.floor(ratio * state.lastPlan.length)));
    }
  }
  list.innerHTML = state.lastPlan.map((step, index) => `
    <div class="planStep ${index === activeIndex ? "active" : ""}" data-index="${index + 1}">
      <strong>${step.name}</strong>
      <span>${step.detail}</span>
    </div>
  `).join("");
  setConfidence(Math.min(96, 58 + state.lastPlan.length * 7));
}

function renderRoute(plan, status, agentProgress = null) {
  const count = Math.max(5, Math.min(7, (plan || []).length || 5));
  let active = status === "idle" ? -1 : 0;
  if (agentProgress && Number.isFinite(Number(agentProgress.current_waypoint_index)) && Number.isFinite(Number(agentProgress.total_waypoints))) {
    const current = Math.max(0, Number(agentProgress.current_waypoint_index));
    const total = Math.max(1, Number(agentProgress.total_waypoints));
    active = Math.max(0, Math.min(count - 1, Math.floor((current / total) * count)));
  }
  el("routeLine").innerHTML = Array.from({ length: count }, (_, index) => {
    const cls = index < active ? "done" : index === active ? "active" : "";
    return `<i class="routeDot ${cls}"></i>`;
  }).join("");
}

function renderToolPreview(preview) {
  const box = el("toolPreview");
  if (!preview) {
    box.className = "toolPreview";
    box.innerHTML = "";
    return;
  }
  const executable = preview.executable || null;
  const toolCall = executable && executable.tool_call;
  const safety = executable && executable.safety;
  const accepted = executable && executable.accepted;
  const toolName = toolCall ? toolCall.tool : "\u6682\u65e0\u5de5\u5177\u6620\u5c04";
  const status = executable ? (accepted ? "\u5c31\u7eea" : "\u5df2\u963b\u6b62") : "\u8ba1\u5212";
  const statusClass = accepted || !executable ? "ok" : "blocked";
  const stats = preview.preview || {};
  const args = toolCall && toolCall.args ? toolCall.args : {};
  const reason = safety && safety.reason ? safety.reason : (stats.message || "");
  const preflight = preview.preflight || {};
  const checks = Array.isArray(preflight.checks) ? preflight.checks : [];
  const checksHtml = checks.map((item) => `
    <div class="preflightItem ${item.status}">
      <b>${item.label}</b>
      <span>${item.detail}</span>
    </div>
  `).join("");
  box.className = "toolPreview visible";
  box.innerHTML = `
    <div class="toolPreviewHead">
      <strong>${toolName}</strong>
      <span class="${statusClass}">${status}</span>
    </div>
    <div class="toolPreviewStats">
      <div><b>\u822a\u70b9</b><em>${stats.waypoint_count || 0}</em></div>
      <div><b>\u8ddd\u79bb</b><em>${fmt(stats.estimated_distance_m || 0)}m</em></div>
      <div><b>\u5b89\u5168</b><em>${reason || "-"}</em></div>
    </div>
    <div class="preflightPanel">
      <div class="preflightTitle">
        <b>\u98de\u884c\u524d\u68c0\u67e5</b>
        <span class="${preflight.overall || "caution"}">${String(preflight.overall || "caution").toUpperCase()}</span>
      </div>
      <div class="preflightList">${checksHtml}</div>
    </div>
    <details>
      <summary>\u5de5\u5177\u53c2\u6570</summary>
      <pre>${JSON.stringify(args, null, 2)}</pre>
    </details>
  `;
}

function renderEvents(events) {
  const visibleEvents = (events || []).filter((item) => {
    const eventTs = Number(item.ts || 0);
    return !eventTs || eventTs >= state.pageStartedAt;
  });
  if (!visibleEvents.length) {
    el("eventList").innerHTML = `<div class="eventItem">\u7b49\u5f85\u4efb\u52a1\u4e8b\u4ef6</div>`;
    return;
  }
  const levelClass = (level) => {
    const key = String(level || "").toUpperCase();
    if (["ERROR", "FAILED"].includes(key)) return "error";
    if (["WARN", "SAFE"].includes(key)) return "warn";
    if (["MISSION", "PLAN", "TOOL"].includes(key)) return "info";
    return "";
  };
  el("eventList").innerHTML = visibleEvents.slice(0, 8).map((item) => `
    <div class="eventItem ${levelClass(item.level)}">[${item.time}] <b>${item.level}</b> | ${item.message}</div>
  `).join("");
}

function mapCanvasSize() {
  const canvas = el("missionMap");
  const rect = canvas.getBoundingClientRect();
  return { width: rect.width, height: rect.height };
}

function resizeMissionMap() {
  const canvas = el("missionMap");
  const rect = canvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.floor(rect.width * ratio));
  canvas.height = Math.max(1, Math.floor(rect.height * ratio));
  const ctx = canvas.getContext("2d");
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  drawMissionMap();
}

function mapToCanvas(x, y) {
  const size = mapCanvasSize();
  return [
    size.width / 2 + (Number(y) - state.mapViewY) * state.mapPxPerMeter,
    size.height / 2 - (Number(x) - state.mapViewX) * state.mapPxPerMeter,
  ];
}

function canvasToMap(cx, cy) {
  const size = mapCanvasSize();
  return [
    state.mapViewX + (size.height / 2 - cy) / state.mapPxPerMeter,
    state.mapViewY + (cx - size.width / 2) / state.mapPxPerMeter,
  ];
}

function setMapZoom(pxPerMeter, anchor = null) {
  const point = anchor || (() => {
    const size = mapCanvasSize();
    return { x: size.width / 2, y: size.height / 2 };
  })();
  const before = canvasToMap(point.x, point.y);
  state.mapPxPerMeter = clamp(pxPerMeter, MAP_ZOOM_MIN, MAP_ZOOM_MAX);
  const after = canvasToMap(point.x, point.y);
  state.mapViewX += before[0] - after[0];
  state.mapViewY += before[1] - after[1];
  updateMapZoomLevel();
  drawMissionMap();
}

function zoomMissionMap(factor, anchor = null) {
  setMapZoom(state.mapPxPerMeter * factor, anchor);
}

function drawMissionMap() {
  const canvas = el("missionMap");
  if (!canvas) return;
  updateMapZoomLevel();
  const ctx = canvas.getContext("2d");
  const size = mapCanvasSize();
  ctx.clearRect(0, 0, size.width, size.height);
  ctx.fillStyle = "#091116";
  ctx.fillRect(0, 0, size.width, size.height);
  drawMapGrid(ctx, size);
  const data = state.map || {};
  if (data.search_area) drawSearchArea(ctx, data.search_area);
  drawBlockedZones(ctx, data.blocked_zones || []);
  drawObstaclePoints(ctx, data.obstacles || []);
  drawPolyline(ctx, data.route || [], "#d9a441", 2, [6, 5]);
  drawPlannerPath(ctx);
  drawPolyline(ctx, data.trail || [], "#43c7b9", 2.5, []);
  drawFormationTargets(ctx);
  drawPoint(ctx, data.home || { x: 0, y: 0 }, "#78aaff", 5, "H");
  for (const target of data.targets || []) {
    drawPoint(ctx, target, "#e15d5d", 6, "T");
  }
  const uavs = visibleUavs(selectedFirstUavs(data.uavs || [], data.uav));
  if (uavs.length >= 1) {
    if (uavs.length > 1) {
      drawFormationLines(ctx, uavs);
    }
    for (let i = 0; i < uavs.length; i++) {
      drawUav(ctx, uavs[i], i === 0, i);
    }
  } else {
    drawUav(ctx, data.uav || { x: 0, y: 0, z: 0 }, true, 0);
  }
}

function visibleUavs(uavs) {
  if (!uavs || uavs.length < 2) return uavs || [];
  const groups = new Map();
  for (let i = 0; i < uavs.length; i++) {
    const item = uavs[i] || {};
    const key = `${Math.round((Number(item.x) || 0) * 10) / 10},${Math.round((Number(item.y) || 0) * 10) / 10}`;
    if (!groups.has(key)) groups.set(key, []);
    groups.get(key).push(i);
  }
  return uavs.map((item, index) => {
    const key = `${Math.round((Number(item.x) || 0) * 10) / 10},${Math.round((Number(item.y) || 0) * 10) / 10}`;
    const group = groups.get(key) || [];
    if (group.length < 2) return item;
    const slot = group.indexOf(index);
    const angle = (Math.PI * 2 * slot) / group.length;
    return {
      ...item,
      screenOffsetX: Math.cos(angle) * 13,
      screenOffsetY: Math.sin(angle) * 13,
    };
  });
}

function selectedFirstUavs(uavs, selectedFallback = null) {
  const selected = state.selectedVehicle || (selectedFallback && selectedFallback.name) || "";
  const list = Array.isArray(uavs) ? [...uavs] : [];
  if (!selected) return list.length ? list : (selectedFallback ? [selectedFallback] : []);
  const index = list.findIndex((item) => item && item.name === selected);
  if (index > 0) {
    const [item] = list.splice(index, 1);
    list.unshift(item);
  } else if (index < 0 && selectedFallback) {
    list.unshift(selectedFallback);
  }
  return list;
}

function drawFormationLines(ctx, uavs) {
  if (uavs.length < 2) return;
  ctx.save();
  ctx.strokeStyle = "rgba(255,165,0,0.4)";
  ctx.lineWidth = 1.5;
  ctx.setLineDash([4, 4]);
  const leader = uavs[0];
  const [lx, ly] = mapToCanvas(leader.x || 0, leader.y || 0);
  for (let i = 1; i < uavs.length; i++) {
    const [x, y] = mapToCanvas(uavs[i].x || 0, uavs[i].y || 0);
    ctx.beginPath();
    ctx.moveTo(lx, ly);
    ctx.lineTo(x, y);
    ctx.stroke();
  }
  ctx.restore();
}

function drawFormationTargets(ctx) {
  const progress = state.agentProgress || {};
  const targets = Array.isArray(progress.formation_targets) ? progress.formation_targets : [];
  const vehicles = Array.isArray(progress.formation_vehicles) ? progress.formation_vehicles : [];
  if (!targets.length && !vehicles.length) return;
  const states = new Map(vehicles.map((item) => [item.vehicle, item]));
  ctx.save();
  ctx.setLineDash([5, 4]);
  ctx.lineWidth = 1.5;
  for (const target of targets) {
    const [tx, ty] = mapToCanvas(target.x, target.y);
    const stateItem = states.get(target.vehicle);
    if (stateItem && stateItem.telemetry) {
      const [ux, uy] = mapToCanvas(stateItem.telemetry.x, stateItem.telemetry.y);
      ctx.strokeStyle = stateItem.arrived ? "rgba(67,199,185,0.7)" : "rgba(217,164,65,0.75)";
      ctx.beginPath();
      ctx.moveTo(ux, uy);
      ctx.lineTo(tx, ty);
      ctx.stroke();
    }
    ctx.setLineDash([]);
    ctx.strokeStyle = "#d9a441";
    ctx.fillStyle = "rgba(217,164,65,0.14)";
    ctx.beginPath();
    ctx.rect(tx - 6, ty - 6, 12, 12);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = "#f5d58a";
    ctx.font = "700 10px Arial";
    const err = stateItem && stateItem.error_m != null ? ` ${fmt(stateItem.error_m)}m` : "";
    ctx.fillText(`${target.vehicle}${err}`, tx + 8, ty - 8);
    ctx.setLineDash([5, 4]);
  }
  ctx.restore();
}

function drawObstaclePoints(ctx, points) {
  if (!points || !points.length) return;
  ctx.save();
  ctx.fillStyle = "rgba(225,93,93,0.72)";
  for (const point of points) {
    const [x, y] = mapToCanvas(point.x, point.y);
    ctx.fillRect(x - 1.5, y - 1.5, 3, 3);
  }
  ctx.restore();
}

function drawBlockedZones(ctx, zones) {
  if (!zones || !zones.length) return;
  ctx.save();
  ctx.fillStyle = "rgba(225,93,93,0.22)";
  ctx.strokeStyle = "rgba(225,93,93,0.68)";
  ctx.lineWidth = 1.2;
  for (const zone of zones) {
    const a = mapToCanvas(zone.x_min, zone.y_min);
    const b = mapToCanvas(zone.x_max, zone.y_max);
    const x = Math.min(a[0], b[0]);
    const y = Math.min(a[1], b[1]);
    const w = Math.abs(a[0] - b[0]);
    const h = Math.abs(a[1] - b[1]);
    ctx.fillRect(x, y, w, h);
    ctx.strokeRect(x, y, w, h);
  }
  ctx.restore();
}

function drawPlannerPath(ctx) {
  const planner = state.agentProgress && state.agentProgress.planner;
  const points = planner && planner.waypoints;
  if (!points || points.length < 2) return;
  drawPolyline(ctx, points, "#78aaff", 2.5, []);
  points.forEach((point, index) => {
    if (index % 2 === 0 || index === points.length - 1) {
      drawPoint(ctx, point, "#78aaff", 3.5, "");
    }
  });
}

function drawMapGrid(ctx, size) {
  const step = Math.max(24, state.mapPxPerMeter * 10);
  ctx.strokeStyle = "rgba(120,170,255,0.09)";
  ctx.lineWidth = 1;
  for (let x = 0; x <= size.width; x += step) {
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, size.height); ctx.stroke();
  }
  for (let y = 0; y <= size.height; y += step) {
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(size.width, y); ctx.stroke();
  }
  const origin = mapToCanvas(0, 0);
  ctx.strokeStyle = "rgba(238,245,247,0.22)";
  ctx.beginPath();
  ctx.moveTo(origin[0], 0); ctx.lineTo(origin[0], size.height);
  ctx.moveTo(0, origin[1]); ctx.lineTo(size.width, origin[1]);
  ctx.stroke();
}

function drawSearchArea(ctx, area) {
  const a = mapToCanvas(area.x_min, area.y_min);
  const b = mapToCanvas(area.x_max, area.y_max);
  const x = Math.min(a[0], b[0]);
  const y = Math.min(a[1], b[1]);
  const w = Math.abs(a[0] - b[0]);
  const h = Math.abs(a[1] - b[1]);
  ctx.fillStyle = "rgba(67,199,185,0.08)";
  ctx.strokeStyle = "rgba(67,199,185,0.55)";
  ctx.lineWidth = 1.5;
  ctx.fillRect(x, y, w, h);
  ctx.strokeRect(x, y, w, h);
}

function drawPolyline(ctx, points, color, width, dash) {
  if (!points || points.length < 2) return;
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.setLineDash(dash);
  ctx.beginPath();
  points.forEach((point, index) => {
    const [x, y] = mapToCanvas(point.x, point.y);
    if (index === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();
  ctx.setLineDash([]);
}

function drawPoint(ctx, point, color, radius, label) {
  const [x, y] = mapToCanvas(point.x || 0, point.y || 0);
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(x, y, radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = "#061014";
  ctx.lineWidth = 2;
  ctx.stroke();
  if (label) {
    ctx.fillStyle = "#dce8ec";
    ctx.font = "700 10px Arial";
    ctx.fillText(label, x + radius + 5, y - radius);
  }
}

function drawUav(ctx, point, isLeader = true, index = 0) {
  const base = mapToCanvas(point.x || 0, point.y || 0);
  const x = base[0] + (Number(point.screenOffsetX) || 0);
  const y = base[1] + (Number(point.screenOffsetY) || 0);
  const name = point.name || (isLeader ? "Drone1" : "W" + (index + 1));
  const size = isLeader ? 16 : 14;
  ctx.save();
  ctx.translate(x, y);
  if (isLeader) {
    ctx.fillStyle = "#43c7b9";
    ctx.strokeStyle = "#03110f";
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(0, -size);
    ctx.lineTo(size * 0.8, size * 0.8);
    ctx.lineTo(0, size * 0.4);
    ctx.lineTo(-size * 0.8, size * 0.8);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = "rgba(67,199,185,0.3)";
    ctx.beginPath();
    ctx.arc(0, 0, size * 1.5, 0, Math.PI * 2);
    ctx.fill();
    ctx.beginPath();
    ctx.fillStyle = "#e9fffb";
    ctx.font = "bold 12px Arial";
    ctx.fillText(name, size, -size + 2);
    ctx.font = "10px Arial";
    ctx.fillText("X:" + Math.round(point.x || 0) + " Y:" + Math.round(point.y || 0), size, -size + 14);
  } else {
    ctx.fillStyle = "#f39c12";
    ctx.strokeStyle = "#03110f";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(0, 0, size, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = "#fff4e0";
    ctx.font = "bold 11px Arial";
    ctx.textAlign = "center";
    ctx.fillText(name, 0, 4);
    ctx.font = "9px Arial";
    ctx.fillText("X:" + Math.round(point.x || 0) + " Y:" + Math.round(point.y || 0), 0, size + 12);
    ctx.textAlign = "left";
  }
  ctx.restore();
}

function renderMapStatus() {
  const data = state.map || {};
  const uav = data.uav || { x: 0, y: 0, z: 0 };
  const uavs = data.uavs || [];
  const lidarCount = (data.obstacles || []).length;
  const label = state.selectedVehicle || uav.name || "\u65e0\u4eba\u673a";
  const fleet = uavs.length > 1 ? ` / ${uavs.length}\u67b6` : "";
  el("mapStatus").textContent = `${label}${fleet}: X ${fmt(uav.x)} / Y ${fmt(uav.y)} / Z ${fmt(uav.z)} | \u8f68\u8ff9 ${(data.trail || []).length} | \u96f7\u8fbe\u70b9 ${lidarCount}`;
}

function renderPerception(data) {
  const safety = data.safety || {};
  const vision = data.vision || {};
  const detection = vision.latest_detection || {};
  const detector = vision.detector || {};
  const obstacle = safety.obstacle || {};
  const collision = safety.collision || {};
  const distances = safety.distance_sensors || {};
  const progress = (data.task && data.task.agent_progress) || {};
  const assessment = progress.area_assessment || {};
  const planner = progress.planner || {};
  const dataset = progress.dataset || {};
  const risk = assessment.risk || obstacle.risk_level || "-";
  el("riskText").textContent = `风险 ${riskText(risk)}`;
  el("riskText").className = `risk ${risk}`;
  const plannerState = planner.ok === true ? "ok" : planner.ok === false ? "blocked" : "wait";
  const plannerReason = planner.reason || progress.message || "\u7b49\u5f85\u8def\u5f84\u89c4\u5212";
  el("plannerText").textContent = `A* ${plannerText(plannerState)} | ${plannerReason}`;
  const detections = Array.isArray(detection.detections) ? detection.detections : [];
  const detectorState = detector.loaded ? "\u5df2\u52a0\u8f7d" : detector.available ? "\u5df2\u914d\u7f6e" : "\u672a\u914d\u7f6e";
  el("detectionText").textContent = detection.message
    ? `YOLO ${detectorState} | ${detection.message}`
    : `YOLO ${detectorState} | \u5c1a\u672a\u68c0\u6d4b`;
  const grid = planner.grid || {};
  const lidarPoints = (safety.lidar && safety.lidar.world_points) || (data.map && data.map.obstacles ? data.map.obstacles.length : 0);
  const nearest = obstacle.nearest_distance_m == null ? "-" : `${fmt(obstacle.nearest_distance_m)}m`;
  const frontDistance = distances.front_m == null ? "-" : `${fmt(distances.front_m)}m`;
  const sideDistances = `${distances.left_m == null ? "-" : fmt(distances.left_m)} / ${distances.right_m == null ? "-" : fmt(distances.right_m)}m`;
  const bubbleText = distances.emergency
    ? `\u7d27\u6025\u505c\u6b62 ${directionText(distances.emergency_direction)}`
    : (distances.min_m == null ? "-" : `${directionText(distances.nearest_direction)} ${fmt(distances.min_m)}m`);
  const collisionName = collision.has_collided ? (collision.object_name || "unknown") : "-";
  const assessmentText = assessment.risk
    ? `${riskText(assessment.risk)} / ${assessment.replanned ? `\u5df2\u8c03\u6574 ${assessment.route_hits_before_replan || 0}->${assessment.route_hits || 0}` : `${assessment.route_hits || 0} \u5904\u76f8\u4ea4`}`
    : "-";
  el("sensorList").innerHTML = `
    <div><b>预扫描</b><span>${assessmentText}</span></div>
    <div><b>雷达</b><span>${lidarPoints} 点</span></div>
    <div><b>最近障碍</b><span>${nearest}</span></div>
    <div><b>前向距离</b><span>${frontDistance}</span></div>
    <div><b>左右距离</b><span>${sideDistances}</span></div>
    <div><b>安全气泡</b><span>${bubbleText}</span></div>
    <div><b>前向点</b><span>${obstacle.front_points || 0} 点</span></div>
    <div><b>A* 栅格</b><span>${grid.raw_obstacles || 0}/${grid.inflated_obstacles || 0}</span></div>
    <div><b>建议侧</b><span>${directionText(obstacle.recommended_side)}</span></div>
    <div><b>碰撞</b><span>${collisionName}</span></div>
    <div><b>采集</b><span>${dataset.saved == null ? "-" : `${dataset.saved}/${dataset.max_images || "-"}`}</span></div>
    <div><b>YOLO</b><span>${detections.length} 个目标</span></div>
    <div><b>模型</b><span>${detector.model_path ? detector.model_path.split(/[\\\\/]/).pop() : "-"}</span></div>
  `;
}

function fitMissionMap() {
  const data = state.map || {};
  const points = [];
  if (data.home) points.push(data.home);
  if (data.uav) points.push(data.uav);
  for (const p of data.uavs || []) points.push(p);
  for (const p of data.route || []) points.push(p);
  for (const p of data.trail || []) points.push(p);
  for (const p of data.obstacles || []) points.push(p);
  for (const zone of data.blocked_zones || []) {
    points.push({ x: zone.x_min, y: zone.y_min });
    points.push({ x: zone.x_max, y: zone.y_max });
  }
  const planner = state.agentProgress && state.agentProgress.planner;
  for (const p of (planner && planner.waypoints) || []) points.push(p);
  const formationTargets = state.agentProgress && state.agentProgress.formation_targets;
  for (const p of formationTargets || []) points.push(p);
  if (data.search_area) {
    points.push({ x: data.search_area.x_min, y: data.search_area.y_min });
    points.push({ x: data.search_area.x_max, y: data.search_area.y_max });
  }
  if (!points.length) return;
  const xs = points.map((p) => Number(p.x) || 0);
  const ys = points.map((p) => Number(p.y) || 0);
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);
  const size = mapCanvasSize();
  state.mapViewX = (minX + maxX) / 2;
  state.mapViewY = (minY + maxY) / 2;
  const spanX = Math.max(8, maxX - minX);
  const spanY = Math.max(8, maxY - minY);
  state.mapPxPerMeter = clamp(Math.min(size.height / (spanX + 12), size.width / (spanY + 12)), 1.2, 12);
  drawMissionMap();
}

function setMapFollow(enabled) {
  state.mapFollow = enabled;
  el("mapFollowBtn").classList.toggle("active", enabled);
}

function setConfidence(value) {
  el("confidenceFill").style.width = `${value}%`;
  el("confidenceText").textContent = `${value}%`;
}

function updateClock() {
  el("clockText").textContent = new Date().toLocaleTimeString("zh-CN", { hour12: false });
}

function toast(title, detail = "") {
  const host = el("toastHost");
  const item = document.createElement("div");
  item.className = "toast";
  item.innerHTML = `<b>${title}</b><span>${detail}</span>`;
  host.appendChild(item);
  setTimeout(() => item.remove(), 3200);
}

async function withBusy(button, work) {
  button.classList.add("isBusy");
  const original = button.textContent;
  button.textContent = "\u5904\u7406\u4e2d...";
  try {
    return await work();
  } finally {
    button.textContent = original;
    button.classList.remove("isBusy");
  }
}

async function previewTask(button = el("previewBtn")) {
  await withBusy(button, async () => {
    const preview = await api("/api/task/preview", { text: el("taskInput").value, params: collectTaskParams() });
    state.lastPreview = preview;
    el("intentLabel").textContent = preview.intent;
    renderPlan(preview.plan);
    renderRoute(preview.plan, "idle");
    renderToolPreview(preview);
    if (preview.map) {
      state.map = preview.map;
      state.previewMapActive = true;
      state.agentProgress = null;
      setMapFollow(false);
      fitMissionMap();
      renderMapStatus();
      drawMissionMap();
    }
    const message = preview.preview && preview.preview.message ? preview.preview.message : "\u4efb\u52a1\u8349\u6848\u5df2\u66f4\u65b0";
    toast("\u9884\u89c8\u5df2\u751f\u6210", message);
  });
}

async function runTask(button = el("runBtn")) {
  const input = el("taskInput").value;
  if (!input.trim()) {
    toast("\u9700\u8981\u4efb\u52a1\u63cf\u8ff0", "\u5148\u8f93\u5165\u4e00\u4e2a\u81ea\u7136\u8bed\u8a00\u4efb\u52a1");
    return;
  }
  await withBusy(button, async () => {
    const data = await api("/api/task/run", { text: input, params: collectTaskParams() });
    el("intentLabel").textContent = "\u4efb\u52a1\u5df2\u63d0\u4ea4";
    state.previewMapActive = false;
    render(data);
    toast("\u4efb\u52a1\u5df2\u63d0\u4ea4", "\u81ea\u7136\u8bed\u8a00\u5df2\u901a\u8fc7\u5b89\u5168\u95e8\u8f6c\u5165\u5de5\u5177\u8c03\u7528");
  });
}

async function command(path, message) {
  const data = await api(path, {});
  state.previewMapActive = false;
  render(data);
  toast(message, "\u5b89\u5168\u64cd\u4f5c\u5df2\u8bb0\u5f55\u5230\u4e8b\u4ef6\u6d41");
}

async function flightCommand(path, message, payload = {}) {
  const data = await api(path, payload);
  state.previewMapActive = false;
  render(data);
  toast(message, "\u98de\u63a7\u547d\u4ee4\u5df2\u53d1\u9001");
}

async function detectLatest(button = el("detectBtn")) {
  await withBusy(button, async () => {
    const payload = {
      target: el("targetSelect").value || "object",
      camera: state.selectedCamera || "front_center",
    };
    const data = await api("/api/detect/latest", payload);
    if (data.state) {
      render(data.state);
    }
    const result = data.result || {};
    const count = result.data && Array.isArray(result.data.detections) ? result.data.detections.length : 0;
    toast("\u76ee\u6807\u68c0\u6d4b\u5b8c\u6210", result.message || `\u68c0\u6d4b\u5230 ${count} \u4e2a\u76ee\u6807`);
  });
}

document.querySelectorAll(".templateChip").forEach((button) => {
  button.addEventListener("click", () => {
    const template = button.dataset.template || "";
    applyTemplateParams(template);
    el("taskInput").value = templates[template] || buildTaskTextFromParams();
    document.querySelectorAll(".templateChip").forEach((item) => item.classList.toggle("selected", item === button));
    previewTask().catch((err) => toast("\u9884\u89c8\u5931\u8d25", err.message));
  });
});

["targetSelect", "forwardInput", "widthInput", "altitudeInput", "speedInput", "spacingInput", "formationCountInput", "formationShapeSelect", "preScanInput", "avoidanceInput"].forEach((id) => {
  el(id).addEventListener("change", () => {
    document.querySelectorAll(".templateChip").forEach((item) => item.classList.remove("selected"));
    el("taskInput").value = buildTaskTextFromParams();
    previewTask().catch((err) => toast("\u9884\u89c8\u5931\u8d25", err.message));
  });
});

el("previewBtn").addEventListener("click", () => previewTask().catch((err) => toast("\u9884\u89c8\u5931\u8d25", err.message)));
el("detectBtn").addEventListener("click", () => detectLatest().catch((err) => toast("\u68c0\u6d4b\u5931\u8d25", err.message)));
el("runBtn").addEventListener("click", () => runTask().catch((err) => toast("\u63d0\u4ea4\u5931\u8d25", err.message)));
el("takeoffBtn").addEventListener("click", () => {
  if (window.confirm("\u6267\u884c\u8d77\u98de\uff1f")) {
    flightCommand("/api/flight/takeoff", "\u8d77\u98de\u547d\u4ee4", { altitude_m: 8 }).catch((err) => toast("\u8d77\u98de\u5931\u8d25", err.message));
  }
});
el("hoverBtn").addEventListener("click", () => flightCommand("/api/flight/hover", "\u60ac\u505c\u547d\u4ee4").catch((err) => toast("\u60ac\u505c\u5931\u8d25", err.message)));
el("landBtn").addEventListener("click", () => {
  if (window.confirm("\u6267\u884c\u964d\u843d\uff1f")) {
    flightCommand("/api/flight/land", "\u964d\u843d\u547d\u4ee4").catch((err) => toast("\u964d\u843d\u5931\u8d25", err.message));
  }
});
el("pauseBtn").addEventListener("click", () => command("/api/task/pause", "\u4efb\u52a1\u5df2\u6682\u505c").catch((err) => toast("\u64cd\u4f5c\u5931\u8d25", err.message)));
el("stopBtn").addEventListener("click", () => {
  if (window.confirm("\u7ec8\u6b62\u5f53\u524d\u4efb\u52a1\uff1f")) {
    command("/api/task/stop", "\u4efb\u52a1\u5df2\u7ec8\u6b62").catch((err) => toast("\u64cd\u4f5c\u5931\u8d25", err.message));
  }
});
el("rtlBtn").addEventListener("click", () => {
  if (window.confirm("\u89e6\u53d1\u8fd4\u822a\uff1f")) {
    command("/api/task/rtl", "\u5df2\u89e6\u53d1\u8fd4\u822a").catch((err) => toast("\u64cd\u4f5c\u5931\u8d25", err.message));
  }
});
el("voiceBtn").addEventListener("click", () => {
  state.isListening = !state.isListening;
  el("voiceBtn").textContent = state.isListening ? "\u505c\u6b62\u5f55\u97f3" : "\u8bed\u97f3";
  el("inputState").textContent = state.isListening ? "\u6b63\u5728\u542c\u53d6\u8bed\u97f3..." : "\u6587\u672c / \u8bed\u97f3";
  toast(state.isListening ? "\u8bed\u97f3\u5165\u53e3\u5df2\u6253\u5f00" : "\u8bed\u97f3\u5165\u53e3\u5df2\u5173\u95ed", "\u4e0b\u4e00\u6b65\u53ef\u63a5 Web Speech API \u6216\u540e\u7aef ASR");
});

el("missionMap").addEventListener("mousemove", (event) => {
  const rect = el("missionMap").getBoundingClientRect();
  if (state.mapDragging) {
    const dx = event.clientX - state.mapLastX;
    const dy = event.clientY - state.mapLastY;
    state.mapViewY -= dx / state.mapPxPerMeter;
    state.mapViewX += dy / state.mapPxPerMeter;
    state.mapLastX = event.clientX;
    state.mapLastY = event.clientY;
    drawMissionMap();
  }
  const [x, y] = canvasToMap(event.clientX - rect.left, event.clientY - rect.top);
  el("mapCursor").textContent = `X ${fmt(x)} / Y ${fmt(y)} / ${fmt(state.mapPxPerMeter)} px/m`;
});

el("missionMap").addEventListener("mousedown", (event) => {
  state.mapDragging = true;
  state.mapLastX = event.clientX;
  state.mapLastY = event.clientY;
  setMapFollow(false);
  el("missionMap").classList.add("dragging");
});

window.addEventListener("mouseup", () => {
  state.mapDragging = false;
  el("missionMap").classList.remove("dragging");
});

el("missionMap").addEventListener("wheel", (event) => {
  event.preventDefault();
  const rect = el("missionMap").getBoundingClientRect();
  const cx = event.clientX - rect.left;
  const cy = event.clientY - rect.top;
  const factor = event.deltaY < 0 ? MAP_ZOOM_STEP : 1 / MAP_ZOOM_STEP;
  zoomMissionMap(factor, { x: cx, y: cy });
  setMapFollow(false);
}, { passive: false });

el("mapFollowBtn").addEventListener("click", () => {
  setMapFollow(!state.mapFollow);
  if (state.mapFollow && state.map && state.map.uav) {
    state.mapViewX = Number(state.map.uav.x) || 0;
    state.mapViewY = Number(state.map.uav.y) || 0;
  }
  drawMissionMap();
});

el("mapFitBtn").addEventListener("click", () => {
  setMapFollow(false);
  fitMissionMap();
});

el("mapZoomInBtn").addEventListener("click", () => {
  setMapFollow(false);
  zoomMissionMap(MAP_ZOOM_STEP);
});

el("mapZoomOutBtn").addEventListener("click", () => {
  setMapFollow(false);
  zoomMissionMap(1 / MAP_ZOOM_STEP);
});

el("connectionPill").addEventListener("click", () => {
  api("/api/airsim/reconnect", {})
    .then((data) => {
      render(data);
      toast("\u8fde\u63a5\u5df2\u5237\u65b0", data.connected ? "AirSim \u5728\u7ebf" : (data.airsim_error || "AirSim \u79bb\u7ebf"));
    })
    .catch((err) => toast("\u8fde\u63a5\u5931\u8d25", err.message));
});

el("visionPill").addEventListener("click", () => {
  api("/api/camera/probe", {})
    .then((data) => {
      const ok = data.results.find((item) => item.ok);
      toast(
        ok ? "\u6444\u50cf\u5934\u53ef\u7528" : "\u672a\u627e\u5230\u53ef\u7528\u6444\u50cf\u5934",
        ok ? `${ok.camera}, ${ok.bytes} bytes` : (data.last_error || "Check AirSim camera name and settings.json")
      );
    })
    .catch((err) => toast("\u6444\u50cf\u5934\u63a2\u6d4b\u5931\u8d25", err.message));
});

document.querySelectorAll(".cameraChip").forEach((button) => {
  button.addEventListener("click", () => {
    const camera = button.dataset.camera || "";
    withBusy(button, async () => {
      const data = await api("/api/camera/select", { camera });
      if (!data.ok) {
        throw new Error(data.detail || "\u6444\u50cf\u5934\u5207\u6362\u5931\u8d25");
      }
      state.selectedCamera = "";
      render(data.state);
      toast("\u89c6\u89d2\u5df2\u5207\u6362", camera);
    }).catch((err) => toast("\u6444\u50cf\u5934\u5207\u6362\u5931\u8d25", err.message));
  });
});

el("vehicleSelect").addEventListener("change", () => {
  const vehicle = el("vehicleSelect").value;
  api("/api/vehicle/select", { vehicle })
    .then((data) => {
      if (!data.ok) {
        throw new Error(data.detail || "\u65e0\u4eba\u673a\u5207\u6362\u5931\u8d25");
      }
      state.selectedCamera = "";
      state.previewMapActive = false;
      state.lastPreview = null;
      render(data.state);
      toast("\u65e0\u4eba\u673a\u5df2\u5207\u6362", vehicle);
    })
    .catch((err) => {
      toast("\u65e0\u4eba\u673a\u5207\u6362\u5931\u8d25", err.message);
      api("/api/state").then(render).catch(() => {});
    });
});

function updateMissionLog(data) {
  const progress = data.task && data.task.agent_progress;
  if (!progress || !progress.message) return;
  
  const status = progress.status || "running";
  const message = progress.message;
  const lastLog = state.missionLogs[state.missionLogs.length - 1];
  
  if (lastLog && lastLog.message === message && lastLog.status === status) {
    return;
  }
  
  const now = new Date();
  const timeStr = `${now.getHours().toString().padStart(2, "0")}:${now.getMinutes().toString().padStart(2, "0")}:${now.getSeconds().toString().padStart(2, "0")}`;
  
  state.missionLogs.push({
    time: timeStr,
    message: message,
    status: status,
  });
  
  if (state.missionLogs.length > 50) {
    state.missionLogs.shift();
  }
  
  renderMissionLog();
}

function renderMissionLog() {
  const container = el("missionLog");
  if (!container) return;
  
  container.innerHTML = state.missionLogs.map((log) => `
    <div class="logEntry status-${log.status}">
      <span class="time">${log.time}</span>
      <span class="msg">${log.message}</span>
      <span class="status">${statusLabel(log.status)}</span>
    </div>
  `).join("");
  
  container.scrollTop = container.scrollHeight;
}

function statusLabel(status) {
  const labels = {
    preparing: "\u51c6\u5907",
    arming: "\u89e3\u9501",
    taking_off: "\u542f\u98de",
    forming: "\u7f16\u961f",
    running: "\u8fd0\u884c",
    completed: "\u5b8c\u6210",
    failed: "\u5931\u8d25",
    stopped: "\u505c\u6b62",
    blocked: "\u963b\u65ad",
    recovering: "\u6062\u590d",
  };
  return labels[status] || status;
}

el("clearLogBtn")?.addEventListener("click", () => {
  state.missionLogs = [];
  renderMissionLog();
});

setInterval(() => {
  api("/api/state").then(render).catch((err) => console.warn(err));
}, 1000);
setInterval(() => {
  api("/api/telemetry").then(renderTelemetry).catch((err) => console.warn(err));
}, 200);
setInterval(updateClock, 1000);
window.addEventListener("resize", resizeMissionMap);

api("/api/state").then(render).catch((err) => console.warn(err));
api("/api/telemetry").then(renderTelemetry).catch((err) => console.warn(err));
resizeMissionMap();
