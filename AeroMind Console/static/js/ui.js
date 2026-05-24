import { state, text, templateParams, el, checked, valueOf, fmt, numInput, statusText, riskText, directionText, plannerText, statusLabel, toast, withBusy } from "./common.js";
import { clearedMissionMap, renderMapStatus, drawMissionMap, fitMissionMap, setMapFollow } from "./map.js";
import { update3dData } from "./three_map.js";

export async function api(path, body = null) {
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

export function updateClock() {
  el("clockText").textContent = new Date().toLocaleTimeString("zh-CN", { hour12: false });
}

export function setConfidence(value) {
  el("confidenceFill").style.width = `${value}%`;
  el("confidenceText").textContent = `${value}%`;
}

export function render(data) {
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
    ? (data.airsim_error || data.video.message || "等待摄像头流")
    : `相机 ${data.video.camera_name || data.video.selected_camera_name || "front"} 在线`;
  const visiblePlan = staleTaskView ? [] : (taskStatus === "idle" ? state.lastPlan : (data.task.plan || []));
  el("taskTitle").textContent = staleTaskView ? text.waitTask : data.task.title;
  const currentWp = Number(state.agentProgress && state.agentProgress.current_waypoint_index);
  const totalWp = Number(state.agentProgress && state.agentProgress.total_waypoints);
  const wpText = Number.isFinite(currentWp) && Number.isFinite(totalWp) && totalWp > 0
    ? ` / 航点 ${Math.max(0, Math.round(currentWp))}/${Math.max(0, Math.round(totalWp))}`
    : "";
  el("taskSubline").textContent = staleTaskView ? "等待提交任务" : `任务状态：${statusText(taskStatus)}${wpText}`;
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
  // Force map follow unless the user is actively dragging
  if (!state.mapFollow && !state.mapDragging) {
    setMapFollow(true);
  }
  if (state.mapFollow && state.map && state.map.uav) {
    state.mapViewX = Number(state.map.uav.x) || 0;
    state.mapViewY = Number(state.map.uav.y) || 0;
  }
  renderMapStatus();
  drawMissionMap();
  update3dData(data.map || null);
  updateClock();
  updateMissionLog(data);
}

export function renderOpsOverview(data) {
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

export function renderTelemetry(data) {
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
  // Auto re-enable map follow when the drone is moving (speed-based)
  // or has drifted far from the view center (distance-based fallback).
  // Force map follow unless the user is actively dragging
  if (!state.mapFollow && !state.mapDragging) {
    setMapFollow(true);
  }
  if (state.mapFollow && state.map.uav) {
    state.mapViewX = Number(state.map.uav.x) || 0;
    state.mapViewY = Number(state.map.uav.y) || 0;
  }
  renderMapStatus();
  drawMissionMap();
}

export function renderVideo(video) {
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

export function renderCameraSwitcher(video) {
  const selected = video.selected_camera_name || video.camera_name || "front_center";
  document.querySelectorAll(".cameraChip").forEach((button) => {
    const camera = button.dataset.camera || "";
    button.classList.toggle("active", camera === selected);
  });
}

export function renderVehicleSelector(vehicle) {
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
  select.style.display = vehicles.length > 1 ? "inline-block" : "none";
}

export function renderPlan(plan, status = state.currentStatus, agentProgress = null) {
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

export function renderRoute(plan, status, agentProgress = null) {
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

export function renderToolPreview(preview) {
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
  const toolName = toolCall ? toolCall.tool : "暂无工具映射";
  const status = executable ? (accepted ? "就绪" : "已阻止") : "计划";
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
      <div><b>航点</b><em>${stats.waypoint_count || 0}</em></div>
      <div><b>距离</b><em>${fmt(stats.estimated_distance_m || 0)}m</em></div>
      <div><b>安全</b><em>${reason || "-"}</em></div>
    </div>
    <div class="preflightPanel">
      <div class="preflightTitle">
        <b>飞行前检查</b>
        <span class="${preflight.overall || "caution"}">${String(preflight.overall || "caution").toUpperCase()}</span>
      </div>
      <div class="preflightList">${checksHtml}</div>
    </div>
    <details>
      <summary>工具参数</summary>
      <pre>${JSON.stringify(args, null, 2)}</pre>
    </details>
  `;
}

export function renderEvents(events) {
  const visibleEvents = (events || []).filter((item) => {
    const eventTs = Number(item.ts || 0);
    return !eventTs || eventTs >= state.pageStartedAt;
  });
  if (!visibleEvents.length) {
    el("eventList").innerHTML = `<div class="eventItem">等待任务事件</div>`;
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

export function renderPerception(data) {
  if (!el("riskText") || !el("plannerText") || !el("sensorList")) {
    return;
  }
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
  const plannerReason = planner.reason || progress.message || "等待路径规划";
  el("plannerText").textContent = `A* ${plannerText(plannerState)} | ${plannerReason}`;
  const detections = Array.isArray(detection.detections) ? detection.detections : [];
  const detectorState = detector.loaded ? "已加载" : detector.available ? "已配置" : "未配置";
  el("detectionText").textContent = detection.message
    ? `YOLO ${detectorState} | ${detection.message}`
    : `YOLO ${detectorState} | 尚未检测`;
  const grid = planner.grid || {};
  const lidarPoints = (safety.lidar && safety.lidar.world_points) || (data.map && data.map.obstacles ? data.map.obstacles.length : 0);
  const nearest = obstacle.nearest_distance_m == null ? "-" : `${fmt(obstacle.nearest_distance_m)}m`;
  const frontDistance = distances.front_m == null ? "-" : `${fmt(distances.front_m)}m`;
  const sideDistances = `${distances.left_m == null ? "-" : fmt(distances.left_m)} / ${distances.right_m == null ? "-" : fmt(distances.right_m)}m`;
  const bubbleText = distances.emergency
    ? `紧急停止 ${directionText(distances.emergency_direction)}`
    : (distances.min_m == null ? "-" : `${directionText(distances.nearest_direction)} ${fmt(distances.min_m)}m`);
  const collisionName = collision.has_collided ? (collision.object_name || "unknown") : "-";
  const assessmentText = assessment.risk
    ? `${riskText(assessment.risk)} / ${assessment.replanned ? `已调整 ${assessment.route_hits_before_replan || 0}->${assessment.route_hits || 0}` : `${assessment.route_hits || 0} 处相交`}`
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

export function collectTaskParams() {
  const forward = numInput("forwardInput", 60);
  const halfWidth = numInput("widthInput", 20);
  return {
    target: valueOf("targetSelect", "object"),
    altitude_m: numInput("altitudeInput", 8),
    speed_mps: numInput("speedInput", 2),
    spacing_m: numInput("spacingInput", 10),
    count: Math.max(2, Math.min(6, Math.round(numInput("formationCountInput", 3)))),
    shape: valueOf("formationShapeSelect", "v"),
    formation_spacing_m: numInput("spacingInput", 10),
    avoidance: checked("avoidanceInput", true),
    planned_avoidance: checked("avoidanceInput", true),
    pre_scan: checked("preScanInput", true),
    pre_scan_stop_on_high_risk: checked("stopOnRiskInput", true),
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

export function applyTemplateParams(name) {
  const params = templateParams[name];
  if (!params) return;
  if (el("targetSelect")) el("targetSelect").value = params.target;
  if (el("forwardInput")) el("forwardInput").value = params.forward_m;
  if (el("widthInput")) el("widthInput").value = params.half_width_m;
  if (el("altitudeInput")) el("altitudeInput").value = params.altitude_m;
  if (el("speedInput")) el("speedInput").value = params.speed_mps;
  if (el("spacingInput")) el("spacingInput").value = params.spacing_m;
  if (el("formationCountInput")) el("formationCountInput").value = params.count;
  if (el("formationShapeSelect")) el("formationShapeSelect").value = params.shape;
  if (el("preScanInput")) el("preScanInput").checked = params.pre_scan;
  if (el("avoidanceInput")) el("avoidanceInput").checked = params.planned_avoidance;
  if (el("scanMarginInput")) el("scanMarginInput").value = params.scan_margin_m;
  if (el("obstacleDistanceInput")) el("obstacleDistanceInput").value = params.obstacle_distance_m;
  if (el("avoidanceOffsetInput")) el("avoidanceOffsetInput").value = params.avoidance_offset_m;
  if (el("stopOnRiskInput")) el("stopOnRiskInput").checked = params.pre_scan_stop_on_high_risk;
  // Toggle formation / advanced fields visibility
  const isFormation = name === "formation";
  document.querySelectorAll(".formationOnly").forEach((el) => {
    el.style.display = isFormation ? "" : "none";
  });
  document.querySelectorAll(".advancedParam").forEach((el) => {
    el.style.display = isFormation ? "" : "none";
  });
}

export function buildTaskTextFromParams() {
  const params = collectTaskParams();
  const targetNames = {
    vehicle: "车辆",
    person: "行人",
    building: "建筑",
    obstacle: "障碍物",
    object: "目标",
  };
  return `搜索前方 ${fmt(params.area.x_max, 0)} 米左右 ${fmt(params.area.y_max, 0)} 米范围内的${targetNames[params.target] || "目标"}，高度 ${fmt(params.altitude_m, 0)} 米`;
}

export async function previewTask(button = el("previewBtn")) {
  await withBusy(button, async () => {
    const preview = await api("/api/task/preview", { text: el("taskInput").value, params: collectTaskParams() });
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
    const message = preview.preview && preview.preview.message ? preview.preview.message : "任务草稿已更新";
    toast("预览已生成", message);
  });
}

export async function runTask(button = el("runBtn")) {
  const input = el("taskInput").value;
  if (!input.trim()) {
    toast("需要任务描述", "先输入一个自然语言任务");
    return;
  }
  await withBusy(button, async () => {
    const data = await api("/api/task/run", { text: input, params: collectTaskParams() });
    el("intentLabel").textContent = "任务已提交";
    state.previewMapActive = false;
    render(data);
    toast("任务已提交", "自然语言已通过安全门转入工具调用");
  });
}

export async function command(path, message) {
  const data = await api(path, {});
  state.previewMapActive = false;
  render(data);
  toast(message, "安全操作已记录到事件流");
}

export async function flightCommand(path, message, payload = {}) {
  const data = await api(path, payload);
  state.previewMapActive = false;
  render(data);
  toast(message, "飞控命令已发送");
}

export async function detectLatest(button = el("detectBtn")) {
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
    toast("目标检测完成", result.message || `检测到 ${count} 个目标`);
  });
}

export function updateMissionLog(data) {
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

export function renderMissionLog() {
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
