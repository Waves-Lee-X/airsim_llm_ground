"use strict";

const COMMAND_LABELS = {
  arm: "解锁",
  disarm: "加锁",
  takeoff: "起飞",
  hold: "悬停",
  land: "降落",
  rtl: "返航",
};

const COMMAND_CONFIRMATIONS = {
  arm: "飞控将进入解锁状态。请确认桨叶区域安全且当前模式允许解锁。",
  disarm: "飞控将立即加锁。飞行中加锁会造成动力中断。",
  takeoff: "飞行器将解锁并上升到设定相对高度。",
};

const state = {
  apiOnline: false,
  config: null,
  status: null,
  telemetry: null,
  websocket: null,
  websocketRetry: null,
  selectedVehicleId: 1,
  track: [],
  commandRows: [],
  pendingCommand: null,
  lastTelemetryAt: null,
  cameraTimer: null,
};

const elements = {};

function byId(id) {
  return document.getElementById(id);
}

function cacheElements() {
  [
    "apiBadge", "fcuBadge", "armedBadge", "simMode", "realMode",
    "vehicleSelect", "vehicleName", "modeValue", "altitudeValue",
    "batteryValue", "gpsValue", "satellitesValue", "prearmValue",
    "posN", "posE", "posD", "velN", "velE", "velD", "linkDot",
    "linkHealth", "gpsDot", "gpsHealth", "ekfDot", "ekfHealth",
    "homeDot", "homeHealth", "statusText", "nedCanvas", "navEmpty",
    "clearTrack", "trackScale", "commandCount", "evidenceList",
    "controlState", "takeoffAltitude", "altitudeMinus", "altitudePlus",
    "controlHint", "cameraBadge", "cameraFrame", "cameraEmpty",
    "cameraDetail", "cameraResolution", "vlmBadge", "semanticTarget",
    "semanticColor", "semanticConfidence", "runtimeLabel", "lastUpdate",
    "confirmDialog", "confirmTitle", "confirmText", "confirmVehicle",
    "confirmMode", "confirmSubmit",
  ].forEach((id) => {
    elements[id] = byId(id);
  });
  elements.commandButtons = Array.from(document.querySelectorAll("[data-command]"));
}

function valueOr(value, fallback = "--") {
  return value === null || value === undefined || value === "" ? fallback : value;
}

function finiteNumber(value) {
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

function fixed(value, digits = 2) {
  const number = finiteNumber(value);
  return number === null ? "--" : number.toFixed(digits);
}

function booleanLabel(value, yes = "正常", no = "异常") {
  if (value === true) return yes;
  if (value === false) return no;
  return "未知";
}

function setBadge(element, className, text) {
  element.className = `status-badge ${className}`;
  const dot = document.createElement("i");
  element.replaceChildren(dot, document.createTextNode(text));
}

function setHealth(dot, value) {
  dot.className = value === true ? "ok" : value === false ? "bad" : "warn";
}

async function requestJson(path, options = {}) {
  const response = await fetch(path, {
    cache: "no-store",
    headers: { "Content-Type": "application/json", ...(options.headers || {}) },
    ...options,
  });
  let payload = null;
  try {
    payload = await response.json();
  } catch (_error) {
    payload = null;
  }
  if (!response.ok) {
    const detail = payload?.detail || payload?.message || `${response.status} ${response.statusText}`;
    throw new Error(detail);
  }
  return payload;
}

function unwrapPayload(payload) {
  if (!payload || typeof payload !== "object") return {};
  if (payload.data && typeof payload.data === "object") return payload.data;
  if (payload.payload && typeof payload.payload === "object") return payload.payload;
  return payload;
}

function normalizeTelemetry(payload) {
  const root = unwrapPayload(payload);
  const candidate = root.telemetry || root.snapshot || root.vehicle?.telemetry || root;
  if (!candidate || typeof candidate !== "object") return null;
  const health = candidate.health || {};
  const position = candidate.local_position_ned_m || candidate.position_m || candidate.position || null;
  const velocity = candidate.velocity_ned_m_s || candidate.velocity_m_s || candidate.velocity || null;
  const attitude = attitudeTuple(candidate.attitude_rpy_rad || candidate.attitude || null);
  const normalizedPosition = vectorTuple(position);
  const relativeAltitude = candidate.relative_altitude_m ?? (
    normalizedPosition?.[2] === null || normalizedPosition?.[2] === undefined
      ? null
      : Math.max(0, -normalizedPosition[2])
  );
  return {
    fcu_link_ok: valueOr(candidate.fcu_link_ok, health.fcu_link_ok),
    armed: candidate.armed,
    mode: candidate.mode,
    relative_altitude_m: relativeAltitude,
    local_position_ned_m: normalizedPosition,
    velocity_ned_m_s: vectorTuple(velocity),
    attitude_rpy_rad: attitude,
    battery_remaining: candidate.battery_remaining,
    battery_voltage_v: candidate.battery_voltage_v,
    gps_fix_type: valueOr(candidate.gps_fix_type, health.gps_fix_type),
    satellites_visible: candidate.satellites_visible,
    gps_healthy: valueOr(candidate.gps_healthy, health.gps_healthy),
    prearm_ok: valueOr(candidate.prearm_ok, health.prearm_ok),
    ekf_flags: candidate.ekf_flags,
    ekf_ok: valueOr(candidate.ekf_ok, health.ekf_ok),
    landed_state: candidate.landed_state,
    home_position_ned_m: vectorTuple(candidate.home_position_ned_m),
    last_status_text: candidate.last_status_text,
    observed_at_utc: candidate.observed_at_utc || root.observed_at_utc,
    field_ages_s: candidate.field_ages_s || {},
  };
}

function attitudeTuple(value) {
  if (value && typeof value === "object" && "w" in value) {
    const w = finiteNumber(value.w);
    const x = finiteNumber(value.x);
    const y = finiteNumber(value.y);
    const z = finiteNumber(value.z);
    if ([w, x, y, z].some((part) => part === null)) return null;
    const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    const sinPitch = Math.max(-1, Math.min(1, 2 * (w * y - z * x)));
    const pitch = Math.asin(sinPitch);
    const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    return [roll, pitch, yaw];
  }
  return vectorTuple(value);
}

function vectorTuple(value) {
  if (Array.isArray(value) && value.length >= 3) {
    return [finiteNumber(value[0]), finiteNumber(value[1]), finiteNumber(value[2])];
  }
  if (value && typeof value === "object") {
    if ("x" in value || "y" in value || "z" in value) {
      return [finiteNumber(value.x), finiteNumber(value.y), finiteNumber(value.z)];
    }
    if ("north" in value || "east" in value || "down" in value) {
      return [finiteNumber(value.north), finiteNumber(value.east), finiteNumber(value.down)];
    }
  }
  return null;
}

function renderConfig(configPayload) {
  const config = unwrapPayload(configPayload);
  state.config = config;
  state.selectedVehicleId = Number(config.vehicle_id || config.vehicles?.[0]?.vehicle_id || 1);
  const deploymentMode = String(config.deployment_mode || "sim").toLowerCase();
  elements.simMode.classList.toggle("active", deploymentMode === "sim");
  elements.realMode.classList.toggle("active", deploymentMode === "real");

  const vehicles = Array.isArray(config.vehicles)
    ? config.vehicles
    : [{ vehicle_id: state.selectedVehicleId, vehicle_name: config.vehicle_name || "UAV 01" }];
  elements.vehicleSelect.replaceChildren();
  vehicles.forEach((vehicle) => {
    const option = document.createElement("option");
    option.value = String(vehicle.vehicle_id);
    option.textContent = vehicle.vehicle_name || vehicle.name || `UAV ${String(vehicle.vehicle_id).padStart(2, "0")}`;
    elements.vehicleSelect.append(option);
  });
  elements.vehicleSelect.value = String(state.selectedVehicleId);
  elements.vehicleName.textContent = elements.vehicleSelect.selectedOptions[0]?.textContent || "UAV 01";

  const mode = String(config.runtime_mode || "manual").toUpperCase();
  const endpoint = config.fcu_endpoint || "--";
  const ownership = config.external_processes_managed === false ? "SITL / UE 外部启动" : "进程托管";
  elements.runtimeLabel.textContent = `${mode} · ${endpoint} · ${ownership}`;
  renderCameraConfig(config.camera || {});
  updateControlState();
}

function renderCameraConfig(camera) {
  const available = camera.stream_available === true;
  elements.cameraBadge.className = `camera-badge ${available ? "online" : "offline"}`;
  elements.cameraBadge.textContent = available ? "已连接" : "未连接";
  elements.cameraDetail.textContent = camera.detail || camera.name || "AirSim front_center";
  const width = camera.width || 1280;
  const height = camera.height || 720;
  elements.cameraResolution.textContent = `RGB · ${width}×${height}`;
  if (available && camera.stream_url) {
    startCameraStream(camera.stream_url);
  } else {
    stopCameraStream();
  }
}

function startCameraStream(url) {
  stopCameraStream();
  const refresh = () => {
    const separator = url.includes("?") ? "&" : "?";
    elements.cameraFrame.src = `${url}${separator}vehicle_id=${state.selectedVehicleId}&t=${Date.now()}`;
  };
  elements.cameraFrame.onload = () => {
    elements.cameraFrame.hidden = false;
    elements.cameraEmpty.hidden = true;
  };
  elements.cameraFrame.onerror = () => {
    elements.cameraFrame.hidden = true;
    elements.cameraEmpty.hidden = false;
  };
  refresh();
  state.cameraTimer = window.setInterval(refresh, 500);
}

function stopCameraStream() {
  if (state.cameraTimer !== null) window.clearInterval(state.cameraTimer);
  state.cameraTimer = null;
  elements.cameraFrame.removeAttribute("src");
  elements.cameraFrame.hidden = true;
  elements.cameraEmpty.hidden = false;
}

function renderStatus(payload) {
  const status = unwrapPayload(payload);
  state.status = status;
  const runtimeStarted = status.runtime_started ?? status.started ?? true;
  const agentConnected = status.agent_connected ?? status.vehicle_connected ?? status.connected ?? false;
  if (status.telemetry || status.snapshot || "fcu_link_ok" in status) {
    renderTelemetry(normalizeTelemetry(status));
  }
  state.apiOnline = true;
  setBadge(elements.apiBadge, runtimeStarted ? "online" : "pending", runtimeStarted ? "地面站在线" : "地面站启动中");
  if (!agentConnected && !state.telemetry?.fcu_link_ok) {
    setBadge(elements.fcuBadge, "offline", "FCU 离线");
  }
  updateControlState();
}

function renderTelemetry(telemetry) {
  if (!telemetry) return;
  state.telemetry = telemetry;
  state.lastTelemetryAt = new Date();
  const linkOk = telemetry.fcu_link_ok === true;
  const isDemo = state.config?.commands_are_simulated === true;
  setBadge(elements.fcuBadge, linkOk ? "online" : isDemo ? "pending" : "offline", linkOk ? "FCU 在线" : isDemo ? "DEMO 无输出" : "FCU 离线");
  setBadge(elements.armedBadge, telemetry.armed ? "armed" : "neutral", telemetry.armed ? "已解锁" : "未解锁");
  elements.modeValue.textContent = String(valueOr(telemetry.mode, "UNKNOWN")).toUpperCase();
  elements.altitudeValue.textContent = fixed(telemetry.relative_altitude_m, 1);
  elements.batteryValue.textContent = formatBattery(telemetry);
  elements.gpsValue.textContent = telemetry.gps_fix_type === null || telemetry.gps_fix_type === undefined ? "--" : `FIX ${telemetry.gps_fix_type}`;
  elements.satellitesValue.textContent = String(valueOr(telemetry.satellites_visible));
  elements.prearmValue.textContent = booleanLabel(telemetry.prearm_ok, "通过", "未通过");

  renderVector(telemetry.local_position_ned_m, elements.posN, elements.posE, elements.posD);
  renderVector(telemetry.velocity_ned_m_s, elements.velN, elements.velE, elements.velD);

  setHealth(elements.linkDot, linkOk);
  elements.linkHealth.textContent = linkOk ? "正常" : "离线";
  setHealth(elements.gpsDot, telemetry.gps_healthy);
  elements.gpsHealth.textContent = booleanLabel(telemetry.gps_healthy);
  const ekfKnown = telemetry.ekf_flags !== null && telemetry.ekf_flags !== undefined;
  const ekfOk = telemetry.ekf_ok === true
    ? true
    : telemetry.ekf_ok === false
      ? false
      : ekfKnown
        ? Number(telemetry.ekf_flags) > 0
        : null;
  setHealth(elements.ekfDot, ekfOk);
  elements.ekfHealth.textContent = ekfKnown
    ? `0x${Number(telemetry.ekf_flags).toString(16).toUpperCase()}`
    : booleanLabel(telemetry.ekf_ok);
  const homeOk = Array.isArray(telemetry.home_position_ned_m);
  setHealth(elements.homeDot, homeOk ? true : null);
  elements.homeHealth.textContent = homeOk ? "已设置" : "未知";
  elements.statusText.textContent = telemetry.last_status_text || "未收到 STATUSTEXT";
  elements.lastUpdate.textContent = `最后遥测：${state.lastTelemetryAt.toLocaleTimeString("zh-CN", { hour12: false })}`;

  appendTrackPoint(telemetry);
  updateControlState();
}

function formatBattery(telemetry) {
  const remaining = finiteNumber(telemetry.battery_remaining);
  if (remaining !== null) return `${Math.round(remaining * 100)}%`;
  const voltage = finiteNumber(telemetry.battery_voltage_v);
  return voltage === null ? "--" : `${voltage.toFixed(1)} V`;
}

function renderVector(vector, first, second, third) {
  first.textContent = fixed(vector?.[0]);
  second.textContent = fixed(vector?.[1]);
  third.textContent = fixed(vector?.[2]);
}

function appendTrackPoint(telemetry) {
  const position = telemetry.local_position_ned_m;
  if (!position || position[0] === null || position[1] === null) return;
  const last = state.track[state.track.length - 1];
  const current = { north: position[0], east: position[1], yaw: telemetry.attitude_rpy_rad?.[2] || 0 };
  if (!last || Math.hypot(current.north - last.north, current.east - last.east) >= 0.04) {
    state.track.push(current);
    if (state.track.length > 800) state.track.splice(0, state.track.length - 800);
  } else {
    state.track[state.track.length - 1] = current;
  }
  elements.navEmpty.hidden = true;
  drawTrack();
}

function canvasSize() {
  const canvas = elements.nedCanvas;
  const rect = canvas.getBoundingClientRect();
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  const width = Math.max(1, Math.round(rect.width * ratio));
  const height = Math.max(1, Math.round(rect.height * ratio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  return { width, height, ratio };
}

function drawTrack() {
  const canvas = elements.nedCanvas;
  if (!canvas) return;
  const { width, height, ratio } = canvasSize();
  const context = canvas.getContext("2d");
  context.clearRect(0, 0, width, height);
  context.fillStyle = "#0a0e11";
  context.fillRect(0, 0, width, height);

  const maxCoordinate = Math.max(10, ...state.track.flatMap((point) => [Math.abs(point.north), Math.abs(point.east)]));
  const halfRange = maxCoordinate <= 20 ? 20 : maxCoordinate <= 50 ? 50 : maxCoordinate <= 100 ? 100 : Math.ceil(maxCoordinate / 50) * 50;
  elements.trackScale.textContent = `± ${halfRange} m`;
  const scale = Math.min(width, height) * 0.42 / halfRange;
  const centerX = width / 2;
  const centerY = height / 2;

  context.lineWidth = ratio;
  context.strokeStyle = "#222a30";
  const gridStep = halfRange <= 20 ? 5 : halfRange <= 50 ? 10 : 25;
  for (let value = -halfRange; value <= halfRange; value += gridStep) {
    const x = centerX + value * scale;
    const y = centerY - value * scale;
    context.beginPath();
    context.moveTo(x, 0);
    context.lineTo(x, height);
    context.stroke();
    context.beginPath();
    context.moveTo(0, y);
    context.lineTo(width, y);
    context.stroke();
  }
  context.strokeStyle = "#52606a";
  context.beginPath();
  context.moveTo(centerX, 0);
  context.lineTo(centerX, height);
  context.moveTo(0, centerY);
  context.lineTo(width, centerY);
  context.stroke();

  context.fillStyle = "#62cf82";
  context.fillRect(centerX - 2 * ratio, centerY - 2 * ratio, 4 * ratio, 4 * ratio);

  if (!state.track.length) return;
  context.strokeStyle = "#29c7d8";
  context.lineWidth = 2 * ratio;
  context.beginPath();
  state.track.forEach((point, index) => {
    const x = centerX + point.east * scale;
    const y = centerY - point.north * scale;
    if (index === 0) context.moveTo(x, y);
    else context.lineTo(x, y);
  });
  context.stroke();

  const current = state.track[state.track.length - 1];
  const x = centerX + current.east * scale;
  const y = centerY - current.north * scale;
  context.save();
  context.translate(x, y);
  context.rotate(current.yaw || 0);
  context.fillStyle = "#f2f5f7";
  context.beginPath();
  context.moveTo(0, -8 * ratio);
  context.lineTo(6 * ratio, 7 * ratio);
  context.lineTo(0, 4 * ratio);
  context.lineTo(-6 * ratio, 7 * ratio);
  context.closePath();
  context.fill();
  context.restore();
}

function updateControlState() {
  const simulated = state.config?.commands_are_simulated === true;
  const outputEnabled = state.config?.flight_output_enabled === true;
  const agentConnected = state.status?.agent_connected ?? state.status?.vehicle_connected ?? false;
  const fcuReady = state.telemetry?.fcu_link_ok === true;
  const enabled = state.apiOnline && (simulated || (outputEnabled && agentConnected && fcuReady));
  elements.commandButtons.forEach((button) => {
    button.disabled = !enabled;
  });
  elements.altitudeMinus.disabled = !enabled;
  elements.altitudePlus.disabled = !enabled;
  elements.takeoffAltitude.disabled = !enabled;
  elements.controlState.className = `control-state ${enabled ? "ready" : "locked"}`;
  elements.controlState.textContent = enabled ? (simulated ? "DEMO" : "就绪") : "锁定";
  if (simulated && state.apiOnline) {
    elements.controlHint.textContent = "DEMO 模式：命令不会发送到 MAVLink";
  } else if (!state.apiOnline) {
    elements.controlHint.textContent = "等待地面站连接";
  } else if (!agentConnected || !fcuReady) {
    elements.controlHint.textContent = "等待手动启动的 SITL 连接 14550";
  } else {
    elements.controlHint.textContent = "MAVLink 已连接，飞行命令可用";
  }
}

function showCommandConfirmation(command) {
  const label = COMMAND_LABELS[command] || command;
  if (!COMMAND_CONFIRMATIONS[command]) {
    void sendCommand(command);
    return;
  }
  state.pendingCommand = command;
  elements.confirmTitle.textContent = `确认${label}`;
  const altitude = finiteNumber(elements.takeoffAltitude.value) || 3;
  elements.confirmText.textContent = command === "takeoff"
    ? `${COMMAND_CONFIRMATIONS[command]} 当前设定：${altitude.toFixed(1)} m。`
    : COMMAND_CONFIRMATIONS[command];
  elements.confirmVehicle.textContent = elements.vehicleName.textContent;
  elements.confirmMode.textContent = String(state.config?.deployment_mode || "SIM").toUpperCase();
  if (typeof elements.confirmDialog.showModal === "function") {
    elements.confirmDialog.showModal();
  } else if (window.confirm(elements.confirmText.textContent)) {
    void sendCommand(command);
  }
}

async function sendCommand(command) {
  const label = COMMAND_LABELS[command] || command;
  const localId = crypto.randomUUID ? crypto.randomUUID() : `${Date.now()}-${Math.random()}`;
  const row = {
    localId,
    command,
    label,
    createdAt: new Date(),
    application: "pending",
    mavlink: "pending",
    physical: "pending",
    status: "running",
    detail: "命令已提交到地面站",
  };
  state.commandRows.unshift(row);
  state.commandRows = state.commandRows.slice(0, 100);
  renderEvidence();

  const body = command === "takeoff"
    ? { altitude_m: finiteNumber(elements.takeoffAltitude.value) || 3 }
    : {};
  try {
    const response = await requestJson(
      `/api/vehicles/${state.selectedVehicleId}/commands/${command}`,
      { method: "POST", body: JSON.stringify(body) },
    );
    applyCommandResult(row, unwrapPayload(response));
  } catch (error) {
    row.application = "fail";
    row.mavlink = "fail";
    row.physical = "fail";
    row.status = "failed";
    row.detail = error.message;
  }
  renderEvidence();
}

function applyCommandResult(row, payload) {
  const result = payload.result || payload.command_result || payload;
  const application = result.application || result.application_acceptance || {};
  const mavlink = result.fcu_ack || result.mavlink_ack || result.ack || {};
  const physical = result.physical_completion || result.physical || {};
  row.requestId = result.command_id || result.request_id || payload.command_message_id || row.requestId;
  row.application = application.accepted === false ? "fail" : "ok";
  row.mavlink = mavlinkStatus(mavlink, result.simulated === true);
  row.physical = physical.confirmed === true || result.physical_completion_confirmed === true ? "ok" : "fail";
  row.status = String(result.terminal?.status || result.status || payload.status || "completed").toLowerCase();
  row.detail = result.detail || result.terminal?.detail || payload.detail || row.status;
}

function renderEvidence() {
  elements.commandCount.textContent = String(state.commandRows.length);
  if (!state.commandRows.length) {
    elements.evidenceList.innerHTML = '<div class="evidence-empty">暂无命令记录</div>';
    return;
  }
  elements.evidenceList.replaceChildren(...state.commandRows.map(createEvidenceRow));
}

function createEvidenceRow(row) {
  const container = document.createElement("div");
  container.className = "evidence-row";
  container.title = row.detail || "";

  const command = document.createElement("div");
  command.className = "evidence-command";
  const commandName = document.createElement("strong");
  commandName.textContent = row.label;
  const commandTime = document.createElement("small");
  commandTime.textContent = row.createdAt.toLocaleTimeString("zh-CN", { hour12: false });
  command.append(commandName, commandTime);
  container.append(command);
  container.append(
    evidenceStep(row.application, row.application === "ok" ? "已接收" : row.application === "fail" ? "拒绝" : "等待"),
    evidenceStep(row.mavlink, row.mavlink === "ok" ? "已确认" : row.mavlink === "na" ? "不适用" : row.mavlink === "fail" ? "失败" : "等待"),
    evidenceStep(row.physical, row.physical === "ok" ? "已确认" : row.physical === "fail" ? "未完成" : "等待"),
  );
  const result = document.createElement("strong");
  const successful = ["completed", "accepted"].includes(row.status);
  result.className = `result-label ${successful ? "success" : row.status === "running" ? "" : "failed"}`;
  result.textContent = statusLabel(row.status);
  container.append(result);
  return container;
}

function evidenceStep(status, label) {
  const element = document.createElement("span");
  element.className = `evidence-step ${status === "na" ? "" : status}`;
  element.textContent = label;
  return element;
}

function mavlinkStatus(evidence, simulated = false) {
  if (simulated || evidence.applicable === false) return "na";
  if (evidence.received !== true) return "fail";
  const result = finiteNumber(evidence.result);
  return result === null || result === 0 || result === 5 ? "ok" : "fail";
}

function statusLabel(status) {
  const labels = {
    running: "执行中",
    completed: "完成",
    accepted: "已接受",
    application_rejected: "应用拒绝",
    dispatch_failed: "发送失败",
    mavlink_rejected: "飞控拒绝",
    ack_timeout: "ACK 超时",
    physical_timeout: "物理超时",
    expired_before_send: "发送前过期",
    expired_during_execution: "执行中过期",
    preempted: "被抢占",
    fcu_link_lost: "链路丢失",
    link_stopped: "链路停止",
    gateway_timeout: "网关超时",
    failed: "失败",
  };
  return labels[status] || status || "未知";
}

function handleWebsocketMessage(event) {
  let payload;
  try {
    payload = JSON.parse(event.data);
  } catch (_error) {
    return;
  }
  const type = String(
    payload.event_type || payload.type || payload.event || payload.message_type || "",
  ).toLowerCase();
  const body = unwrapPayload(payload);
  if (type === "snapshot") {
    renderStatus(body.status || body);
    if (body.telemetry?.available && body.telemetry.telemetry) {
      renderTelemetry(normalizeTelemetry(body.telemetry));
    } else if (body.telemetry) {
      renderTelemetry(normalizeTelemetry({ telemetry: body.telemetry }));
    }
    return;
  }
  if (type.includes("telemetry") || body.telemetry || body.snapshot) {
    if (body.available !== false) renderTelemetry(normalizeTelemetry(body));
  }
  if (type === "status" || type === "service_status") renderStatus(body);
  if (type === "command_progress") {
    applyCommandProgress(body);
  } else if (type === "command_result") {
    const requestId = body.command_id || body.request_id;
    const row = state.commandRows.find((item) => item.requestId === requestId)
      || state.commandRows.find((item) => item.command === body.command && item.status === "running");
    if (row) {
      applyCommandResult(row, body);
      renderEvidence();
    }
  }
}

function applyCommandProgress(progress) {
  const commandId = progress.command_id || progress.request_id;
  const row = state.commandRows.find((item) => item.requestId === commandId)
    || state.commandRows.find((item) => item.command === progress.command && item.status === "running");
  if (!row) return;
  if (commandId) row.requestId = commandId;
  const stage = String(progress.stage || "").toLowerCase();
  if (stage === "sent") row.application = "pending";
  if (stage === "accepted") row.application = "ok";
  if (stage === "running") {
    row.application = "ok";
    const result = finiteNumber(progress.apm_command_result);
    row.mavlink = progress.simulated
      ? "na"
      : result === null
        ? "pending"
        : result === 0 || result === 5
          ? "ok"
          : "fail";
  }
  if (["completed", "failed", "cancelled", "expired"].includes(stage)) {
    row.application = "ok";
    const result = finiteNumber(progress.apm_command_result);
    row.mavlink = progress.simulated
      ? "na"
      : result === 0 || result === 5
        ? "ok"
        : "fail";
    row.physical = progress.physical_completion_confirmed ? "ok" : "fail";
    row.status = stage;
  }
  row.detail = progress.detail || row.detail;
  renderEvidence();
}

function connectWebsocket() {
  if (state.websocketRetry !== null) window.clearTimeout(state.websocketRetry);
  const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
  const socket = new WebSocket(`${protocol}//${window.location.host}/ws`);
  state.websocket = socket;
  socket.addEventListener("message", handleWebsocketMessage);
  socket.addEventListener("close", () => {
    if (state.websocket === socket) state.websocket = null;
    state.websocketRetry = window.setTimeout(connectWebsocket, 1500);
  });
  socket.addEventListener("error", () => socket.close());
}

async function refreshStatus() {
  try {
    const payload = await requestJson("/api/status");
    renderStatus(payload);
    if (!state.telemetry) {
      try {
        const telemetry = await requestJson(`/api/vehicles/${state.selectedVehicleId}/telemetry`);
        renderTelemetry(normalizeTelemetry(telemetry));
      } catch (_error) {
        // Status remains authoritative while telemetry is not available.
      }
    }
  } catch (error) {
    state.apiOnline = false;
    setBadge(elements.apiBadge, "offline", "地面站离线");
    elements.statusText.textContent = error.message;
    updateControlState();
  }
}

function bindEvents() {
  elements.commandButtons.forEach((button) => {
    button.addEventListener("click", () => showCommandConfirmation(button.dataset.command));
  });
  elements.confirmSubmit.addEventListener("click", () => {
    const command = state.pendingCommand;
    state.pendingCommand = null;
    elements.confirmDialog.close();
    if (command) void sendCommand(command);
  });
  elements.confirmDialog.addEventListener("close", () => {
    state.pendingCommand = null;
  });
  elements.altitudeMinus.addEventListener("click", () => adjustAltitude(-1));
  elements.altitudePlus.addEventListener("click", () => adjustAltitude(1));
  elements.clearTrack.addEventListener("click", () => {
    state.track = [];
    elements.navEmpty.hidden = false;
    drawTrack();
  });
  elements.vehicleSelect.addEventListener("change", () => {
    state.selectedVehicleId = Number(elements.vehicleSelect.value);
    elements.vehicleName.textContent = elements.vehicleSelect.selectedOptions[0]?.textContent || `UAV ${state.selectedVehicleId}`;
    state.track = [];
    void refreshStatus();
  });
  window.addEventListener("resize", drawTrack);
  if (window.ResizeObserver) {
    new ResizeObserver(drawTrack).observe(elements.nedCanvas.parentElement);
  }
}

function adjustAltitude(delta) {
  const input = elements.takeoffAltitude;
  const minimum = finiteNumber(input.min) ?? 1;
  const maximum = finiteNumber(input.max) ?? 20;
  const current = finiteNumber(input.value) ?? 3;
  input.value = String(Math.max(minimum, Math.min(maximum, current + delta)));
}

async function bootstrap() {
  cacheElements();
  bindEvents();
  drawTrack();
  try {
    const [config, status] = await Promise.all([
      requestJson("/api/config"),
      requestJson("/api/status"),
    ]);
    renderConfig(config);
    renderStatus(status);
  } catch (error) {
    state.apiOnline = false;
    setBadge(elements.apiBadge, "offline", "地面站离线");
    elements.statusText.textContent = error.message;
    updateControlState();
  }
  connectWebsocket();
  window.setInterval(refreshStatus, 2000);
}

document.addEventListener("DOMContentLoaded", bootstrap);
