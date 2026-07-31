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
  situationTrails: {},
  situationPayload: null,
  fleetStatus: null,
  situationView: null,
  situationAutoView: null,
  situationDrag: null,
  commandRows: [],
  pendingCommand: null,
  pendingAgentDraft: null,
  lastTelemetryAt: null,
  cameraTimer: null,
  cameraStreamUrl: null,
  serialPorts: [],
  serialApplying: false,
  semanticBusy: false,
  semanticMode: "vision",
  semanticView: "structured",
  semanticLatest: null,
  semanticResults: { vision: null, mission: null },
  agentSessionId: null,
  agentDraft: null,
  agentReplyKeys: new Set(),
  georeference: null,
  geoApplying: false,
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
    "semanticColor", "semanticConfidence", "semanticRisk", "visualSummary",
    "visionAnalysisForm", "visionPrompt", "analyzeVision", "missionParseForm",
    "missionInstruction", "parseMission", "semanticLatency", "semanticResult",
    "visionWorkspace", "missionWorkspace", "visionFrameState", "visionRequestState",
    "missionRequestState", "missionSummary", "semanticResultTitle",
    "semanticConfidenceBar", "semanticRiskCell",
    "visionCrosscheck", "visionCrossStatus",
    "visionVlmColor", "visionOpenCvColor", "visionAgreement",
    "visionConsensus", "visionEvidenceList",
    "fleetPhase", "fleetApply", "fleetLeaderX",
    "fleetLeaderY", "fleetLeaderZ", "fleetSlots",
    "fleetAltitude", "fleetHold", "fleetExecute", "fleetSequenceDemo", "fleetCancel",
    "fleetMissionPhase", "fleetMissionVehicles",
    "gotoX", "gotoY", "gotoZ", "gotoFly",
    "situationMap", "situationEmpty", "situationLegend", "situationStamp",
    "situationZoomIn", "situationZoomOut", "situationZoomReset",
    "agentVehicleState", "agentLinkState", "agentGpsState",
    "agentPermissionState", "resetAgentSession", "agentConversation",
    "agentDraft", "agentDraftAction", "agentDraftStatus",
    "agentDraftBlockers", "cancelAgentDraft", "confirmAgentDraft",
    "runtimeLabel", "linkDiagnostics", "lastUpdate",
    "p9Badge", "agentBadge",
    "confirmDialog", "confirmTitle", "confirmText", "confirmVehicle",
    "confirmMode", "confirmSubmit",
    "serialSettingsButton", "serialDialog", "serialForm", "serialPort",
    "serialPortOptions", "serialBaudrate", "refreshSerialPorts",
    "serialPortList", "serialSettingsStatus", "serialCancel", "serialApply",
    "geoSettingsButton", "geoDialog", "geoForm", "geoCalibrationId",
    "geoStatus", "geoHeading", "geoAltitudeDatum", "geoMapLatitude",
    "geoMapLongitude", "geoMapAltitude", "geoAirSimLatitude",
    "geoAirSimLongitude", "geoAirSimAltitude", "geoHomes", "geoAddHome",
    "geoNotes", "geoCurrentStatus", "geoCurrentHash", "geoRoundTrip",
    "geoSettingsStatus", "geoCancel", "geoApply",
  ].forEach((id) => {
    elements[id] = byId(id);
  });
  elements.commandButtons = Array.from(document.querySelectorAll("[data-command]"));
  elements.semanticTabs = Array.from(document.querySelectorAll("[data-semantic-tab]"));
  elements.semanticModes = Array.from(document.querySelectorAll("[data-semantic-mode]"));
  elements.fleetFormations = Array.from(document.querySelectorAll("[data-fleet-formation]"));
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
    gps_hdop: valueOr(candidate.gps_hdop, health.gps_hdop),
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

function configuredVehicles() {
  if (Array.isArray(state.config?.vehicles) && state.config.vehicles.length) {
    return state.config.vehicles;
  }
  return state.config ? [state.config] : [];
}

function selectedVehicleConfig() {
  return configuredVehicles().find(
    (vehicle) => Number(vehicle.vehicle_id) === Number(state.selectedVehicleId),
  ) || state.config || {};
}

function selectedDeploymentMode() {
  return String(selectedVehicleConfig().deployment_mode || "sim").toLowerCase();
}

function renderSelectedVehicleConfig() {
  const config = selectedVehicleConfig();
  const deploymentMode = selectedDeploymentMode();
  elements.simMode.classList.toggle("active", deploymentMode === "sim");
  elements.realMode.classList.toggle("active", deploymentMode === "real");
  elements.p9Badge.hidden = deploymentMode !== "real";
  elements.agentBadge.hidden = deploymentMode !== "real";
  const mode = String(config.runtime_mode || "manual").toUpperCase();
  const endpoint = deploymentMode === "real"
    ? `${config.ground_serial_port || "--"} @ ${config.ground_serial_baudrate || "--"}`
    : config.fcu_endpoint || "--";
  const ownership = deploymentMode === "real"
    ? "机载代理外部运行"
    : config.external_processes_managed === false
      ? "SITL / UE 外部启动"
      : "进程托管";
  elements.runtimeLabel.textContent = `${mode} · ${endpoint} · ${ownership}`;
  renderCameraConfig(config.camera || {});
}

function renderConfig(configPayload) {
  const config = unwrapPayload(configPayload);
  const previousVehicleId = state.selectedVehicleId;
  state.config = config;
  elements.serialSettingsButton.hidden = config.serial_runtime_configurable !== true;
  if (config.ground_serial_port) elements.serialPort.value = config.ground_serial_port;
  if (config.ground_serial_baudrate) {
    elements.serialBaudrate.value = String(config.ground_serial_baudrate);
  }

  const vehicles = Array.isArray(config.vehicles)
    ? config.vehicles
    : [{ ...config, vehicle_id: config.vehicle_id || 1 }];
  const configuredIds = vehicles.map((vehicle) => Number(vehicle.vehicle_id));
  state.selectedVehicleId = configuredIds.includes(Number(previousVehicleId))
    ? Number(previousVehicleId)
    : Number(config.vehicle_id || vehicles[0]?.vehicle_id || 1);
  elements.vehicleSelect.replaceChildren();
  vehicles.forEach((vehicle) => {
    const option = document.createElement("option");
    option.value = String(vehicle.vehicle_id);
    const deploymentMode = String(vehicle.deployment_mode || "sim").toLowerCase();
    const typeLabel = deploymentMode === "real" ? "实机" : "仿真";
    const name = vehicle.vehicle_name || vehicle.name
      || `UAV ${String(vehicle.vehicle_id).padStart(2, "0")}`;
    option.textContent = `[${typeLabel}] ${name}`;
    elements.vehicleSelect.append(option);
  });
  elements.vehicleSelect.value = String(state.selectedVehicleId);
  elements.vehicleName.textContent = elements.vehicleSelect.selectedOptions[0]?.textContent || "UAV 01";
  renderSelectedVehicleConfig();
  renderSemanticStatus(config.semantic || {});
  updateControlState();
}

function renderCameraConfig(camera) {
  const available = camera.stream_available === true;
  const cameraState = String(camera.state || (available ? "online" : "offline"));
  elements.cameraBadge.className = `camera-badge ${available ? "online" : "offline"}`;
  elements.cameraBadge.textContent = available
    ? cameraState === "degraded" ? "缓存画面" : "已连接"
    : cameraState === "connecting" ? "连接中" : "未连接";
  const metrics = [];
  if (Number.isFinite(Number(camera.source_fps))) metrics.push(`${Number(camera.source_fps).toFixed(1)} FPS`);
  if (Number.isFinite(Number(camera.frame_age_s))) metrics.push(`帧龄 ${(Number(camera.frame_age_s) * 1000).toFixed(0)} ms`);
  elements.cameraDetail.textContent = [camera.detail || camera.name || "AirSim front_center", ...metrics].join(" · ");
  const width = camera.width || 1280;
  const height = camera.height || 720;
  elements.cameraResolution.textContent = `RGB · ${width}×${height}`;
  const frameAge = finiteNumber(camera.frame_age_s);
  elements.visionFrameState.textContent = available
    ? frameAge === null ? "LATEST RGB" : `LATEST · ${(frameAge * 1000).toFixed(0)} MS`
    : "NO FRAME";
  if (available && camera.stream_url) {
    startCameraStream(camera.stream_url);
  } else {
    stopCameraStream();
  }
}

function startCameraStream(url) {
  if (state.cameraStreamUrl === url) return;
  stopCameraStream();
  state.cameraStreamUrl = url;
  const refresh = () => {
    state.cameraTimer = null;
    if (state.cameraStreamUrl !== url) return;
    const separator = url.includes("?") ? "&" : "?";
    elements.cameraFrame.src = `${url}${separator}vehicle_id=${state.selectedVehicleId}&t=${Date.now()}`;
  };
  const schedule = (delayMs) => {
    if (state.cameraStreamUrl !== url) return;
    if (state.cameraTimer !== null) window.clearTimeout(state.cameraTimer);
    state.cameraTimer = window.setTimeout(refresh, delayMs);
  };
  elements.cameraFrame.onload = () => {
    elements.cameraFrame.hidden = false;
    elements.cameraEmpty.hidden = true;
    schedule(50);
  };
  elements.cameraFrame.onerror = () => {
    elements.cameraFrame.hidden = true;
    elements.cameraEmpty.hidden = false;
    schedule(500);
  };
  schedule(0);
}

function stopCameraStream() {
  if (state.cameraTimer !== null) window.clearTimeout(state.cameraTimer);
  state.cameraTimer = null;
  state.cameraStreamUrl = null;
  elements.cameraFrame.onload = null;
  elements.cameraFrame.onerror = null;
  elements.cameraFrame.removeAttribute("src");
  elements.cameraFrame.hidden = true;
  elements.cameraEmpty.hidden = false;
}

function renderSemanticStatus(payload) {
  const semantic = payload && typeof payload === "object" ? payload : {};
  const visionAvailable = semantic.vision_available === true;
  const missionAvailable = semantic.agent_available === true
    || semantic.mission_parser_available === true;
  const busy = semantic.busy === true || state.semanticBusy;
  const available = visionAvailable || missionAvailable;
  elements.vlmBadge.className = `camera-badge ${available ? busy ? "offline" : "online" : "unavailable"}`;
  elements.vlmBadge.textContent = available
    ? busy ? "分析中" : semantic.vision_model || semantic.mission_model || "已配置"
    : "未配置";
  const cameraAvailable = state.status?.camera?.stream_available
    ?? selectedVehicleConfig().camera?.stream_available
    ?? false;
  elements.analyzeVision.disabled = busy || !visionAvailable || !cameraAvailable;
  elements.parseMission.disabled = busy || !missionAvailable;
}

function semanticResultRow(label, value) {
  const row = document.createElement("div");
  row.className = "semantic-result-row";
  const key = document.createElement("span");
  key.textContent = label;
  const content = document.createElement("span");
  content.textContent = semanticDisplayValue(value);
  row.append(key, content);
  return row;
}

function semanticDisplayValue(value) {
  if (value === null || value === undefined || value === "") return "--";
  if (typeof value === "string") return value;
  if (Array.isArray(value)) {
    if (!value.length) return "--";
    return value.map((item) => typeof item === "string" ? item : JSON.stringify(item)).join(" · ");
  }
  return JSON.stringify(value, null, 2);
}

function setSemanticMode(mode) {
  if (!['vision', 'mission'].includes(mode)) return;
  state.semanticMode = mode;
  elements.visionWorkspace.hidden = mode !== "vision";
  elements.missionWorkspace.hidden = mode !== "mission";
  elements.semanticModes.forEach((button) => {
    const active = button.dataset.semanticMode === mode;
    button.classList.toggle("active", active);
    button.setAttribute("aria-selected", String(active));
  });
  state.semanticLatest = state.semanticResults[mode];
  elements.semanticResultTitle.textContent = mode === "vision" ? "识别结果" : "Agent 状态";
  const current = state.semanticLatest;
  elements.semanticLatency.textContent = current
    ? `${current.model || (mode === "vision" ? "VLM" : "LLM")} · ${fixed(current.latency_ms, 0)} ms`
    : "--";
  renderSemanticResult();
}

function renderSemanticResult() {
  const payload = state.semanticLatest;
  if (!payload) {
    const empty = document.createElement("span");
    empty.className = "semantic-empty";
    empty.textContent = state.semanticMode === "vision" ? "尚无视觉识别结果" : "尚无 Agent 对话结果";
    elements.semanticResult.replaceChildren(empty);
    return;
  }
  if (state.semanticView === "raw") {
    elements.semanticResult.textContent = payload.raw_response || "模型原文不可用";
    return;
  }
  const list = document.createElement("div");
  list.className = "semantic-result-list";
  if (payload.kind === "visual_analysis") {
    const result = payload.result || {};
    const target = result.target || {};
    list.append(
      semanticResultRow("场景", result.scene),
      semanticResultRow("目标证据", target.evidence),
      semanticResultRow("标识面", target.face),
      semanticResultRow("对象", result.objects),
      semanticResultRow("建议", result.suggestion),
      semanticResultRow("任务关联", result.mission_relevance),
      semanticResultRow("格式", result.format_warning || "JSON 已校验"),
    );
  } else if (payload.kind === "mission_agent_reply") {
    list.append(
      semanticResultRow("状态摘要", payload.state_summary),
      semanticResultRow("风险等级", payload.risk_level),
      semanticResultRow("待确认问题", payload.questions),
      semanticResultRow("工具草案", payload.draft?.action),
      semanticResultRow("门禁结果", payload.draft?.blockers),
      semanticResultRow("执行策略", "operator_confirmation_required"),
    );
  } else {
    const plan = payload.plan || {};
    list.append(
      semanticResultRow("任务摘要", plan.summary),
      semanticResultRow("任务意图", plan.intent),
      semanticResultRow("无人机", plan.vehicle_ids),
      semanticResultRow("目标", plan.targets),
      semanticResultRow("约束", plan.constraints),
      semanticResultRow("安全检查", plan.safety_checks),
      semanticResultRow("待确认", plan.ambiguities),
      semanticResultRow("执行策略", plan.execution_policy || "preview_only"),
    );
    const fleetPlan = plan.fleet_plan;
    if (fleetPlan) {
      const formationLabels = { line: "一字", v: "V 字", diamond: "正方形" };
      list.append(
        semanticResultRow("编队计划", formationLabels[fleetPlan.formation] || fleetPlan.formation),
        semanticResultRow("领机目标", Array.isArray(fleetPlan.leader_target_map_m)
          ? fleetPlan.leader_target_map_m.map((value) => fixed(value, 1)).join(", ")
          : "--"),
        semanticResultRow("间距 / 高度", `${fleetPlan.spacing_m ?? "--"} m / ${fleetPlan.altitude_m ?? "--"} m`),
        semanticResultRow("保持时间", `${fleetPlan.hold_s ?? "--"} s`),
        semanticResultRow("参与飞机", Array.isArray(fleetPlan.vehicle_ids)
          ? fleetPlan.vehicle_ids.join(", ")
          : "--"),
      );
    }
    const steps = document.createElement("div");
    steps.className = "mission-steps";
    (Array.isArray(plan.steps) ? plan.steps : []).forEach((step, index) => {
      const item = document.createElement("div");
      item.className = "mission-step";
      const order = step.order ?? index + 1;
      const vehicle = step.vehicle_id ? ` · UAV ${step.vehicle_id}` : "";
      const target = step.target ? ` · ${typeof step.target === "string" ? step.target : JSON.stringify(step.target)}` : "";
      const marker = document.createElement("b");
      marker.textContent = String(order);
      const detail = document.createElement("span");
      detail.textContent = `${step.action || "待定义"}${vehicle}${target}`;
      item.append(marker, detail);
      steps.append(item);
    });
    if (steps.childElementCount) list.append(steps);
  }
  elements.semanticResult.replaceChildren(list);
}

function renderVisualSemantic(payload) {
  if (!payload || payload.kind !== "visual_analysis") return;
  state.semanticResults.vision = payload;
  const result = payload.result || {};
  const target = result.target || {};
  elements.semanticTarget.textContent = target.found
    ? target.box_id || target.label || "已发现"
    : "未发现";
  elements.semanticColor.textContent = target.color || "--";
  const confidence = finiteNumber(target.confidence);
  elements.semanticConfidence.textContent = confidence === null
    ? "--"
    : `${(confidence * 100).toFixed(1)}%`;
  elements.semanticConfidenceBar.style.width = confidence === null
    ? "0%"
    : `${Math.max(0, Math.min(100, confidence * 100))}%`;
  elements.semanticRisk.textContent = result.risk_level || "--";
  elements.semanticRiskCell.dataset.level = String(result.risk_level || "unknown").toLowerCase();
  renderVisionCrossValidation(payload.cross_validation);
  renderVisionConsensus(payload.consensus);
  void refreshVisionEvidence();
  void refreshFleetStatus();
  elements.visualSummary.textContent = [result.message, result.scene, result.suggestion]
    .filter(Boolean)
    .join(" · ") || "视觉分析已完成";
  elements.visionRequestState.textContent = "分析完成";
  if (state.semanticMode === "vision") {
    state.semanticLatest = payload;
    elements.semanticLatency.textContent = `${payload.model || "VLM"} · ${fixed(payload.latency_ms, 0)} ms`;
    elements.semanticResultTitle.textContent = "识别结果";
    renderSemanticResult();
  }
}

function renderMissionSemantic(payload) {
  if (!payload || payload.kind !== "mission_preview") return;
  state.semanticResults.mission = payload;
  const plan = payload.plan || {};
  elements.missionSummary.textContent = plan.summary || "任务草案解析完成";
  elements.missionRequestState.textContent = "草案已生成";
  if (state.semanticMode === "mission") {
    state.semanticLatest = payload;
    elements.semanticLatency.textContent = `${payload.model || "LLM"} · ${fixed(payload.latency_ms, 0)} ms`;
    elements.semanticResultTitle.textContent = "任务草案";
    renderSemanticResult();
  }
}

function renderAgentContext() {
  const telemetry = state.telemetry || {};
  const health = telemetry.health || {};
  const status = state.status || {};
  const vehicleId = status.vehicle_id || state.selectedVehicleId || "--";
  const vehicleLabel = vehicleId === "--" ? "--" : String(vehicleId).padStart(2, "0");
  elements.agentVehicleState.textContent = `UAV ${vehicleLabel} · ${telemetry.armed ? "ARM" : "SAFE"}`;
  const agentOnline = status.onboard_agent_connected
    ?? status.agent_connected
    ?? status.vehicle_connected
    ?? false;
  const linkLabel = selectedDeploymentMode() === "real" ? "P9 / FCU" : "SITL / FCU";
  elements.agentLinkState.textContent = agentOnline && telemetry.fcu_link_ok === true
    ? `${linkLabel} 正常`
    : agentOnline ? "FCU 等待" : "Agent 离线";
  const fix = finiteNumber(telemetry.gps_fix_type ?? health.gps_fix_type);
  const hdop = finiteNumber(telemetry.gps_hdop ?? health.gps_hdop);
  elements.agentGpsState.textContent = fix === null
    ? "未知"
    : `FIX ${fix}${hdop === null ? "" : ` · ${hdop.toFixed(2)}`}`;
  const simulated = selectedVehicleConfig().commands_are_simulated === true;
  const allowed = Array.isArray(status.allowed_commands) ? status.allowed_commands : [];
  elements.agentPermissionState.textContent = simulated
    ? "DEMO"
    : allowed.length ? allowed.map((item) => COMMAND_LABELS[item] || item).join(" / ") : "只读";
}

function agentDraftStatusLabel(status) {
  const labels = {
    pending_confirmation: "等待人工确认",
    blocked: "门禁阻断",
    executing: "执行中",
    completed: "已完成",
    failed: "执行失败",
    cancelled: "已取消",
    expired: "已过期",
  };
  return labels[status] || status || "未知";
}

function renderAgentDraft(draft) {
  state.agentDraft = draft || null;
  if (!draft) {
    elements.agentDraft.hidden = true;
    return;
  }
  elements.agentDraft.hidden = false;
  const label = COMMAND_LABELS[draft.action] || draft.action || "未知动作";
  const altitude = finiteNumber(draft.arguments?.altitude_m);
  elements.agentDraftAction.textContent = altitude === null
    ? label
    : `${label} ${altitude.toFixed(1)} m`;
  elements.agentDraftStatus.textContent = agentDraftStatusLabel(draft.status);
  elements.agentDraft.dataset.status = draft.status || "unknown";
  elements.missionSummary.textContent = draft.reason || "Mission Agent 工具草案";
  const blockers = Array.isArray(draft.blockers) ? draft.blockers : [];
  elements.agentDraftBlockers.replaceChildren(...blockers.map((blocker) => {
    const item = document.createElement("span");
    item.textContent = blocker;
    return item;
  }));
  elements.agentDraftBlockers.hidden = blockers.length === 0;
  elements.confirmAgentDraft.disabled = draft.executable !== true;
  elements.cancelAgentDraft.disabled = draft.status !== "pending_confirmation";
}

function createAgentMessage(role, content, payload = null) {
  const row = document.createElement("div");
  row.className = `agent-message ${role}`;
  const heading = document.createElement("div");
  const identity = document.createElement("strong");
  identity.textContent = role === "user" ? "操作员" : role === "assistant" ? "Mission Agent" : "系统";
  const meta = document.createElement("span");
  meta.textContent = payload?.risk_level && payload.risk_level !== "unknown"
    ? `风险 ${String(payload.risk_level).toUpperCase()}`
    : "";
  heading.append(identity, meta);
  const text = document.createElement("p");
  text.textContent = content || "--";
  row.append(heading, text);
  return row;
}

function appendAgentMessage(role, content, payload = null) {
  const empty = elements.agentConversation.querySelector(".agent-empty");
  if (empty) empty.remove();
  elements.agentConversation.append(createAgentMessage(role, content, payload));
  elements.agentConversation.scrollTop = elements.agentConversation.scrollHeight;
}

function renderAgentSession(session) {
  elements.agentConversation.replaceChildren();
  const turns = Array.isArray(session?.turns) ? session.turns : [];
  turns.forEach((turn) => appendAgentMessage(turn.role, turn.content, turn.payload));
  if (!turns.length) {
    const empty = document.createElement("div");
    empty.className = "agent-empty";
    empty.textContent = "尚无对话";
    elements.agentConversation.append(empty);
  }
  const latestAssistant = [...turns].reverse().find((turn) => turn.role === "assistant");
  if (latestAssistant) {
    const payload = {
      kind: "mission_agent_reply",
      session_id: session.session_id,
      created_at_utc: latestAssistant.created_at_utc,
      reply: latestAssistant.content,
      ...(latestAssistant.payload || {}),
    };
    state.semanticResults.mission = payload;
    if (state.semanticMode === "mission") {
      state.semanticLatest = payload;
      elements.semanticResultTitle.textContent = "Agent 状态";
      renderSemanticResult();
    }
  }
  renderAgentDraft(session?.latest_draft || null);
}

function renderAgentReply(payload) {
  if (!payload || payload.kind !== "mission_agent_reply") return;
  const key = `${payload.session_id || "session"}:${payload.created_at_utc || payload.latency_ms}`;
  if (state.agentReplyKeys.has(key)) return;
  state.agentReplyKeys.add(key);
  if (payload.user_message) appendAgentMessage("user", payload.user_message);
  appendAgentMessage("assistant", payload.reply, {
    risk_level: payload.risk_level,
  });
  renderAgentDraft(payload.draft || null);
  state.semanticResults.mission = payload;
  if (state.semanticMode === "mission") {
    state.semanticLatest = payload;
    elements.semanticLatency.textContent = `${payload.model || "LLM"} · ${fixed(payload.latency_ms, 0)} ms`;
    elements.semanticResultTitle.textContent = "Agent 状态";
    renderSemanticResult();
  }
  elements.missionRequestState.textContent = payload.draft
    ? agentDraftStatusLabel(payload.draft.status)
    : "回复完成";
}

function persistAgentSession(sessionId) {
  state.agentSessionId = sessionId || null;
  try {
    if (sessionId) window.sessionStorage.setItem("aeromindAgentSession", sessionId);
    else window.sessionStorage.removeItem("aeromindAgentSession");
  } catch (_error) {
    // Private browser sessions may disable sessionStorage.
  }
}

async function createAgentSession() {
  const session = await requestJson("/api/agent/sessions", { method: "POST" });
  persistAgentSession(session.session_id);
  state.agentReplyKeys.clear();
  renderAgentSession(session);
  elements.missionRequestState.textContent = "会话已就绪";
  return session.session_id;
}

async function restoreAgentSession() {
  let saved = null;
  try {
    saved = window.sessionStorage.getItem("aeromindAgentSession");
  } catch (_error) {
    saved = null;
  }
  if (!saved) {
    await createAgentSession();
    return;
  }
  try {
    const session = await requestJson(`/api/agent/sessions/${saved}`);
    persistAgentSession(session.session_id);
    renderAgentSession(session);
    elements.missionRequestState.textContent = "会话已恢复";
  } catch (_error) {
    persistAgentSession(null);
    await createAgentSession();
  }
}

async function resetAgentSession() {
  const previous = state.agentSessionId;
  persistAgentSession(null);
  state.agentReplyKeys.clear();
  if (previous) {
    try {
      await requestJson(`/api/agent/sessions/${previous}`, { method: "DELETE" });
    } catch (_error) {
      // A server restart already invalidates the in-memory session.
    }
  }
  await createAgentSession();
}

function setSemanticBusy(busy) {
  state.semanticBusy = busy;
  renderSemanticStatus(state.status?.semantic || state.config?.semantic || {});
}

async function analyzeVision(event) {
  event.preventDefault();
  if (state.semanticBusy) return;
  setSemanticBusy(true);
  elements.visionRequestState.textContent = "分析中...";
  elements.visualSummary.textContent = "正在分析当前相机帧...";
  try {
    const payload = await requestJson("/api/semantic/vision/analyze", {
      method: "POST",
      body: JSON.stringify({
        prompt: elements.visionPrompt.value.trim(),
        vehicle_id: state.selectedVehicleId,
      }),
    });
    renderVisualSemantic(payload);
  } catch (error) {
    elements.visionRequestState.textContent = "请求失败";
    elements.visualSummary.textContent = `视觉分析失败：${error.message}`;
  } finally {
    setSemanticBusy(false);
    await refreshSemanticStatus();
  }
}

function renderVisionCrossValidation(cross) {
  const value = cross || {};
  elements.visionCrossStatus.textContent =
    value.status === "agreed" ? "VLM 与 OpenCV 一致"
    : value.status === "disagreed" ? "VLM 与 OpenCV 不一致"
    : value.status === "vlm_color_missing" ? "VLM 颜色缺失"
    : value.status === "vlm_not_analyzed" ? "尚未识别"
    : value.status === "unavailable" ? "检测不可用"
    : "等待交叉验证";
  elements.visionCrossStatus.title = value.note || "";
  elements.visionCrossStatus.dataset.level =
    value.status === "agreed" ? "ok"
    : value.status === "disagreed" ? "bad"
    : value.status === "vlm_color_missing" || value.status === "vlm_not_analyzed" || value.status === "unavailable" ? "warn"
    : "none";
  setColorSwatch(elements.visionVlmColor, value.vlm_color);
  setColorSwatch(elements.visionOpenCvColor, value.opencv_color);
  elements.visionAgreement.dataset.level =
    value.agreement === true ? "ok"
    : value.agreement === false ? "bad"
    : "none";
  elements.visionAgreement.textContent =
    value.agreement === true ? "一致"
    : value.agreement === false ? "不一致"
    : "--";
}
function setColorSwatch(host, color) {
  const swatch = host.querySelector("i");
  const label = host.querySelector("b");
  if (swatch) swatch.dataset.color = color || "none";
  if (label) label.textContent = color || "--";
}
function renderVisionConsensus(consensus) {
  const value = consensus || {};
  const color = value.confirmed_color || (value.best && value.best.color) || null;
  elements.visionConsensus.dataset.level = value.confirmed ? "ok" : value.window_size ? "warn" : "none";
  elements.visionConsensus.textContent = value.confirmed
    ? "已确认 · " + color + " (" + value.window_size + "/" + value.required + ")"
    : value.window_size
      ? "观察中 " + value.window_size + "/" + value.required
      : "--";
}
async function runVisionCrosscheck() {
  if (state.semanticBusy) return;
  elements.visionCrossStatus.textContent = "交叉验证中...";
  try {
    const payload = await requestJson("/api/vision/crosscheck", {
      method: "POST",
      body: JSON.stringify({ vehicle_id: state.selectedVehicleId }),
    });
    renderVisionCrossValidation(payload.cross_validation);
    renderVisionConsensus(payload.consensus);
  } catch (error) {
    elements.visionCrossStatus.textContent = "交叉验证失败";
    elements.visionEvidenceList.replaceChildren();
    const note = document.createElement("span");
    note.className = "semantic-empty";
    note.textContent = "交叉验证失败：" + error.message;
    elements.visionEvidenceList.append(note);
  }
  await refreshVisionEvidence();
}

async function refreshVisionEvidence() {
  try {
    const payload = await requestJson("/api/vision/evidence");
    const items = Array.isArray(payload.evidence) ? payload.evidence : [];
    elements.visionEvidenceList.replaceChildren();
    if (!items.length) {
      const empty = document.createElement("span");
      empty.className = "semantic-empty";
      empty.textContent = "尚无视觉证据";
      elements.visionEvidenceList.append(empty);
      return;
    }
    const list = document.createElement("ol");
    items.slice(-8).reverse().forEach((item) => {
      const row = document.createElement("li");
      const swatch = document.createElement("i");
      swatch.className = "color-swatch";
      swatch.dataset.color = item.opencv_color || item.vlm_color || "none";
      const label = document.createElement("a");
      const color = item.opencv_color || item.vlm_color || "?";
      const status = item.agreement === true ? "一致" : item.agreement === false ? "冲突" : "无";
      label.href = "/api/vision/evidence/" + item.evidence_id + "/frame";
      label.target = "_blank";
      label.rel = "noopener";
      label.textContent = "#" + item.frame_sequence + " " + color + " " + status + (item.confirmed ? " · 已确认" : "");
      row.append(swatch, label);
      list.append(row);
    });
    elements.visionEvidenceList.append(list);
  } catch (_error) {
    // Evidence history is optional while the rest of the station stays usable.
  }
}

function renderFleetStatus(payload) {
  const value = payload || {};
  const phaseLabels = {
    idle: "未编排",
    syncing: "同步中",
    transit: "切换中",
    formation_hold: "编队保持",
    degraded: "降级编队",
    aborted: "中止",
  };
  elements.fleetPhase.textContent = (value.formation_label || value.formation || "line") + " · " + (phaseLabels[value.phase] || value.phase || "--");
  elements.fleetPhase.dataset.level =
    value.phase === "formation_hold" ? "ok"
    : value.phase === "degraded" || value.phase === "aborted" ? "bad"
    : value.phase === "idle" ? "none"
    : "warn";
  elements.fleetPhase.title = value.reason || "";
  if (Array.isArray(value.leader_target_map_m) && value.leader_target_map_m.length === 3) {
    elements.fleetLeaderX.value = value.leader_target_map_m[0];
    elements.fleetLeaderY.value = value.leader_target_map_m[1];
    elements.fleetLeaderZ.value = value.leader_target_map_m[2];
  }
  elements.fleetFormations.forEach((button) => {
    const active = button.dataset.fleetFormation === value.formation;
    button.classList.toggle("active", active);
    button.setAttribute("aria-selected", String(active));
  });
  const execution = value.execution || {};
  const missionLabels = {
    idle: "未执行",
    starting: "启动中",
    running: "执行中",
    cancelling: "取消中",
    cancelled: "已取消",
    done: "已完成",
    failed: "失败",
  };
  elements.fleetMissionPhase.textContent = "任务：" + (missionLabels[execution.phase] || execution.phase || "--")
    + (execution.stage ? " · " + execution.stage : "");
  elements.fleetMissionPhase.dataset.level =
    execution.phase === "done" ? "ok"
    : execution.phase === "failed" ? "bad"
    : execution.phase === "running" || execution.phase === "cancelling" ? "warn"
    : "none";
  elements.fleetMissionPhase.title = execution.error || "";
  elements.fleetExecute.disabled = execution.busy === true;
  elements.fleetSequenceDemo.disabled = execution.busy === true;
  elements.fleetCancel.disabled = execution.busy !== true;
  const missionVehicles = Array.isArray(execution.vehicles) ? execution.vehicles : [];
  elements.fleetMissionVehicles.replaceChildren();
  if (missionVehicles.length) {
    const missionList = document.createElement("ol");
    missionVehicles.forEach((item) => {
      const row = document.createElement("li");
      const id = document.createElement("span");
      id.className = "fleet-slot-id";
      id.textContent = "UAV" + String(item.vehicle_id).padStart(2, "0");
      const step = document.createElement("span");
      step.className = "fleet-slot-action " + (item.state || "pending");
      step.textContent = item.step || "--";
      const detail = document.createElement("span");
      detail.className = "fleet-slot-note";
      detail.textContent = item.detail || item.state || "";
      row.append(id, step, detail);
      missionList.append(row);
    });
    elements.fleetMissionVehicles.append(missionList);
  }

  const directives = Array.isArray(value.directives) ? value.directives : [];
  elements.fleetSlots.replaceChildren();
  if (!directives.length) {
    const empty = document.createElement("span");
    empty.className = "semantic-empty";
    empty.textContent = value.reason || "尚无编队数据";
    elements.fleetSlots.append(empty);
    return;
  }
  const list = document.createElement("ol");
  directives.forEach((item) => {
    const row = document.createElement("li");
    const id = document.createElement("span");
    id.className = "fleet-slot-id";
    id.textContent = "UAV" + String(item.vehicle_id).padStart(2, "0");
    const action = document.createElement("span");
    action.className = "fleet-slot-action " + (item.action || "hold");
    action.textContent = item.action || "--";
    const target = document.createElement("span");
    target.className = "fleet-slot-target";
    target.textContent = item.target_map_m
      ? item.target_map_m.map((value) => fixed(value, 1)).join(", ")
      : "--";
    const note = document.createElement("span");
    note.className = "fleet-slot-note";
    note.textContent = item.slot !== null && item.slot !== undefined
      ? "槽位 " + item.slot + (item.reason ? " · " + item.reason : "")
      : item.reason || "";
    row.append(id, action, target, note);
    list.append(row);
  });
  elements.fleetSlots.append(list);
}

function switchDeploymentMode(mode) {
  const vehicles = Array.isArray(state.config?.vehicles) ? state.config.vehicles : [];
  const target = mode === "real"
    ? vehicles.find((vehicle) => String(vehicle.deployment_mode || "sim").toLowerCase() === "real")
    : vehicles.find((vehicle) => String(vehicle.deployment_mode || "sim").toLowerCase() !== "real");
  if (!target) return;
  const vehicleId = Number(target.vehicle_id);
  if (vehicleId === state.selectedVehicleId) return;
  elements.vehicleSelect.value = String(vehicleId);
  elements.vehicleSelect.dispatchEvent(new Event("change"));
}

async function sendGoto() {
  const x = finiteNumber(elements.gotoX.value);
  const y = finiteNumber(elements.gotoY.value);
  const z = finiteNumber(elements.gotoZ.value);
  if (x === null || y === null || z === null) {
    elements.statusText.textContent = "请填写有效的 NED 目标坐标";
    return;
  }
  elements.gotoFly.disabled = true;
  try {
    const result = await requestJson(`/api/vehicles/${state.selectedVehicleId}/commands/goto`, {
      method: "POST",
      body: JSON.stringify({
        target_position_ned_m: [x, y, z],
        reason: "operator goto",
      }),
    });
    elements.statusText.textContent = `v${state.selectedVehicleId} 飞往 (${x}, ${y}, ${z})：${result.successful ? "已完成" : "未完成"}`;
  } catch (error) {
    elements.statusText.textContent = error.message;
  } finally {
    elements.gotoFly.disabled = false;
    void refreshStatus();
  }
}

async function executeFleetMission() {
  const active = elements.fleetFormations.find((button) => button.classList.contains("active"));
  const formation = active ? active.dataset.fleetFormation : "line";
  const leader = [
    finiteNumber(elements.fleetLeaderX.value) ?? 0,
    finiteNumber(elements.fleetLeaderY.value) ?? 0,
    finiteNumber(elements.fleetLeaderZ.value) ?? -3,
  ];
  const simIds = (state.config?.vehicles || [])
    .filter((vehicle) => String(vehicle.deployment_mode || "sim").toLowerCase() !== "real")
    .map((vehicle) => Number(vehicle.vehicle_id));
  const vehicleIds = simIds.length >= 2 ? simIds : [1, 2, 3, 4];
  const payload = {
    formation,
    leader_target_map_m: leader,
    spacing_m: 3.0,
    altitude_m: finiteNumber(elements.fleetAltitude.value) ?? 2,
    hold_s: finiteNumber(elements.fleetHold.value) ?? 5,
    vehicle_ids: vehicleIds,
    land_after: true,
  };
  elements.fleetExecute.disabled = true;
  try {
    const result = await requestJson("/api/fleet/execute", {
      method: "POST",
      body: JSON.stringify(payload),
    });
    elements.statusText.textContent = `编队任务已启动：${result.stage || result.phase || ""}`;
  } catch (error) {
    elements.statusText.textContent = error.message;
    elements.fleetExecute.disabled = false;
  }
  void refreshFleetStatus();
}

async function executeFleetSequence() {
  const leader = [
    finiteNumber(elements.fleetLeaderX.value) ?? 0,
    finiteNumber(elements.fleetLeaderY.value) ?? 0,
    finiteNumber(elements.fleetLeaderZ.value) ?? -3,
  ];
  const simIds = (state.config?.vehicles || [])
    .filter((vehicle) => String(vehicle.deployment_mode || "sim").toLowerCase() !== "real")
    .map((vehicle) => Number(vehicle.vehicle_id));
  const vehicleIds = simIds.length >= 2 ? simIds : [1, 2, 3, 4];
  const payload = {
    formation: "line",
    formations: ["line", "v", "diamond"],
    leader_target_map_m: leader,
    spacing_m: 3.0,
    altitude_m: finiteNumber(elements.fleetAltitude.value) ?? 2,
    hold_s: finiteNumber(elements.fleetHold.value) ?? 5,
    vehicle_ids: vehicleIds,
    land_after: true,
  };
  elements.fleetExecute.disabled = true;
  elements.fleetSequenceDemo.disabled = true;
  try {
    const result = await requestJson("/api/fleet/execute", {
      method: "POST",
      body: JSON.stringify(payload),
    });
    elements.statusText.textContent = `编队序列已启动：${result.config?.formations?.join(" -> ") || "line -> v -> diamond"}`;
  } catch (error) {
    elements.statusText.textContent = error.message;
    elements.fleetExecute.disabled = false;
    elements.fleetSequenceDemo.disabled = false;
  }
  void refreshFleetStatus();
}

async function cancelFleetMission() {
  try {
    await requestJson("/api/fleet/execute/cancel", { method: "POST" });
    elements.statusText.textContent = "正在取消编队任务并降落";
  } catch (error) {
    elements.statusText.textContent = error.message;
  }
  void refreshFleetStatus();
}

async function refreshFleetStatus() {
  try {
    const payload = await requestJson("/api/fleet/status");
    state.fleetStatus = payload;
    renderFleetStatus(payload);
  } catch (_error) {
    // Fleet panel stays optional while the rest of the station keeps working.
  }
}

async function applyFleetFormation() {
  const active = elements.fleetFormations.find((button) => button.classList.contains("active"));
  const formation = active ? active.dataset.fleetFormation : "line";
  try {
    const payload = await requestJson("/api/fleet/formation", {
      method: "POST",
      body: JSON.stringify({
        formation,
        leader_target_map_m: [
          finiteNumber(elements.fleetLeaderX.value) ?? 0,
          finiteNumber(elements.fleetLeaderY.value) ?? 0,
          finiteNumber(elements.fleetLeaderZ.value) ?? -3,
        ],
      }),
    });
    void payload;
    await refreshFleetStatus();
  } catch (error) {
    elements.fleetPhase.textContent = "应用失败";
    elements.fleetPhase.dataset.level = "bad";
    elements.fleetPhase.title = error.message;
  }
}

async function parseMission(event) {
  event.preventDefault();
  if (state.semanticBusy) return;
  const instruction = elements.missionInstruction.value.trim();
  if (!instruction) {
    elements.semanticResult.textContent = "请输入任务指令";
    return;
  }
  setSemanticBusy(true);
  elements.missionRequestState.textContent = "Agent 思考中...";
  try {
    const sessionId = state.agentSessionId || await createAgentSession();
    const payload = await requestJson(`/api/agent/sessions/${sessionId}/messages`, {
      method: "POST",
      body: JSON.stringify({
        message: instruction,
        vehicle_id: state.selectedVehicleId,
      }),
    });
    renderAgentReply(payload);
    elements.missionInstruction.value = "";
  } catch (error) {
    elements.missionRequestState.textContent = "请求失败";
    appendAgentMessage("system", `Agent 请求失败：${error.message}`);
  } finally {
    setSemanticBusy(false);
    await refreshSemanticStatus();
  }
}

function showAgentDraftConfirmation() {
  const draft = state.agentDraft;
  if (!draft?.executable) return;
  state.pendingCommand = null;
  state.pendingAgentDraft = draft;
  const label = COMMAND_LABELS[draft.action] || draft.action;
  const altitude = finiteNumber(draft.arguments?.altitude_m);
  elements.confirmTitle.textContent = `确认 Agent ${label}`;
  elements.confirmText.textContent = altitude === null
    ? `${draft.reason}。确认后将提交到现有飞行命令证据链。`
    : `${draft.reason}。目标高度 ${altitude.toFixed(1)} m。`;
  elements.confirmVehicle.textContent = `UAV ${draft.vehicle_id}`;
  elements.confirmMode.textContent = selectedDeploymentMode().toUpperCase();
  elements.confirmSubmit.textContent = "确认执行";
  if (typeof elements.confirmDialog.showModal === "function") {
    elements.confirmDialog.showModal();
  } else if (window.confirm(elements.confirmText.textContent)) {
    state.pendingAgentDraft = null;
    void executeAgentDraft(draft);
  } else {
    state.pendingAgentDraft = null;
  }
}

async function executeAgentDraft(draft) {
  elements.confirmAgentDraft.disabled = true;
  elements.cancelAgentDraft.disabled = true;
  elements.missionRequestState.textContent = "执行中...";
  try {
    const payload = await requestJson(`/api/agent/drafts/${draft.draft_id}/confirm`, {
      method: "POST",
      body: JSON.stringify({
        confirmed: true,
        vehicle_id: draft.vehicle_id,
      }),
    });
    renderAgentDraft(payload.draft);
    const command = payload.command_result || {};
    const row = {
      localId: command.command_id || draft.draft_id,
      requestId: command.command_id,
      command: draft.action,
      label: COMMAND_LABELS[draft.action] || draft.action,
      createdAt: new Date(),
      application: "pending",
      mavlink: "pending",
      physical: "pending",
      status: "running",
      detail: draft.reason,
    };
    applyCommandResult(row, command);
    state.commandRows.unshift(row);
    state.commandRows = state.commandRows.slice(0, 100);
    renderEvidence();
    appendAgentMessage("system", command.successful
      ? `${row.label}已完成，物理结果已确认。`
      : `${row.label}未完成：${command.detail || "未知原因"}`);
    elements.missionRequestState.textContent = command.successful ? "执行完成" : "执行失败";
  } catch (error) {
    elements.missionRequestState.textContent = "确认失败";
    appendAgentMessage("system", `草案确认失败：${error.message}`);
    try {
      const session = await requestJson(`/api/agent/sessions/${state.agentSessionId}`);
      renderAgentDraft(session.latest_draft);
    } catch (_ignored) {
      renderAgentDraft(state.agentDraft);
    }
  }
}

async function cancelAgentDraft() {
  const draft = state.agentDraft;
  if (!draft || draft.status !== "pending_confirmation") return;
  try {
    const payload = await requestJson(`/api/agent/drafts/${draft.draft_id}/cancel`, {
      method: "POST",
    });
    renderAgentDraft(payload);
    elements.missionRequestState.textContent = "草案已取消";
  } catch (error) {
    elements.missionRequestState.textContent = `取消失败：${error.message}`;
  }
}

function renderStatus(payload) {
  const status = unwrapPayload(payload);
  if (status.vehicle_id && Number(status.vehicle_id) !== state.selectedVehicleId) return;
  state.status = status;
  renderSemanticStatus(status.semantic || state.config?.semantic || {});
  const runtimeStarted = status.runtime_started ?? status.started ?? true;
  const agentConnected = status.onboard_agent_connected
    ?? status.agent_connected
    ?? status.vehicle_connected
    ?? status.connected
    ?? false;
  if (status.telemetry || status.snapshot || "fcu_link_ok" in status) {
    renderTelemetry(normalizeTelemetry(status));
  }
  state.apiOnline = true;
  const runtimeError = status.runtime_error || status.ground_link?.error;
  setBadge(
    elements.apiBadge,
    runtimeError ? "offline" : runtimeStarted ? "online" : "pending",
    runtimeError ? "串口连接失败" : runtimeStarted ? "地面站在线" : "地面站启动中",
  );
  const isReal = selectedDeploymentMode() === "real";
  const p9Open = isReal && runtimeStarted && !runtimeError && !status.ground_link?.reconnecting;
  setBadge(
    elements.p9Badge,
    runtimeError ? "offline" : p9Open ? "online" : "pending",
    runtimeError ? "P9 串口失败" : p9Open ? "P9 串口已打开" : "P9 连接中",
  );
  setBadge(
    elements.agentBadge,
    agentConnected ? "online" : "offline",
    agentConnected ? "机载 Agent 在线" : "机载 Agent 离线",
  );
  renderLinkDiagnostics(status);
  if (runtimeError) elements.statusText.textContent = runtimeError;
  if (!agentConnected && !state.telemetry?.fcu_link_ok) {
    setBadge(elements.fcuBadge, "offline", "FCU 离线");
  }
  updateControlState();
  renderAgentContext();
}

function renderLinkDiagnostics(status) {
  const stats = status?.ground_link?.stats || {};
  const camera = status?.camera || {};
  const received = Number(stats.received_frames || 0);
  const duplicates = Number(stats.duplicate_frames || 0);
  const crcErrors = Number(stats.crc_errors || 0);
  const retries = Number(stats.retries || 0);
  const fps = Number(camera.source_fps);
  const video = Number.isFinite(fps) && fps > 0 ? `${fps.toFixed(1)} FPS` : "视频等待中";
  elements.linkDiagnostics.textContent = selectedDeploymentMode() === "real"
    ? `Lite v1 · P9 RX ${received} · 重复 ${duplicates} · CRC ${crcErrors} · 重传 ${retries} · ${video}`
    : `SITL UDP · FCU ${status?.fcu_link_ok ? "在线" : "离线"} · ${video}`;
}

function renderTelemetry(telemetry) {
  if (!telemetry) return;
  state.telemetry = telemetry;
  state.lastTelemetryAt = new Date();
  const linkOk = telemetry.fcu_link_ok === true;
  const isDemo = selectedVehicleConfig().commands_are_simulated === true;
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
  const hdop = finiteNumber(telemetry.gps_hdop);
  elements.gpsHealth.textContent = hdop === null
    ? booleanLabel(telemetry.gps_healthy)
    : `${booleanLabel(telemetry.gps_healthy)} · HDOP ${hdop.toFixed(2)}`;
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
  renderAgentContext();
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
  void refreshSituationMap();
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

const SITUATION_COLORS = {
  1: "#ff5d5d",
  2: "#4da3ff",
  3: "#3ddc84",
  4: "#c58aff",
  5: "#ffb84d",
};

function situationColor(vehicleId) {
  return SITUATION_COLORS[vehicleId] || "#9ad1ff";
}

function situationCanvasSize() {
  const canvas = elements.situationMap;
  if (!canvas) return null;
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

function drawSituationStar(context, x, y, radius, color) {
  context.save();
  context.translate(x, y);
  context.fillStyle = color;
  context.beginPath();
  for (let i = 0; i < 10; i += 1) {
    const angle = -Math.PI / 2 + i * Math.PI / 5;
    const arm = i % 2 === 0 ? radius : radius * 0.45;
    const px = Math.cos(angle) * arm;
    const py = Math.sin(angle) * arm;
    if (i === 0) context.moveTo(px, py);
    else context.lineTo(px, py);
  }
  context.closePath();
  context.fill();
  context.restore();
}

function renderSituationMap() {
  const canvas = elements.situationMap;
  if (!canvas) return;
  const size = situationCanvasSize();
  if (!size) return;
  const context = canvas.getContext("2d");
  const payload = state.situationPayload;
  const vehicles = (payload?.vehicles || []).filter(
    (vehicle) => vehicle.available && vehicle.position_m,
  );
  const leader = payload?.formation?.leader_target_map_m;
  const hasLeader = Array.isArray(leader)
    && leader.length >= 2
    && Number.isFinite(leader[0])
    && Number.isFinite(leader[1]);

  if (elements.situationEmpty) elements.situationEmpty.hidden = vehicles.length > 0;
  if (elements.situationStamp) {
    const range = state.situationView
      ? Math.round(state.situationView.halfRange)
      : null;
    elements.situationStamp.textContent = vehicles.length > 0
      ? `${vehicles.length} 机在线${range ? ` · ±${range}m` : ""}`
      : "--";
  }

  context.clearRect(0, 0, size.width, size.height);
  context.fillStyle = "#0a0e11";
  context.fillRect(0, 0, size.width, size.height);

  const plannedPoints = ((state.fleetStatus || {}).directives || [])
    .filter((item) => Array.isArray(item.target_map_m) && item.target_map_m.length >= 2)
    .map((item) => ({ x: item.target_map_m[0], y: item.target_map_m[1] }));
  const points = vehicles.map((vehicle) => ({
    x: vehicle.position_m.x,
    y: vehicle.position_m.y,
  }));
  if (hasLeader) points.push({ x: leader[0], y: leader[1] });
  points.push(...plannedPoints);
  if (!points.length) {
    if (elements.situationLegend) {
      elements.situationLegend.replaceChildren(
        Object.assign(document.createElement("span"), {
          className: "semantic-empty",
          textContent: "暂无在线无人机",
        }),
      );
    }
    return;
  }

  let centerX;
  let centerY;
  let halfRange;
  const fixedView = state.situationView;
  if (fixedView) {
    centerX = fixedView.centerX;
    centerY = fixedView.centerY;
    halfRange = fixedView.halfRange;
  } else {
    const minX = Math.min(...points.map((point) => point.x));
    const maxX = Math.max(...points.map((point) => point.x));
    const minY = Math.min(...points.map((point) => point.y));
    const maxY = Math.max(...points.map((point) => point.y));
    centerX = (minX + maxX) / 2;
    centerY = (minY + maxY) / 2;
    halfRange = Math.max(maxX - minX, maxY - minY) / 2 * 1.15 + 6;
    state.situationAutoView = { centerX, centerY, halfRange };
  }
  const scale = Math.min(size.width, size.height) * 0.44 / halfRange;
  const toScreen = (x, y) => ({
    sx: size.width / 2 + (x - centerX) * scale,
    sy: size.height / 2 - (y - centerY) * scale,
  });

  context.lineWidth = size.ratio;
  context.strokeStyle = "#222a30";
  const gridStep = halfRange <= 20 ? 5 : halfRange <= 50 ? 10 : 25;
  context.beginPath();
  for (let value = Math.floor((centerX - halfRange) / gridStep) * gridStep;
    value <= centerX + halfRange;
    value += gridStep) {
    const line = toScreen(value, centerY);
    context.moveTo(line.sx, 0);
    context.lineTo(line.sx, size.height);
  }
  for (let value = Math.floor((centerY - halfRange) / gridStep) * gridStep;
    value <= centerY + halfRange;
    value += gridStep) {
    const line = toScreen(centerX, value);
    context.moveTo(0, line.sy);
    context.lineTo(size.width, line.sy);
  }
  context.stroke();

  context.strokeStyle = "#52606a";
  context.beginPath();
  const northAxis = toScreen(0, centerY);
  const eastAxis = toScreen(centerX, 0);
  context.moveTo(northAxis.sx, 0);
  context.lineTo(northAxis.sx, size.height);
  context.moveTo(0, eastAxis.sy);
  context.lineTo(size.width, eastAxis.sy);
  context.stroke();

  const home = toScreen(0, 0);
  context.strokeStyle = "#62cf82";
  context.beginPath();
  context.arc(home.sx, home.sy, 5 * size.ratio, 0, Math.PI * 2);
  context.stroke();

  if (hasLeader) {
    const leaderPoint = toScreen(leader[0], leader[1]);
    drawSituationStar(context, leaderPoint.sx, leaderPoint.sy, 9 * size.ratio, "#f5d76e");
  }

  if (plannedPoints.length >= 2) {
    context.strokeStyle = "#5c6b76";
    context.lineWidth = 1.2 * size.ratio;
    context.setLineDash([5 * size.ratio, 4 * size.ratio]);
    context.beginPath();
    plannedPoints.forEach((point, index) => {
      const projected = toScreen(point.x, point.y);
      if (index === 0) context.moveTo(projected.sx, projected.sy);
      else context.lineTo(projected.sx, projected.sy);
    });
    context.stroke();
    context.setLineDash([]);
    plannedPoints.forEach((point) => {
      const projected = toScreen(point.x, point.y);
      context.strokeStyle = "#8fa3b3";
      context.lineWidth = 1.2 * size.ratio;
      context.beginPath();
      context.moveTo(projected.sx - 5 * size.ratio, projected.sy - 5 * size.ratio);
      context.lineTo(projected.sx + 5 * size.ratio, projected.sy + 5 * size.ratio);
      context.moveTo(projected.sx + 5 * size.ratio, projected.sy - 5 * size.ratio);
      context.lineTo(projected.sx - 5 * size.ratio, projected.sy + 5 * size.ratio);
      context.stroke();
    });
  }

  for (const vehicle of vehicles) {
    const trail = state.situationTrails[vehicle.vehicle_id] || [];
    if (trail.length < 2) continue;
    context.strokeStyle = situationColor(vehicle.vehicle_id);
    context.lineWidth = 1.5 * size.ratio;
    context.globalAlpha = 0.55;
    context.beginPath();
    trail.forEach((point, index) => {
      const projected = toScreen(point.x, point.y);
      if (index === 0) context.moveTo(projected.sx, projected.sy);
      else context.lineTo(projected.sx, projected.sy);
    });
    context.stroke();
    context.globalAlpha = 1;
  }

  for (const vehicle of vehicles) {
    const point = toScreen(vehicle.position_m.x, vehicle.position_m.y);
    const color = situationColor(vehicle.vehicle_id);
    if (Number.isFinite(vehicle.heading_deg)) {
      context.save();
      context.translate(point.sx, point.sy);
      context.rotate((vehicle.heading_deg || 0) * Math.PI / 180);
      context.fillStyle = color;
      context.beginPath();
      context.moveTo(0, -9 * size.ratio);
      context.lineTo(5.5 * size.ratio, 6 * size.ratio);
      context.lineTo(0, 3.2 * size.ratio);
      context.lineTo(-5.5 * size.ratio, 6 * size.ratio);
      context.closePath();
      context.fill();
      context.restore();
    } else {
      context.fillStyle = color;
      context.beginPath();
      context.arc(point.sx, point.sy, 5 * size.ratio, 0, Math.PI * 2);
      context.fill();
    }
    if (vehicle.vehicle_id === state.selectedVehicleId) {
      context.strokeStyle = "#ffffff";
      context.lineWidth = 1.6 * size.ratio;
      context.beginPath();
      context.arc(point.sx, point.sy, 11 * size.ratio, 0, Math.PI * 2);
      context.stroke();
    }
    context.fillStyle = "#f2f5f7";
    context.font = `bold ${Math.max(10, 11 * size.ratio)}px "Cascadia Mono", monospace`;
    context.fillText(`v${vehicle.vehicle_id}`, point.sx + 9 * size.ratio, point.sy - 8 * size.ratio);
  }

  if (elements.situationLegend) {
    const items = [];
    for (const vehicle of vehicles) {
      const item = document.createElement("span");
      item.className = "situation-legend-item";
      const swatch = document.createElement("i");
      swatch.style.background = situationColor(vehicle.vehicle_id);
      const label = document.createElement("b");
      label.textContent = vehicle.vehicle_name
        || `UAV ${String(vehicle.vehicle_id).padStart(2, "0")}`;
      const detail = document.createElement("span");
      detail.textContent = `${vehicle.mode || "--"} · ${vehicle.armed ? "ARM" : "SAFE"} · ${vehicle.fcu_link_ok ? "在线" : "离线"}`;
      item.append(swatch, label, detail);
      items.push(item);
    }
    for (const vehicle of payload?.vehicles || []) {
      if (vehicle.available && vehicle.position_m) continue;
      const item = document.createElement("span");
      item.className = "situation-legend-item offline";
      const swatch = document.createElement("i");
      const label = document.createElement("b");
      label.textContent = vehicle.vehicle_name
        || `UAV ${String(vehicle.vehicle_id).padStart(2, "0")}`;
      const detail = document.createElement("span");
      detail.textContent = "离线";
      item.append(swatch, label, detail);
      items.push(item);
    }
    if (!items.length) {
      items.push(Object.assign(document.createElement("span"), {
        className: "semantic-empty",
        textContent: "暂无在线无人机",
      }));
    }
    elements.situationLegend.replaceChildren(...items);
  }
}

function situationCanvasPoint(event) {
  const rect = elements.situationMap.getBoundingClientRect();
  return {
    x: event.clientX - rect.left,
    y: event.clientY - rect.top,
  };
}

function situationViewScale(size, halfRange) {
  return Math.min(size.width, size.height) * 0.44 / halfRange;
}

function ensureSituationView() {
  if (!state.situationView && state.situationAutoView) {
    state.situationView = { ...state.situationAutoView };
  }
}

function situationZoomAt(factor, px, py) {
  ensureSituationView();
  const canvas = elements.situationMap;
  const size = situationCanvasSize();
  if (!state.situationView || !size) return;
  const view = state.situationView;
  const scale = situationViewScale(size, view.halfRange);
  const worldX = view.centerX + (px - size.width / 2) / scale;
  const worldY = view.centerY - (py - size.height / 2) / scale;
  view.halfRange = Math.min(200, Math.max(6, view.halfRange * factor));
  const newScale = situationViewScale(size, view.halfRange);
  view.centerX = worldX - (px - size.width / 2) / newScale;
  view.centerY = worldY + (py - size.height / 2) / newScale;
  renderSituationMap();
}

function bindSituationMapInteractions() {
  const canvas = elements.situationMap;
  if (!canvas) return;
  canvas.addEventListener("wheel", (event) => {
    event.preventDefault();
    const point = situationCanvasPoint(event);
    situationZoomAt(event.deltaY > 0 ? 1.25 : 0.8, point.x, point.y);
  }, { passive: false });
  canvas.addEventListener("pointerdown", (event) => {
    if (event.button !== 0) return;
    ensureSituationView();
    const point = situationCanvasPoint(event);
    state.situationDrag = {
      x: point.x,
      y: point.y,
      view: state.situationView ? { ...state.situationView } : null,
    };
    canvas.setPointerCapture(event.pointerId);
    canvas.classList.add("dragging");
  });
  canvas.addEventListener("pointermove", (event) => {
    if (!state.situationDrag || !state.situationDrag.view) return;
    const size = situationCanvasSize();
    if (!size) return;
    const point = situationCanvasPoint(event);
    const view = state.situationDrag.view;
    const scale = situationViewScale(size, view.halfRange);
    state.situationView.centerX = view.centerX - (point.x - state.situationDrag.x) / scale;
    state.situationView.centerY = view.centerY + (point.y - state.situationDrag.y) / scale;
    renderSituationMap();
  });
  const endDrag = () => {
    state.situationDrag = null;
    elements.situationMap.classList.remove("dragging");
  };
  canvas.addEventListener("pointerup", endDrag);
  canvas.addEventListener("pointercancel", endDrag);
  canvas.addEventListener("dblclick", () => {
    state.situationView = null;
    renderSituationMap();
  });
  elements.situationZoomIn.addEventListener("click", () => {
    const size = situationCanvasSize();
    if (size) situationZoomAt(0.8, size.width / 2, size.height / 2);
  });
  elements.situationZoomOut.addEventListener("click", () => {
    const size = situationCanvasSize();
    if (size) situationZoomAt(1.25, size.width / 2, size.height / 2);
  });
  elements.situationZoomReset.addEventListener("click", () => {
    state.situationView = null;
    renderSituationMap();
  });
}

async function refreshSituationMap() {
  let payload;
  try {
    payload = await requestJson("/api/fleet/map");
  } catch (_error) {
    return;
  }
  state.situationPayload = payload;
  const seen = new Set();
  for (const vehicle of payload.vehicles || []) {
    if (!vehicle.available || !vehicle.position_m) continue;
    seen.add(vehicle.vehicle_id);
    if (!state.situationTrails[vehicle.vehicle_id]) {
      state.situationTrails[vehicle.vehicle_id] = [];
    }
    const trail = state.situationTrails[vehicle.vehicle_id];
    const previous = trail[trail.length - 1];
    const point = { x: vehicle.position_m.x, y: vehicle.position_m.y };
    if (!previous
      || Math.abs(previous.x - point.x) > 0.05
      || Math.abs(previous.y - point.y) > 0.05) {
      trail.push(point);
      if (trail.length > 240) trail.splice(0, trail.length - 240);
    }
  }
  for (const vehicleId of Object.keys(state.situationTrails)) {
    if (!seen.has(Number(vehicleId))) state.situationTrails[vehicleId] = [];
  }
  renderSituationMap();
}

function updateControlState() {
  const selectedConfig = selectedVehicleConfig();
  const simulated = selectedConfig.commands_are_simulated === true;
  const deploymentMode = selectedDeploymentMode();
  const onboardOutputEnabled = state.status?.command_output_enabled === true;
  const allowedCommands = new Set(
    Array.isArray(state.status?.allowed_commands)
      ? state.status.allowed_commands.map((command) => String(command).toLowerCase())
      : [],
  );
  const outputEnabled = selectedConfig.flight_output_enabled === true
    && (deploymentMode !== "real" || onboardOutputEnabled);
  const agentConnected = state.status?.onboard_agent_connected
    ?? state.status?.agent_connected
    ?? state.status?.vehicle_connected
    ?? false;
  const fcuReady = state.telemetry?.fcu_link_ok === true;
  const enabled = state.apiOnline && (simulated || (outputEnabled && agentConnected && fcuReady));
  const anyCommandEnabled = enabled
    && (simulated || deploymentMode !== "real" || allowedCommands.size > 0);
  elements.commandButtons.forEach((button) => {
    const actionAllowed = simulated || deploymentMode !== "real"
      || allowedCommands.has(String(button.dataset.command || "").toLowerCase());
    button.disabled = !(enabled && actionAllowed);
  });
  elements.gotoFly.disabled = !(enabled
    && (simulated || deploymentMode !== "real" || allowedCommands.has("goto")));
  const takeoffEnabled = enabled
    && (simulated || deploymentMode !== "real" || allowedCommands.has("takeoff"));
  elements.altitudeMinus.disabled = !takeoffEnabled;
  elements.altitudePlus.disabled = !takeoffEnabled;
  elements.takeoffAltitude.disabled = !takeoffEnabled;
  elements.controlState.className = `control-state ${anyCommandEnabled ? "ready" : "locked"}`;
  elements.controlState.textContent = anyCommandEnabled
    ? (simulated ? "DEMO" : "就绪")
    : "锁定";
  if (simulated && state.apiOnline) {
    elements.controlHint.textContent = "DEMO 模式：命令不会发送到 MAVLink";
  } else if (!state.apiOnline) {
    elements.controlHint.textContent = "等待地面站连接";
  } else if (deploymentMode === "real" && !onboardOutputEnabled) {
    elements.controlHint.textContent = "只读验收模式：机载飞控命令输出已禁用";
  } else if (deploymentMode === "real" && allowedCommands.size === 0) {
    elements.controlHint.textContent = "机载命令白名单为空，实机控制保持禁用";
  } else if (deploymentMode === "real" && allowedCommands.size > 0) {
    const labels = [...allowedCommands].map((command) => COMMAND_LABELS[command] || command);
    elements.controlHint.textContent = `实机白名单：${labels.join("、")}`;
  } else if (!agentConnected || !fcuReady) {
    elements.controlHint.textContent = deploymentMode === "real"
      ? "等待 P9 链路、机载代理和飞控连接"
      : "等待手动启动的 SITL 连接 14550";
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
  state.pendingAgentDraft = null;
  elements.confirmTitle.textContent = `确认${label}`;
  const altitude = finiteNumber(elements.takeoffAltitude.value) || 3;
  elements.confirmText.textContent = command === "takeoff"
    ? `${COMMAND_CONFIRMATIONS[command]} 当前设定：${altitude.toFixed(1)} m。`
    : COMMAND_CONFIRMATIONS[command];
  elements.confirmVehicle.textContent = elements.vehicleName.textContent;
  elements.confirmMode.textContent = selectedDeploymentMode().toUpperCase();
  elements.confirmSubmit.textContent = "确认发送";
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
  const eventVehicleId = Number(body.vehicle_id);
  const isVehicleEvent = Number.isFinite(eventVehicleId) && eventVehicleId > 0;
  if (isVehicleEvent && eventVehicleId !== state.selectedVehicleId) return;
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
  if (type === "georeference_updated") {
    renderGeoreferenceMeta(body);
  } else if (type === "visual_semantic_result") {
    renderVisualSemantic(body);
  } else if (type === "mission_parse_result") {
    renderMissionSemantic(body);
  } else if (type === "mission_agent_reply") {
    renderAgentReply(body);
  } else if (type === "mission_agent_draft_updated") {
    renderAgentDraft(body);
  } else if (type === "mission_agent_execution") {
    renderAgentDraft(body.draft);
  } else if (type === "command_progress") {
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
    const payload = await requestJson(`/api/status?vehicle_id=${state.selectedVehicleId}`);
    renderStatus(payload);
    try {
      const telemetry = await requestJson(`/api/vehicles/${state.selectedVehicleId}/telemetry`);
      if (telemetry.available) renderTelemetry(normalizeTelemetry(telemetry));
    } catch (_error) {
      // Status remains authoritative while telemetry is not available.
    }
  } catch (error) {
    state.apiOnline = false;
    setBadge(elements.apiBadge, "offline", "地面站离线");
    elements.statusText.textContent = error.message;
    updateControlState();
  }
}

async function refreshCameraStatus() {
  try {
    const camera = await requestJson(
      `/api/camera/status?vehicle_id=${state.selectedVehicleId}`,
    );
    renderCameraConfig(camera);
    if (state.status) {
      state.status.camera = camera;
      renderLinkDiagnostics(state.status);
    }
  } catch (error) {
    renderCameraConfig({
      state: "offline",
      stream_available: false,
      detail: `相机状态不可用：${error.message}`,
    });
  }
}

async function refreshSemanticStatus() {
  try {
    const semantic = await requestJson("/api/semantic/status");
    if (state.status) state.status.semantic = semantic;
    renderSemanticStatus(semantic);
  } catch (_error) {
    renderSemanticStatus({});
  }
}

async function restoreSemanticResults() {
  try {
    const payload = await requestJson("/api/semantic/latest");
    if (payload.visual) renderVisualSemantic(payload.visual);
    if (payload.mission) renderMissionSemantic(payload.mission);
    setSemanticMode(state.semanticMode);
    renderSemanticStatus(payload.status || {});
  } catch (_error) {
    // The ground station remains usable when the semantic service is absent.
  }
}

async function openSerialSettings() {
  elements.serialPort.value = state.config?.ground_serial_port || "COM3";
  elements.serialBaudrate.value = String(state.config?.ground_serial_baudrate || 57600);
  setSerialSettingsStatus("");
  if (typeof elements.serialDialog.showModal === "function") {
    elements.serialDialog.showModal();
  }
  await refreshSerialPorts();
}

async function refreshSerialPorts() {
  elements.refreshSerialPorts.disabled = true;
  elements.serialPortList.textContent = "正在扫描串口...";
  try {
    const payload = await requestJson("/api/serial/ports");
    state.serialPorts = Array.isArray(payload.ports) ? payload.ports : [];
    elements.serialPortOptions.replaceChildren(...state.serialPorts.map((port) => {
      const option = document.createElement("option");
      option.value = port.device;
      option.label = port.description || port.hwid || port.device;
      return option;
    }));
    elements.serialPortList.textContent = state.serialPorts.length
      ? state.serialPorts.map((port) => `${port.device} · ${port.description || "未知设备"}`).join("\n")
      : "未检测到串口；仍可手工输入 COM 端口或设备路径。";
  } catch (error) {
    elements.serialPortList.textContent = `扫描失败：${error.message}`;
  } finally {
    elements.refreshSerialPorts.disabled = false;
  }
}

function setSerialSettingsStatus(message, type = "") {
  elements.serialSettingsStatus.textContent = message;
  elements.serialSettingsStatus.className = `serial-settings-status ${type}`.trim();
}

async function applySerialSettings(event) {
  event.preventDefault();
  if (state.serialApplying) return;
  const port = elements.serialPort.value.trim();
  const baudrate = finiteNumber(elements.serialBaudrate.value);
  if (!port || baudrate === null || baudrate < 300 || baudrate > 4000000) {
    setSerialSettingsStatus("请输入有效串口和 300-4000000 范围内的波特率。", "error");
    return;
  }

  state.serialApplying = true;
  elements.serialApply.disabled = true;
  elements.serialCancel.disabled = true;
  setSerialSettingsStatus(`正在连接 ${port} @ ${baudrate}...`);
  try {
    const result = await requestJson("/api/serial/config", {
      method: "PUT",
      body: JSON.stringify({ port, baudrate }),
    });
    const [config, status] = await Promise.all([
      requestJson("/api/config"),
      requestJson(`/api/status?vehicle_id=${state.selectedVehicleId}`),
    ]);
    renderConfig(config);
    renderStatus(status);
    setSerialSettingsStatus(
      `${result.port} @ ${result.baudrate} 已打开，正在等待机载端。`,
      "success",
    );
    window.setTimeout(() => elements.serialDialog.close(), 700);
  } catch (error) {
    setSerialSettingsStatus(error.message, "error");
    await refreshStatus();
  } finally {
    state.serialApplying = false;
    elements.serialApply.disabled = false;
    elements.serialCancel.disabled = false;
  }
}

function setInputValue(element, value) {
  element.value = value === null || value === undefined ? "" : String(value);
}

function createGeoHomeRow(home = {}) {
  const row = document.createElement("div");
  row.className = "geo-home-row";
  row.dataset.vehicleHome = "";
  const fields = [
    ["vehicle_id", home.vehicle_id ?? "", "机号", "1", "255", "1"],
    ["x", home.map_position_m?.[0] ?? 0, "Home X", "-100000", "100000", "0.01"],
    ["y", home.map_position_m?.[1] ?? 0, "Home Y", "-100000", "100000", "0.01"],
    ["z", home.map_position_m?.[2] ?? 0, "Home Z", "-10000", "10000", "0.01"],
  ];
  fields.forEach(([name, value, label, minimum, maximum, step]) => {
    const input = document.createElement("input");
    input.type = "number";
    input.dataset.geoHomeField = name;
    input.value = String(value);
    input.min = minimum;
    input.max = maximum;
    input.step = step;
    input.inputMode = "decimal";
    input.required = true;
    input.setAttribute("aria-label", label);
    row.append(input);
  });
  const remove = document.createElement("button");
  remove.type = "button";
  remove.title = "移除飞行器 Home";
  remove.setAttribute("aria-label", "移除飞行器 Home");
  remove.textContent = "×";
  remove.addEventListener("click", () => {
    if (elements.geoHomes.children.length > 1) row.remove();
  });
  row.append(remove);
  return row;
}

function renderGeoHomes(homes) {
  const values = Array.isArray(homes) && homes.length
    ? homes
    : [{ vehicle_id: 1, map_position_m: [0, 0, 0] }];
  elements.geoHomes.replaceChildren(...values.map(createGeoHomeRow));
}

function renderGeoreferenceMeta(payload) {
  if (!payload?.configuration) return;
  state.georeference = payload;
  const configuration = payload.configuration;
  const surveyed = configuration.status === "surveyed";
  elements.geoCurrentStatus.textContent = surveyed ? "已实测 / 冻结" : "草稿 / 可编辑";
  elements.geoCurrentStatus.className = surveyed ? "ok" : "warn";
  elements.geoCurrentHash.textContent = payload.config_hash?.slice(0, 16) || "--";
  elements.geoCurrentHash.title = payload.config_hash || "";
  const report = payload.round_trip_report || {};
  if (!report.ready) {
    elements.geoRoundTrip.textContent = "未就绪";
    elements.geoRoundTrip.className = "warn";
  } else if (report.passed) {
    elements.geoRoundTrip.textContent = `通过 · ${Number(report.maximum_error_m || 0).toExponential(2)} m`;
    elements.geoRoundTrip.className = "ok";
  } else {
    elements.geoRoundTrip.textContent = "误差超限";
    elements.geoRoundTrip.className = "warn";
  }
}

function populateGeoreferenceForm(payload) {
  renderGeoreferenceMeta(payload);
  const config = payload.configuration;
  elements.geoCalibrationId.value = config.calibration_id || "venue-draft";
  elements.geoStatus.value = config.status || "draft";
  setInputValue(elements.geoHeading, config.map_x_heading_from_true_north_deg);
  elements.geoAltitudeDatum.value = config.height_reference?.datum || "amsl";
  setInputValue(elements.geoMapLatitude, config.map_origin_wgs84?.latitude_deg);
  setInputValue(elements.geoMapLongitude, config.map_origin_wgs84?.longitude_deg);
  setInputValue(elements.geoMapAltitude, config.map_origin_wgs84?.altitude_m);
  setInputValue(elements.geoAirSimLatitude, config.airsim_origin_wgs84?.latitude_deg);
  setInputValue(elements.geoAirSimLongitude, config.airsim_origin_wgs84?.longitude_deg);
  setInputValue(elements.geoAirSimAltitude, config.airsim_origin_wgs84?.altitude_m);
  elements.geoNotes.value = config.notes || "";
  renderGeoHomes(config.vehicle_homes);
}

function setGeoSettingsStatus(message, type = "") {
  elements.geoSettingsStatus.textContent = message;
  elements.geoSettingsStatus.className = `serial-settings-status ${type}`.trim();
}

async function refreshGeoreference() {
  const payload = await requestJson("/api/georeference");
  renderGeoreferenceMeta(payload);
  return payload;
}

async function openGeoreferenceSettings() {
  setGeoSettingsStatus("正在读取场地配置...");
  if (typeof elements.geoDialog.showModal === "function") {
    elements.geoDialog.showModal();
  }
  try {
    populateGeoreferenceForm(await refreshGeoreference());
    setGeoSettingsStatus("");
  } catch (error) {
    setGeoSettingsStatus(error.message, "error");
  }
}

function readGeoPosition(latitude, longitude, altitude, label) {
  const inputs = [latitude, longitude, altitude];
  const present = inputs.map((input) => input.value.trim() !== "");
  if (!present.some(Boolean)) return null;
  if (!present.every(Boolean) || inputs.some((input) => !input.checkValidity())) {
    throw new Error(`${label}必须同时填写有效的纬度、经度和高度。`);
  }
  return {
    latitude_deg: Number(latitude.value),
    longitude_deg: Number(longitude.value),
    altitude_m: Number(altitude.value),
  };
}

function collectGeoHomes() {
  const homes = Array.from(elements.geoHomes.querySelectorAll("[data-vehicle-home]")).map((row) => {
    const value = (name) => row.querySelector(`[data-geo-home-field="${name}"]`);
    const inputs = [value("vehicle_id"), value("x"), value("y"), value("z")];
    if (inputs.some((input) => !input.checkValidity() || finiteNumber(input.value) === null)) {
      throw new Error("每个 Home 都必须填写有效的机号和 X/Y/Z 坐标。");
    }
    return {
      vehicle_id: Number(inputs[0].value),
      map_position_m: inputs.slice(1).map((input) => Number(input.value)),
      expected_wgs84: null,
    };
  });
  if (!homes.length) throw new Error("至少保留一个飞行器 Home。");
  if (new Set(homes.map((home) => home.vehicle_id)).size !== homes.length) {
    throw new Error("飞行器 Home 的机号不能重复。");
  }
  return homes;
}

function collectGeoreference() {
  const calibrationId = elements.geoCalibrationId.value.trim();
  if (!calibrationId || !/^[A-Za-z0-9][A-Za-z0-9._-]*$/.test(calibrationId)) {
    throw new Error("标定版本 ID 只能使用字母、数字、点、下划线和连字符。");
  }
  const mapOrigin = readGeoPosition(
    elements.geoMapLatitude,
    elements.geoMapLongitude,
    elements.geoMapAltitude,
    "真实场地原点",
  );
  const airsimOrigin = readGeoPosition(
    elements.geoAirSimLatitude,
    elements.geoAirSimLongitude,
    elements.geoAirSimAltitude,
    "AirSim 原点",
  );
  const heading = elements.geoHeading.value.trim() === ""
    ? null
    : finiteNumber(elements.geoHeading.value);
  if (heading !== null && !elements.geoHeading.checkValidity()) {
    throw new Error("map X 航向必须在 -180° 到 180° 之间。");
  }
  if (elements.geoStatus.value === "surveyed" && (!mapOrigin || !airsimOrigin || heading === null)) {
    throw new Error("冻结为已实测版本前，必须填写两个原点和 map X 真北航向。");
  }
  const previous = state.georeference?.configuration || {};
  return {
    schema_version: "1.0",
    calibration_id: calibrationId,
    status: elements.geoStatus.value,
    map_definition: previous.map_definition || {
      origin_marker: "O",
      positive_x_marker: "X1",
      x_axis_reference: "true_north",
      z_axis: "up",
      units: "m",
    },
    map_origin_wgs84: mapOrigin,
    map_x_heading_from_true_north_deg: heading,
    airsim_origin_wgs84: airsimOrigin,
    map_scale: 1.0,
    height_reference: {
      datum: elements.geoAltitudeDatum.value,
      map_z_zero: "map_origin",
    },
    vehicle_homes: collectGeoHomes(),
    notes: elements.geoNotes.value,
  };
}

async function applyGeoreference(event) {
  event.preventDefault();
  if (state.geoApplying) return;
  let payload;
  try {
    payload = collectGeoreference();
  } catch (error) {
    setGeoSettingsStatus(error.message, "error");
    return;
  }
  state.geoApplying = true;
  elements.geoApply.disabled = true;
  elements.geoCancel.disabled = true;
  setGeoSettingsStatus("正在校验并保存标定配置...");
  try {
    const result = await requestJson("/api/georeference", {
      method: "PUT",
      body: JSON.stringify(payload),
    });
    populateGeoreferenceForm(result);
    setGeoSettingsStatus(
      result.immutable ? "已保存并冻结；修改时必须使用新的标定版本 ID。" : "草稿已保存。",
      "success",
    );
    window.setTimeout(() => elements.geoDialog.close(), 900);
  } catch (error) {
    setGeoSettingsStatus(error.message, "error");
  } finally {
    state.geoApplying = false;
    elements.geoApply.disabled = false;
    elements.geoCancel.disabled = false;
  }
}

function bindEvents() {
  elements.commandButtons.forEach((button) => {
    button.addEventListener("click", () => showCommandConfirmation(button.dataset.command));
  });
  elements.confirmSubmit.addEventListener("click", () => {
    const command = state.pendingCommand;
    const draft = state.pendingAgentDraft;
    state.pendingCommand = null;
    state.pendingAgentDraft = null;
    elements.confirmDialog.close();
    if (draft) void executeAgentDraft(draft);
    else if (command) void sendCommand(command);
  });
  elements.confirmDialog.addEventListener("close", () => {
    state.pendingCommand = null;
    state.pendingAgentDraft = null;
    elements.confirmSubmit.textContent = "确认发送";
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
    state.status = null;
    state.telemetry = null;
    state.track = [];
    renderSelectedVehicleConfig();
    elements.statusText.textContent = "正在切换车辆链路...";
    updateControlState();
    void refreshStatus();
    void refreshCameraStatus();
  });
  elements.serialSettingsButton.addEventListener("click", () => {
    void openSerialSettings();
  });
  elements.refreshSerialPorts.addEventListener("click", () => {
    void refreshSerialPorts();
  });
  elements.serialForm.addEventListener("submit", (event) => {
    void applySerialSettings(event);
  });
  elements.serialCancel.addEventListener("click", () => {
    if (!state.serialApplying) elements.serialDialog.close();
  });
  elements.geoSettingsButton.addEventListener("click", () => {
    void openGeoreferenceSettings();
  });
  elements.geoForm.addEventListener("submit", (event) => {
    void applyGeoreference(event);
  });
  elements.geoCancel.addEventListener("click", () => {
    if (!state.geoApplying) elements.geoDialog.close();
  });
  elements.geoAddHome.addEventListener("click", () => {
    const used = Array.from(elements.geoHomes.querySelectorAll('[data-geo-home-field="vehicle_id"]'))
      .map((input) => Number(input.value));
    let vehicleId = 1;
    while (used.includes(vehicleId)) vehicleId += 1;
    elements.geoHomes.append(createGeoHomeRow({ vehicle_id: vehicleId, map_position_m: [0, 0, 0] }));
  });
  elements.visionAnalysisForm.addEventListener("submit", (event) => {
    void analyzeVision(event);
  });
  elements.visionCrosscheck.addEventListener("click", () => {
    void runVisionCrosscheck();
  });
  elements.fleetFormations.forEach((button) => {
    button.addEventListener("click", () => {
      elements.fleetFormations.forEach((candidate) => {
        const active = candidate === button;
        candidate.classList.toggle("active", active);
        candidate.setAttribute("aria-selected", String(active));
      });
      void applyFleetFormation();
    });
  });
  elements.simMode.addEventListener("click", () => {
    switchDeploymentMode("sim");
  });
  elements.realMode.addEventListener("click", () => {
    switchDeploymentMode("real");
  });
  elements.fleetApply.addEventListener("click", () => {
    void applyFleetFormation();
  });
  elements.fleetExecute.addEventListener("click", () => {
    void executeFleetMission();
  });
  elements.fleetSequenceDemo.addEventListener("click", () => {
    void executeFleetSequence();
  });
  elements.fleetCancel.addEventListener("click", () => {
    void cancelFleetMission();
  });
  elements.gotoFly.addEventListener("click", () => {
    void sendGoto();
  });
  elements.missionParseForm.addEventListener("submit", (event) => {
    void parseMission(event);
  });
  elements.resetAgentSession.addEventListener("click", () => {
    void resetAgentSession();
  });
  elements.confirmAgentDraft.addEventListener("click", showAgentDraftConfirmation);
  elements.cancelAgentDraft.addEventListener("click", () => {
    void cancelAgentDraft();
  });
  elements.semanticTabs.forEach((tab) => {
    tab.addEventListener("click", () => {
      state.semanticView = tab.dataset.semanticTab;
      elements.semanticTabs.forEach((candidate) => {
        const active = candidate === tab;
        candidate.classList.toggle("active", active);
        candidate.setAttribute("aria-selected", String(active));
      });
      renderSemanticResult();
    });
  });
  elements.semanticModes.forEach((button) => {
    button.addEventListener("click", () => {
      setSemanticMode(button.dataset.semanticMode);
    });
  });
  window.addEventListener("resize", drawTrack);
  if (window.ResizeObserver) {
    new ResizeObserver(drawTrack).observe(elements.nedCanvas.parentElement);
  }
  window.addEventListener("resize", renderSituationMap);
  if (window.ResizeObserver && elements.situationMap) {
    new ResizeObserver(renderSituationMap).observe(elements.situationMap.parentElement);
  }
  bindSituationMapInteractions();
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
  setSemanticMode("vision");
  drawTrack();
  try {
    const [config, status] = await Promise.all([
      requestJson("/api/config"),
      requestJson("/api/status"),
    ]);
    renderConfig(config);
    renderStatus(status);
    try {
      await refreshGeoreference();
    } catch (_error) {
      // Flight monitoring remains available if the calibration file is unavailable.
    }
    await refreshCameraStatus();
    await restoreSemanticResults();
    await restoreAgentSession();
  } catch (error) {
    state.apiOnline = false;
    setBadge(elements.apiBadge, "offline", "地面站离线");
    elements.statusText.textContent = error.message;
    updateControlState();
  }
  connectWebsocket();
  window.setInterval(refreshStatus, 2000);
  window.setInterval(refreshCameraStatus, 2000);
  window.setInterval(refreshSemanticStatus, 3000);
  window.setInterval(refreshFleetStatus, 3000);
  window.setInterval(refreshSituationMap, 1000);
  void refreshVisionEvidence();
}

document.addEventListener("DOMContentLoaded", bootstrap);
