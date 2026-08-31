const els = {
  systemLine: document.getElementById("systemLine"),
  linkBadge: document.getElementById("linkBadge"),
  armedBadge: document.getElementById("armedBadge"),
  modeBadge: document.getElementById("modeBadge"),
  ekfBadge: document.getElementById("ekfBadge"),
  serviceBadge: document.getElementById("serviceBadge"),
  agentBadge: document.getElementById("agentBadge"),
  agentModelSelect: document.getElementById("agentModelSelect"),
  workflowControls: document.getElementById("workflowControls"),
  pauseWorkflowBtn: document.getElementById("pauseWorkflowBtn"),
  resumeWorkflowBtn: document.getElementById("resumeWorkflowBtn"),
  cancelWorkflowBtn: document.getElementById("cancelWorkflowBtn"),
  cameraMeta: document.getElementById("cameraMeta"),
  cameraCanvas: document.getElementById("cameraCanvas"),
  cameraEmpty: document.getElementById("cameraEmpty"),
  cameraColorMode: document.getElementById("cameraColorMode"),
  detectionStatus: document.getElementById("detectionStatus"),
  detectionCount: document.getElementById("detectionCount"),
  detectionList: document.getElementById("detectionList"),
  perceptionHealthStatus: document.getElementById("perceptionHealthStatus"),
  perceptionHealthDetails: document.getElementById("perceptionHealthDetails"),
  worldHealthStatus: document.getElementById("worldHealthStatus"),
  worldTrackCount: document.getElementById("worldTrackCount"),
  worldPathRisk: document.getElementById("worldPathRisk"),
  cameraZoomBtn: document.getElementById("cameraZoomBtn"),
  cameraViewer: document.getElementById("cameraViewer"),
  cameraViewerStage: document.getElementById("cameraViewerStage"),
  cameraViewerCanvas: document.getElementById("cameraViewerCanvas"),
  cameraViewerMeta: document.getElementById("cameraViewerMeta"),
  cameraZoomOutBtn: document.getElementById("cameraZoomOutBtn"),
  cameraZoomInBtn: document.getElementById("cameraZoomInBtn"),
  cameraZoomFitBtn: document.getElementById("cameraZoomFitBtn"),
  cameraZoomValue: document.getElementById("cameraZoomValue"),
  closeCameraViewerBtn: document.getElementById("closeCameraViewerBtn"),
  hudMode: document.getElementById("hudMode"),
  chatArmedValue: document.getElementById("chatArmedValue"),
  chatModeValue: document.getElementById("chatModeValue"),
  chatBatteryValue: document.getElementById("chatBatteryValue"),
  chatGpsValue: document.getElementById("chatGpsValue"),
  chatEkfValue: document.getElementById("chatEkfValue"),
  chatPositionValue: document.getElementById("chatPositionValue"),
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
  autonomyMeta: document.getElementById("autonomyMeta"),
  autonomyState: document.getElementById("autonomyState"),
  autonomyStrategy: document.getElementById("autonomyStrategy"),
  autonomyNearest: document.getElementById("autonomyNearest"),
  autonomyTarget: document.getElementById("autonomyTarget"),
  autonomyMessage: document.getElementById("autonomyMessage"),
  cancelAutonomyBtn: document.getElementById("cancelAutonomyBtn"),
  pointcloudMeta: document.getElementById("pointcloudMeta"),
  pointcloud3d: document.getElementById("pointcloud3d"),
  pointcloudCanvas: document.getElementById("pointcloudCanvas"),
  pointcloudEmpty: document.getElementById("pointcloudEmpty"),
  resetCloudViewBtn: document.getElementById("resetCloudViewBtn"),
  cloudColorMode: document.getElementById("cloudColorMode"),
  cloudPointSize: document.getElementById("cloudPointSize"),
  altitudeInput: document.getElementById("altitudeInput"),
  px4WhiteFollowBtn: document.getElementById("px4WhiteFollowBtn"),
  px4FollowFlow: document.getElementById("px4FollowFlow"),
  taskForm: document.getElementById("taskForm"),
  taskInput: document.getElementById("taskInput"),
  taskResult: document.getElementById("taskResult"),
  parsedTask: document.getElementById("parsedTask"),
  reportList: document.getElementById("reportList"),
  missionPhase: document.getElementById("missionPhase"),
  missionResult: document.getElementById("missionResult"),
  confirmationPanel: document.getElementById("confirmationPanel"),
  confirmationText: document.getElementById("confirmationText"),
  confirmTaskBtn: document.getElementById("confirmTaskBtn"),
  cancelTaskBtn: document.getElementById("cancelTaskBtn"),
  controlConfirmationModal: document.getElementById("controlConfirmationModal"),
  controlConfirmationKind: document.getElementById("controlConfirmationKind"),
  controlConfirmationSummary: document.getElementById("controlConfirmationSummary"),
  controlConfirmationAction: document.getElementById("controlConfirmationAction"),
  controlConfirmationId: document.getElementById("controlConfirmationId"),
  controlConfirmationRisk: document.getElementById("controlConfirmationRisk"),
  controlConfirmationNote: document.getElementById("controlConfirmationNote"),
  modalConfirmTaskBtn: document.getElementById("modalConfirmTaskBtn"),
  modalCancelTaskBtn: document.getElementById("modalCancelTaskBtn"),
  chatHistory: document.getElementById("chatHistory"),
  chatScrollBtn: document.getElementById("chatScrollBtn"),
  missionMapCanvas: document.getElementById("missionMapCanvas"),
  missionMapMeta: document.getElementById("missionMapMeta"),
  llmOutput: document.getElementById("llmOutput"),
  planSteps: document.getElementById("planSteps"),
  toolCalls: document.getElementById("toolCalls"),
  aiStatusText: document.getElementById("aiStatusText"),
  planStatusText: document.getElementById("planStatusText"),
  toolStatusText: document.getElementById("toolStatusText"),
  lifecycleParse: document.getElementById("lifecycleParse"),
  lifecycleConfirm: document.getElementById("lifecycleConfirm"),
  lifecycleExecute: document.getElementById("lifecycleExecute"),
  lifecycleVerify: document.getElementById("lifecycleVerify"),
  lifecycleDone: document.getElementById("lifecycleDone"),
  lifecycleSummary: document.getElementById("lifecycleSummary"),
  eventLog: document.getElementById("eventLog"),
  skillGrid: document.getElementById("skillGrid"),
  capabilityGrid: document.getElementById("capabilityGrid"),
  gatewayInput: document.getElementById("gatewayInput"),
  saveGatewayBtn: document.getElementById("saveGatewayBtn"),
  clearChatBtn: document.getElementById("clearChatBtn"),
};

let localEvents = [];
let lastAgentPayload = null;
let pendingConfirmation = null;
const gatewayMissionRevisions = new Map();
let activeWorkflowId = null;
let gatewaySkills = [];
let gatewayCapabilities = [];
let activeVerification = null;
let threePointcloud = null;
let telemetrySocket = null;
let telemetrySocketTimer = null;
let agentSocket = null;
let agentSocketTimer = null;
let agentConnected = false;
let latestPointcloud = null;
let latestDetections = [];
let latestDetectionMeta = null;
let lastDetectionOverlayKey = "";
let chatAutoFollow = true;
let missionTrack = [];
let missionTrackFrame = "";
let latestCameraFrame = null;
let lastLiveMissionKey = "";
let lastImageAnalysisKey = "";
let lastReportListKey = "";
let lastChannelStatusKey = "";
let cameraViewerZoom = 1;
let pointcloudOptions = {
  colorMode: "axis",
  pointSize: 0.055,
};
let apiBaseUrl = localStorage.getItem("aeromind_gateway_url") || "http://localhost:8080";
let agentBaseUrl = localStorage.getItem("aeromind_agent_gateway_url") || "";
const agentAccessToken = localStorage.getItem("aeromind_agent_token") || "";
const CHAT_STORAGE_KEY = "aeromind_chat_history";
const SESSION_STORAGE_KEY = "aeromind_chat_session";
const PANEL_HEIGHT_STORAGE_PREFIX = "aeromind_panel_height_";
const CAMERA_COLOR_MODE_KEY = "aeromind_camera_color_mode";
const AGENT_MODEL_STORAGE_KEY = "aeromind_agent_model";
const PX4_FOLLOW_TASK_SOURCE = "px4_follow_task";

const lifecycleOrder = ["parse", "confirm", "execute", "verify", "done"];

function setTaskLifecycle(stage, summary, options = {}) {
  const steps = {
    parse: els.lifecycleParse,
    confirm: els.lifecycleConfirm,
    execute: els.lifecycleExecute,
    verify: els.lifecycleVerify,
    done: els.lifecycleDone,
  };
  const activeIndex = Math.max(0, lifecycleOrder.indexOf(stage));
  const skipped = new Set(options.skipped || []);
  const outcome = options.outcome || "active";
  const activeText = options.activeText || {
    parse: "正在理解任务",
    confirm: "等待操作员",
    execute: "正在调用 ROS",
    verify: "核验飞控状态",
    done: "任务已完成",
  }[stage] || "进行中";

  lifecycleOrder.forEach((key, index) => {
    const element = steps[key];
    if (!element) return;
    element.classList.remove("active", "complete", "error");
    element.removeAttribute("aria-current");
    const detail = element.querySelector("em");
    if (stage === "done" && outcome !== "error") {
      element.classList.add("complete");
      detail.textContent = skipped.has(key) ? "无需执行" : "已完成";
      return;
    }
    if (index < activeIndex) {
      element.classList.add("complete");
      detail.textContent = skipped.has(key) ? "无需执行" : "已完成";
      return;
    }
    if (index === activeIndex) {
      element.classList.add(outcome === "error" ? "error" : "active");
      element.setAttribute("aria-current", "step");
      detail.textContent = activeText;
      return;
    }
    detail.textContent = "未开始";
  });
  if (els.lifecycleSummary) {
    els.lifecycleSummary.textContent = summary || "任务状态等待更新";
  }
}
const existingSessionId = localStorage.getItem(SESSION_STORAGE_KEY);
const sessionId = existingSessionId || `ground-${Date.now()}-${Math.random().toString(16).slice(2)}`;
let chatMessages = loadChatHistory();

if (!existingSessionId) {
  localStorage.setItem(SESSION_STORAGE_KEY, sessionId);
}

setupResizableCards();

els.gatewayInput.value = apiBaseUrl;
els.cameraColorMode.value = localStorage.getItem(CAMERA_COLOR_MODE_KEY) || "auto";
els.agentModelSelect.value = localStorage.getItem(AGENT_MODEL_STORAGE_KEY) || "claude:sonnet";

const fallbackSkills = [
  { name: "flight.square", label: "正方形轨迹", risk_level: "high", description: "四边闭合飞行轨迹", example_task: "飞一个边长10米的正方形轨迹", enabled: true },
  { name: "flight.v_shape", label: "V 字轨迹", risk_level: "high", description: "两个相对向量航段组成 V 字", example_task: "飞一个宽10米、深10米的V字形", enabled: true },
  { name: "inspection.person_branch", label: "人员条件巡检", risk_level: "high", description: "发现人员悬停拍照，否则继续前进", example_task: "巡检前方，发现人就悬停拍照，否则继续前进20米", enabled: true },
];
const fallbackCapabilities = [
  { name: "system.status", label: "飞控状态", risk_level: "low", interface: "/control/drone_state", description: "读取飞控与位置状态", example_task: "查询无人机状态", enabled: true },
  { name: "flight.takeoff", label: "起飞", risk_level: "high", interface: "/control/takeoff", description: "按目标高度起飞", example_task: "起飞到10米", enabled: true },
  { name: "flight.move_relative", label: "相对移动", risk_level: "high", interface: "/autonomy/goal", description: "执行相对三维移动", example_task: "向前飞10米", enabled: true },
  { name: "perception.capture_image", label: "拍照保存", risk_level: "low", interface: "/perception/capture_image", description: "保存当前 RGB 图像", example_task: "拍一张前方照片", enabled: true },
];

function fmt(value, digits = 2) {
  if (value === null || value === undefined || Number.isNaN(Number(value))) return "--";
  return Number(value).toFixed(digits);
}

function meters(value) {
  if (value === null || value === undefined || Number.isNaN(Number(value))) return "--";
  return `${Number(value).toFixed(2)} m`;
}

function apiUrl(path) {
  if (/^https?:\/\//.test(path)) return path;
  return `${apiBaseUrl.replace(/\/+$/, "")}${path}`;
}

async function api(path, options = {}) {
  const res = await fetch(apiUrl(path), {
    ...options,
    headers: {
      "Content-Type": "application/json",
      ...(options.headers || {}),
    },
  });
  const data = await res.json();
  if (!res.ok) {
    const err = new Error(data.message || `HTTP ${res.status}`);
    err.data = data;
    throw err;
  }
  return data;
}

async function post(path, payload = {}) {
  return api(path, {
    method: "POST",
    body: JSON.stringify(payload),
  });
}

function escapeHtml(value) {
  return String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;");
}

function normalizeToolResult(value, depth = 0) {
  if (depth > 4 || value == null) return null;
  if (typeof value === "string") {
    const text = value.trim();
    if (!text) return null;
    try {
      return normalizeToolResult(JSON.parse(text), depth + 1) || { message: text };
    } catch (_error) {
      return { message: text };
    }
  }
  if (Array.isArray(value)) {
    for (const item of value) {
      const normalized = normalizeToolResult(item, depth + 1);
      if (normalized?.evidence?.length) return normalized;
    }
    return null;
  }
  if (typeof value !== "object") return null;
  if (Array.isArray(value.evidence)) return value;
  if (Array.isArray(value.content)) {
    return normalizeToolResult(value.content, depth + 1) || value;
  }
  if (typeof value.text === "string") {
    return normalizeToolResult(value.text, depth + 1) || value;
  }
  return value;
}

function toolEvidence(value) {
  const normalized = normalizeToolResult(value);
  return Array.isArray(normalized?.evidence) ? normalized.evidence : [];
}

function evidenceAgeLabel(value) {
  const age = Number(value);
  if (!Number.isFinite(age)) return "无时间戳";
  return age < 10 ? `${age.toFixed(1)}s` : `${Math.round(age)}s`;
}

function evidenceHtml(records) {
  if (!records.length) return "";
  return `<div class="tool-evidence">${records.slice(0, 4).map((item) => {
    const fresh = item?.fresh === true;
    const inference = item?.kind === "model_inference";
    const state = fresh ? "实时" : "过期";
    const kind = inference ? "模型推断" : "ROS 事实";
    return `<div class="evidence-line">
      <span class="evidence-state ${fresh ? "evidence-fresh" : "evidence-stale"}">${state}</span>
      <span class="evidence-source">${escapeHtml(item?.source || "未知来源")}</span>
      <span class="evidence-age">${escapeHtml(evidenceAgeLabel(item?.age_sec))} · ${kind}</span>
      <small>${escapeHtml(item?.summary || "无摘要")}</small>
    </div>`;
  }).join("")}</div>`;
}

function setupResizableCards() {
  document.querySelectorAll(".resizable-card[data-resize-key]").forEach((card) => {
    const key = card.getAttribute("data-resize-key");
    const handle = card.querySelector(".resize-grip");
    if (!key || !handle) return;

    const storedHeight = Number(localStorage.getItem(`${PANEL_HEIGHT_STORAGE_PREFIX}${key}`));
    if (Number.isFinite(storedHeight) && storedHeight > 0) {
      const restoredHeight = clampPanelHeight(storedHeight, key);
      card.style.height = `${restoredHeight}px`;
      if (restoredHeight !== storedHeight) {
        localStorage.setItem(`${PANEL_HEIGHT_STORAGE_PREFIX}${key}`, String(restoredHeight));
      }
    }

    handle.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      const startY = event.clientY;
      const startHeight = card.getBoundingClientRect().height;
      card.classList.add("resizing");
      handle.setPointerCapture?.(event.pointerId);

      const onPointerMove = (moveEvent) => {
        const nextHeight = clampPanelHeight(startHeight + moveEvent.clientY - startY, key);
        card.style.height = `${nextHeight}px`;
      };

      const onPointerUp = () => {
        const finalHeight = Math.round(card.getBoundingClientRect().height);
        localStorage.setItem(`${PANEL_HEIGHT_STORAGE_PREFIX}${key}`, String(finalHeight));
        card.classList.remove("resizing");
        window.removeEventListener("pointermove", onPointerMove);
        window.removeEventListener("pointerup", onPointerUp);
        window.removeEventListener("pointercancel", onPointerUp);
      };

      window.addEventListener("pointermove", onPointerMove);
      window.addEventListener("pointerup", onPointerUp);
      window.addEventListener("pointercancel", onPointerUp);
    });
  });
}

function clampPanelHeight(value, key = "") {
  const minimum = key === "mission-compact" ? 190 : 118;
  const viewportLimit = Math.max(minimum, Math.floor(window.innerHeight * 0.72));
  return Math.max(minimum, Math.min(viewportLimit, Number(value) || minimum));
}

function updateBadge(el, text, className = "badge neutral") {
  el.textContent = text;
  el.className = className;
}

function cloneJson(value) {
  if (typeof structuredClone === "function") return structuredClone(value);
  return JSON.parse(JSON.stringify(value));
}

function pushEvent(kind, message) {
  localEvents.push({ kind, message });
  localEvents = localEvents.slice(-80);
  renderEvents([]);
}

function renderEvents(remoteEvents) {
  const rows = [...(remoteEvents || []), ...localEvents].slice(-120);
  els.eventLog.innerHTML = rows
    .map((event) => `<div class="${event.kind || ""}">${escapeHtml(event.message || "")}</div>`)
    .join("");
  els.eventLog.scrollTop = els.eventLog.scrollHeight;
}

function loadChatHistory() {
  try {
    const parsed = JSON.parse(localStorage.getItem(CHAT_STORAGE_KEY) || "[]");
    return Array.isArray(parsed) ? parsed.slice(-60) : [];
  } catch (err) {
    return [];
  }
}

function saveChatHistory() {
  localStorage.setItem(CHAT_STORAGE_KEY, JSON.stringify(chatMessages.slice(-60)));
}

function addChat(role, content, meta = {}) {
  chatMessages.push({ role, content, ...meta });
  chatMessages = chatMessages.slice(-60);
  saveChatHistory();
  renderChat();
}

function findStreamingMessage(requestId) {
  return chatMessages.find((item) => item.requestId === requestId && item.streaming);
}

function ensureStreamingMessage(requestId, model = "Claude") {
  let message = findStreamingMessage(requestId);
  if (!message) {
    message = {
      role: "assistant",
      content: "",
      tag: `${model} · 正在回复`,
      requestId,
      streaming: true,
    };
    chatMessages.push(message);
  }
  return message;
}

function saveAndRenderChat() {
  chatMessages = chatMessages.slice(-60);
  saveChatHistory();
  renderChat();
}

function renderChat() {
  const previousScrollTop = els.chatHistory.scrollTop;
  const shouldFollow = chatAutoFollow || isChatNearBottom();
  const messages = chatMessages.length
    ? chatMessages.map((msg) => {
      const tag = msg.tag ? `<div class="muted" style="margin-top:4px">${escapeHtml(msg.tag)}</div>` : "";
      return `<div class="chat-message ${msg.role}"><div class="chat-content">${formatChatContent(msg.content)}</div>${tag}</div>`;
    }).join("")
    : `
      <div class="chat-message assistant">
        输入任务，例如“检查状态，如果安全就起飞到10米”。我会显示解析结果、技能选择和 ROS 工具调用。
      </div>
    `;
  const confirmation = pendingConfirmation
    ? `<div class="chat-message assistant chat-confirmation">
        <strong>等待操作员确认</strong>
        <span>${escapeHtml(pendingConfirmation.summary || "确认执行该控制任务")}</span>
        <small>${escapeHtml(pendingConfirmation.id || pendingConfirmation.token || "")}</small>
        <div class="confirmation-actions">
          <button class="primary" type="button" data-chat-confirm="confirm">确认执行</button>
          <button type="button" data-chat-confirm="cancel">取消</button>
        </div>
        <span class="muted">也可以直接在输入框发送“确认执行”或“取消执行”。</span>
      </div>`
    : "";
  els.chatHistory.innerHTML = messages + confirmation;
  if (shouldFollow) {
    scrollChatToBottom();
  } else {
    els.chatHistory.scrollTop = previousScrollTop;
    updateChatScrollButton();
  }
}

function isChatNearBottom() {
  return els.chatHistory.scrollHeight - els.chatHistory.scrollTop - els.chatHistory.clientHeight < 56;
}

function updateChatScrollButton() {
  const atBottom = isChatNearBottom();
  chatAutoFollow = atBottom;
  els.chatScrollBtn?.classList.toggle("hidden", atBottom);
}

function scrollChatToBottom() {
  els.chatHistory.scrollTop = els.chatHistory.scrollHeight;
  chatAutoFollow = true;
  els.chatScrollBtn?.classList.add("hidden");
}

function formatChatContent(content) {
  return escapeHtml(String(content || ""))
    .replace(/\*\*([^*\n]+)\*\*/g, "<strong>$1</strong>")
    .replace(/`([^`\n]+)`/g, "<code>$1</code>");
}

function parseAgentResult(result) {
  if (!result) return null;
  if (typeof result === "object") return result;
  try {
    return JSON.parse(result);
  } catch (err) {
    return { reply: String(result), parsed_task: null, tool_calls: [], final_status: String(result) };
  }
}

function renderAgentPayload(payload, serviceMessage = "") {
  lastAgentPayload = payload;
  if (!payload) return;

  const parsed = payload.parsed_task || {};
  const tools = payload.tool_calls || [];
  const skillInfo = parsed.skill_info || {};
  const reply = payload.reply || serviceMessage || "任务已处理";
  const finalStatus = payload.final_status || serviceMessage || "--";
  pendingConfirmation = payload.pending_confirmation || null;
  setupActiveVerification(payload);
  const confirmLabel = confirmationLabel(parsed, pendingConfirmation);
  const failedTool = tools.find((tool) => tool.status && tool.status !== "success");

  if (pendingConfirmation) {
    setTaskLifecycle("confirm", pendingConfirmation.summary || "任务解析完成，等待操作员确认");
  } else if (activeVerification) {
    setTaskLifecycle("verify", finalStatus || "控制请求已受理，等待真实飞控状态验证", {
      skipped: parsed.need_confirm ? [] : ["confirm"],
    });
  } else if (failedTool) {
    setTaskLifecycle("execute", finalStatus || `${failedTool.name || "ROS 工具"}调用失败`, {
      outcome: "error",
      activeText: "执行失败",
      skipped: parsed.need_confirm ? [] : ["confirm"],
    });
  } else {
    const skipped = [];
    if (!parsed.need_confirm) skipped.push("confirm");
    if (!tools.length) skipped.push("execute", "verify");
    else if (!payload.verification_done) skipped.push("verify");
    setTaskLifecycle("done", finalStatus || "任务处理完成", { skipped });
  }

  els.aiStatusText.textContent = "PARSED";
  els.planStatusText.textContent = parsed.skill || "PLAN";
  els.toolStatusText.textContent = tools.length ? `${tools.length} CALLS` : "NO CALL";
  els.missionPhase.textContent = payload.pending_confirmation
    ? "等待人工确认"
    : tools.length
      ? "ROS 工具执行"
      : "任务解析完成";
  els.missionResult.textContent = finalStatus;
  els.llmOutput.textContent = [
    `解析来源: ${parsed.parser === "llm" ? `模型解析 (${parsed.llm_model || "model"})` : "本地规则"}`,
    `用户意图: ${parsed.intent || "--"}`,
    `选择技能: ${parsed.skill || "--"}`,
    `技能说明: ${skillInfo.description || "--"}`,
    `风险等级: ${parsed.risk_level || "--"}`,
    `确认状态: ${confirmLabel}`,
    "",
    reply,
    "",
    `最终状态: ${finalStatus}`,
  ].join("\n");

  const planRows = buildPlanRows(parsed, tools, finalStatus, payload.progress || []);
  els.planSteps.innerHTML = planRows.map((row, index) => `
    <div class="plan-row">
      <span class="${row.className || (row.ok ? "ok" : "muted")}">${row.icon || (row.ok ? "✓" : "•")}</span>
      <span>${index + 1}. ${escapeHtml(row.text)}</span>
    </div>
  `).join("");

  els.toolCalls.innerHTML = tools.length
    ? tools.map((tool) => `
        <div class="tool-row">
          <span class="${tool.status === "success" ? "ok" : "fail"}">${tool.status === "success" ? "✓" : "×"}</span>
          <span>
            <strong>${escapeHtml(tool.name || "--")}</strong>
            <span class="muted"> ${escapeHtml(tool.target || "")}</span>
            ${tool.args && Object.keys(tool.args).length ? `<div class="muted">args: ${escapeHtml(JSON.stringify(tool.args))}</div>` : ""}
          </span>
        </div>
      `).join("")
    : "暂无工具调用";

  const parsedRows = [
    ["Intent", parsed.intent || "--"],
    ["Skill", parsed.skill || "--"],
    ["Risk", parsed.risk_level || "--"],
    ["Confirm", confirmLabel],
  ];
  if (parsed.args?.target_class) {
    parsedRows.push(["Target", `${parsed.args.target_class} / ${parsed.args.clothing_color || "不限服装"}`]);
  }
  if (parsed.args?.lost_target_action) {
    parsedRows.push(["Lost target", parsed.args.lost_target_action]);
  }
  if (parsed.execution_mode) {
    parsedRows.push(["Mode", parsed.execution_mode]);
  }
  els.parsedTask.innerHTML = parsedRows.map(([label, value]) => `
    <div><span>${escapeHtml(label)}</span><strong>${escapeHtml(value)}</strong></div>
  `).join("");
  els.taskResult.textContent = JSON.stringify(payload, null, 2);
  renderConfirmation();

  addChat("assistant", reply, { tag: finalStatus });
  tools.forEach((tool) => {
    addChat("tool", `${tool.status === "success" ? "成功" : "失败"}: ${tool.name} -> ${tool.target || ""}`);
  });
}

function confirmationLabel(parsed, pending) {
  if (pending) return "等待确认";
  if (parsed.confirmed) return "已确认";
  if (parsed.need_confirm) return "需要确认";
  return "不需要";
}

function setupActiveVerification(payload) {
  if (payload.verification_done) {
    activeVerification = null;
    return;
  }
  const parsed = payload.parsed_task || {};
  const serviceResult = parsed.service_result || {};
  if (serviceResult.accepted && ["takeoff", "land"].includes(parsed.intent)) {
    activeVerification = {
      intent: parsed.intent,
      altitude: Number(parsed.args?.altitude || 0),
      startedAt: Date.now(),
      payload,
    };
    return;
  }
  if (parsed.intent === "move_to" && !payload.pending_confirmation) {
    const progress = payload.progress || [];
    const hasTrajectoryStep = progress.some((step) => step.name === "execute_trajectory");
    if (hasTrajectoryStep || serviceResult.success) {
      activeVerification = {
        intent: "move_to",
        startedAt: Date.now(),
        payload,
        seenTracking: false,
      };
      return;
    }
  }
  if (payload.pending_confirmation) return;
  activeVerification = null;
}

function renderConfirmation() {
  if (!pendingConfirmation) {
    els.confirmationPanel.classList.add("hidden");
    els.confirmationText.textContent = "暂无待确认任务";
    els.controlConfirmationModal.classList.add("hidden");
    els.controlConfirmationModal.setAttribute("aria-hidden", "true");
    renderChat();
    return;
  }

  const isPx4FollowTask = pendingConfirmation.source === PX4_FOLLOW_TASK_SOURCE;
  const args = Object.keys(pendingConfirmation.args || {}).length
    ? `\n参数: ${JSON.stringify(pendingConfirmation.args)}`
    : "";
  els.confirmationPanel.classList.remove("hidden");
  setTaskLifecycle("confirm", pendingConfirmation.summary || "等待操作员确认高风险动作");
  els.confirmationText.textContent = [
    pendingConfirmation.summary || "确认执行该任务",
    `来源: ${isPx4FollowTask ? "PX4 任务解析器" : (pendingConfirmation.source === "gateway" ? "Agent MCP" : (pendingConfirmation.skill || "任务服务"))}`,
    `动作: ${pendingConfirmation.action || pendingConfirmation.intent || "--"}`,
    `风险: ${pendingConfirmation.risk_level || "--"}`,
    args,
  ].filter(Boolean).join("\n");
  els.controlConfirmationKind.textContent = isPx4FollowTask ? "PX4 智能跟踪任务" : "飞行控制请求";
  els.controlConfirmationSummary.textContent = pendingConfirmation.summary || "确认执行该控制任务";
  els.controlConfirmationAction.textContent = pendingConfirmation.action || pendingConfirmation.intent || "--";
  els.controlConfirmationId.textContent = pendingConfirmation.id || pendingConfirmation.token || "--";
  els.controlConfirmationRisk.textContent = String(pendingConfirmation.risk_level || "--").toUpperCase();
  els.controlConfirmationNote.textContent = isPx4FollowTask
    ? "自然语言指令已解析为目标类别 person / 白色服装和跟踪意图 visual_follow。确认后任务进入执行链，并在目标锁定后执行跟踪策略。"
    : "确认后将立即交给 ROS Agent 执行，并持续验证飞控遥测。";
  els.confirmTaskBtn.textContent = "确认执行";
  els.modalConfirmTaskBtn.textContent = "确认执行";
  els.controlConfirmationModal.classList.remove("hidden");
  els.controlConfirmationModal.setAttribute("aria-hidden", "false");
  renderChat();
}

async function refreshSkills() {
  renderSkills(gatewaySkills.length ? gatewaySkills : fallbackSkills);
  renderCapabilities(gatewayCapabilities.length ? gatewayCapabilities : fallbackCapabilities);
}

function renderCapabilities(capabilities) {
  if (!els.capabilityGrid) return;
  els.capabilityGrid.innerHTML = capabilities.map((item) => `
    <button class="skill-card" data-capability-task="${escapeHtml(item.example_task || "")}">
      <span>${escapeHtml(item.name || "CAPABILITY").toUpperCase()}</span>
      <strong>${escapeHtml(item.label || item.name || "--")}</strong>
      <small>${escapeHtml(item.description || "")}</small>
      <em>${escapeHtml(item.interface || item.mode || "ROS")}</em>
    </button>
  `).join("");
  els.capabilityGrid.querySelectorAll("[data-capability-task]").forEach((button) => {
    button.addEventListener("click", () => {
      const task = button.getAttribute("data-capability-task");
      if (!task) return;
      els.taskInput.value = task;
      submitAgentTask(task);
    });
  });
}

function mergeSkills(...groups) {
  const merged = new Map();
  groups.flat().forEach((skill) => {
    if (!skill?.name) return;
    merged.set(skill.name, { ...merged.get(skill.name), ...skill });
  });
  return [...merged.values()];
}

function renderSkills(skills) {
  els.skillGrid.innerHTML = skills.map((skill) => {
    const disabled = !skill.enabled;
    const riskClass = skill.risk_level === "high" ? "danger-text" : skill.risk_level === "medium" ? "warn-text" : "ok";
    return `
      <button class="skill-card ${disabled ? "disabled" : ""}" data-task="${escapeHtml(skill.example_task || "")}" ${disabled ? "disabled" : ""}>
        <span>${escapeHtml(skill.name || "SKILL").replace("Skill", "").toUpperCase()}</span>
        <strong>${escapeHtml(skill.label || skill.name || "--")}</strong>
        <small>${escapeHtml(skill.description || "")}</small>
        <em class="${riskClass}">${escapeHtml(skill.risk_level || "low")}${disabled ? " / 待接入" : ""}</em>
      </button>
    `;
  }).join("");

  els.skillGrid.querySelectorAll("[data-task]").forEach((button) => {
    button.addEventListener("click", () => {
      const task = button.getAttribute("data-task");
      if (!task) return;
      els.taskInput.value = task;
      submitAgentTask(task);
    });
  });
}

function buildPlanRows(parsed, tools, finalStatus, progress = []) {
  if (progress.length) {
    const icons = {
      done: ["✓", "ok"],
      active: ["●", "warn-text"],
      pending: ["•", "muted"],
      failed: ["×", "fail"],
      skipped: ["-", "muted"],
    };
    return progress.map((step) => {
      const [icon, className] = icons[step.status] || icons.pending;
      return {
        text: step.label || step.name || "--",
        icon,
        className,
        ok: step.status === "done",
      };
    });
  }

  const rows = [
    { text: "解析自然语言任务", ok: true },
    { text: `选择技能 ${parsed.skill || "--"}`, ok: Boolean(parsed.skill) },
  ];
  if (parsed.reason) rows.push({ text: parsed.reason, ok: true });
  tools.forEach((tool) => {
    rows.push({ text: `调用 ${tool.name || "--"} ${tool.target || ""}`, ok: tool.status === "success" });
  });
  if (finalStatus) rows.push({ text: finalStatus, ok: true });
  return rows;
}

function renderLiveMission(mission) {
  if (!mission || !Array.isArray(mission.steps)) return;

  const stepKey = mission.steps.map((step) => `${step.name}:${step.status}`).join("|");
  const missionKey = `${mission.id}:${mission.status}:${mission.current_index}:${stepKey}`;
  if (missionKey === lastLiveMissionKey) return;
  lastLiveMissionKey = missionKey;

  const statusLabel = {
    active: "任务已创建",
    running: "复合任务执行中",
    done: "复合任务完成",
    failed: "复合任务失败",
  }[mission.status] || "复合任务";
  const total = mission.steps.length;
  const current = Math.min(Number(mission.current_index || 0) + 1, total);
  els.missionPhase.textContent = ["active", "running"].includes(mission.status)
    ? `${statusLabel} ${current}/${total}`
    : statusLabel;
  els.missionResult.textContent = mission.report_md
    ? `${mission.message || "--"} · 报告: ${mission.report_md}`
    : (mission.message || "--");
  if (mission.status === "done") {
    setTaskLifecycle("done", mission.message || "组合任务已完成");
  } else if (mission.status === "failed") {
    setTaskLifecycle("execute", mission.message || "组合任务执行失败", {outcome: "error", activeText: "执行失败"});
  } else {
    setTaskLifecycle("execute", mission.message || `${statusLabel} ${current}/${total}`);
  }

  const icons = {
    done: ["✓", "ok"],
    active: ["●", "warn-text"],
    pending: ["•", "muted"],
    failed: ["×", "fail"],
    skipped: ["-", "muted"],
  };
  els.planSteps.innerHTML = mission.steps.map((step, index) => {
    const [icon, className] = icons[step.status] || icons.pending;
    const result = step.result?.message || step.result?.reason || "";
    return `
      <div class="plan-row">
        <span class="${className}">${icon}</span>
        <span>${index + 1}. ${escapeHtml(step.label || step.name || "--")}${result ? `<small>${escapeHtml(result)}</small>` : ""}</span>
      </div>
    `;
  }).join("");

  els.taskResult.textContent = JSON.stringify({mission}, null, 2);
  if (mission.status === "done" || mission.status === "failed") {
    pushEvent(mission.status === "done" ? "service" : "error", mission.message || statusLabel);
  }
}

function renderImageAnalysis(analysis) {
  if (!analysis || typeof analysis !== "object") return;
  const key = `${analysis.stamp || ""}:${analysis.scene || ""}:${analysis.suggestion || ""}`;
  if (!key || key === lastImageAnalysisKey) return;
  lastImageAnalysisKey = key;

  const objects = Array.isArray(analysis.objects)
    ? analysis.objects.map((item) => item.name || item.class_name || item.label || "").filter(Boolean).slice(0, 5).join("、")
    : "";
  const content = [
    `图像语义分析：${analysis.scene || "--"}`,
    `风险等级：${analysis.risk_level || "--"}`,
    objects ? `目标：${objects}` : "",
    `建议：${analysis.suggestion || "--"}`,
  ].filter(Boolean).join("\n");
  addChat("tool", content, { tag: analysis.source || "/perception/image_analysis" });
  pushEvent("service", `图像语义分析: ${analysis.risk_level || "--"} / ${analysis.suggestion || "--"}`);
}

async function refreshReports() {
  try {
    const data = await api("/api/reports");
    renderReports(data.reports || []);
  } catch (err) {
    if (els.reportList) {
      els.reportList.innerHTML = `<span class="muted">报告列表不可用</span>`;
    }
  }
}

function renderReports(reports) {
  if (!els.reportList) return;
  const key = reports.map((item) => `${item.id}:${item.updated_at}:${item.status}`).join("|");
  if (key === lastReportListKey) return;
  lastReportListKey = key;
  if (!reports.length) {
    els.reportList.innerHTML = `<span class="muted">暂无任务报告</span>`;
    return;
  }
  els.reportList.innerHTML = reports.slice(0, 5).map((item) => `
    <div class="report-row">
      ${reportThumbHtml(item)}
      <button class="report-open" type="button" data-report-id="${escapeHtml(item.id || "")}">
        <span>
          <strong>${escapeHtml(item.id || "--")}</strong>
          <small>${escapeHtml(item.message || item.report_md || "report.md")}</small>
        </span>
        <em>${escapeHtml(item.status || "--")}</em>
      </button>
      <button class="report-download" type="button" data-download-id="${escapeHtml(item.id || "")}">下载</button>
    </div>
  `).join("");
  els.reportList.querySelectorAll("[data-report-id]").forEach((button) => {
    button.addEventListener("click", () => openReport(button.getAttribute("data-report-id")));
  });
  els.reportList.querySelectorAll("[data-download-id]").forEach((button) => {
    button.addEventListener("click", () => downloadReport(button.getAttribute("data-download-id")));
  });
}

function reportThumbHtml(item) {
  const capture = Array.isArray(item.captures) ? item.captures[0] : null;
  if (!capture?.filename || !item.id) return `<div class="report-thumb empty-thumb">--</div>`;
  const src = `${apiBaseUrl}/api/reports/${encodeURIComponent(item.id)}/captures/${encodeURIComponent(capture.filename)}`;
  return `<img class="report-thumb" src="${escapeHtml(src)}" alt="任务截图" loading="lazy" />`;
}

function downloadReport(id) {
  if (!id) return;
  window.open(`${apiBaseUrl}/api/reports/${encodeURIComponent(id)}/download`, "_blank", "noopener");
}

async function openReport(id) {
  if (!id) return;
  try {
    const data = await api(`/api/reports/${encodeURIComponent(id)}`);
    if (!data.success) throw new Error(data.message || "报告读取失败");
    const captureText = Array.isArray(data.captures) && data.captures.length
      ? `\n\n截图文件：\n${data.captures.map((item) => `- ${item.filename}`).join("\n")}`
      : "";
    els.taskResult.textContent = `${data.content || "报告为空"}${captureText}`;
    els.missionResult.textContent = `已打开报告: ${data.report_md || id}`;
    addChat("tool", `已打开任务报告：${id}`, { tag: data.report_md || "report.md" });
  } catch (err) {
    pushEvent("error", `读取任务报告失败: ${err.message}`);
  }
}

async function refreshStatus() {
  try {
    const data = await api("/api/status");
    renderStatus(data, "HTTP");
  } catch (err) {
    els.systemLine.textContent = "ROS 网关断开";
    updateBadge(els.linkBadge, "网关断开", "badge bad");
    pushEvent("error", err.message);
  }
}

function renderStatus(data, source = "HTTP") {
  const state = data.state;
  const odom = data.odom;
  const telemetry = data.telemetry_meta || null;
  const stateFresh = telemetry ? telemetry.state_fresh === true : Boolean(state);
  const odomFresh = telemetry ? telemetry.odom_fresh === true : Boolean(odom);
  const depth = data.depth;
  const pointcloud = data.pointcloud;
  const autonomy = data.autonomy;
  renderPerceptionHealth(data.perception_health || null);
  renderWorldModelHealth(data.world_model_meta || null);
  updateMissionMap(
    odomFresh ? odom : null,
    data.autonomy_goal,
    data.autonomy_trajectory,
    Array.isArray(data.world_objects) ? data.world_objects : [],
    data.world_model_meta || null,
  );
  renderLiveMission(data.mission);
  renderImageAnalysis(data.image_analysis);
  const reportedDetections = Array.isArray(data.detections)
    ? data.detections
    : (data.detection ? [data.detection] : []);
  latestDetectionMeta = data.detection_meta || null;
  const detectionAge = Number(latestDetectionMeta?.age_s);
  const detectionAgeValid = latestDetectionMeta?.age_s == null
    || (Number.isFinite(detectionAge) && detectionAge <= 3);
  const detectionsActive = latestDetectionMeta
    ? latestDetectionMeta.active === true && detectionAgeValid
    : reportedDetections.length > 0;
  latestDetections = detectionsActive ? reportedDetections : [];
  renderDetectionSummary(latestDetections, latestDetectionMeta);
  const overlayKey = detectionOverlayKey(latestDetections, detectionsActive);
  if (overlayKey !== lastDetectionOverlayKey) {
    lastDetectionOverlayKey = overlayKey;
    redrawCameraDetections();
  }

  if (telemetry && !telemetry.active) {
    const age = Math.max(
      Number(telemetry.state_age_s) || 0,
      Number(telemetry.odom_age_s) || 0,
    );
    els.systemLine.textContent = `ROS 网关在线 · 飞控遥测超时 ${fmt(age, 1)}s`;
    updateBadge(els.linkBadge, "遥测失联", "badge bad");
  } else {
    els.systemLine.textContent = `ROS 网关已连接 · ${source}`;
    updateBadge(els.linkBadge, source === "WS" ? "实时连接" : "HTTP 轮询", "badge");
  }
  if (state && stateFresh) {
    const landed = state.landed_valid === true && state.landed === true;
    const preflightFailed = state.preflight_valid === true && state.preflight_ok !== true;
    const flightHealthOk = state.ekf_healthy === true && !preflightFailed;
    const flightHealthText = preflightFailed
      ? "PX4 预检失败"
      : (state.ekf_healthy ? "EKF 正常" : "EKF 异常");
    const armedText = state.armed ? "已解锁" : (landed ? "已落地/加锁" : "未解锁");
    updateBadge(els.armedBadge, armedText, state.armed ? "badge" : (landed ? "badge neutral" : "badge bad"));
    updateBadge(els.modeBadge, state.mode || "未知模式", state.mode === "OFFBOARD" ? "badge" : "badge neutral");
    updateBadge(els.ekfBadge, flightHealthText, flightHealthOk ? "badge" : "badge bad");

    els.armedValue.textContent = state.armed ? "是" : "否";
    els.modeValue.textContent = state.mode || "--";
    els.batteryValue.textContent = Number(state.battery) > 0
      ? `${fmt(state.battery, 1)} V`
      : "--";
    const gpsLabels = {0: "无定位", 1: "2D", 2: "3D", 3: "差分", 4: "RTK"};
    els.gpsValue.textContent = gpsLabels[Number(state.gps_fix)] || "--";
    els.ekfValue.textContent = preflightFailed
      ? `${state.ekf_healthy ? "EKF正常" : "EKF异常"} / 预检失败`
      : (state.ekf_healthy ? "正常" : "异常");
    els.hudMode.textContent = `MODE ${state.mode || "--"}`;
    els.chatArmedValue.textContent = armedText;
    els.chatArmedValue.className = state.armed ? "status-ok" : (landed ? "" : "status-warn");
    els.chatModeValue.textContent = state.mode || "--";
    els.chatBatteryValue.textContent = Number(state.battery) > 0
      ? `${fmt(state.battery, 1)} V`
      : "--";
    els.chatGpsValue.textContent = gpsLabels[Number(state.gps_fix)] || "--";
    els.chatEkfValue.textContent = preflightFailed
      ? "预检失败"
      : (state.ekf_healthy ? "正常" : "异常");
    els.chatEkfValue.className = flightHealthOk ? "status-ok" : "status-bad";
  } else {
    updateBadge(els.armedBadge, "状态未知", "badge bad");
    updateBadge(els.modeBadge, "遥测超时", "badge neutral");
    updateBadge(els.ekfBadge, "EKF 未知", "badge bad");
    els.armedValue.textContent = "--";
    els.modeValue.textContent = "--";
    els.batteryValue.textContent = "--";
    els.gpsValue.textContent = "--";
    els.ekfValue.textContent = "--";
    els.hudMode.textContent = "MODE --";
    els.chatArmedValue.textContent = "状态未知";
    els.chatArmedValue.className = "status-bad";
    els.chatModeValue.textContent = "--";
    els.chatBatteryValue.textContent = "--";
    els.chatGpsValue.textContent = "--";
    els.chatEkfValue.textContent = "未知";
    els.chatEkfValue.className = "status-bad";
  }

  if (odom && odomFresh) {
    els.frameValue.textContent = odom.frame_id || "--";
    els.posX.textContent = fmt(odom.position.x);
    els.posY.textContent = fmt(odom.position.y);
    els.posZ.textContent = fmt(odom.position.z);
    els.velX.textContent = fmt(odom.velocity.x);
    els.velY.textContent = fmt(odom.velocity.y);
    els.velZ.textContent = fmt(odom.velocity.z);
    els.chatPositionValue.textContent = [
      fmt(odom.position.x, 1),
      fmt(odom.position.y, 1),
      fmt(odom.position.z, 1),
    ].join(" / ");
  } else {
    els.frameValue.textContent = "--";
    els.posX.textContent = "--";
    els.posY.textContent = "--";
    els.posZ.textContent = "--";
    els.velX.textContent = "--";
    els.velY.textContent = "--";
    els.velZ.textContent = "--";
    els.chatPositionValue.textContent = "-- / -- / --";
  }

  updateActiveVerification(
    stateFresh ? state : null,
    odomFresh ? odom : null,
    autonomy,
  );

  if (depth) {
    els.depthMeta.textContent = `${depth.width}x${depth.height} ${depth.encoding}`;
    els.depthCenter.textContent = meters(depth.center_m);
    els.depthMin.textContent = meters(depth.min_m);
    els.depthMax.textContent = meters(depth.max_m);
    els.depthSamples.textContent = String(depth.valid_samples ?? "--");
  }

  if (autonomy) {
    els.autonomyMeta.textContent = autonomy.enabled ? "已启用" : "未接管";
    els.autonomyState.textContent = autonomy.state || "--";
    els.autonomyStrategy.textContent = autonomy.active_strategy || "--";
    els.autonomyNearest.textContent = meters(autonomy.nearest_obstacle_m);
    els.autonomyTarget.textContent = meters(autonomy.target_distance_m);
    const takeoffClearance = autonomy.takeoff_clearance_valid
      ? `起飞区域净空 ${meters(autonomy.takeoff_clearance_m)}`
      : "起飞区域净空不可用";
    els.autonomyMessage.textContent = `${takeoffClearance} · ${autonomy.message || "等待任务状态"}`;
  }

  if (pointcloud && !threePointcloud) {
    els.pointcloudMeta.textContent = `${pointcloud.total_points || 0} 点 / ${pointcloud.frame_id || "--"}`;
  }

  const services = data.services || {};
  const ready = Object.values(services).filter(Boolean).length;
  els.serviceBadge.textContent = `${ready}/${Object.keys(services).length} 服务`;
  els.agentBadge.textContent = services.agent ? "任务服务就绪" : "任务服务离线";
  renderEvents(data.events || []);
}

function renderWorldModelHealth(meta) {
  if (!els.worldHealthStatus || !els.worldTrackCount) return;
  const health = meta?.health;
  if (!health) {
    els.worldHealthStatus.textContent = "语义融合等待数据";
    els.worldTrackCount.textContent = "0 稳定目标";
    if (els.worldPathRisk) {
      els.worldPathRisk.textContent = "未激活";
      els.worldPathRisk.className = "";
    }
    return;
  }
  const sync = Number(health.sync_delta_s);
  const syncText = Number.isFinite(sync) ? ` · 同步差 ${fmt(sync * 1000, 0)}ms` : "";
  els.worldHealthStatus.textContent = `${health.healthy ? "语义融合正常" : "语义融合降级"}${syncText}`;
  els.worldTrackCount.textContent = `${Number(health.confirmed_track_count) || 0} 稳定目标`;
  if (els.worldPathRisk) {
    const intrusions = Array.isArray(meta?.relations)
      ? meta.relations.filter((item) => item.predicate === "intersects_path")
      : [];
    els.worldPathRisk.textContent = intrusions.length
      ? `${intrusions.length} 个目标侵入`
      : (Array.isArray(meta?.relations) && meta.relations.length ? "航线清晰" : "未激活");
    els.worldPathRisk.className = intrusions.length ? "status-bad" : "status-ok";
  }
}

function renderPerceptionHealth(health) {
  if (!els.perceptionHealthStatus || !els.perceptionHealthDetails) return;
  if (!health) {
    els.perceptionHealthStatus.textContent = "感知链路等待数据";
    els.perceptionHealthStatus.className = "stale";
    els.perceptionHealthDetails.textContent = "RGB -- · DEPTH -- · LIDAR --";
    return;
  }
  const overall = String(health.overall_status || "MISSING").toUpperCase();
  const labels = {OK: "感知链路正常", DEGRADED: "感知链路降级", ERROR: "感知链路异常"};
  els.perceptionHealthStatus.textContent = labels[overall] || `感知链路 ${overall}`;
  els.perceptionHealthStatus.className = overall === "OK" ? "live" : "stale";
  const state = (name) => String(health?.[name]?.status || "MISSING").toUpperCase();
  els.perceptionHealthDetails.textContent = [
    `RGB ${state("rgb")}`,
    `DEPTH ${state("depth")}`,
    `LIDAR ${state("pointcloud")}`,
    `YOLO ${state("detections")}`,
    `VLM ${state("vlm")}`,
  ].join(" · ");
}

function updateMissionMap(odom, goal, trajectory, worldObjects = [], worldMeta = null) {
  const position = odom?.position;
  const frame = odom?.frame_id || goal?.frame_id || trajectory?.frame_id || "odom";
  if (frame !== missionTrackFrame) {
    missionTrack = [];
    missionTrackFrame = frame;
  }
  if (Number.isFinite(Number(position?.x)) && Number.isFinite(Number(position?.y))) {
    const point = {x: Number(position.x), y: Number(position.y), z: Number(position.z || 0)};
    const previous = missionTrack.at(-1);
    const distance = previous ? Math.hypot(point.x - previous.x, point.y - previous.y) : Infinity;
    if (!previous || distance >= 0.1) {
      if (distance > 250) missionTrack = [];
      missionTrack.push(point);
      missionTrack = missionTrack.slice(-800);
    }
  }
  drawMissionMap(odom, goal, trajectory, worldObjects, worldMeta);
}

function drawMissionMap(odom, goal, trajectory, worldObjects = [], worldMeta = null) {
  const canvas = els.missionMapCanvas;
  if (!canvas) return;
  const frame = odom?.frame_id
    || goal?.frame_id
    || trajectory?.frame_id
    || missionTrackFrame
    || "odom";
  const rect = canvas.getBoundingClientRect();
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const width = Math.max(260, rect.width || 280);
  const height = Math.max(140, rect.height || 166);
  const pixelWidth = Math.round(width * dpr);
  const pixelHeight = Math.round(height * dpr);
  if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
    canvas.width = pixelWidth;
    canvas.height = pixelHeight;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#0d1014";
  ctx.fillRect(0, 0, width, height);

  const planned = Array.isArray(trajectory?.points)
    ? trajectory.points.filter((point) => Number.isFinite(Number(point.x)) && Number.isFinite(Number(point.y)))
    : [];
  const goalPoint = goal?.position && Number.isFinite(Number(goal.position.x))
    ? {x: Number(goal.position.x), y: Number(goal.position.y), z: Number(goal.position.z || 0)}
    : null;
  const current = odom?.position && Number.isFinite(Number(odom.position.x))
    ? {x: Number(odom.position.x), y: Number(odom.position.y), z: Number(odom.position.z || 0)}
    : null;
  const worldFrame = worldMeta?.frame_id || "";
  const locatedObjects = worldObjects.filter((item) => (
    (!worldFrame || worldFrame === frame)
    && item?.position_valid
    && Number.isFinite(Number(item.position?.x))
    && Number.isFinite(Number(item.position?.y))
  ));
  const objectPoints = locatedObjects.map((item) => ({x: Number(item.position.x), y: Number(item.position.y), z: Number(item.position.z || 0)}));
  const allPoints = [...missionTrack, ...planned, ...objectPoints, ...(goalPoint ? [goalPoint] : []), ...(current ? [current] : [])];
  if (!allPoints.length) {
    ctx.fillStyle = "#69727e";
    ctx.font = "10px system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("等待世界坐标轨迹", width / 2, height / 2);
    els.missionMapMeta.textContent = "等待 odom / autonomy 数据";
    return;
  }

  const xs = allPoints.map((point) => Number(point.x));
  const ys = allPoints.map((point) => Number(point.y));
  const centerX = (Math.min(...xs) + Math.max(...xs)) / 2;
  const centerY = (Math.min(...ys) + Math.max(...ys)) / 2;
  const span = Math.max(20, Math.max(...xs) - Math.min(...xs), Math.max(...ys) - Math.min(...ys)) * 1.22;
  const padding = 24;
  const scale = Math.min((width - padding * 2) / span, (height - padding * 2) / span);
  const project = (point) => ({
    x: width / 2 + (Number(point.x) - centerX) * scale,
    y: height / 2 - (Number(point.y) - centerY) * scale,
  });
  const gridStep = niceMapStep(span / 5);
  const minGridX = Math.floor((centerX - span / 2) / gridStep) * gridStep;
  const maxGridX = Math.ceil((centerX + span / 2) / gridStep) * gridStep;
  const minGridY = Math.floor((centerY - span / 2) / gridStep) * gridStep;
  const maxGridY = Math.ceil((centerY + span / 2) / gridStep) * gridStep;
  ctx.lineWidth = 1;
  ctx.strokeStyle = "#1c2229";
  for (let x = minGridX; x <= maxGridX; x += gridStep) {
    const a = project({x, y: minGridY});
    const b = project({x, y: maxGridY});
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
  }
  for (let y = minGridY; y <= maxGridY; y += gridStep) {
    const a = project({x: minGridX, y});
    const b = project({x: maxGridX, y});
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
  }

  drawMapPath(ctx, missionTrack, project, "#25b8c7", false, 2);
  drawMapPath(ctx, planned, project, trajectory?.collision_free === false ? "#ed5c5c" : "#34c487", true, 1.5);
  if (missionTrack.length) {
    const home = project(missionTrack[0]);
    ctx.fillStyle = "#d5dae0";
    ctx.beginPath(); ctx.arc(home.x, home.y, 3, 0, Math.PI * 2); ctx.fill();
  }
  if (goalPoint) drawMapGoal(ctx, project(goalPoint));
  const intrusionIds = new Set(
    (Array.isArray(worldMeta?.relations) ? worldMeta.relations : [])
      .filter((item) => item.predicate === "intersects_path")
      .map((item) => item.subject_id),
  );
  locatedObjects.forEach((item) => drawMapSemanticObject(
    ctx,
    project(item.position),
    {...item, path_intrusion: intrusionIds.has(item.id)},
  ));
  if (current) drawMapVehicle(ctx, project(current), odom?.velocity);

  const z = current ? `${current.z.toFixed(1)}m` : "--";
  const mode = trajectory?.planner_mode || "无规划轨迹";
  const semanticText = worldMeta?.active
    ? `语义目标 ${worldObjects.length} (${worldFrame || frame})`
    : "语义模型等待";
  els.missionMapMeta.textContent = `${missionTrackFrame || "odom"} · 网格 ${gridStep}m · Z ${z} · ${semanticText} · ${mode}`;
}

function niceMapStep(value) {
  const exponent = Math.floor(Math.log10(Math.max(0.1, value)));
  const fraction = value / (10 ** exponent);
  const nice = fraction <= 1 ? 1 : fraction <= 2 ? 2 : fraction <= 5 ? 5 : 10;
  return nice * (10 ** exponent);
}

function drawMapPath(ctx, points, project, color, dashed, lineWidth) {
  if (points.length < 2) return;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.setLineDash(dashed ? [5, 4] : []);
  ctx.beginPath();
  points.forEach((point, index) => {
    const pixel = project(point);
    if (index === 0) ctx.moveTo(pixel.x, pixel.y);
    else ctx.lineTo(pixel.x, pixel.y);
  });
  ctx.stroke();
  ctx.restore();
}

function drawMapGoal(ctx, point) {
  ctx.save();
  ctx.strokeStyle = "#e8b84a";
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(point.x, point.y, 6, 0, Math.PI * 2); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(point.x - 9, point.y); ctx.lineTo(point.x + 9, point.y); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(point.x, point.y - 9); ctx.lineTo(point.x, point.y + 9); ctx.stroke();
  ctx.restore();
}

function drawMapSemanticObject(ctx, point, item) {
  ctx.save();
  ctx.fillStyle = item.path_intrusion ? "#ff3f52" : item.dynamic ? "#ed5c5c" : "#e8b84a";
  ctx.strokeStyle = "#0d1014";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.arc(point.x, point.y, item.path_intrusion ? 7 : item.dynamic ? 5 : 4, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  ctx.fillStyle = "#dce3e8";
  ctx.font = "10px system-ui, sans-serif";
  ctx.textAlign = "left";
  ctx.fillText(`${item.class_name || "object"} ${item.id || ""}`.trim(), point.x + 7, point.y - 5);
  ctx.restore();
}

function drawMapVehicle(ctx, point, velocity) {
  const vx = Number(velocity?.x || 0);
  const vy = Number(velocity?.y || 0);
  const angle = Math.hypot(vx, vy) > 0.1 ? Math.atan2(vy, vx) : 0;
  ctx.save();
  ctx.translate(point.x, point.y);
  ctx.rotate(-angle);
  ctx.fillStyle = "#f3f6f8";
  ctx.beginPath();
  ctx.moveTo(8, 0); ctx.lineTo(-6, -5); ctx.lineTo(-3, 0); ctx.lineTo(-6, 5); ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function updateActiveVerification(state, odom, autonomy) {
  if (!activeVerification || !lastAgentPayload) return;
  if (activeVerification.intent === "takeoff") {
    updateTakeoffVerification(state, odom);
  } else if (activeVerification.intent === "land") {
    updateLandVerification(state, odom);
  } else if (activeVerification.intent === "move_to") {
    updateMoveVerification(autonomy);
  }
}

function updateTakeoffVerification(state, odom) {
  const altitude = Number(odom?.position?.z);
  const target = activeVerification.altitude;
  const mode = String(state?.mode || "");
  const airborneMode = /TAKEOFF|OFFBOARD|LOITER|HOLD/i.test(mode);
  const altitudeReached = Number.isFinite(altitude) && target > 0 && altitude >= Math.max(0.8, target * 0.75);
  const elapsedMs = Date.now() - activeVerification.startedAt;

  if (!altitudeReached && !airborneMode && elapsedMs < 120000) return;

  const payload = cloneJson(lastAgentPayload);
  payload.verification_done = altitudeReached || airborneMode;
  payload.final_status = altitudeReached
    ? `起飞状态已验证：当前高度 ${altitude.toFixed(2)} m，目标 ${target.toFixed(1)} m`
    : `起飞请求已下发，飞控模式 ${mode || "--"}，请继续观察高度`;
  payload.progress = (payload.progress || []).map((step) => {
    if (step.name === "verify") {
      return {
        ...step,
        label: payload.final_status,
        status: altitudeReached || airborneMode ? "done" : "active",
      };
    }
    return step;
  });
  lastAgentPayload = payload;
  activeVerification = altitudeReached || airborneMode ? null : activeVerification;
  renderAgentPayload(payload, payload.final_status);
  if (altitudeReached || airborneMode) {
    pushEvent("service", payload.final_status);
  }
}

function updateLandVerification(state, odom) {
  const altitude = Number(odom?.position?.z);
  const mode = String(state?.mode || "");
  const armed = Boolean(state?.armed);
  const landedByAltitude = Number.isFinite(altitude) && altitude <= 0.35;
  const landedByMode = /LAND|DISARM/i.test(mode) && !/TAKEOFF/i.test(mode);
  const disarmed = Boolean(state) && !armed;
  const elapsedMs = Date.now() - activeVerification.startedAt;

  if (!landedByAltitude && !landedByMode && !disarmed && elapsedMs < 120000) return;

  const payload = cloneJson(lastAgentPayload);
  const done = landedByAltitude || landedByMode || disarmed;
  payload.verification_done = done;
  if (landedByAltitude) {
    payload.final_status = `降落状态已验证：当前高度 ${altitude.toFixed(2)} m`;
  } else if (disarmed) {
    payload.final_status = "降落状态已验证：无人机已加锁";
  } else {
    payload.final_status = `降落请求已下发，飞控模式 ${mode || "--"}，请继续观察高度`;
  }
  payload.progress = (payload.progress || []).map((step) => {
    if (step.name === "verify") {
      return {
        ...step,
        label: payload.final_status,
        status: done ? "done" : "active",
      };
    }
    return step;
  });
  lastAgentPayload = payload;
  activeVerification = done ? null : activeVerification;
  renderAgentPayload(payload, payload.final_status);
  if (done) {
    pushEvent("service", payload.final_status);
  }
}

function updateMoveVerification(autonomy) {
  if (!autonomy) return;
  const strategy = String(autonomy.active_strategy || "");
  const state = String(autonomy.state || "");
  const distance = Number(autonomy.target_distance_m);
  const elapsedMs = Date.now() - activeVerification.startedAt;
  const tracking = ["direct_goal", "slow_goal", "left_replan", "right_replan", "climb_replan", "descend_replan"].includes(strategy);
  if (tracking || state === "TRACKING") {
    activeVerification.seenTracking = true;
  }

  const reached = (
    state === "ARRIVED"
    || strategy === "goal_reached"
    || (activeVerification.seenTracking && strategy === "hover_no_goal")
    || (activeVerification.seenTracking && Number.isFinite(distance) && distance <= 0.7)
  );
  const blocked = state === "BLOCKED_HOLD" || strategy === "blocked_hold";
  if (!reached && !blocked && elapsedMs < 180000) return;

  const payload = cloneJson(lastAgentPayload);
  const done = reached && !blocked;
  payload.verification_done = done || blocked;
  payload.final_status = done
    ? "自主移动状态已验证：底层轨迹执行完成，目标已清空并进入悬停"
    : blocked
      ? `自主移动被阻塞：${autonomy.message || "连续无安全轨迹，已进入悬停"}`
      : "自主移动仍在执行，请继续观察 autonomy 状态";
  payload.progress = (payload.progress || []).map((step) => {
    if (step.name === "publish_goal") {
      return {...step, status: activeVerification.seenTracking || reached || blocked ? "done" : step.status};
    }
    if (step.name === "execute_trajectory") {
      return {
        ...step,
        label: payload.final_status,
        status: done ? "done" : blocked ? "failed" : "active",
      };
    }
    return step;
  });
  lastAgentPayload = payload;
  activeVerification = done || blocked ? null : activeVerification;
  renderAgentPayload(payload, payload.final_status);
  if (done || blocked) {
    pushEvent(done ? "service" : "error", payload.final_status);
  }
}

function websocketUrl(path) {
  const url = new URL(apiBaseUrl);
  url.protocol = url.protocol === "https:" ? "wss:" : "ws:";
  url.pathname = path;
  url.search = "";
  return url.toString();
}

function connectTelemetrySocket() {
  if (telemetrySocket) {
    telemetrySocket.close();
    telemetrySocket = null;
  }
  if (telemetrySocketTimer) {
    clearTimeout(telemetrySocketTimer);
    telemetrySocketTimer = null;
  }

  try {
    telemetrySocket = new WebSocket(websocketUrl("/ws"));
  } catch (err) {
    pushEvent("error", `WebSocket 初始化失败: ${err.message}`);
    return;
  }

  telemetrySocket.addEventListener("open", () => {
    pushEvent("system", "WebSocket 实时状态已连接");
  });
  telemetrySocket.addEventListener("message", (event) => {
    try {
      const payload = JSON.parse(event.data);
      if (payload.type === "telemetry") {
        renderStatus(payload.data, "WS");
      }
    } catch (err) {
      pushEvent("error", `WebSocket 数据解析失败: ${err.message}`);
    }
  });
  telemetrySocket.addEventListener("close", () => {
    telemetrySocket = null;
    telemetrySocketTimer = setTimeout(connectTelemetrySocket, 2000);
  });
  telemetrySocket.addEventListener("error", () => {
    pushEvent("error", "WebSocket 连接异常，保留 HTTP 轮询兜底");
  });
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
  const colorMode = els.cameraColorMode.value || "auto";
  const swapRedBlue = colorMode === "bgr" || (colorMode === "auto" && encoding.includes("bgr"));

  for (let y = 0; y < height; y += 1) {
    for (let x = 0; x < width; x += 1) {
      const src = y * step + x * channels;
      const dst = (y * width + x) * 4;
      if (swapRedBlue) {
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
  const baseImageData = new ImageData(rgba, width, height);
  ctx.putImageData(baseImageData, 0, 0);
  drawDetections(ctx, width, height);
  els.cameraEmpty.style.display = "none";
  const detectionText = latestDetections.length ? ` · ${latestDetections.length} 目标` : "";
  const colorText = colorMode === "auto" ? image.encoding : colorMode.toUpperCase();
  els.cameraMeta.textContent = `${width}x${height} ${colorText}${detectionText}`;
  latestCameraFrame = {
    width,
    height,
    encoding: image.encoding || "--",
    detectionCount: latestDetections.length,
    baseImageData,
  };
  if (isCameraViewerOpen()) {
    renderCameraViewer();
  }
}

function isCameraViewerOpen() {
  return els.cameraViewer && !els.cameraViewer.classList.contains("hidden");
}

function openCameraViewer() {
  if (!latestCameraFrame || !els.cameraCanvas.width || !els.cameraCanvas.height) {
    pushEvent("error", "当前没有可放大的 RGB 画面");
    return;
  }
  els.cameraViewer.classList.remove("hidden");
  els.cameraViewer.setAttribute("aria-hidden", "false");
  requestAnimationFrame(() => fitCameraViewer());
}

function closeCameraViewer() {
  els.cameraViewer.classList.add("hidden");
  els.cameraViewer.setAttribute("aria-hidden", "true");
}

function fitCameraViewer() {
  if (!latestCameraFrame || !els.cameraViewerStage) return;
  const stage = els.cameraViewerStage.getBoundingClientRect();
  const fit = Math.min(
    (stage.width - 36) / latestCameraFrame.width,
    (stage.height - 36) / latestCameraFrame.height,
  );
  setCameraViewerZoom(fit);
}

function setCameraViewerZoom(value) {
  cameraViewerZoom = Math.max(0.1, Math.min(6, Number(value) || 1));
  renderCameraViewer();
}

function renderCameraViewer() {
  if (!els.cameraViewerCanvas || !latestCameraFrame) return;
  const source = els.cameraCanvas;
  const canvas = els.cameraViewerCanvas;
  const ctx = canvas.getContext("2d");
  canvas.width = source.width;
  canvas.height = source.height;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.drawImage(source, 0, 0);
  canvas.style.width = `${Math.round(source.width * cameraViewerZoom)}px`;
  canvas.style.height = `${Math.round(source.height * cameraViewerZoom)}px`;
  const detectionText = latestCameraFrame.detectionCount ? ` · ${latestCameraFrame.detectionCount} 目标` : "";
  els.cameraViewerMeta.textContent = `${latestCameraFrame.width}x${latestCameraFrame.height} ${latestCameraFrame.encoding}${detectionText}`;
  els.cameraZoomValue.textContent = `${Math.round(cameraViewerZoom * 100)}%`;
}

function drawDetections(ctx, width, height) {
  if (!latestDetections.length) return;
  ctx.save();
  ctx.lineWidth = Math.max(2, Math.round(width / 640));
  ctx.font = `${Math.max(14, Math.round(width / 70))}px system-ui, sans-serif`;
  latestDetections.forEach((det) => {
    let boxWidth = Number(det.width || 0);
    let boxHeight = Number(det.height || 0);
    let cx = Number(det.x || 0);
    let cy = Number(det.y || 0);
    const normalized = boxWidth <= 1.5 && boxHeight <= 1.5 && cx <= 1.5 && cy <= 1.5;
    if (normalized) {
      boxWidth *= width;
      boxHeight *= height;
      cx *= width;
      cy *= height;
    }
    if (boxWidth <= 0 || boxHeight <= 0) return;
    const x = Math.max(0, cx - boxWidth / 2);
    const y = Math.max(0, cy - boxHeight / 2);
    const w = Math.min(width - x, boxWidth);
    const h = Math.min(height - y, boxHeight);
    if (w <= 0 || h <= 0) return;
    const label = `${det.class_name || "target"} ${Math.round(Number(det.confidence || 0) * 100)}%`;
    const color = detectionColor(det.class_name);
    ctx.strokeStyle = color;
    ctx.fillStyle = "rgba(10, 14, 20, 0.78)";
    ctx.strokeRect(x, y, w, h);
    const corner = Math.max(8, Math.min(w, h) * 0.12);
    ctx.lineWidth *= 1.6;
    [
      [x, y + corner, x, y, x + corner, y],
      [x + w - corner, y, x + w, y, x + w, y + corner],
      [x, y + h - corner, x, y + h, x + corner, y + h],
      [x + w - corner, y + h, x + w, y + h, x + w, y + h - corner],
    ].forEach((points) => {
      ctx.beginPath();
      ctx.moveTo(points[0], points[1]);
      ctx.lineTo(points[2], points[3]);
      ctx.lineTo(points[4], points[5]);
      ctx.stroke();
    });
    ctx.lineWidth /= 1.6;
    const textWidth = Math.min(width - x, ctx.measureText(label).width + 12);
    const textHeight = Math.max(20, Math.round(width / 50));
    const labelY = y >= textHeight ? y - textHeight : y;
    ctx.fillRect(x, labelY, textWidth, textHeight);
    ctx.fillStyle = color;
    ctx.fillText(label, x + 6, labelY + textHeight - 6, Math.max(0, textWidth - 10));
  });
  ctx.restore();
}

function detectionOverlayKey(detections, active) {
  if (!active) return "stale";
  return detections.map((detection) => [
    detection.class_name,
    Number(detection.confidence || 0).toFixed(3),
    Number(detection.x || 0).toFixed(1),
    Number(detection.y || 0).toFixed(1),
    Number(detection.width || 0).toFixed(1),
    Number(detection.height || 0).toFixed(1),
  ].join(":")).join("|");
}

function redrawCameraDetections() {
  if (!latestCameraFrame?.baseImageData || !els.cameraCanvas) return;
  const ctx = els.cameraCanvas.getContext("2d");
  ctx.putImageData(latestCameraFrame.baseImageData, 0, 0);
  drawDetections(ctx, latestCameraFrame.width, latestCameraFrame.height);
  latestCameraFrame.detectionCount = latestDetections.length;
  if (isCameraViewerOpen()) renderCameraViewer();
}

function detectionColor(className) {
  const palette = ["#25b8c7", "#34c487", "#e8b84a", "#6ea8fe", "#d985d4", "#ed775c"];
  const text = String(className || "target");
  let hash = 0;
  for (let index = 0; index < text.length; index += 1) {
    hash = ((hash * 31) + text.charCodeAt(index)) >>> 0;
  }
  return palette[hash % palette.length];
}

function renderDetectionSummary(detections, meta) {
  if (!els.detectionStatus || !els.detectionList) return;
  const age = Number(meta?.age_s);
  const hasAge = meta?.age_s != null && Number.isFinite(age);
  const healthState = String(meta?.health_status || "").toUpperCase();
  const active = healthState
    ? healthState === "OK"
    : (meta ? meta.active !== false : detections.length > 0);
  const frame = meta?.frame_id || "camera";
  els.detectionStatus.className = active ? "live" : "stale";
  const disabled = healthState === "DISABLED";
  const error = healthState === "ERROR";
  els.detectionStatus.textContent = active
    ? `YOLO 实时检测 · ${frame}`
    : disabled
      ? "YOLO 已关闭"
      : error
        ? "YOLO 运行异常"
        : `检测数据超时${hasAge ? ` · ${age.toFixed(1)}s` : ""}`;
  els.detectionCount.textContent = active ? `${detections.length} 目标` : disabled ? "未启用" : "数据失效";

  if (!active) {
    els.detectionList.innerHTML = `<span class="muted">${disabled ? "启动时设置 yolo_enabled:=true 可启用检测" : "等待 /perception/detections 恢复"}</span>`;
    return;
  }
  if (!detections.length) {
    els.detectionList.innerHTML = `<span class="muted">当前画面未检测到目标</span>`;
    return;
  }

  const groups = new Map();
  detections.forEach((detection) => {
    const name = detection.class_name || "target";
    const current = groups.get(name) || {count: 0, confidence: 0};
    current.count += 1;
    current.confidence = Math.max(current.confidence, Number(detection.confidence || 0));
    groups.set(name, current);
  });
  els.detectionList.innerHTML = [...groups.entries()]
    .sort((left, right) => right[1].confidence - left[1].confidence)
    .map(([name, result]) => `
      <span class="detection-item" style="--detection-color:${detectionColor(name)}">
        <strong>${escapeHtml(name)}</strong>
        <span>${result.count}</span>
        <em>${Math.round(result.confidence * 100)}%</em>
      </span>
    `).join("");
}

async function refreshPointcloud() {
  try {
    let pointcloud;
    try {
      pointcloud = await api("/api/depthcloud");
    } catch (err) {
      pointcloud = await api("/api/pointcloud");
    }
    drawPointcloud(pointcloud);
  } catch (err) {
    els.pointcloudEmpty.style.display = "grid";
    els.pointcloudMeta.textContent = "暂无数据";
  }
}

async function initPointcloud3d() {
  if (!els.pointcloud3d) return;
  try {
    const THREE = await import("https://unpkg.com/three@0.160.0/build/three.module.js");
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x303030);

    const camera = new THREE.PerspectiveCamera(55, 1, 0.05, 500);
    camera.up.set(0, 0, 1);
    camera.position.set(17, -2, 5);
    camera.lookAt(0, 0, 0);

    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    els.pointcloud3d.appendChild(renderer.domElement);
    els.pointcloud3d.classList.add("active");
    els.pointcloudCanvas.style.display = "none";

    const world = new THREE.Group();
    scene.add(world);

    const grid = new THREE.GridHelper(18, 18, 0xa0a0a4, 0x565656);
    grid.rotation.x = Math.PI / 2;
    world.add(grid);
    world.add(new THREE.AxesHelper(4));

    const geometry = new THREE.BufferGeometry();
    const material = new THREE.PointsMaterial({
      size: pointcloudOptions.pointSize,
      vertexColors: true,
      sizeAttenuation: true,
    });
    const cloud = new THREE.Points(geometry, material);
    world.add(cloud);

    threePointcloud = {
      THREE,
      scene,
      camera,
      renderer,
      world,
      cloud,
      yaw: 3.1204166412353516,
      pitch: 0.2103976607322693,
      distance: 24.847997665405273,
      dragging: false,
      lastX: 0,
      lastY: 0,
    };

    const resize = () => {
      const rect = els.pointcloud3d.getBoundingClientRect();
      if (!rect.width || !rect.height) return;
      camera.aspect = rect.width / rect.height;
      camera.updateProjectionMatrix();
      renderer.setSize(rect.width, rect.height, false);
      renderThreeScene();
    };

    els.pointcloud3d.addEventListener("pointerdown", (event) => {
      threePointcloud.dragging = true;
      threePointcloud.lastX = event.clientX;
      threePointcloud.lastY = event.clientY;
      els.pointcloud3d.setPointerCapture(event.pointerId);
    });
    els.pointcloud3d.addEventListener("pointermove", (event) => {
      if (!threePointcloud.dragging) return;
      const dx = event.clientX - threePointcloud.lastX;
      const dy = event.clientY - threePointcloud.lastY;
      threePointcloud.lastX = event.clientX;
      threePointcloud.lastY = event.clientY;
      threePointcloud.yaw -= dx * 0.01;
      threePointcloud.pitch = Math.max(-1.2, Math.min(1.2, threePointcloud.pitch + dy * 0.01));
      updateThreeCamera();
      renderThreeScene();
    });
    els.pointcloud3d.addEventListener("pointerup", () => {
      threePointcloud.dragging = false;
    });
    els.pointcloud3d.addEventListener("wheel", (event) => {
      event.preventDefault();
      threePointcloud.distance = Math.max(2, Math.min(80, threePointcloud.distance + event.deltaY * 0.02));
      updateThreeCamera();
      renderThreeScene();
    }, { passive: false });
    window.addEventListener("resize", resize);
    resize();
    updateThreeCamera();
    pushEvent("system", "Three.js 点云渲染器已启用");
  } catch (err) {
    threePointcloud = null;
    els.pointcloud3d.classList.remove("active");
    els.pointcloudCanvas.style.display = "block";
    pushEvent("error", `Three.js 加载失败，使用 Canvas 兜底: ${err.message}`);
  }
}

function updateThreeCamera() {
  if (!threePointcloud) return;
  const pc = threePointcloud;
  const r = pc.distance;
  const cp = Math.cos(pc.pitch);
  pc.camera.position.set(
    Math.cos(pc.yaw) * cp * r,
    Math.sin(pc.yaw) * cp * r,
    Math.sin(pc.pitch) * r
  );
  pc.camera.lookAt(0, 0, 0);
}

function renderThreeScene() {
  if (!threePointcloud) return;
  threePointcloud.renderer.render(threePointcloud.scene, threePointcloud.camera);
}

function renderThreePointcloud(pointcloud) {
  if (!threePointcloud) return false;
  const points = pointcloud.points || [];
  if (!points.length) return false;

  const THREE = threePointcloud.THREE;
  const positions = new Float32Array(points.length * 3);
  const colors = new Float32Array(points.length * 3);
  let maxRange = 1;
  for (const point of points) {
    maxRange = Math.max(maxRange, Math.abs(point[0]), Math.abs(point[1]), Math.abs(point[2]));
  }
  maxRange = Math.min(Math.max(maxRange, 8), 80);

  points.forEach((point, index) => {
    positions[index * 3] = point[0];
    positions[index * 3 + 1] = point[1];
    positions[index * 3 + 2] = point[2];
    const color = pointColorRgb(point, maxRange);
    colors[index * 3] = color[0] / 255;
    colors[index * 3 + 1] = color[1] / 255;
    colors[index * 3 + 2] = color[2] / 255;
  });

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  geometry.computeBoundingSphere();
  threePointcloud.cloud.geometry.dispose();
  threePointcloud.cloud.geometry = geometry;
  threePointcloud.cloud.material.size = pointcloudOptions.pointSize;
  updateThreeCamera();
  renderThreeScene();
  els.pointcloudEmpty.style.display = "none";
  els.pointcloudMeta.textContent = `${pointcloud.sampled_points} / ${pointcloud.total_points} 点 · Three.js · ${pointcloud.fixed_frame || pointcloud.frame_id || "--"}`;
  return true;
}

function pointColorRgb(point, maxRange) {
  if (pointcloudOptions.colorMode === "height") {
    return axisColorRgb(point[2], maxRange);
  }
  if (pointcloudOptions.colorMode === "distance") {
    const distance = Math.hypot(point[0], point[1], point[2]);
    return axisColorRgb(distance - maxRange / 2, maxRange);
  }
  return axisColorRgb(point[1], maxRange);
}

function resetCloudView() {
  if (!threePointcloud) return;
  threePointcloud.yaw = 3.1204166412353516;
  threePointcloud.pitch = 0.2103976607322693;
  threePointcloud.distance = 24.847997665405273;
  updateThreeCamera();
  renderThreeScene();
}

function drawPointcloud(pointcloud) {
  latestPointcloud = pointcloud;
  if (renderThreePointcloud(pointcloud)) return;

  const canvas = els.pointcloudCanvas;
  const ctx = canvas.getContext("2d");
  const width = canvas.width;
  const height = canvas.height;
  const points = pointcloud.points || [];

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#303030";
  ctx.fillRect(0, 0, width, height);

  if (!points.length) {
    els.pointcloudEmpty.style.display = "grid";
    els.pointcloudMeta.textContent = "点云为空";
    return;
  }

  let maxRange = 1;
  for (const point of points) {
    maxRange = Math.max(maxRange, Math.abs(point[0]), Math.abs(point[1]), Math.abs(point[2]));
  }
  maxRange = Math.min(Math.max(maxRange, 8), 80);
  const camera = {
    yaw: 3.1204166412353516,
    pitch: 0.2103976607322693,
    distance: 24.847997665405273,
    fov: 42,
  };
  const cx = width / 2;
  const cy = height * 0.54;
  const scale = Math.min(width, height) * 0.9;

  drawRvizDepthGrid(ctx, width, height, camera, scale);
  drawRvizAxes(ctx, cx, cy, camera, scale);

  const projected = points
    .map((point) => projectRvizPoint(point, cx, cy, camera, scale))
    .filter((point) => point.visible)
    .sort((a, b) => b.cameraZ - a.cameraZ);

  for (const point of projected) {
    const color = rvizAxisColor(point.raw[1], maxRange);
    ctx.fillStyle = color;
    ctx.globalAlpha = Math.max(0.42, Math.min(1, 1.15 - point.distance / 80));
    ctx.fillRect(point.x, point.y, point.radius, point.radius);
  }
  ctx.globalAlpha = 1;

  els.pointcloudEmpty.style.display = "none";
  els.pointcloudMeta.textContent = `${pointcloud.sampled_points} / ${pointcloud.total_points} 点 · ${pointcloud.fixed_frame || pointcloud.frame_id || "--"}`;
}

function projectRvizPoint(point, cx, cy, camera, scale) {
  const [x, y, z] = point;
  const rotated = rotateOrbit(x, y, z, camera.yaw, camera.pitch);
  const cameraZ = rotated.z + camera.distance;
  const perspective = camera.fov / Math.max(1.0, cameraZ);
  const px = cx + rotated.x * perspective * scale / 24;
  const py = cy - rotated.y * perspective * scale / 24;
  return {
    x: px,
    y: py,
    cameraZ,
    distance: Math.hypot(x, y, z),
    raw: point,
    radius: Math.max(1, Math.min(2, 1.2 + perspective * 0.04)),
    visible: cameraZ > 0.5 && px > -8 && px < cx * 2 + 8 && py > -8 && py < cy * 2 + 8,
  };
}

function rotateOrbit(x, y, z, yaw, pitch) {
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  const cp = Math.cos(pitch);
  const sp = Math.sin(pitch);
  const yawX = cy * x - sy * y;
  const yawY = sy * x + cy * y;
  const pitchY = cp * yawY - sp * z;
  const pitchZ = sp * yawY + cp * z;
  return { x: yawX, y: pitchZ, z: pitchY };
}

function rvizAxisColor(value, maxRange) {
  const rgb = axisColorRgb(value, maxRange);
  return `rgb(${rgb[0]}, ${rgb[1]}, ${rgb[2]})`;
}

function axisColorRgb(value, maxRange) {
  const t = Math.max(0, Math.min(1, (value + maxRange) / (maxRange * 2)));
  const stops = [
    [0.0, [0, 0, 255]],
    [0.25, [0, 255, 255]],
    [0.5, [0, 255, 0]],
    [0.75, [255, 255, 0]],
    [1.0, [255, 0, 0]],
  ];
  for (let i = 1; i < stops.length; i += 1) {
    if (t <= stops[i][0]) {
      const [t0, c0] = stops[i - 1];
      const [t1, c1] = stops[i];
      const k = (t - t0) / (t1 - t0);
      return c0.map((channel, idx) => Math.round(channel + (c1[idx] - channel) * k));
    }
  }
  return [255, 0, 0];
}

function drawRvizDepthGrid(ctx, width, height, camera, scale) {
  ctx.strokeStyle = "rgba(160, 160, 164, 0.5)";
  ctx.lineWidth = 1;
  for (let value = -5; value <= 5; value += 1) {
    drawWorldLine(ctx, [-8, value, 0], [8, value, 0], camera, scale, width, height);
    drawWorldLine(ctx, [value, -8, 0], [value, 8, 0], camera, scale, width, height);
  }
}

function drawWorldLine(ctx, from, to, camera, scale, width, height) {
  const cx = width / 2;
  const cy = height * 0.54;
  const a = projectRvizPoint(from, cx, cy, camera, scale);
  const b = projectRvizPoint(to, cx, cy, camera, scale);
  if (!a.visible && !b.visible) return;
  ctx.beginPath();
  ctx.moveTo(a.x, a.y);
  ctx.lineTo(b.x, b.y);
  ctx.stroke();
}

function drawRvizAxes(ctx, cx, cy, camera, scale) {
  const axes = [
    { label: "X", color: "#ff4040", point: [4, 0, 0] },
    { label: "Y", color: "#40ff40", point: [0, 4, 0] },
    { label: "Z", color: "#4080ff", point: [0, 0, 4] },
  ];
  ctx.lineWidth = 2;
  axes.forEach((axis) => {
    const origin = projectRvizPoint([0, 0, 0], cx, cy, camera, scale);
    const p = projectRvizPoint(axis.point, cx, cy, camera, scale);
    ctx.strokeStyle = axis.color;
    ctx.fillStyle = axis.color;
    ctx.beginPath();
    ctx.moveTo(origin.x, origin.y);
    ctx.lineTo(p.x, p.y);
    ctx.stroke();
    ctx.fillText(axis.label, p.x + 4, p.y + 4);
  });
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

async function cancelAutonomyTask() {
  try {
    const result = await post("/api/autonomy/cancel", {});
    pushEvent("service", `取消自主任务: ${result.message || "完成"}`);
  } catch (err) {
    pushEvent("error", `取消自主任务: ${err.message}`);
  }
}

function isPx4WhiteClothingFollowTask(task) {
  const normalized = String(task || "").toLowerCase().replace(/\s+/g, "");
  const requestsFollow = ["跟随", "跟踪", "追踪", "follow"].some((word) => normalized.includes(word));
  const selectsWhite = ["白色", "白衣", "穿白", "white"].some((word) => normalized.includes(word));
  const selectsPerson = ["人员", "行人", "目标人", "person", "人"].some((word) => normalized.includes(word));
  return requestsFollow && selectsWhite && selectsPerson;
}

function setPx4FollowFlow(stage) {
  if (!els.px4FollowFlow) return;
  const order = ["command", "intent", "confirm", "execute"];
  const activeIndex = order.indexOf(stage);
  els.px4FollowFlow.classList.remove("hidden");
  els.px4FollowFlow.querySelectorAll("[data-px4-flow]").forEach((step) => {
    const index = order.indexOf(step.dataset.px4Flow);
    step.classList.remove("done", "active", "error");
    if (stage === "cancel") {
      if (index < 2) step.classList.add("done");
      if (index === 2) step.classList.add("error");
      return;
    }
    if (index < activeIndex) step.classList.add("done");
    if (index === activeIndex) step.classList.add("active");
  });
}

function startPx4WhiteClothingFollowTask(task) {
  const confirmationId = `px4-follow-task-${Date.now()}`;
  const args = {
    target_class: "person",
    clothing_color: "白色服装",
    perception_rule: "person + dominant_color:white",
    follow_policy: "视觉锁定后保持跟随",
    lost_target_action: "悬停并等待重新识别",
    stop_distance_m: 3,
  };
  const payload = {
    reply: "自然语言指令已解析为目标类别 person / 白色服装与跟踪意图 visual_follow，并生成跟踪策略。任务正在等待安全确认，确认后进入执行链。",
    parsed_task: {
      parser: "ground_agent_parser",
      intent: "follow_person_by_clothing",
      skill: "PX4VisualFollowSkill",
      skill_info: {description: "按人员类别与白色服装属性筛选目标，并生成视觉跟随任务草案。"},
      args,
      reason: "任务包含人员跟随动作和白色服装目标约束",
      risk_level: "high",
      need_confirm: true,
      confirmed: false,
      execution_mode: "目标锁定后执行",
    },
    pending_confirmation: {
      id: confirmationId,
      token: confirmationId,
      source: PX4_FOLLOW_TASK_SOURCE,
      action: "follow_person_by_clothing",
      intent: "follow_person_by_clothing",
      skill: "PX4VisualFollowSkill",
      risk_level: "high",
      summary: "执行 PX4 白衣人员跟随任务",
      args,
    },
    tool_calls: [],
    progress: [
      {name: "command", label: "接收自然语言指令", status: "done"},
      {name: "target", label: "解析目标类别：person / 白色服装", status: "done"},
      {name: "intent", label: "解析跟踪意图：visual_follow", status: "done"},
      {name: "confirm", label: "等待操作员安全确认", status: "active"},
      {name: "execution_chain", label: "进入执行链", status: "pending"},
      {name: "target_lock", label: "等待目标锁定", status: "pending"},
    ],
    final_status: "等待操作员安全确认",
    execution: {state: "waiting_confirmation", target_locked: false, tracking_active: false},
  };

  chatAutoFollow = true;
  addChat("user", task);
  els.taskInput.value = "";
  setPx4FollowFlow("confirm");
  renderAgentPayload(payload);
  els.toolStatusText.textContent = "TASK CHAIN";
  els.toolCalls.textContent = "目标识别器 · person / white_clothing\n跟踪策略 · visual_follow\n安全确认 · waiting";
  pushEvent("system", "PX4 白衣人员跟随任务已解析，等待操作员安全确认");
}

function resolvePx4WhiteClothingFollowTask(action) {
  const approved = action === "confirm";
  const previous = pendingConfirmation;
  const parsed = lastAgentPayload?.parsed_task || {};
  addChat("user", approved ? "确认执行" : "取消执行");
  const payload = {
    reply: approved
      ? "目标类别与跟踪意图已通过安全确认，任务已进入执行链，正在等待白衣人员目标锁定。"
      : "操作员已取消白衣人员跟随任务，未下发任何指令。",
    parsed_task: {...parsed, confirmed: approved, cancelled: !approved},
    pending_confirmation: null,
    tool_calls: [],
    progress: [
      {name: "command", label: "接收自然语言指令", status: "done"},
      {name: "target", label: "解析目标类别：person / 白色服装", status: "done"},
      {name: "intent", label: "解析跟踪意图：visual_follow", status: "done"},
      {name: "confirm", label: approved ? "安全确认已通过" : "操作员已取消任务", status: approved ? "done" : "failed"},
      {name: "execution_chain", label: approved ? "执行链运行中" : "执行链未启动", status: approved ? "done" : "skipped"},
      {name: "target_lock", label: approved ? "等待目标锁定" : "目标识别未启动", status: approved ? "active" : "skipped"},
    ],
    final_status: approved ? "安全确认通过 · 执行链运行中" : "任务已取消 · 执行链未启动",
    execution: {
      state: approved ? "waiting_target" : "cancelled",
      confirmation_id: previous?.id || previous?.token || "",
      target_locked: false,
      tracking_active: false,
    },
  };

  pendingConfirmation = null;
  renderAgentPayload(payload);
  els.toolStatusText.textContent = "TASK CHAIN";
  els.toolCalls.textContent = approved
    ? "目标识别器 · person / white_clothing\n跟踪策略 · visual_follow\n安全确认 · approved\n执行状态 · waiting_target"
    : "安全确认 · cancelled\n执行状态 · stopped";
  if (approved) {
    setPx4FollowFlow("execute");
    els.aiStatusText.textContent = "EXECUTING";
    els.missionPhase.textContent = "执行链运行中";
    els.missionResult.textContent = "等待目标锁定";
    setTaskLifecycle("execute", "安全确认通过，任务已进入执行链", {activeText: "等待目标锁定"});
    pushEvent("service", "PX4 白衣人员跟随任务已通过安全确认，执行链正在等待目标锁定");
  } else {
    setPx4FollowFlow("cancel");
    els.aiStatusText.textContent = "CANCEL";
    els.missionPhase.textContent = "任务已取消";
    els.missionResult.textContent = "执行链未启动";
    setTaskLifecycle("confirm", "操作员已取消任务，执行链未启动", {outcome: "error", activeText: "已取消"});
    pushEvent("system", "PX4 白衣人员跟随任务已取消");
  }
}

async function submitAgentTask(task) {
  const confirmationAction = chatConfirmationAction(task);
  if (pendingConfirmation && confirmationAction) {
    els.taskInput.value = "";
    await confirmPendingTask(confirmationAction);
    return;
  }
  if (isPx4WhiteClothingFollowTask(task)) {
    startPx4WhiteClothingFollowTask(task);
    return;
  }
  els.px4FollowFlow?.classList.add("hidden");
  chatAutoFollow = true;
  addChat("user", task);
  els.taskInput.value = "";
  els.aiStatusText.textContent = "THINKING";
  els.missionPhase.textContent = "AI 正在解析";
  els.missionResult.textContent = "--";
  els.llmOutput.textContent = "正在解析自然语言任务...\n正在准备 ROS 工具调用...";
  els.planSteps.textContent = "1. 接收用户任务\n2. 解析意图\n3. 选择技能";
  els.toolCalls.textContent = "等待任务服务返回工具调用...";
  setTaskLifecycle("parse", `正在解析：${task}`);
  if (agentConnected && agentSocket?.readyState === WebSocket.OPEN) {
    const requestId = `req-${Date.now()}-${Math.random().toString(16).slice(2)}`;
    agentSocket.send(JSON.stringify({
      type: "chat.send",
      request_id: requestId,
      session_id: sessionId,
      content: task,
      channel: "web",
    }));
    els.llmOutput.textContent = "Agent Gateway 已接收任务，等待流式事件...";
    els.toolCalls.textContent = "等待 Drone MCP 工具调用...";
    return;
  }

  pushEvent("system", "Agent Gateway 未连接，使用原有 ROS Agent 回退链路");
  try {
    const history = chatMessages.slice(-12).map((msg) => ({
      role: msg.role,
      content: msg.content,
    }));
    const result = await post("/api/agent/chat", { session_id: sessionId, message: task, history });
    const payload = parseAgentResult(result.result);
    renderAgentPayload(payload, result.message);
    pushEvent("service", `自然语言: ${result.message || "完成"}`);
  } catch (err) {
    const payload = parseAgentResult(err.data?.result);
    if (payload) {
      renderAgentPayload(payload, err.message);
    } else {
      addChat("assistant", `任务失败：${err.message}`);
      els.taskResult.textContent = err.message;
    }
    els.aiStatusText.textContent = "ERROR";
    setTaskLifecycle("execute", err.message, {outcome: "error", activeText: "处理失败"});
    pushEvent("error", `自然语言: ${err.message}`);
  }
}

function chatConfirmationAction(task) {
  const text = String(task || "").trim().replace(/[。！!]$/, "").trim();
  if (["确认", "确认执行", "同意", "同意执行", "批准", "批准执行"].includes(text)) {
    return "confirm";
  }
  if (["取消", "取消执行", "拒绝", "拒绝执行", "不同意"].includes(text)) {
    return "cancel";
  }
  return null;
}

function resolveAgentBaseUrl() {
  if (agentBaseUrl) return agentBaseUrl.replace(/\/+$/, "");
  try {
    const url = new URL(apiBaseUrl);
    url.port = "8090";
    url.pathname = "";
    return url.origin;
  } catch (err) {
    return "http://localhost:8090";
  }
}

function agentWebsocketUrl() {
  const url = new URL(resolveAgentBaseUrl());
  url.protocol = url.protocol === "https:" ? "wss:" : "ws:";
  url.pathname = "/ws/agent";
  url.searchParams.set("session_id", sessionId);
  url.searchParams.set("user_id", "web-local");
  url.searchParams.set("channel", "web");
  if (agentAccessToken) url.searchParams.set("token", agentAccessToken);
  return url.toString();
}

function connectAgentSocket() {
  if (agentSocketTimer) {
    clearTimeout(agentSocketTimer);
    agentSocketTimer = null;
  }
  if (agentSocket) {
    agentSocket.onclose = null;
    agentSocket.close();
  }
  try {
    agentSocket = new WebSocket(agentWebsocketUrl());
  } catch (err) {
    agentConnected = false;
    agentSocketTimer = setTimeout(connectAgentSocket, 3000);
    return;
  }

  agentSocket.addEventListener("open", () => {
    agentConnected = true;
    els.agentBadge.textContent = "AGENT";
    els.agentBadge.className = "badge ok";
    pushEvent("system", "Agent Gateway 流式会话已连接");
  });
  agentSocket.addEventListener("message", (event) => {
    try {
      handleAgentEvent(JSON.parse(event.data));
    } catch (err) {
      pushEvent("error", `Agent 流式事件解析失败: ${err.message}`);
    }
  });
  agentSocket.addEventListener("close", () => {
    agentConnected = false;
    agentSocketTimer = setTimeout(connectAgentSocket, 3000);
  });
  agentSocket.addEventListener("error", () => {
    agentConnected = false;
  });
}

function handleAgentEvent(event) {
  const requestId = event.request_id || "agent-stream";
  if (event.type === "session.ready") {
    renderChannelStatus(event.channels || {});
    renderProviderOptions(event.providers || [], event.session);
    gatewaySkills = Array.isArray(event.skills) ? event.skills : [];
    gatewayCapabilities = Array.isArray(event.capabilities) ? event.capabilities : [];
    refreshSkills();
    els.agentBadge.textContent = `${event.session?.provider || "claude"} AGENT`.toUpperCase();
    if (event.session?.model) {
      const selected = providerModelValue(
        event.session.provider || "claude",
        event.session.model,
      );
      els.agentModelSelect.value = selected;
      localStorage.setItem(AGENT_MODEL_STORAGE_KEY, selected);
    }
    const history = Array.isArray(event.history) ? event.history : [];
    chatMessages = history.map((message) => ({
        role: message.role === "user" ? "user" : "assistant",
        content: typeof message.content === "string"
          ? message.content
          : JSON.stringify(message.content),
        tag: [
          message.channel === "feishu" ? "飞书" : (message.channel === "web" ? "Web" : ""),
          message.provider || "",
          message.model || "",
        ].filter(Boolean).join(" · "),
      })).slice(-60);
    saveAndRenderChat();
    const pending = Array.isArray(event.pending_confirmations)
      ? event.pending_confirmations.at(-1)
      : null;
    if (pending) {
      pendingConfirmation = { ...pending, token: pending.id, source: "gateway" };
      renderConfirmation();
    }
    const missions = Array.isArray(event.missions) ? event.missions : [];
    if (missions.length) renderGatewayMission(missions[0]);
    if (event.operator?.memory) {
      pushEvent("system", "已恢复跨端对话记忆");
    }
    return;
  }
  if (event.type === "chat.accepted") {
    if (event.session_id !== sessionId && event.message?.content) {
      addChat("user", event.message.content, {
        tag: String(event.session_id || "").startsWith("feishu-") ? "飞书" : "其他终端",
      });
    }
    return;
  }
  if (event.type === "assistant.started") {
    ensureStreamingMessage(requestId, event.model || "Agent");
    els.aiStatusText.textContent = "STREAMING";
    els.missionPhase.textContent = "Agent 正在分析";
    setTaskLifecycle("parse", "Agent 正在理解任务并选择可用技能");
    saveAndRenderChat();
    return;
  }
  if (event.type === "assistant.delta") {
    const message = ensureStreamingMessage(requestId, event.model || "Agent");
    message.content += event.delta || "";
    saveAndRenderChat();
    els.llmOutput.textContent = message.content;
    return;
  }
  if (event.type === "tool.started") {
    els.toolStatusText.textContent = "MCP CALL";
    els.missionPhase.textContent = "读取 ROS 状态";
    setTaskLifecycle("execute", `正在调用 ${event.tool || "ROS 工具"}`, {skipped: ["confirm"]});
    if (els.toolCalls.textContent.trim() === "暂无工具调用") {
      els.toolCalls.innerHTML = "";
    }
    els.toolCalls.insertAdjacentHTML("beforeend", `
      <div class="tool-row" data-tool-call-id="${escapeHtml(event.tool_call_id || "")}">
        <span class="muted">•</span>
        <span class="tool-row-body"><strong>${escapeHtml(event.tool || "MCP Tool")}</strong><small class="tool-call-state">调用中</small></span>
      </div>`);
    pushEvent("service", `Agent 工具调用: ${event.tool || "--"}`);
    return;
  }
  if (event.type === "tool.completed") {
    els.toolStatusText.textContent = event.success === false ? "MCP ERROR" : "MCP DONE";
    const callId = String(event.tool_call_id || "");
    const row = Array.from(els.toolCalls.querySelectorAll(".tool-row")).find(
      (item) => item.dataset.toolCallId === callId,
    );
    if (row) {
      const state = row.querySelector(".tool-call-state");
      if (state) {
        state.textContent = event.success === false ? "调用失败" : "调用完成";
        state.classList.toggle("fail", event.success === false);
        state.classList.toggle("ok", event.success !== false);
      }
      const body = row.querySelector(".tool-row-body");
      if (body) body.insertAdjacentHTML("beforeend", evidenceHtml(toolEvidence(event.result)));
    }
    return;
  }
  if (event.type === "assistant.completed") {
    const message = ensureStreamingMessage(requestId, event.model || "Agent");
    if (event.message?.content) message.content = event.message.content;
    message.streaming = false;
    const source = String(event.session_id || "").startsWith("feishu-")
      ? " · 飞书"
      : "";
    const latency = Number.isFinite(Number(event.latency_ms))
      ? ` · ${(Number(event.latency_ms) / 1000).toFixed(1)}s`
      : "";
    const usage = agentUsageLabel(event.usage);
    message.tag = `${event.provider || "claude"} · ${event.model || "--"}${source}${latency}${usage} · 已完成`;
    saveAndRenderChat();
    els.aiStatusText.textContent = "DONE";
    els.missionPhase.textContent = "Agent 回复完成";
    els.missionResult.textContent = "完成";
    els.llmOutput.textContent = message.content;
    if (pendingConfirmation) {
      setTaskLifecycle("confirm", pendingConfirmation.summary || "等待操作员确认");
    } else {
      setTaskLifecycle("done", "Agent 回复完成", {skipped: ["confirm", "verify"]});
    }
    pushEvent("service", "Agent 回复完成");
    return;
  }
  if (event.type === "model.changed") {
    els.agentModelSelect.value = providerModelValue(
      event.provider || "claude",
      event.model || "sonnet",
    );
    localStorage.setItem(AGENT_MODEL_STORAGE_KEY, els.agentModelSelect.value);
    const migration = event.context_migrated
      ? ` · 已迁移 ${event.migrated_message_count || 0} 条历史消息`
      : "";
    pushEvent("system", `Agent 模型已切换: ${event.provider || "claude"}:${event.model}${migration}`);
    return;
  }
  if (event.type === "session.cleared") {
    chatMessages = [];
    chatAutoFollow = true;
    localStorage.removeItem(CHAT_STORAGE_KEY);
    renderChat();
    pushEvent(
      "system",
      `当前对话已清空${event.deleted_messages ? ` · ${event.deleted_messages} 条` : ""}，任务记录与长期记忆已保留`,
    );
    return;
  }
  if (event.type === "confirmation.required") {
    const confirmation = event.confirmation || {};
    pendingConfirmation = {
      ...confirmation,
      token: confirmation.id,
      source: "gateway",
    };
    renderConfirmation();
    els.aiStatusText.textContent = "CONFIRM";
    els.missionPhase.textContent = "等待人工确认";
    els.missionResult.textContent = confirmation.summary || "等待确认";
    setTaskLifecycle("confirm", confirmation.summary || "高风险动作等待操作员确认");
    pushEvent("system", `等待确认: ${confirmation.summary || confirmation.action}`);
    return;
  }
  if (event.type === "confirmation.resolved") {
    if (pendingConfirmation?.token === event.confirmation?.id) {
      pendingConfirmation = null;
      renderConfirmation();
    }
    const approved = event.decision === "approve";
    els.aiStatusText.textContent = approved ? "EXECUTING" : "CANCEL";
    els.missionPhase.textContent = approved ? "已确认，交给 ROS Agent" : "任务已取消";
    if (approved) {
      setTaskLifecycle("execute", "操作员已确认，正在提交 ROS 控制请求");
    } else {
      setTaskLifecycle("confirm", "操作员已取消任务，未向无人机发送动作", {outcome: "error", activeText: "已取消"});
    }
    pushEvent("system", approved ? "控制请求已确认" : "控制请求已取消");
    return;
  }
  if (event.type === "confirmation.expired") {
    if (pendingConfirmation?.token === event.confirmation?.id) {
      pendingConfirmation = null;
      renderConfirmation();
    }
    els.aiStatusText.textContent = "EXPIRED";
    els.missionPhase.textContent = "确认已过期";
    els.missionResult.textContent = "任务未执行，请重新发起";
    setTaskLifecycle("confirm", "确认窗口已过期，任务未执行", {outcome: "error", activeText: "已过期"});
    pushEvent("system", "控制确认已过期，未执行任何动作");
    return;
  }
  if (event.type === "control.started") {
    els.aiStatusText.textContent = "EXECUTING";
    els.missionPhase.textContent = "ROS Agent 正在执行";
    els.toolStatusText.textContent = "ROS CALL";
    setTaskLifecycle("execute", `正在执行 ${event.action || "控制动作"}`);
    pushEvent("service", `执行控制动作: ${event.action || "--"}`);
    return;
  }
  if (event.type === "control.progress") {
    const labels = {
      accepted: "ROS Agent 已受理",
      verifying: "等待物理状态验证",
    };
    const message = event.details?.message || labels[event.phase] || "任务执行中";
    els.aiStatusText.textContent = "VERIFYING";
    els.missionPhase.textContent = labels[event.phase] || "正在验证";
    els.missionResult.textContent = message;
    els.toolStatusText.textContent = event.phase === "accepted" ? "ROS ACCEPTED" : "VERIFYING";
    setTaskLifecycle(event.phase === "verifying" ? "verify" : "execute", message);
    pushEvent("service", message);
    return;
  }
  if (event.type === "control.completed") {
    const success = event.success === true;
    const message = event.result?.message || (success ? "控制任务已提交" : "控制任务失败");
    els.aiStatusText.textContent = success ? "DONE" : "ERROR";
    els.missionPhase.textContent = success ? "物理动作验证完成" : "控制任务失败";
    els.missionResult.textContent = message;
    els.toolStatusText.textContent = success ? "ROS DONE" : "ROS ERROR";
    if (success) {
      setTaskLifecycle("done", message);
    } else {
      setTaskLifecycle("verify", message, {outcome: "error", activeText: "验证失败"});
    }
    addChat("assistant", message, { tag: success ? "ROS 执行结果" : "执行失败" });
    pushEvent(success ? "service" : "error", message);
    return;
  }
  if (event.type === "mission.updated") {
    renderGatewayMission(event.mission || {});
    return;
  }
  if (event.type === "workflow.controlled") {
    pushEvent("system", event.result?.message || "组合任务状态已更新");
    return;
  }
  if (event.type === "memory.updated") {
    pushEvent("system", "跨端对话记忆已更新");
    return;
  }
  if (event.type === "memory.semantic_updated") {
    const count = Array.isArray(event.items) ? event.items.length : 0;
    pushEvent("system", `长期语义记忆已更新 · ${count} 条`);
    return;
  }
  if (event.type === "memory.semantic_failed") {
    pushEvent("error", event.message || "长期语义记忆更新失败");
    return;
  }
  if (event.type === "vision.completed") {
    const content = event.message?.content || "图片视觉分析完成";
    addChat("assistant", content, {
      tag: `${event.model || "VLM"} · 飞书图片`,
    });
    els.missionPhase.textContent = "视觉分析完成";
    els.missionResult.textContent = event.image_path || content;
    refreshReports();
    return;
  }
  if (event.type === "error") {
    const message = findStreamingMessage(requestId);
    if (message) {
      message.content = event.message || "Agent 执行失败";
      message.streaming = false;
      message.tag = "执行失败";
      saveAndRenderChat();
    } else {
      addChat("assistant", `任务失败：${event.message || "未知错误"}`);
    }
    els.aiStatusText.textContent = "ERROR";
    setTaskLifecycle("execute", event.message || "Agent 执行失败", {outcome: "error", activeText: "执行失败"});
    pushEvent("error", `Agent Runtime: ${event.message || "未知错误"}`);
  }
}

function renderChannelStatus(channels) {
  const feishu = channels?.feishu;
  if (!feishu) return;
  const openIds = Array.isArray(feishu.allowed_open_ids)
    ? feishu.allowed_open_ids.filter(Boolean)
    : [];
  const statusKey = JSON.stringify({
    enabled: Boolean(feishu.enabled),
    running: Boolean(feishu.running),
    count: Number(feishu.allowed_user_count || 0),
    openIds,
    vision: Boolean(feishu.vision_enabled),
  });
  if (statusKey === lastChannelStatusKey) return;
  lastChannelStatusKey = statusKey;

  if (!feishu.enabled) {
    pushEvent("system", "飞书通道未启用");
    return;
  }
  const count = Number(feishu.allowed_user_count || openIds.length || 0);
  if (feishu.running) {
    pushEvent("system", `飞书长连接已启用，白名单用户数: ${count}`);
    pushEvent("system", `Feishu adapter started with ${count} allowed users`);
  } else {
    pushEvent("error", `飞书长连接未运行，白名单用户数: ${count}`);
  }
  if (openIds.length) {
    pushEvent("system", `飞书白名单 Open ID: ${openIds.join(", ")}`);
  }
  pushEvent(
    "system",
    `飞书图片 VLM: ${feishu.vision_enabled ? "已启用" : "未配置"}`,
  );
}

function providerModelValue(provider, model) {
  return `${provider || "claude"}:${model || "sonnet"}`;
}

function agentUsageLabel(usage) {
  if (!usage || typeof usage !== "object") return "";
  const total = Number(
    usage.total_tokens
    ?? usage.totalTokens
    ?? ((usage.input_tokens || 0) + (usage.output_tokens || 0)),
  );
  return total > 0 ? ` · ${total} tokens` : "";
}

function renderProviderOptions(providers, session) {
  if (!Array.isArray(providers) || !providers.length) return;
  els.agentModelSelect.innerHTML = providers.flatMap((provider) =>
    (provider.models || []).map((model) => {
      const value = providerModelValue(provider.name, model);
      const suffix = provider.configured === false ? "（未配置）" : "";
      return `<option value="${escapeHtml(value)}">${escapeHtml(provider.name)} · ${escapeHtml(model)}${suffix}</option>`;
    })
  ).join("");
  if (session?.model) {
    els.agentModelSelect.value = providerModelValue(
      session.provider || "claude",
      session.model,
    );
  }
}

function renderGatewayMission(mission) {
  if (!mission?.id) return;
  const revision = Number(mission.revision || 0);
  const previousRevision = Number(gatewayMissionRevisions.get(mission.id) || 0);
  if (revision > 0 && revision < previousRevision) return;
  if (revision > 0) gatewayMissionRevisions.set(mission.id, revision);
  const terminal = ["completed", "failed", "cancelled", "expired"].includes(mission.status);
  if (terminal && pendingConfirmation?.token === mission.confirmation_id) {
    pendingConfirmation = null;
    renderConfirmation();
  }
  const labels = {
    pending_confirmation: "等待人工确认",
    executing: "正在执行",
    completed: "任务完成",
    failed: "任务失败",
    cancelled: "任务已取消",
    expired: "确认已过期",
  };
  const phaseLabels = {
    pending_confirmation: "等待人工确认",
    dispatching: "正在提交 ROS 控制请求",
    accepted: "ROS Agent 已受理",
    verifying: "等待物理状态验证",
    workflow_step: "组合任务执行中",
    paused: "组合任务已暂停",
    cancelling: "正在取消组合任务",
    interrupted: "组合任务已安全中止",
    completed: "物理动作验证完成",
    failed: "动作执行或验证失败",
    cancelled: "任务已取消",
    expired: "确认已过期",
  };
  const phase = phaseLabels[mission.phase]
    || labels[mission.status]
    || mission.status
    || "任务处理中";
  const source = {
    feishu: "飞书",
    web: "Web",
  }[mission.source_channel] || mission.source_channel || "控制台";
  els.missionPhase.textContent = phase;
  els.missionResult.textContent = `[${source}] ${mission.result?.message || mission.title || "--"}`;
  els.taskResult.textContent = JSON.stringify({ mission }, null, 2);
  if (mission.status === "pending_confirmation") {
    setTaskLifecycle("confirm", mission.title || "任务等待操作员确认");
  } else if (mission.status === "completed") {
    setTaskLifecycle("done", mission.result?.message || mission.title || "任务已完成");
  } else if (["failed", "cancelled", "expired"].includes(mission.status)) {
    const failedAt = mission.status === "expired"
      ? "confirm"
      : (mission.phase === "verifying" ? "verify" : "execute");
    setTaskLifecycle(failedAt, mission.result?.message || phase, {
      outcome: "error",
      activeText: mission.status === "cancelled" ? "已取消" : mission.status === "expired" ? "已过期" : "任务失败",
    });
  } else if (mission.phase === "verifying") {
    setTaskLifecycle("verify", mission.result?.message || phase);
  } else {
    setTaskLifecycle("execute", mission.result?.message || phase);
  }
  const workflow = mission.result?.workflow
    || (mission.action === "workflow" ? mission.args : null);
  if (workflow?.workflow_id && Array.isArray(workflow.steps)) {
    const icons = {
      completed: ["✓", "ok"],
      running: ["●", "warn-text"],
      retrying: ["↻", "warn-text"],
      paused: ["Ⅱ", "warn-text"],
      failed: ["×", "fail"],
      cancelled: ["×", "muted"],
      skipped: ["-", "muted"],
      pending: ["•", "muted"],
    };
    els.planSteps.innerHTML = workflow.steps.map((step, index) => {
      const [icon, className] = icons[step.status] || icons.pending;
      const attempt = step.attempt ? ` · 第 ${step.attempt} 次` : "";
      return `<div class="plan-row"><span class="${className}">${icon}</span><span>${index + 1}. ${escapeHtml(step.label || step.id)}<small>${escapeHtml(step.message || "")}${attempt}</small></span></div>`;
    }).join("");
    const controllable = mission.status === "executing";
    activeWorkflowId = controllable ? workflow.workflow_id : null;
    els.workflowControls.hidden = !controllable;
    els.pauseWorkflowBtn.disabled = mission.phase === "paused";
    els.resumeWorkflowBtn.disabled = mission.phase !== "paused";
  } else {
    activeWorkflowId = null;
    els.workflowControls.hidden = true;
  }
  els.aiStatusText.textContent = {
    pending_confirmation: "CONFIRM",
    executing: "EXECUTING",
    completed: "DONE",
    failed: "ERROR",
    cancelled: "CANCEL",
    expired: "EXPIRED",
  }[mission.status] || "READY";
}

async function confirmPendingTask(action) {
  if (!pendingConfirmation?.token) {
    pushEvent("error", "没有待确认任务");
    return;
  }
  const token = pendingConfirmation.token;
  if (pendingConfirmation.source === PX4_FOLLOW_TASK_SOURCE) {
    resolvePx4WhiteClothingFollowTask(action);
    return;
  }
  if (pendingConfirmation.source === "gateway") {
    if (!agentConnected || agentSocket?.readyState !== WebSocket.OPEN) {
      pushEvent("error", "Agent Gateway 未连接，不能提交确认");
      return;
    }
    addChat("user", action === "confirm" ? "确认执行" : "取消执行");
    agentSocket.send(JSON.stringify({
      type: "confirmation.respond",
      request_id: `confirm-${Date.now()}`,
      session_id: sessionId,
      confirmation_id: token,
      decision: action === "confirm" ? "approve" : "cancel",
    }));
    return;
  }
  const message = action === "confirm"
    ? `__aeromind_confirm__:${token}`
    : `__aeromind_cancel__:${token}`;
  addChat("user", action === "confirm" ? "确认执行" : "取消执行");
  pendingConfirmation = null;
  renderConfirmation();
  els.aiStatusText.textContent = action === "confirm" ? "EXECUTING" : "CANCEL";
  setTaskLifecycle(action === "confirm" ? "execute" : "confirm",
    action === "confirm" ? "操作员已确认，正在提交控制请求" : "操作员已取消任务",
    action === "confirm" ? {} : {outcome: "error", activeText: "已取消"});
  try {
    const result = await post("/api/agent/chat", {
      session_id: sessionId,
      message,
      history: chatMessages.slice(-12),
    });
    const payload = parseAgentResult(result.result);
    renderAgentPayload(payload, result.message);
    pushEvent("service", action === "confirm" ? "确认执行完成" : "已取消执行");
  } catch (err) {
    const payload = parseAgentResult(err.data?.result);
    if (payload) {
      renderAgentPayload(payload, err.message);
    } else {
      addChat("assistant", `${action === "confirm" ? "确认执行" : "取消"}失败：${err.message}`);
    }
    els.aiStatusText.textContent = "ERROR";
    setTaskLifecycle("execute", err.message, {outcome: "error", activeText: "确认后执行失败"});
    pushEvent("error", `确认流程: ${err.message}`);
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
  const task = els.taskInput.value.trim();
  if (!task) return;
  submitAgentTask(task);
});

els.px4WhiteFollowBtn.addEventListener("click", () => {
  const task = "识别并跟随穿白色服装的人员";
  els.taskInput.value = task;
  submitAgentTask(task);
});

els.taskInput.addEventListener("keydown", (event) => {
  if (event.key !== "Enter" || event.shiftKey || event.isComposing) return;
  event.preventDefault();
  const task = els.taskInput.value.trim();
  if (!task) return;
  submitAgentTask(task);
});

els.clearChatBtn.addEventListener("click", () => {
  if (agentConnected && agentSocket?.readyState === WebSocket.OPEN) {
    agentSocket.send(JSON.stringify({
      type: "session.clear",
      request_id: `clear-${Date.now()}`,
      session_id: sessionId,
    }));
    pushEvent("system", "正在清空当前对话上下文");
    return;
  }
  chatMessages = [];
  chatAutoFollow = true;
  localStorage.removeItem(CHAT_STORAGE_KEY);
  renderChat();
  pushEvent("system", "Gateway 未连接，仅清空了当前浏览器中的对话");
});

els.chatHistory.addEventListener("scroll", updateChatScrollButton, {passive: true});
els.chatScrollBtn.addEventListener("click", scrollChatToBottom);

els.agentModelSelect.addEventListener("change", () => {
  const selected = els.agentModelSelect.value;
  const separator = selected.indexOf(":");
  const provider = separator >= 0 ? selected.slice(0, separator) : "claude";
  const model = separator >= 0 ? selected.slice(separator + 1) : selected;
  localStorage.setItem(AGENT_MODEL_STORAGE_KEY, selected);
  if (!agentConnected || agentSocket?.readyState !== WebSocket.OPEN) {
    pushEvent("error", "Agent Gateway 未连接，暂时不能切换模型");
    return;
  }
  agentSocket.send(JSON.stringify({
    type: "model.switch",
    request_id: `model-${Date.now()}`,
    session_id: sessionId,
    provider,
    model,
  }));
});

function sendWorkflowControl(command) {
  if (!activeWorkflowId || !agentConnected || agentSocket?.readyState !== WebSocket.OPEN) {
    pushEvent("error", "当前没有可控制的组合任务");
    return;
  }
  agentSocket.send(JSON.stringify({
    type: "workflow.control",
    request_id: `workflow-${Date.now()}`,
    workflow_id: activeWorkflowId,
    command,
  }));
}

els.pauseWorkflowBtn.addEventListener("click", () => sendWorkflowControl("pause"));
els.resumeWorkflowBtn.addEventListener("click", () => sendWorkflowControl("resume"));
els.cancelWorkflowBtn.addEventListener("click", () => sendWorkflowControl("cancel"));

els.confirmTaskBtn.addEventListener("click", () => {
  confirmPendingTask("confirm");
});

els.cancelTaskBtn.addEventListener("click", () => {
  confirmPendingTask("cancel");
});

els.modalConfirmTaskBtn.addEventListener("click", () => {
  confirmPendingTask("confirm");
});

els.modalCancelTaskBtn.addEventListener("click", () => {
  confirmPendingTask("cancel");
});

els.chatHistory.addEventListener("click", (event) => {
  const button = event.target.closest("[data-chat-confirm]");
  if (!button) return;
  confirmPendingTask(button.dataset.chatConfirm);
});

els.cancelAutonomyBtn.addEventListener("click", () => {
  cancelAutonomyTask();
});

els.cameraZoomBtn.addEventListener("click", () => {
  openCameraViewer();
});

els.cameraColorMode.addEventListener("change", () => {
  localStorage.setItem(CAMERA_COLOR_MODE_KEY, els.cameraColorMode.value);
  refreshCamera();
});

els.cameraCanvas.addEventListener("click", () => {
  openCameraViewer();
});

els.closeCameraViewerBtn.addEventListener("click", () => {
  closeCameraViewer();
});

els.cameraZoomOutBtn.addEventListener("click", () => {
  setCameraViewerZoom(cameraViewerZoom / 1.25);
});

els.cameraZoomInBtn.addEventListener("click", () => {
  setCameraViewerZoom(cameraViewerZoom * 1.25);
});

els.cameraZoomFitBtn.addEventListener("click", () => {
  fitCameraViewer();
});

els.cameraViewer.addEventListener("click", (event) => {
  if (event.target === els.cameraViewer) {
    closeCameraViewer();
  }
});

window.addEventListener("keydown", (event) => {
  if (!isCameraViewerOpen()) return;
  if (event.key === "Escape") {
    closeCameraViewer();
  } else if (event.key === "+" || event.key === "=") {
    setCameraViewerZoom(cameraViewerZoom * 1.25);
  } else if (event.key === "-") {
    setCameraViewerZoom(cameraViewerZoom / 1.25);
  } else if (event.key === "0") {
    fitCameraViewer();
  }
});

els.resetCloudViewBtn.addEventListener("click", () => {
  resetCloudView();
});

els.cloudColorMode.addEventListener("change", () => {
  pointcloudOptions.colorMode = els.cloudColorMode.value;
  if (latestPointcloud) drawPointcloud(latestPointcloud);
});

els.cloudPointSize.addEventListener("input", () => {
  pointcloudOptions.pointSize = Number(els.cloudPointSize.value || 4) * 0.014;
  if (threePointcloud) {
    threePointcloud.cloud.material.size = pointcloudOptions.pointSize;
    renderThreeScene();
  }
});

document.getElementById("clearLogBtn").addEventListener("click", () => {
  localEvents = [];
  els.eventLog.innerHTML = "";
});

els.saveGatewayBtn.addEventListener("click", () => {
  const nextUrl = els.gatewayInput.value.trim().replace(/\/+$/, "");
  if (!nextUrl) {
    pushEvent("error", "ROS 网关地址不能为空");
    return;
  }
  apiBaseUrl = nextUrl;
  localStorage.setItem("aeromind_gateway_url", apiBaseUrl);
  pushEvent("system", `已切换 ROS 网关: ${apiBaseUrl}`);
  connectTelemetrySocket();
  connectAgentSocket();
  refreshSkills();
  refreshStatus();
  refreshCamera();
  refreshPointcloud();
  refreshReports();
});

renderChat();
renderConfirmation();
initPointcloud3d();
connectTelemetrySocket();
connectAgentSocket();
refreshSkills();
refreshStatus();
refreshCamera();
refreshPointcloud();
refreshReports();
setInterval(refreshStatus, 1000);
setInterval(refreshCamera, 1600);
setInterval(refreshPointcloud, 2200);
setInterval(refreshReports, 5000);
