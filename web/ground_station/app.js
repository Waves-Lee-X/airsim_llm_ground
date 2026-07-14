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
  chatHistory: document.getElementById("chatHistory"),
  llmOutput: document.getElementById("llmOutput"),
  planSteps: document.getElementById("planSteps"),
  toolCalls: document.getElementById("toolCalls"),
  aiStatusText: document.getElementById("aiStatusText"),
  planStatusText: document.getElementById("planStatusText"),
  toolStatusText: document.getElementById("toolStatusText"),
  eventLog: document.getElementById("eventLog"),
  skillGrid: document.getElementById("skillGrid"),
  gatewayInput: document.getElementById("gatewayInput"),
  saveGatewayBtn: document.getElementById("saveGatewayBtn"),
  clearChatBtn: document.getElementById("clearChatBtn"),
};

let localEvents = [];
let lastAgentPayload = null;
let pendingConfirmation = null;
let activeWorkflowId = null;
let gatewaySkills = [];
let activeVerification = null;
let threePointcloud = null;
let telemetrySocket = null;
let telemetrySocketTimer = null;
let agentSocket = null;
let agentSocketTimer = null;
let agentConnected = false;
let latestPointcloud = null;
let latestDetections = [];
let latestCameraFrame = null;
let lastLiveMissionKey = "";
let lastImageAnalysisKey = "";
let lastReportListKey = "";
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
  { name: "StatusSkill", label: "状态检查", type: "hard", risk_level: "low", description: "读取飞控状态", example_task: "查询状态", enabled: true },
  { name: "ArmSkill", label: "解锁/加锁", type: "hard", risk_level: "medium", description: "调用 /control/arm", example_task: "解锁", enabled: true },
  { name: "TakeoffSkill", label: "安全起飞", type: "hard", risk_level: "medium", description: "调用 /control/takeoff", example_task: "起飞到10米", enabled: true },
  { name: "LandSkill", label: "降落", type: "hard", risk_level: "medium", description: "调用 /control/land", example_task: "降落", enabled: true },
  { name: "PlanningSkill", label: "路径规划", type: "soft", risk_level: "high", description: "发布 /autonomy/goal", example_task: "飞到前方20米", enabled: true },
  { name: "HoverSkill", label: "悬停保持", type: "hard", risk_level: "medium", description: "取消自主目标并发布零速度", example_task: "原地悬停", enabled: true },
  { name: "ReturnHomeSkill", label: "返航", type: "hard", risk_level: "high", description: "调用 PX4 原生 RTL", example_task: "返航并降落", enabled: true },
  { name: "EmergencyStopSkill", label: "急停", type: "hard", risk_level: "high", description: "取消任务、零速度、尝试加锁", example_task: "立即急停", enabled: true },
  { name: "PerceptionSkill", label: "环境感知", type: "perception", risk_level: "low", description: "读取相机/深度/点云摘要", example_task: "检查前方有没有障碍物", enabled: true },
  { name: "CaptureImageSkill", label: "拍照取证", type: "perception", risk_level: "low", description: "保存当前 RGB 图像", example_task: "拍一张前方照片", enabled: true },
  { name: "ScanAreaSkill", label: "区域扫描", type: "soft", risk_level: "medium", description: "生成区域扫描摘要", example_task: "扫描前方区域", enabled: true },
  { name: "TargetSearchSkill", label: "目标搜索", type: "soft", risk_level: "medium", description: "匹配当前 detection", example_task: "检测人", enabled: true },
  { name: "SemanticImageSkill", label: "图像语义分析", type: "perception", risk_level: "low", description: "分析当前 RGB 画面", example_task: "分析当前画面中有什么", enabled: true },
  { name: "MissionSequenceSkill", label: "复合任务", type: "soft", risk_level: "high", description: "拆解起飞、移动和目标检测", example_task: "起飞，向左飞20米，并检测人", enabled: true },
  { name: "MissionReportSkill", label: "任务报告", type: "soft", risk_level: "low", description: "汇总当前任务报告", example_task: "生成当前任务报告", enabled: true },
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

function setupResizableCards() {
  document.querySelectorAll(".resizable-card[data-resize-key]").forEach((card) => {
    const key = card.getAttribute("data-resize-key");
    const handle = card.querySelector(".resize-grip");
    if (!key || !handle) return;

    const storedHeight = Number(localStorage.getItem(`${PANEL_HEIGHT_STORAGE_PREFIX}${key}`));
    if (Number.isFinite(storedHeight) && storedHeight > 0) {
      card.style.height = `${clampPanelHeight(storedHeight)}px`;
    }

    handle.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      const startY = event.clientY;
      const startHeight = card.getBoundingClientRect().height;
      card.classList.add("resizing");
      handle.setPointerCapture?.(event.pointerId);

      const onPointerMove = (moveEvent) => {
        const nextHeight = clampPanelHeight(startHeight + moveEvent.clientY - startY);
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

function clampPanelHeight(value) {
  const viewportLimit = Math.max(160, Math.floor(window.innerHeight * 0.72));
  return Math.max(118, Math.min(viewportLimit, Number(value) || 118));
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
  els.chatHistory.scrollTop = els.chatHistory.scrollHeight;
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

  els.parsedTask.innerHTML = `
    <div><span>Intent</span><strong>${escapeHtml(parsed.intent || "--")}</strong></div>
    <div><span>Skill</span><strong>${escapeHtml(parsed.skill || "--")}</strong></div>
    <div><span>Risk</span><strong>${escapeHtml(parsed.risk_level || "--")}</strong></div>
    <div><span>Confirm</span><strong>${escapeHtml(confirmLabel)}</strong></div>
  `;
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
    renderChat();
    return;
  }

  const args = Object.keys(pendingConfirmation.args || {}).length
    ? `\n参数: ${JSON.stringify(pendingConfirmation.args)}`
    : "";
  els.confirmationPanel.classList.remove("hidden");
  els.confirmationText.textContent = [
    pendingConfirmation.summary || "确认执行该任务",
    `来源: ${pendingConfirmation.source === "gateway" ? "Agent MCP" : (pendingConfirmation.skill || "任务服务")}`,
    `动作: ${pendingConfirmation.action || pendingConfirmation.intent || "--"}`,
    `风险: ${pendingConfirmation.risk_level || "--"}`,
    args,
  ].filter(Boolean).join("\n");
  renderChat();
}

async function refreshSkills() {
  try {
    const data = await api("/api/skills");
    renderSkills(mergeSkills(data.skills || fallbackSkills, gatewaySkills));
  } catch (err) {
    renderSkills(mergeSkills(fallbackSkills, gatewaySkills));
    pushEvent("error", `技能库加载失败，使用本地 fallback: ${err.message}`);
  }
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
  const depth = data.depth;
  const pointcloud = data.pointcloud;
  const autonomy = data.autonomy;
  renderLiveMission(data.mission);
  renderImageAnalysis(data.image_analysis);
  latestDetections = Array.isArray(data.detections)
    ? data.detections
    : (data.detection ? [data.detection] : []);

  els.systemLine.textContent = `ROS 网关已连接 · ${source}`;
  updateBadge(els.linkBadge, source === "WS" ? "实时连接" : "HTTP 轮询", "badge");
  if (state) {
    updateBadge(els.armedBadge, state.armed ? "已解锁" : "未解锁", state.armed ? "badge" : "badge bad");
    updateBadge(els.modeBadge, state.mode || "未知模式", state.mode === "OFFBOARD" ? "badge" : "badge neutral");
    updateBadge(els.ekfBadge, state.ekf_healthy ? "EKF 正常" : "EKF 异常", state.ekf_healthy ? "badge" : "badge bad");

    els.armedValue.textContent = state.armed ? "是" : "否";
    els.modeValue.textContent = state.mode || "--";
    els.batteryValue.textContent = `${fmt(state.battery, 1)} V`;
    els.gpsValue.textContent = String(state.gps_fix ?? "--");
    els.ekfValue.textContent = state.ekf_healthy ? "正常" : "异常";
    els.hudMode.textContent = `MODE ${state.mode || "--"}`;
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

  updateActiveVerification(state, odom, autonomy);

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
    els.autonomyMessage.textContent = autonomy.message || "等待任务状态";
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
  ctx.putImageData(new ImageData(rgba, width, height), 0, 0);
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
    const boxWidth = Number(det.width || 0);
    const boxHeight = Number(det.height || 0);
    const cx = Number(det.x || 0);
    const cy = Number(det.y || 0);
    if (boxWidth <= 0 || boxHeight <= 0) return;
    const x = Math.max(0, cx - boxWidth / 2);
    const y = Math.max(0, cy - boxHeight / 2);
    const w = Math.min(width - x, boxWidth);
    const h = Math.min(height - y, boxHeight);
    const label = `${det.class_name || "target"} ${Math.round(Number(det.confidence || 0) * 100)}%`;
    ctx.strokeStyle = "#22c55e";
    ctx.fillStyle = "rgba(10, 14, 20, 0.78)";
    ctx.strokeRect(x, y, w, h);
    const textWidth = ctx.measureText(label).width + 12;
    const textHeight = Math.max(20, Math.round(width / 50));
    ctx.fillRect(x, Math.max(0, y - textHeight), textWidth, textHeight);
    ctx.fillStyle = "#bbf7d0";
    ctx.fillText(label, x + 6, Math.max(14, y - 6));
  });
  ctx.restore();
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

async function submitAgentTask(task) {
  const confirmationAction = chatConfirmationAction(task);
  if (pendingConfirmation && confirmationAction) {
    els.taskInput.value = "";
    await confirmPendingTask(confirmationAction);
    return;
  }
  addChat("user", task);
  els.taskInput.value = "";
  els.aiStatusText.textContent = "THINKING";
  els.missionPhase.textContent = "AI 正在解析";
  els.missionResult.textContent = "--";
  els.llmOutput.textContent = "正在解析自然语言任务...\n正在准备 ROS 工具调用...";
  els.planSteps.textContent = "1. 接收用户任务\n2. 解析意图\n3. 选择技能";
  els.toolCalls.textContent = "等待任务服务返回工具调用...";
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
    renderProviderOptions(event.providers || [], event.session);
    gatewaySkills = Array.isArray(event.skills) ? event.skills : [];
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
    const timeline = Array.isArray(event.timeline) && event.timeline.length
      ? event.timeline
      : (Array.isArray(event.history) ? event.history : []);
    if (timeline.length) {
      chatMessages = timeline.map((message) => ({
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
    }
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
    els.toolCalls.innerHTML += `
      <div class="tool-row">
        <span class="muted">•</span>
        <span><strong>${escapeHtml(event.tool || "MCP Tool")}</strong><small>调用中</small></span>
      </div>`;
    pushEvent("service", `Agent 工具调用: ${event.tool || "--"}`);
    return;
  }
  if (event.type === "tool.completed") {
    els.toolStatusText.textContent = event.success === false ? "MCP ERROR" : "MCP DONE";
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
    pushEvent("system", "控制确认已过期，未执行任何动作");
    return;
  }
  if (event.type === "control.started") {
    els.aiStatusText.textContent = "EXECUTING";
    els.missionPhase.textContent = "ROS Agent 正在执行";
    els.toolStatusText.textContent = "ROS CALL";
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
    pushEvent("error", `Agent Runtime: ${event.message || "未知错误"}`);
  }
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
  const workflow = mission.result?.workflow
    || (mission.action === "workflow" ? mission.args : null);
  if (workflow?.workflow_id && Array.isArray(workflow.steps)) {
    const icons = {
      completed: ["✓", "ok"],
      running: ["●", "warn-text"],
      retrying: ["↻", "warn-text"],
      paused: ["Ⅱ", "warn-text"],
      failed: ["×", "fail"],
      skipped: ["-", "muted"],
      pending: ["•", "muted"],
    };
    els.planSteps.innerHTML = workflow.steps.map((step, index) => {
      const [icon, className] = icons[step.status] || icons.pending;
      const attempt = step.attempt ? ` · 第 ${step.attempt} 次` : "";
      return `<div class="plan-row"><span class="${className}">${icon}</span><span>${index + 1}. ${escapeHtml(step.label || step.id)}<small>${escapeHtml(step.message || "")}${attempt}</small></span></div>`;
    }).join("");
    activeWorkflowId = workflow.workflow_id;
    const controllable = mission.status === "executing";
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

els.taskInput.addEventListener("keydown", (event) => {
  if (event.key !== "Enter" || event.shiftKey || event.isComposing) return;
  event.preventDefault();
  const task = els.taskInput.value.trim();
  if (!task) return;
  submitAgentTask(task);
});

els.clearChatBtn.addEventListener("click", () => {
  chatMessages = [];
  pendingConfirmation = null;
  localStorage.removeItem(CHAT_STORAGE_KEY);
  renderChat();
  renderConfirmation();
  pushEvent("system", "已清空 AI 对话");
});

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
