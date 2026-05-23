export const state = {
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
  previewMapActive: false,
  missionLogs: [],
  pageStartedAt: Date.now() / 1000,
  serverStartedAt: null,
  selectedVehicle: "",
  clickTarget: null,
  pointFlyEnabled: true,
};

export const MAP_ZOOM_MIN = 0.6;
export const MAP_ZOOM_MAX = 36;
export const MAP_ZOOM_STEP = 1.18;

export const text = {
  simConnected: "仿真已连接",
  simOffline: "仿真未连接",
  visionPending: "视觉待接入",
  cameraOnline: "相机在线",
  idle: "空闲",
  planning: "规划中",
  holding: "悬停保持",
  paused: "已暂停",
  stopped: "已终止",
  returning: "返航中",
  waitTask: "等待任务",
  waitDetail: "输入自然语言任务后，这里会展示大模型拆解的执行步骤。",
};

export const riskLabels = {
  clear: "清空",
  low: "低",
  medium: "中",
  high: "高",
  critical: "危急",
  unknown: "未知",
  "-": "-",
};

export const directionLabels = {
  none: "无",
  front: "前方",
  left: "左侧",
  right: "右侧",
  hard_left: "左后",
  hard_right: "右后",
};

export const plannerLabels = {
  ok: "已生成",
  blocked: "受阻",
  wait: "等待",
};

export const templates = {
  search: "搜索前方 60 米区域，发现车辆后悬停并报告坐标",
  scan: "绕目标建筑扫描一圈，保持 8 米高度并保存关键帧",
  formation: "三架无人机组成 V 字编队，沿 X 轴飞行 40 米并保持避障",
};

export const templateParams = {
  search: { target: "vehicle", forward_m: 60, half_width_m: 20, altitude_m: 8, speed_mps: 2, spacing_m: 10, count: 3, shape: "v", pre_scan: false, planned_avoidance: true, scan_margin_m: 4, obstacle_distance_m: 8, avoidance_offset_m: 6, pre_scan_stop_on_high_risk: true },
  scan: { target: "building", forward_m: 35, half_width_m: 12, altitude_m: 8, speed_mps: 1.5, spacing_m: 8, count: 3, shape: "v", pre_scan: false, planned_avoidance: true, scan_margin_m: 4, obstacle_distance_m: 8, avoidance_offset_m: 6, pre_scan_stop_on_high_risk: true },
  formation: { target: "object", forward_m: 40, half_width_m: 15, altitude_m: 10, speed_mps: 2, spacing_m: 10, count: 3, shape: "v", pre_scan: false, planned_avoidance: true, scan_margin_m: 4, obstacle_distance_m: 8, avoidance_offset_m: 6, pre_scan_stop_on_high_risk: true },
};

export function el(id) {
  return document.getElementById(id);
}

export function on(id, event, handler, options) {
  const node = el(id);
  if (!node) return;
  node.addEventListener(event, handler, options);
}

export function checked(id, fallback = false) {
  const node = el(id);
  return node ? Boolean(node.checked) : fallback;
}

export function valueOf(id, fallback = "") {
  const node = el(id);
  if (!node) return fallback;
  const value = node.value;
  return value == null || value === "" ? fallback : value;
}

export function fmt(value, digits = 1) {
  const n = Number(value);
  return Number.isFinite(n) ? n.toFixed(digits) : "-";
}

export function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

export function numInput(id, fallback) {
  const node = el(id);
  if (!node) return fallback;
  const value = Number(node.value);
  return Number.isFinite(value) ? value : fallback;
}

export function statusText(status) {
  return text[status] || status;
}

export function riskText(value) {
  const key = String(value || "unknown").toLowerCase();
  return riskLabels[key] || value || "未知";
}

export function directionText(value) {
  const key = String(value || "none").toLowerCase();
  return directionLabels[key] || value || "-";
}

export function plannerText(value) {
  return plannerLabels[value] || value;
}

export function statusLabel(status) {
  const labels = {
    preparing: "准备",
    arming: "解锁",
    taking_off: "起飞",
    forming: "编队",
    running: "运行",
    completed: "完成",
    failed: "失败",
    stopped: "停止",
    blocked: "阻断",
    recovering: "恢复",
  };
  return labels[status] || status;
}

export function toast(title, detail = "") {
  const host = el("toastHost");
  if (!host) return;
  const item = document.createElement("div");
  item.className = "toast";
  item.innerHTML = `<b>${title}</b><span>${detail}</span>`;
  host.appendChild(item);
  setTimeout(() => item.remove(), 3200);
}

export async function withBusy(button, work) {
  button.classList.add("isBusy");
  const original = button.textContent;
  button.textContent = "处理中...";
  try {
    return await work();
  } finally {
    button.textContent = original;
    button.classList.remove("isBusy");
  }
}
