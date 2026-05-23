import { state, MAP_ZOOM_STEP, templates, el, on, fmt, toast, withBusy } from "./js/common.js";
import { canvasToMap, resizeMissionMap, drawMissionMap, zoomMissionMap, setMapFollow, fitMissionMap } from "./js/map.js";
import { api, render, renderTelemetry, updateClock, previewTask, runTask, command, flightCommand, detectLatest, applyTemplateParams, buildTaskTextFromParams, renderMissionLog } from "./js/ui.js";

document.querySelectorAll(".templateChip").forEach((button) => {
  button.addEventListener("click", () => {
    const template = button.dataset.template || "";
    applyTemplateParams(template);
    el("taskInput").value = templates[template] || buildTaskTextFromParams();
    document.querySelectorAll(".templateChip").forEach((item) => item.classList.toggle("selected", item === button));
    previewTask().catch((err) => toast("预览失败", err.message));
  });
});

["targetSelect", "forwardInput", "widthInput", "altitudeInput", "speedInput", "spacingInput", "formationCountInput", "formationShapeSelect", "preScanInput", "avoidanceInput"].forEach((id) => {
  const node = el(id);
  if (!node) return;
  node.addEventListener("change", () => {
    document.querySelectorAll(".templateChip").forEach((item) => item.classList.remove("selected"));
    el("taskInput").value = buildTaskTextFromParams();
    previewTask().catch((err) => toast("预览失败", err.message));
  });
});

on("previewBtn", "click", () => previewTask().catch((err) => toast("预览失败", err.message)));
on("detectBtn", "click", () => detectLatest().catch((err) => toast("检测失败", err.message)));
on("runBtn", "click", () => runTask().catch((err) => toast("提交失败", err.message)));
on("takeoffBtn", "click", () => {
  if (window.confirm("执行起飞？")) {
    flightCommand("/api/flight/takeoff", "起飞命令", { altitude_m: 8 }).catch((err) => toast("起飞失败", err.message));
  }
});
on("hoverBtn", "click", () => flightCommand("/api/flight/hover", "悬停命令").catch((err) => toast("悬停失败", err.message)));
on("landBtn", "click", () => {
  if (window.confirm("执行降落？")) {
    flightCommand("/api/flight/land", "降落命令").catch((err) => toast("降落失败", err.message));
  }
});
on("pauseBtn", "click", () => command("/api/task/pause", "任务已暂停").catch((err) => toast("操作失败", err.message)));
on("stopBtn", "click", () => {
  if (window.confirm("终止当前任务？")) {
    command("/api/task/stop", "任务已终止").catch((err) => toast("操作失败", err.message));
  }
});
on("rtlBtn", "click", () => {
  if (window.confirm("触发返航？")) {
    command("/api/task/rtl", "已触发返航").catch((err) => toast("操作失败", err.message));
  }
});
on("voiceBtn", "click", () => {
  state.isListening = !state.isListening;
  el("voiceBtn").textContent = state.isListening ? "停止录音" : "语音";
  el("inputState").textContent = state.isListening ? "正在听取语音..." : "文本 / 语音";
  toast(state.isListening ? "语音入口已打开" : "语音入口已关闭", "下一步可接 Web Speech API 或后端 ASR");
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

let _lastMousedownTime = 0;

el("missionMap").addEventListener("mousedown", (event) => {
  const now = Date.now();
  if (now - _lastMousedownTime < 380) {
    state.mapDragging = false;
    el("missionMap").classList.remove("dragging");
    return;
  }
  _lastMousedownTime = now;
  state.mapDragging = true;
  state.mapLastX = event.clientX;
  state.mapLastY = event.clientY;
  setMapFollow(false);
  el("missionMap").classList.add("dragging");
});

el("missionMap").addEventListener("dblclick", (event) => {
  if (!state.pointFlyEnabled) return;
  if (!state.map || !state.map.uav) {
    toast("指点飞行", "无无人机位置信息，请先连接仿真");
    return;
  }
  resizeMissionMap();
  const rect = el("missionMap").getBoundingClientRect();
  const cx = event.clientX - rect.left;
  const cy = event.clientY - rect.top;
  const [worldX, worldY] = canvasToMap(cx, cy);
  const currentZ = state.map.uav.z || -8;
  const altitudeM = Math.abs(currentZ);
  const targetX = Math.round(worldX * 10) / 10;
  const targetY = Math.round(worldY * 10) / 10;

  state.clickTarget = { x: worldX, y: worldY, z: currentZ };
  drawMissionMap();

  const confirmed = window.confirm(
    `确认指点飞行？\n\n目标坐标: X ${fmt(targetX)}, Y ${fmt(targetY)}\n高度: ${altitudeM.toFixed(1)}m\n\n将使用 A* 避障自主导航至目标点。`
  );
  if (!confirmed) {
    state.clickTarget = null;
    drawMissionMap();
    return;
  }

  setMapFollow(false);

  const payload = {
    tool: "autonomous_nav",
    args: {
      x: targetX,
      y: targetY,
      z: -altitudeM,
      speed_mps: 2.0,
      replan_interval_s: 1.0,
      control_dt_s: 0.2,
      lookahead_m: 3.0,
      timeout_s: 120.0,
    },
  };
  api("/api/tools/call", payload)
    .then((data) => {
      if (!data.accepted) {
        const reason = (data.safety && data.safety.reason) || "安全门拒绝";
        state.clickTarget = null;
        drawMissionMap();
        toast("指点飞行被拒绝", reason);
        return;
      }
      const resultOk = data.result && data.result.ok;
      if (!resultOk) {
        const msg = (data.result && data.result.message) || "未知错误";
        state.clickTarget = null;
        drawMissionMap();
        toast("指点飞行失败", msg);
        return;
      }
      render(data.state);
      toast("指点飞行", `正在避障飞往 X ${fmt(targetX)}, Y ${fmt(targetY)}, 高度 ${altitudeM.toFixed(1)}m`);
    })
    .catch((err) => {
      state.clickTarget = null;
      drawMissionMap();
      toast("指点飞行失败", err.message);
    });
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

on("mapFollowBtn", "click", () => {
  setMapFollow(!state.mapFollow);
  if (state.mapFollow && state.map && state.map.uav) {
    state.mapViewX = Number(state.map.uav.x) || 0;
    state.mapViewY = Number(state.map.uav.y) || 0;
  }
  drawMissionMap();
});

on("mapFitBtn", "click", () => {
  setMapFollow(false);
  fitMissionMap();
});

on("mapZoomInBtn", "click", () => {
  setMapFollow(false);
  zoomMissionMap(MAP_ZOOM_STEP);
});

on("mapZoomOutBtn", "click", () => {
  setMapFollow(false);
  zoomMissionMap(1 / MAP_ZOOM_STEP);
});

on("pointFlyBtn", "click", () => {
  state.pointFlyEnabled = !state.pointFlyEnabled;
  const btn = el("pointFlyBtn");
  if (btn) {
    btn.classList.toggle("active", state.pointFlyEnabled);
    btn.textContent = state.pointFlyEnabled ? "指点:开" : "指点:关";
  }
  if (!state.pointFlyEnabled) {
    state.clickTarget = null;
  }
  drawMissionMap();
  const label = state.pointFlyEnabled ? "双击地图任意位置发起避障飞行" : "指点飞行已关闭";
  toast("指点飞行", label);
});

on("connectionPill", "click", () => {
  api("/api/airsim/reconnect", {})
    .then((data) => {
      render(data);
      toast("连接已刷新", data.connected ? "AirSim 在线" : (data.airsim_error || "AirSim 离线"));
    })
    .catch((err) => toast("连接失败", err.message));
});

on("visionPill", "click", () => {
  api("/api/camera/probe", {})
    .then((data) => {
      const ok = data.results.find((item) => item.ok);
      toast(
        ok ? "摄像头可用" : "未找到可用摄像头",
        ok ? `${ok.camera}, ${ok.bytes} bytes` : (data.last_error || "Check AirSim camera name and settings.json")
      );
    })
    .catch((err) => toast("摄像头探测失败", err.message));
});

document.querySelectorAll(".cameraChip").forEach((button) => {
  button.addEventListener("click", () => {
    const camera = button.dataset.camera || "";
    withBusy(button, async () => {
      const data = await api("/api/camera/select", { camera });
      if (!data.ok) {
        throw new Error(data.detail || "摄像头切换失败");
      }
      state.selectedCamera = "";
      render(data.state);
      toast("视角已切换", camera);
    }).catch((err) => toast("摄像头切换失败", err.message));
  });
});

on("vehicleSelect", "change", () => {
  const vehicle = el("vehicleSelect").value;
  api("/api/vehicle/select", { vehicle })
    .then((data) => {
      if (!data.ok) {
        throw new Error(data.detail || "无人机切换失败");
      }
      state.selectedCamera = "";
      state.previewMapActive = false;
      render(data.state);
      toast("无人机已切换", vehicle);
    })
    .catch((err) => {
      toast("无人机切换失败", err.message);
      api("/api/state").then(render).catch(() => {});
    });
});

el("clearLogBtn")?.addEventListener("click", () => {
  state.missionLogs = [];
  renderMissionLog();
});

function connectWS() {
  const protocol = location.protocol === "https:" ? "wss:" : "ws:";
  const ws = new WebSocket(`${protocol}//${location.host}/ws`);

  ws.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);
      if (msg.type === "state") {
        render(msg.data);
      } else if (msg.type === "telemetry") {
        renderTelemetry(msg.data);
      }
    } catch (e) {
      console.warn("WS message parse error", e);
    }
  };

  ws.onclose = () => {
    setTimeout(connectWS, 1200);
  };

  ws.onerror = () => {
    ws.close();
  };
}

setInterval(updateClock, 1000);
window.addEventListener("resize", resizeMissionMap);

api("/api/state").then(render).catch((err) => console.warn(err));
api("/api/telemetry").then(renderTelemetry).catch((err) => console.warn(err));
resizeMissionMap();
connectWS();
