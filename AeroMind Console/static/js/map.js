import { state, MAP_ZOOM_MIN, MAP_ZOOM_MAX, MAP_ZOOM_STEP, el, fmt, clamp } from "./common.js";

export function mapZoomLevel() {
  const range = Math.log(MAP_ZOOM_MAX / MAP_ZOOM_MIN);
  const value = Math.log(clamp(state.mapPxPerMeter, MAP_ZOOM_MIN, MAP_ZOOM_MAX) / MAP_ZOOM_MIN) / range;
  return clamp(Math.round(1 + value * 9), 1, 10);
}

export function updateMapZoomLevel() {
  const badge = el("mapZoomLevel");
  if (!badge) return;
  badge.textContent = `L${mapZoomLevel()}`;
  badge.title = `${fmt(state.mapPxPerMeter)} px/m`;
}

export function mapCanvasSize() {
  const canvas = el("missionMap");
  const rect = canvas.getBoundingClientRect();
  return { width: rect.width, height: rect.height };
}

export function resizeMissionMap() {
  const canvas = el("missionMap");
  const rect = canvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.floor(rect.width * ratio));
  canvas.height = Math.max(1, Math.floor(rect.height * ratio));
  const ctx = canvas.getContext("2d");
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  drawMissionMap();
}

export function mapToCanvas(x, y) {
  const size = mapCanvasSize();
  return [
    size.width / 2 + (Number(y) - state.mapViewY) * state.mapPxPerMeter,
    size.height / 2 - (Number(x) - state.mapViewX) * state.mapPxPerMeter,
  ];
}

export function canvasToMap(cx, cy) {
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

export function zoomMissionMap(factor, anchor = null) {
  setMapZoom(state.mapPxPerMeter * factor, anchor);
}

export function setMapFollow(enabled) {
  state.mapFollow = enabled;
  el("mapFollowBtn").classList.toggle("active", enabled);
}

export function clearedMissionMap(map) {
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

export function renderMapStatus() {
  const data = state.map || {};
  const uav = data.uav || { x: 0, y: 0, z: 0 };
  const uavs = data.uavs || [];
  const lidarCount = (data.obstacles || []).length;
  const occCount = (data.occupancy_grid || []).length;
  const label = state.selectedVehicle || uav.name || "无人机";
  const fleet = uavs.length > 1 ? ` / ${uavs.length}架` : "";
  el("mapStatus").textContent = `${label}${fleet}: X ${fmt(uav.x)} / Y ${fmt(uav.y)} / Z ${fmt(uav.z)} | 轨迹 ${(data.trail || []).length} | 雷达 ${lidarCount} | 建图 ${occCount}`;
}

export function drawMissionMap() {
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
  drawOccupancyGrid(ctx, data.occupancy_grid || []);
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
  if (state.clickTarget && state.pointFlyEnabled) {
    drawClickTarget(ctx, state.clickTarget);
  }
}

export function visibleUavs(uavs) {
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

export function selectedFirstUavs(uavs, selectedFallback = null) {
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

export function drawFormationLines(ctx, uavs) {
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

export function drawFormationTargets(ctx) {
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

export function drawObstaclePoints(ctx, points) {
  if (!points || !points.length) return;
  ctx.save();
  ctx.fillStyle = "rgba(225,93,93,0.72)";
  for (const point of points) {
    const [x, y] = mapToCanvas(point.x, point.y);
    ctx.fillRect(x - 1.5, y - 1.5, 3, 3);
  }
  ctx.restore();
}

export function drawOccupancyGrid(ctx, cells) {
  if (!cells || !cells.length) return;
  const ppm = state.mapPxPerMeter || 4;
  ctx.save();
  for (const cell of cells) {
    const [cx, cy] = mapToCanvas(cell.x, cell.y);
    const half = Math.max(2.5, cell.r * ppm);
    // outer glow
    ctx.fillStyle = "rgba(255,80,60,0.28)";
    ctx.fillRect(cx - half - 1, cy - half - 1, (half + 1) * 2, (half + 1) * 2);
    // core cell
    ctx.fillStyle = "rgba(255,110,80,0.55)";
    ctx.fillRect(cx - half, cy - half, half * 2, half * 2);
    // border
    ctx.strokeStyle = "rgba(255,140,110,0.70)";
    ctx.lineWidth = 0.6;
    ctx.strokeRect(cx - half, cy - half, half * 2, half * 2);
  }
  ctx.restore();
}

export function drawBlockedZones(ctx, zones) {
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

export function drawPlannerPath(ctx) {
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

export function drawMapGrid(ctx, size) {
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

export function drawSearchArea(ctx, area) {
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

export function drawPolyline(ctx, points, color, width, dash) {
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

export function drawPoint(ctx, point, color, radius, label) {
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

export function drawClickTarget(ctx, target) {
  const [x, y] = mapToCanvas(target.x || 0, target.y || 0);
  const pulse = 8 + 3 * Math.sin(Date.now() / 400);
  ctx.strokeStyle = "#ff6b6b";
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.arc(x, y, pulse, 0, Math.PI * 2);
  ctx.stroke();
  ctx.strokeStyle = "#ff6b6b";
  ctx.lineWidth = 1.2;
  ctx.beginPath();
  ctx.arc(x, y, pulse + 5, 0, Math.PI * 2);
  ctx.stroke();
  ctx.fillStyle = "#ff6b6b";
  ctx.beginPath();
  ctx.arc(x, y, 4, 0, Math.PI * 2);
  ctx.fill();
  const altText = target.z ? `${fmt(-target.z)}m` : "?m";
  ctx.fillStyle = "#ff6b6b";
  ctx.font = "700 11px Arial";
  ctx.fillText(`+ ${fmt(target.x)}, ${fmt(target.y)} ${altText}`, x + 14, y - 8);
}

export function drawUav(ctx, point, isLeader = true, index = 0) {
  const base = mapToCanvas(point.x || 0, point.y || 0);
  const x = base[0] + (Number(point.screenOffsetX) || 0);
  const y = base[1] + (Number(point.screenOffsetY) || 0);
  const name = point.name || (isLeader ? "Drone1" : "W" + (index + 1));
  const size = isLeader ? 10 : 8;
  ctx.save();
  ctx.translate(x, y);
  if (isLeader) {
    ctx.fillStyle = "#43c7b9";
    ctx.strokeStyle = "#03110f";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, -size);
    ctx.lineTo(size * 0.8, size * 0.8);
    ctx.lineTo(0, size * 0.4);
    ctx.lineTo(-size * 0.8, size * 0.8);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = "rgba(67,199,185,0.25)";
    ctx.beginPath();
    ctx.arc(0, 0, size * 1.3, 0, Math.PI * 2);
    ctx.fill();
    ctx.beginPath();
    ctx.fillStyle = "#e9fffb";
    ctx.font = "bold 10px Arial";
    ctx.fillText(name, size, -size + 1);
    ctx.font = "9px Arial";
    ctx.fillText("X:" + Math.round(point.x || 0) + " Y:" + Math.round(point.y || 0), size, -size + 12);
  } else {
    ctx.fillStyle = "#f39c12";
    ctx.strokeStyle = "#03110f";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(0, 0, size, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = "#fff4e0";
    ctx.font = "bold 9px Arial";
    ctx.textAlign = "center";
    ctx.fillText(name, 0, 4);
    ctx.font = "8px Arial";
    ctx.fillText("X:" + Math.round(point.x || 0) + " Y:" + Math.round(point.y || 0), 0, size + 10);
    ctx.textAlign = "left";
  }
  ctx.restore();
}

export function fitMissionMap() {
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
