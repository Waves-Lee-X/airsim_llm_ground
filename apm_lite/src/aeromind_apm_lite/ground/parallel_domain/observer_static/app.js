const canvas = document.querySelector("#map");
const ctx = canvas.getContext("2d");
const colors = { px4: "#1769e0", apm: "#00875a" };
let snapshot = null;

function number(value, digits = 2) {
  return Number.isFinite(Number(value)) ? Number(value).toFixed(digits) : "--";
}

function updateLine(lineId, line) {
  const state = document.querySelector(`#${lineId}State`);
  state.textContent = String(line?.state || "waiting").toUpperCase();
  state.className = `state ${line?.state || "waiting"}`;
  state.title = line?.reason || "";
  document.querySelector(`#${lineId}Sequence`).textContent = line?.observed_sequence || 0;
  document.querySelector(`#${lineId}Sim`).textContent = line?.latest
    ? `${number(line.latest.sim_time, 1)} s`
    : "--";
  document.querySelector(`#${lineId}Position`).textContent = line?.latest
    ? `${number(line.latest.x)} / ${number(line.latest.y)} / ${number(line.latest.z)}`
    : "--";
}

function niceStep(span) {
  const raw = Math.max(span / 8, 0.1);
  const power = 10 ** Math.floor(Math.log10(raw));
  const unit = raw / power;
  return (unit <= 1 ? 1 : unit <= 2 ? 2 : unit <= 5 ? 5 : 10) * power;
}

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  const width = Math.max(1, Math.round(rect.width * ratio));
  const height = Math.max(1, Math.round(rect.height * ratio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  draw();
}

function draw() {
  const width = canvas.width;
  const height = canvas.height;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#fafbfc";
  ctx.fillRect(0, 0, width, height);
  const all = snapshot
    ? Object.values(snapshot.lines).flatMap((line) => line.points || [])
    : [];
  document.querySelector("#empty").hidden = all.length > 0;
  if (!all.length) return;

  const xs = all.map((point) => Number(point.x)).concat(0);
  const ys = all.map((point) => Number(point.y)).concat(0);
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);
  const span = Math.max(maxX - minX, maxY - minY, 4);
  const padding = Math.max(40, Math.min(width, height) * 0.08);
  const scale = Math.min((width - padding * 2) / span, (height - padding * 2) / span);
  const centerX = (minX + maxX) / 2;
  const centerY = (minY + maxY) / 2;
  const project = (point) => ({
    x: width / 2 + (Number(point.x) - centerX) * scale,
    y: height / 2 - (Number(point.y) - centerY) * scale,
  });

  const step = niceStep(span);
  ctx.lineWidth = 1;
  ctx.strokeStyle = "#e1e4e7";
  ctx.fillStyle = "#7b838a";
  ctx.font = `${Math.max(10, Math.round(width / 140))}px ui-monospace, monospace`;
  const startX = Math.floor((centerX - span) / step) * step;
  const endX = Math.ceil((centerX + span) / step) * step;
  for (let x = startX; x <= endX; x += step) {
    const pixel = project({ x, y: centerY }).x;
    ctx.beginPath(); ctx.moveTo(pixel, 0); ctx.lineTo(pixel, height); ctx.stroke();
    ctx.fillText(`${Number(x.toFixed(2))}m`, pixel + 4, height - 10);
  }
  const startY = Math.floor((centerY - span) / step) * step;
  const endY = Math.ceil((centerY + span) / step) * step;
  for (let y = startY; y <= endY; y += step) {
    const pixel = project({ x: centerX, y }).y;
    ctx.beginPath(); ctx.moveTo(0, pixel); ctx.lineTo(width, pixel); ctx.stroke();
    ctx.fillText(`${Number(y.toFixed(2))}m`, 8, pixel - 5);
  }

  const origin = project({ x: 0, y: 0 });
  ctx.strokeStyle = "#9ca3a9";
  ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(origin.x, 0); ctx.lineTo(origin.x, height); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(0, origin.y); ctx.lineTo(width, origin.y); ctx.stroke();

  for (const lineId of ["px4", "apm"]) {
    const points = snapshot.lines[lineId]?.points || [];
    if (!points.length) continue;
    ctx.strokeStyle = colors[lineId];
    ctx.lineWidth = Math.max(2, width / 600);
    ctx.lineJoin = "round";
    ctx.beginPath();
    points.forEach((point, index) => {
      const pixel = project(point);
      if (index === 0) ctx.moveTo(pixel.x, pixel.y);
      else ctx.lineTo(pixel.x, pixel.y);
    });
    ctx.stroke();
    const latest = project(points[points.length - 1]);
    ctx.fillStyle = colors[lineId];
    ctx.beginPath(); ctx.arc(latest.x, latest.y, Math.max(5, width / 180), 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 2; ctx.stroke();
  }
}

async function refresh() {
  try {
    const response = await fetch("/api/observed", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    snapshot = await response.json();
    updateLine("px4", snapshot.lines.px4);
    updateLine("apm", snapshot.lines.apm);
    document.querySelector("#clock").textContent = new Date(snapshot.generated_at_utc).toLocaleTimeString();
    const line = snapshot.lines.px4?.calibration_sha256
      ? snapshot.lines.px4
      : snapshot.lines.apm;
    document.querySelector("#calibration").textContent = line?.calibration_sha256
      ? `${String(line.calibration_status || "unknown").toUpperCase()} · ${line.calibration_id} · ${line.calibration_sha256.slice(0, 12)}`
      : "CALIBRATION --";
    draw();
  } catch (error) {
    for (const lineId of ["px4", "apm"]) {
      const state = document.querySelector(`#${lineId}State`);
      state.textContent = "OFFLINE";
      state.className = "state stale";
    }
  }
}

new ResizeObserver(resizeCanvas).observe(canvas.parentElement);
refresh();
setInterval(refresh, 500);
