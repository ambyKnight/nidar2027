/**
 * AirMouse GCS Web Dashboard
 * Connects to rosbridge_server (port 9090) and renders the live grid, drone pose,
 * exploration state, survivor detections and a mission event log.
 */

// --- Tunables ---
const TOF_DANGER = 0.30;   // m
const TOF_CAUTION = 0.50;  // m
const TOF_RANGE = 2.00;    // m, full-scale for the clearance bars
const STALE_AFTER = 3.0;   // s without a packet before the link is called stale
const MAX_LOG_ROWS = 200;

// --- Global State ---
const state = {
  ws: null,
  connected: false,
  reconnectTimer: null,
  lastMsgAt: 0,
  msgCount: 0,
  msgRate: 0,

  // Drone & Mission
  dronePose: { x: 0.0, y: 0.0, z: 0.0, yaw: 0.0 },
  lastPose: null,          // { x, y, t } for ground-speed estimation
  speed: 0.0,
  currentCell: [0, 0],
  cells: {},               // "i,j" -> { "+x": "...", "-x": "...", "+y": "...", "-y": "...", "seen": true }
  visitedCells: new Set(),
  queue: [],               // [[i, j], ...]
  trail: [],               // [{x, y}, ...]
  explorer: {
    state: "STANDBY",
    visited: 0,
    mapped: 0,
    going_home: false,
    camera_seen: 0,
    guard_events: 0,
    tof: {},
    elapsed: 0.0
  },
  survivors: [],           // [{id, tag, cell, x, y, confidence}]

  // Derived / alerting
  prevPhase: null,
  prevGuardEvents: 0,
  prevSurvivorIds: new Set(),
  tofDangerSides: new Set(),

  // Canvas & Viewport
  canvas: null,
  ctx: null,
  panX: 0,
  panY: 0,
  zoom: 55,                // pixels per meter
  followDrone: true,
  isDragging: false,
  dragStartX: 0,
  dragStartY: 0,
  cellSize: 1.0
};

// --- DOM Elements ---
const el = {};
function bindDom() {
  const ids = [
    "connection-pill", "connection-label", "phase-badge", "flight-timer", "msg-rate",
    "val-pos-x", "val-pos-y", "val-pos-z", "val-yaw", "val-speed", "val-current-cell",
    "val-visited", "val-mapped", "val-queue", "val-camera-seen", "val-going-home",
    "val-guard-events", "val-survivor-count", "survivor-list", "compass-needle",
    "tof-fwd", "tof-bwd", "tof-left", "tof-right",
    "tof-fwd-box", "tof-bwd-box", "tof-left-box", "tof-right-box",
    "map-canvas", "btn-zoom-in", "btn-zoom-out", "btn-fit-view", "btn-follow-drone",
    "btn-clear-trail", "btn-clear-log", "btn-theme", "theme-icon",
    "map-grid-count", "map-scale-info", "map-cursor",
    "alert-banner", "alert-banner-text", "alert-dismiss",
    "event-log", "toast-stack", "status-endpoint", "status-last-msg"
  ];
  for (const id of ids) {
    el[id.replace(/-([a-z])/g, (_, c) => c.toUpperCase())] = document.getElementById(id);
  }
}

// --- Event log, toasts and the critical-alert banner ---
function clockStamp() {
  const d = new Date();
  return [d.getHours(), d.getMinutes(), d.getSeconds()]
    .map((n) => n.toString().padStart(2, "0")).join(":");
}

function logEvent(message, level = "info") {
  const row = document.createElement("div");
  row.className = `log-row ${level}`;
  row.innerHTML = `<span class="log-time">${clockStamp()}</span><span>${message}</span>`;
  el.eventLog.appendChild(row);
  while (el.eventLog.childElementCount > MAX_LOG_ROWS) {
    el.eventLog.removeChild(el.eventLog.firstChild);
  }
  el.eventLog.scrollTop = el.eventLog.scrollHeight;
}

function toast(message, level = "info", ttl = 4000) {
  const node = document.createElement("div");
  node.className = `toast ${level}`;
  node.textContent = message;
  el.toastStack.appendChild(node);
  setTimeout(() => node.remove(), ttl);
}

// Critical states must be unmissable: the banner stays up until the cause clears.
function setBanner(message) {
  if (!message) {
    el.alertBanner.classList.add("hidden");
    return;
  }
  el.alertBannerText.textContent = message;
  el.alertBanner.classList.remove("hidden");
}

function refreshBanner() {
  if (!state.connected) {
    setBanner("TELEMETRY LINK DOWN — reconnecting to rosbridge…");
  } else if (state.tofDangerSides.size > 0) {
    const sides = [...state.tofDangerSides].join(", ").toUpperCase();
    setBanner(`OBSTACLE PROXIMITY — ${sides} below ${TOF_DANGER.toFixed(2)} m`);
  } else if (state.explorer.state === "ABORTED") {
    setBanner("MISSION ABORTED — drone is no longer exploring");
  } else {
    setBanner(null);
  }
}

// --- WebSocket & Rosbridge Communication ---
function initWebSocket() {
  const host = window.location.hostname || "localhost";
  const wsUrl = `ws://${host}:9090`;
  el.statusEndpoint.textContent = wsUrl;

  updateConnectionStatus(false, "CONNECTING...");

  try {
    state.ws = new WebSocket(wsUrl);
  } catch (err) {
    console.error("WebSocket init error:", err);
    scheduleReconnect();
    return;
  }

  state.ws.onopen = () => {
    updateConnectionStatus(true, "CONNECTED");
    logEvent(`Link established to ${wsUrl}`, "good");
    toast("Telemetry link established", "good", 2500);
    if (state.reconnectTimer) {
      clearTimeout(state.reconnectTimer);
      state.reconnectTimer = null;
    }

    // Subscribe to topics
    subscribeTopic("/airmouse/grid", "std_msgs/msg/String");
    subscribeTopic("/airmouse/explorer", "std_msgs/msg/String");
    subscribeTopic("/airmouse/survivors", "std_msgs/msg/String");
    subscribeTopic("/mavros/local_position/pose", "geometry_msgs/msg/PoseStamped");
  };

  state.ws.onmessage = (event) => {
    state.msgCount += 1;
    state.lastMsgAt = Date.now();
    try {
      const data = JSON.parse(event.data);
      handleRosMessage(data);
    } catch (e) {
      console.warn("Failed to parse JSON:", e, event.data);
    }
  };

  state.ws.onclose = () => {
    if (state.connected) logEvent("Telemetry link lost", "error");
    updateConnectionStatus(false, "DISCONNECTED");
    scheduleReconnect();
  };

  state.ws.onerror = (err) => {
    console.warn("WebSocket error:", err);
    state.ws.close();
  };
}

function scheduleReconnect() {
  if (!state.reconnectTimer) {
    state.reconnectTimer = setTimeout(() => {
      state.reconnectTimer = null;
      initWebSocket();
    }, 2000);
  }
}

function subscribeTopic(topic, type) {
  if (state.ws && state.ws.readyState === WebSocket.OPEN) {
    state.ws.send(JSON.stringify({ op: "subscribe", topic: topic, type: type }));
  }
}

function updateConnectionStatus(connected, label) {
  const wasConnected = state.connected;
  state.connected = connected;
  el.connectionLabel.textContent = label;
  el.connectionPill.classList.toggle("connected", connected);
  el.connectionPill.classList.toggle("disconnected", !connected);
  if (wasConnected && !connected) toast("Telemetry link lost", "error", 6000);
  refreshBanner();
}

// Link health ticker: packet rate, staleness and the "last packet" readout.
function startHealthTicker() {
  setInterval(() => {
    state.msgRate = state.msgCount;
    state.msgCount = 0;
    el.msgRate.innerHTML = `${state.msgRate} <span class="unit">Hz</span>`;

    if (state.lastMsgAt === 0) {
      el.statusLastMsg.textContent = "last packet —";
    } else {
      const age = (Date.now() - state.lastMsgAt) / 1000;
      el.statusLastMsg.textContent = `last packet ${age.toFixed(1)}s ago`;
      if (state.connected && age > STALE_AFTER) {
        updateConnectionStatus(true, "STALE");
      } else if (state.connected && el.connectionLabel.textContent === "STALE") {
        updateConnectionStatus(true, "CONNECTED");
      }
    }
  }, 1000);
}

// --- ROS Message Handling ---
function handleRosMessage(msg) {
  if (!msg || msg.op !== "publish") return;

  const topic = msg.topic;

  if (topic === "/airmouse/grid") {
    handleGridMessage(msg.msg);
  } else if (topic === "/airmouse/explorer") {
    handleExplorerMessage(msg.msg);
  } else if (topic === "/airmouse/survivors") {
    handleSurvivorsMessage(msg.msg);
  } else if (topic === "/mavros/local_position/pose") {
    handlePoseMessage(msg.msg);
  }
}

function handleSurvivorsMessage(msgData) {
  try {
    const payload = typeof msgData.data === "string" ? JSON.parse(msgData.data) : msgData.data;
    state.survivors = payload.survivors || [];
    el.valSurvivorCount.textContent = state.survivors.length;

    if (state.survivors.length === 0) {
      el.survivorList.innerHTML = '<span class="no-data">No survivors tagged yet</span>';
    } else {
      el.survivorList.innerHTML = state.survivors.map((s) => `
        <div class="survivor-item">
          <span class="survivor-tag">✛ ${s.tag}</span>
          <span class="survivor-coords">Cell (${s.cell[0]}, ${s.cell[1]})</span>
        </div>
      `).join("");
    }

    // Announce newly tagged survivors only once.
    for (const s of state.survivors) {
      const key = String(s.id ?? s.tag);
      if (!state.prevSurvivorIds.has(key)) {
        state.prevSurvivorIds.add(key);
        logEvent(`Survivor ${s.tag} tagged at cell (${s.cell[0]}, ${s.cell[1]})`, "warn");
        toast(`Survivor ${s.tag} detected`, "warn", 5000);
      }
    }
  } catch (err) {
    console.warn("Error parsing survivors message:", err);
  }
}

function handleGridMessage(msgData) {
  try {
    const payload = typeof msgData.data === "string" ? JSON.parse(msgData.data) : msgData.data;
    if (payload.cells) {
      state.cells = payload.cells;
      state.cellSize = payload.cell_size || 1.0;
      el.mapGridCount.textContent = `${Object.keys(state.cells).length} cells rendered`;
    }
  } catch (err) {
    console.warn("Error parsing grid message:", err);
  }
}

function handleExplorerMessage(msgData) {
  try {
    const data = typeof msgData.data === "string" ? JSON.parse(msgData.data) : msgData.data;
    state.explorer = Object.assign(state.explorer, data);

    // Phase badge
    const phase = data.state || "STANDBY";
    el.phaseBadge.textContent = phase;
    el.phaseBadge.className = "badge";
    if (["FLY", "LOOK", "SETTLE", "SPIN"].includes(phase)) {
      el.phaseBadge.classList.add("badge-active");
    } else if (["HOME", "EXIT", "LANDING"].includes(phase)) {
      el.phaseBadge.classList.add("badge-home");
    } else if (phase === "ABORTED") {
      el.phaseBadge.classList.add("badge-alert");
    } else {
      el.phaseBadge.classList.add("badge-idle");
    }
    if (phase !== state.prevPhase) {
      if (state.prevPhase !== null) {
        const level = phase === "ABORTED" ? "error" : "info";
        logEvent(`Phase ${state.prevPhase} → ${phase}`, level);
        if (phase === "ABORTED") toast("Mission aborted", "error", 8000);
      }
      state.prevPhase = phase;
      refreshBanner();
    }

    // Mission timer
    if (data.elapsed !== undefined) {
      const totalSec = Math.floor(data.elapsed);
      const m = Math.floor(totalSec / 60).toString().padStart(2, "0");
      const s = (totalSec % 60).toString().padStart(2, "0");
      el.flightTimer.textContent = `${m}:${s}`;
    }

    // Current cell
    if (Array.isArray(data.current)) {
      state.currentCell = data.current;
      el.valCurrentCell.textContent = `(${data.current[0]}, ${data.current[1]})`;
      state.visitedCells.add(`${data.current[0]},${data.current[1]}`);
    }

    // Counts
    if (data.visited !== undefined) el.valVisited.textContent = data.visited;
    if (data.mapped !== undefined) el.valMapped.textContent = data.mapped;
    if (Array.isArray(data.queue)) {
      state.queue = data.queue;
      el.valQueue.textContent = data.queue.length;
    }
    if (data.camera_seen !== undefined) el.valCameraSeen.textContent = data.camera_seen;
    if (data.going_home !== undefined) {
      el.valGoingHome.textContent = data.going_home ? "YES" : "NO";
      el.valGoingHome.className = `kpi-value ${data.going_home ? "status-on" : "status-off"}`;
    }
    if (data.guard_events !== undefined) {
      el.valGuardEvents.textContent = data.guard_events;
      if (data.guard_events > state.prevGuardEvents) {
        logEvent(`Wall guard triggered (total ${data.guard_events})`, "warn");
      }
      state.prevGuardEvents = data.guard_events;
    }

    // ToF clearance
    if (data.tof) {
      updateTofDisplay("front", data.tof.front ?? data.tof.forward ?? data.tof.fwd, el.tofFwd, el.tofFwdBox);
      updateTofDisplay("back", data.tof.back ?? data.tof.backward ?? data.tof.bwd, el.tofBwd, el.tofBwdBox);
      updateTofDisplay("left", data.tof.left, el.tofLeft, el.tofLeftBox);
      updateTofDisplay("right", data.tof.right, el.tofRight, el.tofRightBox);
      refreshBanner();
    }
  } catch (err) {
    console.warn("Error parsing explorer state:", err);
  }
}

function updateTofDisplay(side, val, valEl, boxEl) {
  const bar = boxEl.querySelector(".tof-bar i");

  if (val === undefined || val === null || val === Infinity) {
    valEl.textContent = "--";
    boxEl.className = "tof-box";
    if (bar) bar.style.width = "0%";
    state.tofDangerSides.delete(side);
    return;
  }

  const dist = typeof val === "number" ? val : parseFloat(val);
  valEl.textContent = `${dist.toFixed(2)} m`;
  boxEl.className = "tof-box";
  if (bar) bar.style.width = `${Math.min(100, (dist / TOF_RANGE) * 100)}%`;

  const wasDanger = state.tofDangerSides.has(side);
  if (dist < TOF_DANGER) {
    boxEl.classList.add("danger");
    state.tofDangerSides.add(side);
    if (!wasDanger) logEvent(`${side.toUpperCase()} clearance ${dist.toFixed(2)} m — danger`, "error");
  } else {
    boxEl.classList.add(dist < TOF_CAUTION ? "warn" : "safe");
    state.tofDangerSides.delete(side);
  }
}

function handlePoseMessage(msgData) {
  const p = msgData.pose.position;
  const q = msgData.pose.orientation;

  state.dronePose.x = p.x;
  state.dronePose.y = p.y;
  state.dronePose.z = p.z;

  // Quaternion to Yaw
  const siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  state.dronePose.yaw = Math.atan2(siny_cosp, cosy_cosp);

  // Ground speed from successive pose samples
  const now = Date.now();
  if (state.lastPose) {
    const dt = (now - state.lastPose.t) / 1000;
    if (dt > 0.05) {
      state.speed = Math.hypot(p.x - state.lastPose.x, p.y - state.lastPose.y) / dt;
      state.lastPose = { x: p.x, y: p.y, t: now };
    }
  } else {
    state.lastPose = { x: p.x, y: p.y, t: now };
  }

  // UI Readouts
  el.valPosX.innerHTML = `${p.x.toFixed(2)} <span class="unit">m</span>`;
  el.valPosY.innerHTML = `${p.y.toFixed(2)} <span class="unit">m</span>`;
  el.valPosZ.innerHTML = `${p.z.toFixed(2)} <span class="unit">m</span>`;
  el.valSpeed.innerHTML = `${state.speed.toFixed(2)} <span class="unit">m/s</span>`;

  const deg = state.dronePose.yaw * 180 / Math.PI;
  el.valYaw.innerHTML = `${deg.toFixed(1)}<span class="unit">°</span>`;
  // Canvas/compass rotate clockwise for a counter-clockwise (ENU) yaw.
  el.compassNeedle.setAttribute("transform", `rotate(${-deg})`);

  // Breadcrumb Trail
  const last = state.trail[state.trail.length - 1];
  if (!last || Math.hypot(p.x - last.x, p.y - last.y) > 0.15) {
    state.trail.push({ x: p.x, y: p.y });
    if (state.trail.length > 500) state.trail.shift();
  }
}

// --- Coordinate Transformations ---
function toScreenX(worldX) { return state.panX + worldX * state.zoom; }
function toScreenY(worldY) { return state.panY - worldY * state.zoom; }  // canvas +y is down
function toWorldX(screenX) { return (screenX - state.panX) / state.zoom; }
function toWorldY(screenY) { return (state.panY - screenY) / state.zoom; }

function cssVar(name, fallback) {
  const v = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  return v || fallback;
}

// --- Canvas 2D Rendering ---
function renderMap() {
  const canvas = state.canvas;
  const ctx = state.ctx;
  if (!canvas || !ctx) return;

  const w = canvas.width;
  const h = canvas.height;

  // Auto-follow drone
  if (state.followDrone) {
    const targetPanX = w / 2 - state.dronePose.x * state.zoom;
    const targetPanY = h / 2 + state.dronePose.y * state.zoom;
    state.panX += (targetPanX - state.panX) * 0.12;
    state.panY += (targetPanY - state.panY) * 0.12;
  }

  ctx.fillStyle = cssVar("--bg", "#0a0d14");
  ctx.fillRect(0, 0, w, h);

  drawGridLines(ctx, w, h);
  drawOrigin(ctx);
  drawCells(ctx);
  drawPlannedRoute(ctx);
  drawTrail(ctx);
  drawSurvivors(ctx);
  drawDrone(ctx);

  requestAnimationFrame(renderMap);
}

function drawGridLines(ctx, w, h) {
  ctx.lineWidth = 1;

  // Visible coordinate bounds in meters
  const minX = Math.floor(toWorldX(0)) - 1;
  const maxX = Math.ceil(toWorldX(w)) + 1;
  const minY = Math.floor(toWorldY(h)) - 1;
  const maxY = Math.ceil(toWorldY(0)) + 1;

  ctx.strokeStyle = isLight() ? "rgba(15, 23, 42, 0.07)" : "rgba(255, 255, 255, 0.04)";
  ctx.beginPath();
  for (let x = minX; x <= maxX; x++) {
    const sx = toScreenX(x);
    ctx.moveTo(sx, 0);
    ctx.lineTo(sx, h);
  }
  for (let y = minY; y <= maxY; y++) {
    const sy = toScreenY(y);
    ctx.moveTo(0, sy);
    ctx.lineTo(w, sy);
  }
  ctx.stroke();
}

function drawOrigin(ctx) {
  const ox = toScreenX(0);
  const oy = toScreenY(0);

  ctx.strokeStyle = "rgba(234, 179, 8, 0.55)";
  ctx.lineWidth = 1.5;
  ctx.setLineDash([]);
  ctx.beginPath();
  ctx.arc(ox, oy, 6, 0, Math.PI * 2);
  ctx.moveTo(ox - 10, oy);
  ctx.lineTo(ox + 10, oy);
  ctx.moveTo(ox, oy - 10);
  ctx.lineTo(ox, oy + 10);
  ctx.stroke();

  ctx.fillStyle = "rgba(234, 179, 8, 0.85)";
  ctx.font = "9px monospace";
  ctx.textAlign = "left";
  ctx.fillText("TAKEOFF (0,0)", ox + 10, oy + 12);
}

function drawCells(ctx) {
  const cell = state.cellSize;

  for (const [key, sides] of Object.entries(state.cells)) {
    const [iStr, jStr] = key.split(",");
    const i = parseInt(iStr, 10);
    const j = parseInt(jStr, 10);

    // Bounds of cell (i, j) in world coordinates
    const sxl = toScreenX((i - 0.5) * cell);
    const sxh = toScreenX((i + 0.5) * cell);
    const syh = toScreenY((j + 0.5) * cell);
    const syl = toScreenY((j - 0.5) * cell);

    const isVisited = state.visitedCells.has(key);
    ctx.fillStyle = isVisited ? "rgba(34, 197, 94, 0.16)" : "rgba(148, 163, 184, 0.06)";
    ctx.fillRect(sxl, syh, sxh - sxl, syl - syh);

    drawSide(ctx, sxh, syl, sxh, syh, sides["+x"]);
    drawSide(ctx, sxl, syl, sxl, syh, sides["-x"]);
    drawSide(ctx, sxl, syh, sxh, syh, sides["+y"]);
    drawSide(ctx, sxl, syl, sxh, syl, sides["-y"]);
  }
}

function drawSide(ctx, x1, y1, x2, y2, classification) {
  ctx.beginPath();
  if (classification === "wall") {
    ctx.strokeStyle = "#ef4444";
    ctx.lineWidth = 3.5;
    ctx.setLineDash([]);
  } else if (classification === "open") {
    ctx.strokeStyle = "#22c55e";
    ctx.lineWidth = 2.0;
    ctx.setLineDash([4, 4]);
  } else {
    ctx.strokeStyle = "rgba(100, 116, 139, 0.45)";
    ctx.lineWidth = 1.0;
    ctx.setLineDash([2, 3]);
  }
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.stroke();
  ctx.setLineDash([]);
}

function drawPlannedRoute(ctx) {
  if (!state.queue || state.queue.length === 0) return;

  ctx.strokeStyle = "rgba(0, 229, 255, 0.7)";
  ctx.lineWidth = 2;
  ctx.setLineDash([6, 4]);
  ctx.beginPath();
  ctx.moveTo(toScreenX(state.dronePose.x), toScreenY(state.dronePose.y));
  for (const wp of state.queue) {
    ctx.lineTo(toScreenX(wp[0] * state.cellSize), toScreenY(wp[1] * state.cellSize));
  }
  ctx.stroke();
  ctx.setLineDash([]);

  for (let idx = 0; idx < state.queue.length; idx++) {
    const wp = state.queue[idx];
    const sx = toScreenX(wp[0] * state.cellSize);
    const sy = toScreenY(wp[1] * state.cellSize);
    ctx.fillStyle = idx === 0 ? "#00e5ff" : "rgba(0, 229, 255, 0.5)";
    ctx.beginPath();
    ctx.arc(sx, sy, idx === 0 ? 5 : 3.5, 0, Math.PI * 2);
    ctx.fill();
  }
}

function drawTrail(ctx) {
  if (state.trail.length < 2) return;

  ctx.strokeStyle = "rgba(0, 229, 255, 0.3)";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  for (let i = 0; i < state.trail.length; i++) {
    const pt = state.trail[i];
    const sx = toScreenX(pt.x);
    const sy = toScreenY(pt.y);
    if (i === 0) ctx.moveTo(sx, sy);
    else ctx.lineTo(sx, sy);
  }
  ctx.stroke();
}

function drawSurvivors(ctx) {
  if (!state.survivors || state.survivors.length === 0) return;

  const pulse = Math.sin(Date.now() / 250) * 3;
  for (const s of state.survivors) {
    const sx = toScreenX(s.x);
    const sy = toScreenY(s.y);

    ctx.strokeStyle = "rgba(249, 115, 22, 0.4)";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(sx, sy, 14 + pulse, 0, Math.PI * 2);
    ctx.stroke();

    ctx.fillStyle = "#f97316";
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(sx, sy, 9, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(sx - 4, sy); ctx.lineTo(sx + 4, sy);
    ctx.moveTo(sx, sy - 4); ctx.lineTo(sx, sy + 4);
    ctx.stroke();

    ctx.fillStyle = isLight() ? "#c2410c" : "#fdba74";
    ctx.font = "bold 10px monospace";
    ctx.textAlign = "center";
    ctx.fillText(`${s.tag} (${s.cell[0]},${s.cell[1]})`, sx, sy - 14);
  }
}

function drawDrone(ctx) {
  const sx = toScreenX(state.dronePose.x);
  const sy = toScreenY(state.dronePose.y);

  ctx.save();
  ctx.translate(sx, sy);
  // Heading angle in canvas screen coordinates: -yaw
  ctx.rotate(-state.dronePose.yaw);

  // Heading Beam / Sensor Cone
  const grad = ctx.createRadialGradient(0, 0, 5, 0, -50, 45);
  grad.addColorStop(0, "rgba(0, 229, 255, 0.35)");
  grad.addColorStop(1, "rgba(0, 229, 255, 0.0)");
  ctx.fillStyle = grad;
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.arc(0, 0, 50, -Math.PI / 2 - 0.45, -Math.PI / 2 + 0.45);
  ctx.closePath();
  ctx.fill();

  // Drone Body (Stylized Quad Arrow)
  ctx.fillStyle = "#00e5ff";
  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(0, -14);
  ctx.lineTo(10, 10);
  ctx.lineTo(0, 5);
  ctx.lineTo(-10, 10);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();

  ctx.fillStyle = "#ffffff";
  ctx.beginPath();
  ctx.arc(0, 0, 2.5, 0, Math.PI * 2);
  ctx.fill();

  ctx.restore();
}

// --- Theme ---
function isLight() {
  return document.documentElement.getAttribute("data-theme") === "light";
}

function applyTheme(theme) {
  document.documentElement.setAttribute("data-theme", theme);
  el.themeIcon.textContent = theme === "light" ? "☀" : "☾";
  try { localStorage.setItem("airmouse-theme", theme); } catch (e) { /* private mode */ }
}

function initTheme() {
  let theme = "dark";
  try { theme = localStorage.getItem("airmouse-theme") || "dark"; } catch (e) { /* private mode */ }
  applyTheme(theme);
}

// --- Viewport & Event Handlers ---
function resizeCanvas() {
  const rect = state.canvas.parentElement.getBoundingClientRect();
  state.canvas.width = rect.width;
  state.canvas.height = rect.height;
  if (!state.followDrone && state.panX === 0 && state.panY === 0) {
    state.panX = rect.width / 2;
    state.panY = rect.height / 2;
  }
}

function setZoom(z, anchorX, anchorY) {
  const worldX = anchorX !== undefined ? toWorldX(anchorX) : null;
  const worldY = anchorY !== undefined ? toWorldY(anchorY) : null;
  state.zoom = Math.max(15, Math.min(250, z));
  if (worldX !== null) {
    state.panX = anchorX - worldX * state.zoom;
    state.panY = anchorY + worldY * state.zoom;
  }
  el.mapScaleInfo.textContent = `Scale: ${Math.round(state.zoom)} px/m`;
}

function setupEvents() {
  window.addEventListener("resize", resizeCanvas);

  state.canvas.addEventListener("mousedown", (e) => {
    state.isDragging = true;
    state.dragStartX = e.clientX - state.panX;
    state.dragStartY = e.clientY - state.panY;
  });

  window.addEventListener("mousemove", (e) => {
    if (state.isDragging) {
      state.panX = e.clientX - state.dragStartX;
      state.panY = e.clientY - state.dragStartY;
      setFollowDrone(false);
    }
  });

  window.addEventListener("mouseup", () => { state.isDragging = false; });

  // Cursor world-coordinate readout
  state.canvas.addEventListener("mousemove", (e) => {
    const rect = state.canvas.getBoundingClientRect();
    const wx = toWorldX(e.clientX - rect.left);
    const wy = toWorldY(e.clientY - rect.top);
    el.mapCursor.textContent = `x ${wx.toFixed(2)} · y ${wy.toFixed(2)}`;
  });

  state.canvas.addEventListener("wheel", (e) => {
    e.preventDefault();
    const rect = state.canvas.getBoundingClientRect();
    const factor = e.deltaY < 0 ? 1.15 : 0.85;
    setZoom(state.zoom * factor, e.clientX - rect.left, e.clientY - rect.top);
    setFollowDrone(false);
  }, { passive: false });

  el.btnZoomIn.addEventListener("click", () => setZoom(state.zoom * 1.25));
  el.btnZoomOut.addEventListener("click", () => setZoom(state.zoom * 0.8));
  el.btnFitView.addEventListener("click", fitView);
  el.btnFollowDrone.addEventListener("click", () => setFollowDrone(!state.followDrone));
  el.btnClearTrail.addEventListener("click", clearTrail);
  el.btnClearLog.addEventListener("click", () => { el.eventLog.innerHTML = ""; });
  el.btnTheme.addEventListener("click", () => applyTheme(isLight() ? "dark" : "light"));
  el.alertDismiss.addEventListener("click", () => el.alertBanner.classList.add("hidden"));

  // Keyboard shortcuts
  window.addEventListener("keydown", (e) => {
    if (e.target instanceof HTMLInputElement) return;
    switch (e.key) {
      case "+": case "=": setZoom(state.zoom * 1.25); break;
      case "-": case "_": setZoom(state.zoom * 0.8); break;
      case "f": case "F": fitView(); break;
      case "c": case "C": setFollowDrone(!state.followDrone); break;
      case "t": case "T": clearTrail(); break;
      case "l": case "L": applyTheme(isLight() ? "dark" : "light"); break;
      default: return;
    }
    e.preventDefault();
  });
}

function clearTrail() {
  state.trail = [];
  logEvent("Breadcrumb trail cleared", "info");
}

function setFollowDrone(active) {
  state.followDrone = active;
  el.btnFollowDrone.classList.toggle("active", active);
}

function fitView() {
  const keys = Object.keys(state.cells);
  let minX = state.dronePose.x - 1;
  let maxX = state.dronePose.x + 1;
  let minY = state.dronePose.y - 1;
  let maxY = state.dronePose.y + 1;

  for (const key of keys) {
    const [i, j] = key.split(",").map(Number);
    minX = Math.min(minX, (i - 0.5) * state.cellSize);
    maxX = Math.max(maxX, (i + 0.5) * state.cellSize);
    minY = Math.min(minY, (j - 0.5) * state.cellSize);
    maxY = Math.max(maxY, (j + 0.5) * state.cellSize);
  }

  const pad = 2.0; // meters
  minX -= pad; maxX += pad;
  minY -= pad; maxY += pad;

  const w = state.canvas.width;
  const h = state.canvas.height;

  state.zoom = Math.max(15, Math.min(180, Math.min(w / (maxX - minX), h / (maxY - minY))));
  state.panX = w / 2 - ((minX + maxX) / 2) * state.zoom;
  state.panY = h / 2 + ((minY + maxY) / 2) * state.zoom;

  el.mapScaleInfo.textContent = `Scale: ${Math.round(state.zoom)} px/m`;
  setFollowDrone(false);
}

// --- Initialization ---
window.addEventListener("DOMContentLoaded", () => {
  bindDom();
  initTheme();

  state.canvas = el.mapCanvas;
  state.ctx = state.canvas.getContext("2d");

  resizeCanvas();
  setupEvents();
  logEvent("Ground control station ready", "good");
  initWebSocket();
  startHealthTicker();
  requestAnimationFrame(renderMap);
});
