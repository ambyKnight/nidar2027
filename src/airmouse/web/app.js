/**
 * AirMouse GCS Web Dashboard
 * Connects to rosbridge_server (port 9090) and renders the live grid, drone pose, and exploration state.
 */

// --- Global State ---
const state = {
  ws: null,
  connected: false,
  reconnectTimer: null,
  
  // Drone & Mission
  dronePose: { x: 0.0, y: 0.0, z: 0.0, yaw: 0.0 },
  currentCell: [0, 0],
  cells: {},           // "i,j" -> { "+x": "...", "-x": "...", "+y": "...", "-y": "...", "seen": true }
  visitedCells: new Set(),
  queue: [],           // [[i, j], ...]
  trail: [],           // [{x, y}, ...]
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
  
  // Canvas & Viewport
  canvas: null,
  ctx: null,
  panX: 0,
  panY: 0,
  zoom: 55,            // pixels per meter
  followDrone: true,
  isDragging: false,
  dragStartX: 0,
  dragStartY: 0,
  cellSize: 1.0
};

// --- DOM Elements ---
const el = {
  connPill: document.getElementById("connection-pill"),
  connLabel: document.getElementById("connection-label"),
  phaseBadge: document.getElementById("phase-badge"),
  flightTimer: document.getElementById("flight-timer"),
  
  valPosX: document.getElementById("val-pos-x"),
  valPosY: document.getElementById("val-pos-y"),
  valPosZ: document.getElementById("val-pos-z"),
  valYaw: document.getElementById("val-yaw"),
  valCurrentCell: document.getElementById("val-current-cell"),
  
  valVisited: document.getElementById("val-visited"),
  valMapped: document.getElementById("val-mapped"),
  valQueue: document.getElementById("val-queue"),
  valCameraSeen: document.getElementById("val-camera-seen"),
  valGoingHome: document.getElementById("val-going-home"),
  valGuardEvents: document.getElementById("val-guard-events"),
  
  tofFwd: document.getElementById("tof-fwd"),
  tofBwd: document.getElementById("tof-bwd"),
  tofLeft: document.getElementById("tof-left"),
  tofRight: document.getElementById("tof-right"),
  tofFwdBox: document.getElementById("tof-fwd-box"),
  tofBwdBox: document.getElementById("tof-bwd-box"),
  tofLeftBox: document.getElementById("tof-left-box"),
  tofRightBox: document.getElementById("tof-right-box"),
  
  mapCanvas: document.getElementById("map-canvas"),
  btnZoomIn: document.getElementById("btn-zoom-in"),
  btnZoomOut: document.getElementById("btn-zoom-out"),
  btnFitView: document.getElementById("btn-fit-view"),
  btnFollowDrone: document.getElementById("btn-follow-drone"),
  mapGridCount: document.getElementById("map-grid-count"),
  mapScaleInfo: document.getElementById("map-scale-info")
};

// --- WebSocket & Rosbridge Communication ---
function initWebSocket() {
  const host = window.location.hostname || "localhost";
  const wsUrl = `ws://${host}:9090`;
  
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
    if (state.reconnectTimer) {
      clearTimeout(state.reconnectTimer);
      state.reconnectTimer = null;
    }
    
    // Subscribe to topics
    subscribeTopic("/airmouse/grid", "std_msgs/msg/String");
    subscribeTopic("/airmouse/explorer", "std_msgs/msg/String");
    subscribeTopic("/mavros/local_position/pose", "geometry_msgs/msg/PoseStamped");
  };
  
  state.ws.onmessage = (event) => {
    try {
      const data = JSON.parse(event.data);
      handleRosMessage(data);
    } catch (e) {
      console.warn("Failed to parse JSON:", e, event.data);
    }
  };
  
  state.ws.onclose = () => {
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
    const subMsg = {
      op: "subscribe",
      topic: topic,
      type: type
    };
    state.ws.send(JSON.stringify(subMsg));
  }
}

function updateConnectionStatus(connected, label) {
  state.connected = connected;
  el.connLabel.textContent = label;
  if (connected) {
    el.connPill.classList.remove("disconnected");
    el.connPill.classList.add("connected");
  } else {
    el.connPill.classList.remove("connected");
    el.connPill.classList.add("disconnected");
  }
}

// --- ROS Message Handling ---
function handleRosMessage(msg) {
  if (!msg || msg.op !== "publish") return;
  
  const topic = msg.topic;
  
  if (topic === "/airmouse/grid") {
    handleGridMessage(msg.msg);
  } else if (topic === "/airmouse/explorer") {
    handleExplorerMessage(msg.msg);
  } else if (topic === "/mavros/local_position/pose") {
    handlePoseMessage(msg.msg);
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
    
    // Update Phase Badge
    const phase = data.state || "STANDBY";
    el.phaseBadge.textContent = phase;
    el.phaseBadge.className = "badge";
    if (["FLY", "LOOK", "SETTLE", "SPIN"].includes(phase)) {
      el.phaseBadge.classList.add("badge-active");
    } else if (phase === "HOME" || phase === "LANDING") {
      el.phaseBadge.classList.add("badge-home");
    } else if (phase === "ABORTED") {
      el.phaseBadge.classList.add("badge-alert");
    } else {
      el.phaseBadge.classList.add("badge-idle");
    }
    
    // Update Timer
    if (data.elapsed !== undefined) {
      const totalSec = Math.floor(data.elapsed);
      const m = Math.floor(totalSec / 60).toString().padStart(2, "0");
      const s = (totalSec % 60).toString().padStart(2, "0");
      el.flightTimer.textContent = `${m}:${s}`;
    }
    
    // Update Current Cell
    if (Array.isArray(data.current)) {
      state.currentCell = data.current;
      el.valCurrentCell.textContent = `(${data.current[0]}, ${data.current[1]})`;
      state.visitedCells.add(`${data.current[0]},${data.current[1]}`);
    }
    
    // Update Counts
    if (data.visited !== undefined) el.valVisited.textContent = data.visited;
    if (data.mapped !== undefined) el.valMapped.textContent = data.mapped;
    if (Array.isArray(data.queue)) {
      state.queue = data.queue;
      el.valQueue.textContent = data.queue.length;
    }
    if (data.camera_seen !== undefined) {
      el.valCameraSeen.textContent = `${data.camera_seen} cells`;
    }
    if (data.going_home !== undefined) {
      el.valGoingHome.textContent = data.going_home ? "YES" : "NO";
      el.valGoingHome.className = `value ${data.going_home ? "status-on" : "status-off"}`;
    }
    if (data.guard_events !== undefined) {
      el.valGuardEvents.textContent = data.guard_events;
    }
    
    // Update ToF Sensors
    if (data.tof) {
      updateTofDisplay("fwd", data.tof.forward ?? data.tof.fwd, el.tofFwd, el.tofFwdBox);
      updateTofDisplay("bwd", data.tof.backward ?? data.tof.bwd, el.tofBwd, el.tofBwdBox);
      updateTofDisplay("left", data.tof.left, el.tofLeft, el.tofLeftBox);
      updateTofDisplay("right", data.tof.right, el.tofRight, el.tofRightBox);
    }
  } catch (err) {
    console.warn("Error parsing explorer state:", err);
  }
}

function updateTofDisplay(side, val, valEl, boxEl) {
  if (val === undefined || val === null || val === Infinity) {
    valEl.textContent = "--";
    boxEl.className = "tof-box";
    return;
  }
  const dist = typeof val === "number" ? val : parseFloat(val);
  valEl.textContent = `${dist.toFixed(2)} m`;
  boxEl.className = "tof-box";
  if (dist < 0.25) {
    boxEl.classList.add("danger");
  } else if (dist < 0.45) {
    boxEl.classList.add("warn");
  } else {
    boxEl.classList.add("safe");
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
  
  // UI Readouts
  el.valPosX.innerHTML = `${p.x.toFixed(2)} <span class="unit">m</span>`;
  el.valPosY.innerHTML = `${p.y.toFixed(2)} <span class="unit">m</span>`;
  el.valPosZ.innerHTML = `${p.z.toFixed(2)} <span class="unit">m</span>`;
  const deg = (state.dronePose.yaw * 180 / Math.PI).toFixed(1);
  el.valYaw.innerHTML = `${deg}<span class="unit">°</span>`;
  
  // Breadcrumb Trail
  const last = state.trail[state.trail.length - 1];
  if (!last || Math.hypot(p.x - last.x, p.y - last.y) > 0.15) {
    state.trail.push({ x: p.x, y: p.y });
    if (state.trail.length > 500) state.trail.shift();
  }
}

// --- Coordinate Transformations ---
function toScreenX(worldX) {
  return state.panX + worldX * state.zoom;
}

function toScreenY(worldY) {
  // In Map frame: +y is up/left; on Canvas: +y is downwards
  return state.panY - worldY * state.zoom;
}

function toWorldX(screenX) {
  return (screenX - state.panX) / state.zoom;
}

function toWorldY(screenY) {
  return (state.panY - screenY) / state.zoom;
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
  
  // Clear Background
  ctx.fillStyle = "#0a0d14";
  ctx.fillRect(0, 0, w, h);
  
  drawGridLines(ctx, w, h);
  drawOrigin(ctx);
  drawCells(ctx);
  drawPlannedRoute(ctx);
  drawTrail(ctx);
  drawDrone(ctx);
  
  requestAnimationFrame(renderMap);
}

function drawGridLines(ctx, w, h) {
  const step = state.zoom; // 1 meter per major step
  ctx.lineWidth = 1;
  
  // Visible coordinate bounds in meters
  const minX = Math.floor(toWorldX(0)) - 1;
  const maxX = Math.ceil(toWorldX(w)) + 1;
  const minY = Math.floor(toWorldY(h)) - 1;
  const maxY = Math.ceil(toWorldY(0)) + 1;
  
  ctx.strokeStyle = "rgba(255, 255, 255, 0.04)";
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
  
  ctx.strokeStyle = "rgba(234, 179, 8, 0.4)";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.arc(ox, oy, 6, 0, Math.PI * 2);
  ctx.moveTo(ox - 10, oy);
  ctx.lineTo(ox + 10, oy);
  ctx.moveTo(ox, oy - 10);
  ctx.lineTo(ox, oy + 10);
  ctx.stroke();
  
  ctx.fillStyle = "rgba(234, 179, 8, 0.7)";
  ctx.font = "9px monospace";
  ctx.fillText("TAKEOFF (0,0)", ox + 10, oy + 12);
}

function drawCells(ctx) {
  const cell = state.cellSize;
  
  for (const [key, sides] of Object.entries(state.cells)) {
    const [iStr, jStr] = key.split(",");
    const i = parseInt(iStr, 10);
    const j = parseInt(jStr, 10);
    
    // Bounds of cell (i, j) in world coordinates
    const xl = (i - 0.5) * cell;
    const xh = (i + 0.5) * cell;
    const yl = (j - 0.5) * cell;
    const yh = (j + 0.5) * cell;
    
    const sxl = toScreenX(xl);
    const sxh = toScreenX(xh);
    const syh = toScreenY(yh);
    const syl = toScreenY(yl);
    
    // Cell Fill
    const isVisited = state.visitedCells.has(key);
    ctx.fillStyle = isVisited ? "rgba(34, 197, 94, 0.14)" : "rgba(148, 163, 184, 0.05)";
    ctx.fillRect(sxl, syh, sxh - sxl, syl - syh);
    
    // Draw 4 Sides
    // +x side (x = xh, from yl to yh)
    drawSide(ctx, sxh, syl, sxh, syh, sides["+x"]);
    // -x side (x = xl, from yl to yh)
    drawSide(ctx, sxl, syl, sxl, syh, sides["-x"]);
    // +y side (y = yh, from xl to xh)
    drawSide(ctx, sxl, syh, sxh, syh, sides["+y"]);
    // -y side (y = yl, from xl to xh)
    drawSide(ctx, sxl, syl, sxh, syl, sides["-y"]);
  }
}

function drawSide(ctx, x1, y1, x2, y2, classification) {
  ctx.beginPath();
  if (classification === "wall") {
    ctx.strokeStyle = "#ef4444";
    ctx.lineWidth = 3.5;
    ctx.setLineDash([]);
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
  } else if (classification === "open") {
    ctx.strokeStyle = "#22c55e";
    ctx.lineWidth = 2.0;
    ctx.setLineDash([4, 4]);
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
    ctx.setLineDash([]);
  } else {
    // unknown
    ctx.strokeStyle = "rgba(71, 85, 105, 0.4)";
    ctx.lineWidth = 1.0;
    ctx.setLineDash([2, 3]);
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
    ctx.setLineDash([]);
  }
}

function drawPlannedRoute(ctx) {
  if (!state.queue || state.queue.length === 0) return;
  
  ctx.strokeStyle = "rgba(0, 229, 255, 0.7)";
  ctx.lineWidth = 2;
  ctx.setLineDash([6, 4]);
  ctx.beginPath();
  
  // Start from current drone position
  ctx.moveTo(toScreenX(state.dronePose.x), toScreenY(state.dronePose.y));
  
  for (const wp of state.queue) {
    const wx = wp[0] * state.cellSize;
    const wy = wp[1] * state.cellSize;
    ctx.lineTo(toScreenX(wx), toScreenY(wy));
  }
  ctx.stroke();
  ctx.setLineDash([]);
  
  // Waypoint targets
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
  
  ctx.strokeStyle = "rgba(0, 229, 255, 0.25)";
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
  
  // Center Ring
  ctx.fillStyle = "#ffffff";
  ctx.beginPath();
  ctx.arc(0, 0, 2.5, 0, Math.PI * 2);
  ctx.fill();
  
  ctx.restore();
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

function setupEvents() {
  window.addEventListener("resize", resizeCanvas);
  
  // Mouse Drag to Pan
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
  
  window.addEventListener("mouseup", () => {
    state.isDragging = false;
  });
  
  // Wheel to Zoom
  state.canvas.addEventListener("wheel", (e) => {
    e.preventDefault();
    const zoomFactor = e.deltaY < 0 ? 1.15 : 0.85;
    const mouseX = e.clientX - state.canvas.getBoundingClientRect().left;
    const mouseY = e.clientY - state.canvas.getBoundingClientRect().top;
    
    // Zoom anchored to mouse
    const worldX = toWorldX(mouseX);
    const worldY = toWorldY(mouseY);
    
    state.zoom = Math.max(15, Math.min(250, state.zoom * zoomFactor));
    state.panX = mouseX - worldX * state.zoom;
    state.panY = mouseY + worldY * state.zoom;
    
    el.mapScaleInfo.textContent = `Scale: ${Math.round(state.zoom)} px/m`;
    setFollowDrone(false);
  });
  
  // Buttons
  el.btnZoomIn.addEventListener("click", () => {
    state.zoom = Math.min(250, state.zoom * 1.25);
    el.mapScaleInfo.textContent = `Scale: ${Math.round(state.zoom)} px/m`;
  });
  
  el.btnZoomOut.addEventListener("click", () => {
    state.zoom = Math.max(15, state.zoom * 0.8);
    el.mapScaleInfo.textContent = `Scale: ${Math.round(state.zoom)} px/m`;
  });
  
  el.btnFitView.addEventListener("click", fitView);
  
  el.btnFollowDrone.addEventListener("click", () => {
    setFollowDrone(!state.followDrone);
  });
}

function setFollowDrone(active) {
  state.followDrone = active;
  if (active) {
    el.btnFollowDrone.classList.add("active");
  } else {
    el.btnFollowDrone.classList.remove("active");
  }
}

function fitView() {
  const keys = Object.keys(state.cells);
  let minX = state.dronePose.x - 1;
  let maxX = state.dronePose.x + 1;
  let minY = state.dronePose.y - 1;
  let maxY = state.dronePose.y + 1;
  
  if (keys.length > 0) {
    for (const key of keys) {
      const [i, j] = key.split(",").map(Number);
      minX = Math.min(minX, (i - 0.5) * state.cellSize);
      maxX = Math.max(maxX, (i + 0.5) * state.cellSize);
      minY = Math.min(minY, (j - 0.5) * state.cellSize);
      maxY = Math.max(maxY, (j + 0.5) * state.cellSize);
    }
  }
  
  const pad = 2.0; // padding in meters
  minX -= pad; maxX += pad;
  minY -= pad; maxY += pad;
  
  const spanX = maxX - minX;
  const spanY = maxY - minY;
  
  const w = state.canvas.width;
  const h = state.canvas.height;
  
  state.zoom = Math.max(15, Math.min(180, Math.min(w / spanX, h / spanY)));
  state.panX = w / 2 - ((minX + maxX) / 2) * state.zoom;
  state.panY = h / 2 + ((minY + maxY) / 2) * state.zoom;
  
  el.mapScaleInfo.textContent = `Scale: ${Math.round(state.zoom)} px/m`;
  setFollowDrone(false);
}

// --- Initialization ---
window.addEventListener("DOMContentLoaded", () => {
  state.canvas = el.mapCanvas;
  state.ctx = state.canvas.getContext("2d");
  
  resizeCanvas();
  setupEvents();
  initWebSocket();
  requestAnimationFrame(renderMap);
});
