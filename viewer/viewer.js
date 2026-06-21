/* ── Three.js scene setup ───────────────────────────────────────────────── */
const canvas   = document.getElementById("threejs-canvas");
const panel    = document.getElementById("panel-3d");

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setClearColor(0x0e0e0e);

const scene  = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(45, 1, 0.001, 100);
camera.position.set(0.5, -0.9, 1.0);
camera.lookAt(0.5, 0, 0.15);

const controls = new THREE.OrbitControls(camera, canvas);
controls.target.set(0.5, 0, 0.15);
controls.enableDamping = true;
controls.dampingFactor = 0.08;

scene.add(new THREE.AmbientLight(0xffffff, 0.5));
const dlight = new THREE.DirectionalLight(0xffffff, 0.9);
dlight.position.set(0.5, -1, 2);
scene.add(dlight);
scene.add(new THREE.AxesHelper(0.1));

/* ── Workspace visualization ────────────────────────────────────────────── */
let wsGroup = null;
let lastWorkspaceJson = null;

function buildWorkspaceViz(ws) {
  if (wsGroup) { scene.remove(wsGroup); wsGroup = null; }
  if (!ws) return;

  wsGroup = new THREE.Group();

  const o  = new THREE.Vector3(...ws.origin);
  const xa = new THREE.Vector3(...ws.x_axis);
  const ya = new THREE.Vector3(...ws.y_axis);
  const xe = ws.x_extent;
  const ye = ws.y_extent;

  const p00 = o.clone();
  const p10 = o.clone().addScaledVector(xa, xe);
  const p11 = o.clone().addScaledVector(xa, xe).addScaledVector(ya, ye);
  const p01 = o.clone().addScaledVector(ya, ye);

  const borderPts = [p00, p10, p11, p01, p00];
  const borderGeo = new THREE.BufferGeometry().setFromPoints(borderPts);
  wsGroup.add(new THREE.Line(borderGeo, new THREE.LineBasicMaterial({ color: 0x444444 })));

  const gridMat = new THREE.LineBasicMaterial({ color: 0x2a2a2a });
  const DIVS = 5;
  for (let i = 1; i < DIVS; i++) {
    const t = i / DIVS;
    const geoX = new THREE.BufferGeometry().setFromPoints([
      o.clone().addScaledVector(xa, xe * t),
      o.clone().addScaledVector(xa, xe * t).addScaledVector(ya, ye),
    ]);
    const geoY = new THREE.BufferGeometry().setFromPoints([
      o.clone().addScaledVector(ya, ye * t),
      o.clone().addScaledVector(xa, xe).addScaledVector(ya, ye * t),
    ]);
    wsGroup.add(new THREE.Line(geoX, gridMat));
    wsGroup.add(new THREE.Line(geoY, gridMat));
  }

  wsGroup.add(makeArrow(o, xa, xe * 0.15, 0xff3333));
  wsGroup.add(makeArrow(o, ya, ye * 0.15, 0x33cc55));

  scene.add(wsGroup);

  const za     = new THREE.Vector3(...ws.z_axis);
  const center = o.clone().addScaledVector(xa, xe / 2).addScaledVector(ya, ye / 2);
  const diag   = Math.sqrt(xe * xe + ye * ye);
  controls.target.copy(center);
  camera.position.copy(center).addScaledVector(za, diag * 1.5);
  camera.up.copy(ya);
  camera.lookAt(center);
  controls.update();
}

function makeArrow(origin, dir, length, color) {
  const tip = origin.clone().addScaledVector(dir, length);
  const geo = new THREE.BufferGeometry().setFromPoints([origin, tip]);
  return new THREE.Line(geo, new THREE.LineBasicMaterial({ color }));
}

/* ── Path preview (set after Capture) ──────────────────────────────────── */
let pathGroup = null;

function buildPathViz(strokes) {
  if (pathGroup) { scene.remove(pathGroup); pathGroup = null; }
  if (!strokes || strokes.length === 0) return;

  pathGroup = new THREE.Group();
  const drawMat   = new THREE.LineBasicMaterial({ color: 0x00e5a0 }); // green — draw
  const travelMat = new THREE.LineBasicMaterial({ color: 0x444466 }); // gray  — travel

  let prevEnd = null;

  for (const stroke of strokes) {
    if (stroke.length === 0) continue;

    // Travel segment from previous stroke end to this stroke start
    if (prevEnd) {
      const startPt = new THREE.Vector3(stroke[0][0], stroke[0][1], stroke[0][2]);
      const tGeo = new THREE.BufferGeometry().setFromPoints([prevEnd, startPt]);
      pathGroup.add(new THREE.Line(tGeo, travelMat));
    }

    // Draw segment
    const pts = stroke.map(p => new THREE.Vector3(p[0], p[1], p[2]));
    if (pts.length > 1) {
      const dGeo = new THREE.BufferGeometry().setFromPoints(pts);
      pathGroup.add(new THREE.Line(dGeo, drawMat));
    }

    prevEnd = new THREE.Vector3(stroke[stroke.length - 1][0],
                                stroke[stroke.length - 1][1],
                                stroke[stroke.length - 1][2]);
  }

  scene.add(pathGroup);
}

/* ── End-effector sphere ────────────────────────────────────────────────── */
const eeMat    = new THREE.MeshStandardMaterial({ color: 0x4488ff, emissive: 0x223366, roughness: 0.4 });
const eeSphere = new THREE.Mesh(new THREE.SphereGeometry(0.014, 20, 20), eeMat);
scene.add(eeSphere);

/* ── Resize handling ────────────────────────────────────────────────────── */
function resizeRenderer() {
  const w = panel.clientWidth;
  const h = panel.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resizeRenderer).observe(panel);
resizeRenderer();

/* ── Render loop ────────────────────────────────────────────────────────── */
(function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
})();

/* ── WebSocket ──────────────────────────────────────────────────────────── */
let ws = null;

function connectWS() {
  const protocol = location.protocol === "https:" ? "wss:" : "ws:";
  ws = new WebSocket(`${protocol}//${location.host}/ws`);

  ws.onopen  = () => setHeaderStatus("ws", true, "WS connected");
  ws.onclose = () => { setHeaderStatus("ws", false, "WS disconnected"); setTimeout(connectWS, 2000); };
  ws.onerror = () => setHeaderStatus("ws", false, "WS error");

  ws.onmessage = (ev) => {
    const data = JSON.parse(ev.data);

    if (data.type === "init") {
      handleInit(data);
    } else if (data.type === "state") {
      updateScene(data);
      updateFooter(data);
      updateSetupPanel(data);
    } else if (data.type === "connection_result") {
      setHeaderStatus("robot", data.success, data.message);
    } else if (data.type === "workspace_status") {
      handleWorkspaceStatus(data);
    } else if (data.type === "capture_result") {
      handleCaptureResult(data);
    } else if (data.type === "execution_update") {
      handleExecutionUpdate(data);
    }
  };
}

connectWS();

/* ── Init ───────────────────────────────────────────────────────────────── */
function handleInit(data) {
  if (data.last_ip) {
    document.getElementById("ip-input").value = data.last_ip;
  }
  if (data.workspace) {
    applyWorkspace(data.workspace);
  }
}

/* ── Workspace status ───────────────────────────────────────────────────── */
function handleWorkspaceStatus(data) {
  if (data.loaded && data.workspace) {
    applyWorkspace(data.workspace);
    showLoadedBanner(data.workspace);
  } else {
    showSetupSteps();
  }
  showOverlay(true);
}

function applyWorkspace(ws) {
  const wsJson = JSON.stringify(ws);
  if (wsJson !== lastWorkspaceJson) {
    lastWorkspaceJson = wsJson;
    buildWorkspaceViz(ws);
  }
}

/* ── Capture result ─────────────────────────────────────────────────────── */
function handleCaptureResult(data) {
  if (!data.success) {
    setHeaderStatus("robot", false, "Capture failed: " + (data.error || "unknown error"));
    return;
  }

  document.getElementById("val-strokes").textContent = data.stroke_count;

  if (data.strokes && data.strokes.length > 0) {
    buildPathViz(data.strokes);
    setButtonsForPhase("captured");
  } else {
    buildPathViz(null);
    setButtonsForPhase("previewing");
  }

  setHeaderStatus("robot", true,
    `Captured: ${data.stroke_count} strokes, ${data.point_count} points`);
}

/* ── Execution update ───────────────────────────────────────────────────── */
function handleExecutionUpdate(data) {
  setButtonsForPhase(data.phase);
  setProgress(data.progress);
}

/* ── Scene update (50ms state broadcast) ───────────────────────────────── */
function updateScene(data) {
  if (data.workspace !== undefined) {
    const wsJson = JSON.stringify(data.workspace);
    if (wsJson !== lastWorkspaceJson) {
      lastWorkspaceJson = wsJson;
      buildWorkspaceViz(data.workspace);
    }
  }

  if (data.ee && data.ee.length >= 3) {
    eeSphere.position.set(data.ee[0], data.ee[1], data.ee[2]);
  }

  eeMat.color.setHex(data.executing ? 0xff6600 : 0x4488ff);
  eeMat.emissive.setHex(data.executing ? 0x331400 : 0x223366);
}

/* ── Footer update ──────────────────────────────────────────────────────── */
function updateFooter(data) {
  document.getElementById("val-phase").textContent   = data.phase || "idle";
  document.getElementById("val-strokes").textContent = data.stroke_count ?? 0;

  setButtonsForPhase(data.phase);
  setProgress(data.progress || 0);
}

function setButtonsForPhase(phase) {
  const btnCapture = document.getElementById("btn-capture");
  const btnRun     = document.getElementById("btn-run");
  const btnCancel  = document.getElementById("btn-cancel");
  const progWrap   = document.getElementById("progress-bar-wrap");

  btnCapture.disabled = !(phase === "previewing" || phase === "captured" || phase === "done");
  btnRun.disabled     = !(phase === "captured" || phase === "done");
  btnCancel.classList.toggle("hidden", phase !== "executing");
  progWrap.classList.toggle("hidden", phase !== "executing");
}

function setProgress(value) {
  document.getElementById("progress-bar").style.width = (value * 100).toFixed(1) + "%";
}

/* ── Setup overlay ──────────────────────────────────────────────────────── */
const overlay = document.getElementById("ws-overlay");

function showOverlay(visible) {
  overlay.classList.toggle("hidden", !visible);
}

function showLoadedBanner(ws) {
  document.getElementById("ws-loaded-banner").classList.remove("hidden");
  document.getElementById("ws-steps").classList.add("hidden");
  document.getElementById("ws-loaded-size").textContent =
    `${ws.x_extent.toFixed(3)} m × ${ws.y_extent.toFixed(3)} m`;
  const o = ws.origin;
  document.getElementById("ws-loaded-origin").textContent =
    `Origin (${o[0].toFixed(3)}, ${o[1].toFixed(3)}, ${o[2].toFixed(3)})`;
}

function showSetupSteps() {
  document.getElementById("ws-loaded-banner").classList.add("hidden");
  document.getElementById("ws-steps").classList.remove("hidden");
  resetSetupStepUI();
}

function resetSetupStepUI() {
  setStepEnabled("ws-step-2", false);
  setStepEnabled("ws-step-3", false);
  ["p0", "px", "py"].forEach(n => {
    document.getElementById(`coords-${n}`).textContent = "not recorded";
    document.getElementById(`coords-${n}`).classList.remove("recorded");
  });
  document.getElementById("btn-confirm-ws").disabled = true;
  document.getElementById("ws-preview-size").textContent = "";
  setFreedriveUI(false);
}

function setStepEnabled(id, enabled) {
  document.getElementById(id).classList.toggle("disabled", !enabled);
}

function setFreedriveUI(active) {
  document.getElementById("btn-start-freedrive").classList.toggle("hidden", active);
  document.getElementById("btn-end-freedrive").classList.toggle("hidden", !active);
  document.getElementById("freedrive-indicator").classList.toggle("hidden", !active);
  setStepEnabled("ws-step-2", active);
}

/* ── Setup panel live updates ───────────────────────────────────────────── */
function updateSetupPanel(data) {
  if (data.freedrive && data.ee && data.ee.length >= 3) {
    document.getElementById("tcp-x").textContent = data.ee[0].toFixed(4);
    document.getElementById("tcp-y").textContent = data.ee[1].toFixed(4);
    document.getElementById("tcp-z").textContent = data.ee[2].toFixed(4);
    setFreedriveUI(true);
  } else if (!data.freedrive) {
    setFreedriveUI(false);
  }

  if (data.ws_points) {
    let allRecorded = true;
    ["p0", "px", "py"].forEach(name => {
      const pts = data.ws_points[name];
      const el  = document.getElementById(`coords-${name}`);
      if (pts) {
        el.textContent = `(${pts[0].toFixed(3)}, ${pts[1].toFixed(3)}, ${pts[2].toFixed(3)})`;
        el.classList.add("recorded");
      } else {
        allRecorded = false;
      }
    });

    if (allRecorded) {
      setStepEnabled("ws-step-3", true);
      document.getElementById("btn-confirm-ws").disabled = false;
    }
  }

  if (data.workspace) {
    const wsJson = JSON.stringify(data.workspace);
    if (wsJson !== lastWorkspaceJson) {
      applyWorkspace(data.workspace);
    }
  }
}

/* ── Header status ──────────────────────────────────────────────────────── */
function setHeaderStatus(type, ok, message) {
  const dot   = document.getElementById("status-dot");
  const label = document.getElementById("header-status-label");
  const msg   = document.getElementById("connection-msg");

  if (type === "robot") {
    dot.className   = ok ? "connected" : "error";
    label.textContent = ok ? "Connected" : "Disconnected";
  }
  msg.textContent = message;
}

/* ── Workspace setup buttons ────────────────────────────────────────────── */
function sendWS(msg) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(msg));
}

document.getElementById("btn-start-freedrive").addEventListener("click", () => {
  sendWS({ type: "start_freedrive" });
});

document.getElementById("btn-end-freedrive").addEventListener("click", () => {
  sendWS({ type: "end_freedrive" });
});

["btn-record-p0", "btn-record-px", "btn-record-py"].forEach(id => {
  const btn = document.getElementById(id);
  btn.addEventListener("click", () => {
    sendWS({ type: "record_point", name: btn.dataset.name });
  });
});

document.getElementById("btn-use-workspace").addEventListener("click", () => {
  sendWS({ type: "use_workspace" });
  showOverlay(false);
});

document.getElementById("btn-confirm-ws").addEventListener("click", () => {
  sendWS({ type: "confirm_workspace" });
  showOverlay(false);
});

document.getElementById("btn-ws-redefine").addEventListener("click", () => {
  sendWS({ type: "reset_workspace" });
  lastWorkspaceJson = null;
  buildWorkspaceViz(null);
  buildPathViz(null);
  showSetupSteps();
});

/* ── Connection buttons ─────────────────────────────────────────────────── */
document.getElementById("btn-connect").addEventListener("click", () => {
  const ip = document.getElementById("ip-input").value.trim();
  if (!ip || !ws || ws.readyState !== WebSocket.OPEN) return;
  ws.send(JSON.stringify({ type: "connect", ip }));
  setHeaderStatus("robot", false, `Connecting to ${ip}…`);
});

document.getElementById("btn-disconnect").addEventListener("click", () => {
  sendWS({ type: "disconnect" });
  showOverlay(false);
});

document.getElementById("ip-input").addEventListener("keydown", (e) => {
  if (e.key === "Enter") document.getElementById("btn-connect").click();
});

/* ── Test mode: simulated workspace (no robot) ─────────────────────────── */
document.getElementById("btn-simulate").addEventListener("click", () => {
  sendWS({ type: "simulate_workspace" });
  showOverlay(false);
  setHeaderStatus("robot", false, "Test mode: simulated workspace (no robot). Press Capture.");
});

/* ── Capture / Run / Cancel buttons ────────────────────────────────────── */
document.getElementById("btn-capture").addEventListener("click", () => {
  sendWS({ type: "capture" });
  setHeaderStatus("robot", true, "Capturing drawing…");
});

document.getElementById("btn-run").addEventListener("click", () => {
  sendWS({ type: "run" });
  setButtonsForPhase("executing");
  setProgress(0);
  document.getElementById("val-phase").textContent = "executing";
});

document.getElementById("btn-cancel").addEventListener("click", () => {
  sendWS({ type: "cancel" });
  setHeaderStatus("robot", true, "Cancelling…");
});

document.getElementById("camera-select").addEventListener("change", (e) => {
  sendWS({ type: "select_camera", index: parseInt(e.target.value) });
});

async function loadCameras() {
  const sel = document.getElementById("camera-select");
  sel.innerHTML = "";
  try {
    const res  = await fetch("/cameras");
    const idxs = await res.json();
    if (idxs.length === 0) throw new Error("no cameras");
    idxs.sort((a, b) => a - b);
    idxs.forEach(i => {
      const opt = document.createElement("option");
      opt.value = i;
      opt.textContent = `Camera ${i}`;
      sel.appendChild(opt);
    });
    sel.value = idxs[0];
  } catch {
    const opt = document.createElement("option");
    opt.value = 0;
    opt.textContent = "Camera 0";
    sel.appendChild(opt);
    sel.value = 0;
  }
}

loadCameras();
