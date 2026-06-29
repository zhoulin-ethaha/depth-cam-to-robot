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
let pathGroup  = null;
let orderGroup = null;
let lastStrokes = null;
let pathMode   = "path";   // "path" | "order"
let orderSize  = 1.0;      // multiplier for order numbers + start/end dots

function buildPathViz(strokes) {
  if (pathGroup) { scene.remove(pathGroup); pathGroup = null; }
  lastStrokes = strokes && strokes.length ? strokes : null;

  if (lastStrokes) {
    pathGroup = new THREE.Group();
    const drawMat   = new THREE.LineBasicMaterial({ color: 0x00e5a0 }); // green — draw
    const travelMat = new THREE.LineBasicMaterial({ color: 0x444466 }); // gray  — travel

    let prevEnd = null;
    for (const stroke of lastStrokes) {
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

  buildOrderViz(lastStrokes);
  applyPathMode();
}

/* Order overlay: a numbered label per stroke + green start / red end markers,
   so the drawing order and each stroke's direction are visible. */
function buildOrderViz(strokes) {
  if (orderGroup) { scene.remove(orderGroup); orderGroup = null; }
  if (!strokes || strokes.length === 0) return;

  orderGroup = new THREE.Group();
  const startMat = new THREE.MeshBasicMaterial({ color: 0x00e5a0 }); // green — start
  const endMat   = new THREE.MeshBasicMaterial({ color: 0xff4444 }); // red   — end
  const dot = new THREE.SphereGeometry(0.005 * orderSize, 12, 12);

  strokes.forEach((stroke, i) => {
    if (stroke.length === 0) return;
    const s = stroke[0];
    const e = stroke[stroke.length - 1];

    const startDot = new THREE.Mesh(dot, startMat);
    startDot.position.set(s[0], s[1], s[2]);
    orderGroup.add(startDot);

    const endDot = new THREE.Mesh(dot, endMat);
    endDot.position.set(e[0], e[1], e[2]);
    orderGroup.add(endDot);

    // Number the stroke, placed just above its start point.
    const label = makeLabelSprite(String(i + 1), 0.03 * orderSize);
    label.position.set(s[0], s[1], s[2] + 0.012 * orderSize);
    orderGroup.add(label);
  });

  scene.add(orderGroup);
}

function makeLabelSprite(text, scale) {
  const size = 128;
  const canvas = document.createElement("canvas");
  canvas.width = canvas.height = size;
  const ctx = canvas.getContext("2d");
  ctx.fillStyle = "rgba(10, 12, 18, 0.78)";
  ctx.beginPath();
  ctx.arc(size / 2, size / 2, size / 2 - 6, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = "#00e5a0";
  ctx.lineWidth = 6;
  ctx.stroke();
  ctx.fillStyle = "#ffffff";
  ctx.font = "bold 70px sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(text, size / 2, size / 2 + 4);

  const tex = new THREE.CanvasTexture(canvas);
  const mat = new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true });
  const sp = new THREE.Sprite(mat);
  sp.scale.set(scale, scale, 1);
  return sp;
}

function applyPathMode() {
  if (orderGroup) orderGroup.visible = (pathMode === "order");
  const legend = document.getElementById("path-legend");
  if (legend) legend.classList.toggle("hidden", pathMode !== "order");
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
    } else if (data.type === "still") {
      handleStill(data);
    } else if (data.type === "preview") {
      handlePreview(data);
    } else if (data.type === "capture_result") {
      handleCaptureResult(data);
    } else if (data.type === "reference_status") {
      handleReferenceStatus(data);
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
    setHeaderStatus("robot", true,
      `Path ready: ${data.stroke_count} strokes, ${data.point_count} points`);
  } else {
    buildPathViz(null);
    setButtonsForPhase("editing");
    setHeaderStatus("robot", false,
      "No grooves found — lower Groove depth / adjust detection and try again.");
  }
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
let robotConnected = false;

function updateFooter(data) {
  if (typeof data.robot_connected === "boolean") robotConnected = data.robot_connected;

  document.getElementById("val-phase").textContent   = data.phase || "idle";
  document.getElementById("val-strokes").textContent = data.stroke_count ?? 0;

  setButtonsForPhase(data.phase);
  syncEditUI(data.phase);
  setProgress(data.progress || 0);
}

function setButtonsForPhase(phase) {
  const cap      = document.getElementById("btn-capture-image");
  const gen      = document.getElementById("btn-generate");
  const retake   = document.getElementById("btn-retake");
  const btnRun   = document.getElementById("btn-run");
  const btnCancel = document.getElementById("btn-cancel");
  const progWrap = document.getElementById("progress-bar-wrap");

  const inEdit     = phase === "editing" || phase === "captured" || phase === "done";
  const executing  = phase === "executing";

  cap.classList.toggle("hidden", inEdit || executing);
  cap.disabled = phase !== "previewing";

  retake.classList.toggle("hidden", !(inEdit && !executing));
  gen.classList.toggle("hidden", !(inEdit && !executing));

  btnRun.disabled = !((phase === "captured" || phase === "done") && !executing);
  btnCancel.classList.toggle("hidden", !executing);
  progWrap.classList.toggle("hidden", !executing);
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
  stillLoaded = false;
  buildWorkspaceViz(null);
  buildPathViz(null);
  showLive();
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
  stillLoaded = false;
  buildPathViz(null);
  showLive();
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

/* ── Capture image / Edit / Generate / Retake ──────────────────────────── */
const leftPanel      = document.getElementById("panel-camera-raw");
const stillContainer = document.getElementById("still-container");
const stillImg       = document.getElementById("still-img");
const cropBox        = document.getElementById("crop-box");
const previewImg     = document.getElementById("preview-img");
const viewToggle     = document.getElementById("view-toggle");
const grooveToggle   = document.getElementById("groove-toggle");
const editPanel      = document.getElementById("edit-panel");
const feedRaw        = document.getElementById("camera-feed-raw");
const feedCanny      = document.getElementById("camera-feed-canny");
const labelRaw       = document.getElementById("label-raw");
const labelProcessed = document.getElementById("label-processed");

let stillLoaded  = false;
let leftMode     = "depth";                          // left panel: "depth" | "rgb"
let grooveMode   = "skeleton";                       // right panel: "skeleton" | "mask"
let lastStill    = { depth: null, rgb: null };       // captured still data URLs
let lastGroove   = { skeleton: null, mask: null };   // captured groove preview data URLs
let crop         = { x: 0, y: 0, w: 1, h: 1 };       // normalized to displayed image

/* True while the captured still is showing (vs. the live feed). The crop overlay
   and groove view follow whichever is active. */
function editingActive() {
  return !stillContainer.classList.contains("hidden");
}

/* The left image currently visible — live feed or captured still. The crop
   overlay's drag math operates on whichever one is showing. */
function activeLeftImg() {
  return editingActive() ? stillImg : feedRaw;
}

/* Apply the Depth/RGB toggle to whichever left view is showing. Guarded so we
   don't reset the MJPEG stream (which would flicker) on every state broadcast. */
function applyLeftView() {
  const want = leftMode === "depth" ? "/depth" : "/rgb";
  if (!feedRaw.src.endsWith(want)) feedRaw.src = want;
  const stillUrl = lastStill[leftMode] || lastStill.depth || lastStill.rgb;
  if (stillLoaded && stillUrl && stillImg.src !== stillUrl) stillImg.src = stillUrl;
  labelRaw.textContent = leftMode === "depth" ? "Depth" : "RGB";
}

/* Apply the Skeleton/Mask toggle to the right panel (live feed or captured preview). */
function applyGrooveView() {
  if (editingActive()) {
    const url = lastGroove[grooveMode];
    if (url && previewImg.src !== url) previewImg.src = url;
  } else {
    const want = grooveMode === "skeleton" ? "/depth/grooves" : "/depth/mask";
    if (!feedCanny.src.endsWith(want)) feedCanny.src = want;
  }
  labelProcessed.textContent = grooveMode === "skeleton" ? "Grooves" : "Mask";
}

/* Show/hide live feeds vs. the captured-still editing UI. Idempotent — safe to
   call every state broadcast. The Detect Grooves panel and crop overlay are
   shown during live preview too, so grooves can be tuned (and the region
   cropped) before an image is captured. */
function showLive(showPanel) {
  feedRaw.classList.remove("hidden");
  feedCanny.classList.remove("hidden");
  stillContainer.classList.add("hidden");
  previewImg.classList.add("hidden");
  viewToggle.classList.remove("hidden");
  grooveToggle.classList.remove("hidden");
  editPanel.classList.toggle("hidden", !showPanel);
  cropBox.classList.toggle("hidden", !showPanel);
  applyLeftView();
  applyGrooveView();
  renderCrop();
}

function showEditing(phase) {
  const executing = phase === "executing";
  feedRaw.classList.add("hidden");
  feedCanny.classList.add("hidden");
  stillContainer.classList.remove("hidden");
  previewImg.classList.remove("hidden");
  viewToggle.classList.remove("hidden");
  grooveToggle.classList.remove("hidden");
  editPanel.classList.toggle("hidden", executing);
  cropBox.classList.toggle("hidden", executing);
  applyLeftView();
  applyGrooveView();
  renderCrop();
}

function syncEditUI(phase) {
  if (!stillLoaded || phase === "previewing" || phase === "idle") {
    showLive(phase === "previewing");   // show the Detect panel + crop while previewing live
    return;
  }
  showEditing(phase);
}

/* ── Capture / Retake / Generate ───────────────────────────────────────── */
document.getElementById("btn-capture-image").addEventListener("click", () => {
  sendWS({ type: "capture_image" });
  setHeaderStatus("robot", true, "Capturing still…");
});

document.getElementById("btn-retake").addEventListener("click", () => {
  stillLoaded = false;
  buildPathViz(null);
  showLive(true);
  sendWS({ type: "retake" });
  requestAdjust(true);                  // keep the live groove feed in sync with the panel
  setHeaderStatus("robot", true, "Back to live feed.");
});

document.getElementById("btn-generate").addEventListener("click", () => {
  sendWS({ type: "generate_path", params: buildParams() });
  setHeaderStatus("robot", true, "Generating tool path…");
});

/* ── Incoming still + preview ──────────────────────────────────────────── */
function handleStill(data) {
  if (!data.depth && !data.rgb) return;
  stillLoaded = true;
  previewImg.src = "";
  lastStill  = { depth: data.depth || null, rgb: data.rgb || null };
  lastGroove = { skeleton: null, mask: null };
  // Keep the crop the user drew on the live view — it carries into capture.

  // onload only re-lays-out the crop box. It must NOT trigger another adjust,
  // because every preview swaps stillImg.src (below) which re-fires onload.
  stillImg.onload = renderCrop;
  stillImg.src = lastStill[leftMode] || lastStill.depth || lastStill.rgb;
  labelRaw.textContent = leftMode === "depth" ? "Depth" : "RGB";
  showEditing("editing");
  requestAdjust(true);                  // …then fetch the cropped grooves + mask
  setHeaderStatus("robot", true, "Crop and adjust, then Generate Path.");
}

function handlePreview(data) {
  // Right panel: cropped grooves (skeleton) or the thick mask, per the toggle.
  if (data.grooves) lastGroove.skeleton = data.grooves;
  if (data.mask)    lastGroove.mask     = data.mask;
  applyGrooveView();

  // The colorized depth can change with the view-range sliders; refresh the
  // captured depth so toggling back to it stays current.
  if (data.depth) {
    lastStill.depth = data.depth;
    if (leftMode === "depth" && stillLoaded) stillImg.src = data.depth;
  }
}

viewToggle.querySelectorAll(".seg").forEach(btn => {
  btn.addEventListener("click", () => {
    leftMode = btn.dataset.view;
    viewToggle.querySelectorAll(".seg").forEach(b =>
      b.classList.toggle("active", b === btn));
    applyLeftView();
  });
});

grooveToggle.querySelectorAll(".seg").forEach(btn => {
  btn.addEventListener("click", () => {
    grooveMode = btn.dataset.groove;
    grooveToggle.querySelectorAll(".seg").forEach(b =>
      b.classList.toggle("active", b === btn));
    applyGrooveView();
  });
});

const pathToggle = document.getElementById("path-toggle");
pathToggle.querySelectorAll(".seg").forEach(btn => {
  btn.addEventListener("click", () => {
    pathMode = btn.dataset.path;
    pathToggle.querySelectorAll(".seg").forEach(b =>
      b.classList.toggle("active", b === btn));
    applyPathMode();
  });
});

document.getElementById("order-size").addEventListener("input", (e) => {
  orderSize = parseFloat(e.target.value) || 1.0;
  buildOrderViz(lastStrokes);   // rebuild numbers + dots at the new size
  applyPathMode();
});

/* ── Adjustment sliders ────────────────────────────────────────────────── */
function adjRows() { return document.querySelectorAll(".adj-row[data-key]"); }

function initAdjustControls() {
  adjRows().forEach(row => {
    const input = row.querySelector("input[type=range]");
    input.min   = row.dataset.min;
    input.max   = row.dataset.max;
    input.step  = row.dataset.step;
    input.value = row.dataset.default;
    updateAdjVal(row);
    input.addEventListener("input", () => { updateAdjVal(row); requestAdjust(); });
  });
  document.getElementById("detect-mode").addEventListener("change", () => requestAdjust(true));
}

function updateAdjVal(row) {
  const input = row.querySelector("input[type=range]");
  const span  = row.querySelector(".adj-val");
  const step  = parseFloat(row.dataset.step);
  span.textContent = step < 1 ? parseFloat(input.value).toFixed(2) : input.value;
}

function readAdjustments() {
  const adj = {};
  adjRows().forEach(row => {
    adj[row.dataset.key] = parseFloat(row.querySelector("input[type=range]").value);
  });
  adj.detect = document.getElementById("detect-mode").value;
  return adj;
}

function cropForSend() {
  const c = crop;
  if (c.w <= 0.001 || c.h <= 0.001) return { x: 0, y: 0, w: 1, h: 1 };
  return { x: c.x, y: c.y, w: c.w, h: c.h };
}

function buildParams() {
  return { crop: cropForSend(), adjustments: readAdjustments() };
}

let adjustTimer = null;
function requestAdjust(immediate) {
  if (adjustTimer) clearTimeout(adjustTimer);
  // Before capture → push params to the live feed; after capture → re-process
  // the frozen still (cropped). Same params either way.
  const fire = () => sendWS({
    type: stillLoaded ? "preview_adjust" : "set_groove_params",
    params: buildParams(),
  });
  if (immediate) fire();
  else adjustTimer = setTimeout(fire, 120);
}

document.getElementById("btn-reset-adjust").addEventListener("click", () => {
  adjRows().forEach(row => {
    const input = row.querySelector("input[type=range]");
    input.value = row.dataset.default;
    updateAdjVal(row);
  });
  document.getElementById("detect-mode").value = "valley";
  requestAdjust(true);
});

document.getElementById("btn-crop-reset").addEventListener("click", () => {
  crop = { x: 0, y: 0, w: 1, h: 1 };
  renderCrop();
  requestAdjust(true);
});

/* ── Reference (background) subtraction ─────────────────────────────────── */
document.getElementById("btn-set-reference").addEventListener("click", () => {
  sendWS({ type: "set_reference" });
  setHeaderStatus("robot", true, "Capturing reference (empty sand)…");
});

document.getElementById("btn-clear-reference").addEventListener("click", () => {
  sendWS({ type: "clear_reference" });
});

function handleReferenceStatus(data) {
  const btn = document.getElementById("btn-set-reference");
  btn.classList.toggle("active", data.active);
  btn.textContent = data.active ? "Reference set ✓ (re-capture)" : "Set Reference (empty sand)";
  setHeaderStatus("robot", true, data.message || "");
  // Re-run detection so the change is reflected immediately (live or captured).
  requestAdjust(true);
}

/* ── Crop overlay (drag to draw / move / resize) ───────────────────────── */
/* The overlay lives on the left panel and tracks the active image (live depth
   feed or captured still), so a region can be cropped before *or* after capture. */
function imgContentRect() {
  const cr = leftPanel.getBoundingClientRect();
  const ir = activeLeftImg().getBoundingClientRect();
  return { ox: ir.left - cr.left, oy: ir.top - cr.top, w: ir.width, h: ir.height };
}

function renderCrop() {
  if (cropBox.classList.contains("hidden")) return;
  const r = imgContentRect();
  if (r.w === 0 || r.h === 0) return;
  cropBox.style.left   = (r.ox + crop.x * r.w) + "px";
  cropBox.style.top    = (r.oy + crop.y * r.h) + "px";
  cropBox.style.width  = (crop.w * r.w) + "px";
  cropBox.style.height = (crop.h * r.h) + "px";
}

function clamp01(v) { return Math.min(Math.max(v, 0), 1); }

function ptNorm(e) {
  const ir = activeLeftImg().getBoundingClientRect();
  return {
    x: clamp01((e.clientX - ir.left) / ir.width),
    y: clamp01((e.clientY - ir.top)  / ir.height),
  };
}

let dragMode = null, dragStart = null, cropStart = null;

leftPanel.addEventListener("mousedown", (e) => {
  // No cropping when the overlay is hidden (idle), or when clicking the toggle.
  if (cropBox.classList.contains("hidden")) return;
  if (e.target.closest("#view-toggle")) return;
  const handle = e.target.closest(".crop-handle");
  const p = ptNorm(e);
  if (handle) {
    dragMode = handle.dataset.h;
  } else if (p.x >= crop.x && p.x <= crop.x + crop.w &&
             p.y >= crop.y && p.y <= crop.y + crop.h) {
    dragMode = "move";
  } else {
    dragMode = "new";
    crop = { x: p.x, y: p.y, w: 0, h: 0 };
  }
  dragStart = p;
  cropStart = { ...crop };
  e.preventDefault();
  window.addEventListener("mousemove", onCropMove);
  window.addEventListener("mouseup", onCropUp);
});

function onCropMove(e) {
  const p = ptNorm(e);
  if (dragMode === "new") {
    crop.x = Math.min(dragStart.x, p.x);
    crop.y = Math.min(dragStart.y, p.y);
    crop.w = Math.abs(p.x - dragStart.x);
    crop.h = Math.abs(p.y - dragStart.y);
  } else if (dragMode === "move") {
    crop.x = clamp01(Math.min(cropStart.x + (p.x - dragStart.x), 1 - cropStart.w));
    crop.y = clamp01(Math.min(cropStart.y + (p.y - dragStart.y), 1 - cropStart.h));
  } else {
    let x0 = cropStart.x, y0 = cropStart.y;
    let x1 = cropStart.x + cropStart.w, y1 = cropStart.y + cropStart.h;
    if (dragMode.includes("w")) x0 = p.x;
    if (dragMode.includes("e")) x1 = p.x;
    if (dragMode.includes("n")) y0 = p.y;
    if (dragMode.includes("s")) y1 = p.y;
    crop.x = Math.min(x0, x1); crop.w = Math.abs(x1 - x0);
    crop.y = Math.min(y0, y1); crop.h = Math.abs(y1 - y0);
  }
  cropBox.classList.remove("hidden");
  renderCrop();
}

function onCropUp() {
  window.removeEventListener("mousemove", onCropMove);
  window.removeEventListener("mouseup", onCropUp);
  if (crop.w < 0.01 || crop.h < 0.01) crop = { x: 0, y: 0, w: 1, h: 1 };
  renderCrop();
  requestAdjust(true);
}

new ResizeObserver(renderCrop).observe(leftPanel);
initAdjustControls();

/* ── Run / Cancel buttons ──────────────────────────────────────────────── */
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
