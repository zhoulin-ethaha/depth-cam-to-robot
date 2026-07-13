/* ── Three.js scene setup ───────────────────────────────────────────────── */
const canvas   = document.getElementById("threejs-canvas");
const panel    = document.getElementById("panel-3d");

// preserveDrawingBuffer lets us grab the canvas as a PNG (for Save) at any time.
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
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

/* ── Target surface visualization ───────────────────────────────────────── */
/* The mesh is sent once in its LOCAL frame; placing it (pose sliders) only
   updates this group's transform, so dragging sliders is instant. */
let surfaceGroup = null;

function buildSurfaceViz(mesh) {
  if (surfaceGroup) { scene.remove(surfaceGroup); surfaceGroup = null; }
  if (!mesh || !mesh.vertices || !mesh.vertices.length) return;

  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.Float32BufferAttribute(mesh.vertices, 3));
  geo.setIndex(mesh.faces);
  geo.computeVertexNormals();

  const mat = new THREE.MeshStandardMaterial({
    color: 0x9a7b4f,            // sand
    roughness: 0.95,
    metalness: 0.0,
    transparent: true,
    opacity: 0.85,
    side: THREE.DoubleSide,
  });
  surfaceGroup = new THREE.Group();
  surfaceGroup.add(new THREE.Mesh(geo, mat));
  const wire = new THREE.LineSegments(
    new THREE.WireframeGeometry(geo),
    new THREE.LineBasicMaterial({ color: 0x5a4a33, transparent: true, opacity: 0.25 })
  );
  surfaceGroup.add(wire);
  scene.add(surfaceGroup);
  applySurfacePose(readSurfacePose());
}

function applySurfacePose(p) {
  if (!surfaceGroup) return;
  const d = Math.PI / 180;
  // 'ZYX' here matches scipy's extrinsic 'xyz' euler on the server.
  surfaceGroup.setRotationFromEuler(new THREE.Euler(p.rx * d, p.ry * d, p.rz * d, "ZYX"));
  surfaceGroup.position.set(p.tx, p.ty, p.tz);
}

/* ── Path preview (set after Capture) ──────────────────────────────────── */
let pathGroup  = null;
let orderGroup = null;
let lastStrokes = null;
let pathMode   = "path";   // "path" | "order"
let orderSize  = 1.0;      // multiplier for order numbers + start/end dots

function buildPathViz(strokes, reachFlags) {
  if (pathGroup) { scene.remove(pathGroup); pathGroup = null; }
  lastStrokes = strokes && strokes.length ? strokes : null;

  if (lastStrokes) {
    pathGroup = new THREE.Group();
    const travelMat = new THREE.LineBasicMaterial({ color: 0x444466 }); // gray — travel
    const okPts  = [];   // reachable draw segments (green)
    const badPts = [];   // unreachable draw segments (red)

    let prevEnd = null;
    lastStrokes.forEach((stroke, si) => {
      if (stroke.length === 0) return;
      const flags = reachFlags && reachFlags[si] ? reachFlags[si] : null;

      // Travel segment from previous stroke end to this stroke start
      if (prevEnd) {
        const startPt = new THREE.Vector3(stroke[0][0], stroke[0][1], stroke[0][2]);
        const tGeo = new THREE.BufferGeometry().setFromPoints([prevEnd, startPt]);
        pathGroup.add(new THREE.Line(tGeo, travelMat));
      }

      // Draw segments — red where either endpoint is outside estimated reach.
      for (let i = 0; i + 1 < stroke.length; i++) {
        const bad = flags && (flags[i] || flags[i + 1]);
        const bucket = bad ? badPts : okPts;
        bucket.push(new THREE.Vector3(stroke[i][0], stroke[i][1], stroke[i][2]),
                    new THREE.Vector3(stroke[i + 1][0], stroke[i + 1][1], stroke[i + 1][2]));
      }

      prevEnd = new THREE.Vector3(stroke[stroke.length - 1][0],
                                  stroke[stroke.length - 1][1],
                                  stroke[stroke.length - 1][2]);
    });

    if (okPts.length) {
      const geo = new THREE.BufferGeometry().setFromPoints(okPts);
      pathGroup.add(new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color: 0x00e5a0 })));
    }
    if (badPts.length) {
      const geo = new THREE.BufferGeometry().setFromPoints(badPts);
      pathGroup.add(new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color: 0xff4444 })));
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
    const m = stroke[Math.floor(stroke.length / 2)];   // midpoint along the stroke

    const startDot = new THREE.Mesh(dot, startMat);
    startDot.position.set(s[0], s[1], s[2]);
    orderGroup.add(startDot);

    const endDot = new THREE.Mesh(dot, endMat);
    endDot.position.set(e[0], e[1], e[2]);
    orderGroup.add(endDot);

    // Number the stroke, placed just above its midpoint.
    const label = makeLabelSprite(String(i + 1), 0.03 * orderSize);
    label.position.set(m[0], m[1], m[2] + 0.012 * orderSize);
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
    } else if (data.type === "connection_result") {
      setHeaderStatus("robot", data.success, data.message);
      // After a successful connect, prompt for the drawing target unless a
      // surface is already loaded (there is no 3-point calibration anymore).
      if (data.success && !surfaceGroup) showOverlay(true);
    } else if (data.type === "still") {
      handleStill(data);
    } else if (data.type === "preview") {
      handlePreview(data);
    } else if (data.type === "capture_result") {
      handleCaptureResult(data);
    } else if (data.type === "reference_status") {
      handleReferenceStatus(data);
    } else if (data.type === "surface_status") {
      handleSurfaceStatus(data);
    } else if (data.type === "save_result") {
      handleSaveResult(data);
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
  if (data.surface && data.surface.loaded) {
    handleSurfaceStatus(data.surface);
  }
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
    buildPathViz(data.strokes, data.reach_flags);
    setButtonsForPhase("captured");
    if (data.reach_out > 0) {
      setHeaderStatus("robot", false,
        `⚠ ${data.reach_out} waypoints outside the arm's estimated reach — shown in RED. ` +
        `Move the surface closer / crop smaller before running.`);
    } else {
      setHeaderStatus("robot", true,
        `Path ready: ${data.stroke_count} strokes, ${data.point_count} points — all within estimated reach`);
    }
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

  // Live TCP marker: bright blue + enlarged while the path is executing, so
  // the dot visibly travels along the strokes during actuation (RTDE, 20 Hz).
  eeMat.color.setHex(data.executing ? 0x2e9fff : 0x4488ff);
  eeMat.emissive.setHex(data.executing ? 0x1a5acc : 0x223366);
  const s = data.executing ? 1.5 : 1.0;
  eeSphere.scale.set(s, s, s);
}

/* ── Footer update ──────────────────────────────────────────────────────── */
let robotConnected = false;

function updateFooter(data) {
  if (typeof data.robot_connected === "boolean") robotConnected = data.robot_connected;

  document.getElementById("val-phase").textContent   = data.phase || "idle";
  document.getElementById("val-strokes").textContent = data.stroke_count ?? 0;

  // Surface executor failures (unreachable pose, robot not in Remote, …) so
  // the reason is visible at the rig, not just in the terminal.
  if (data.phase === "error" && data.exec_error) {
    setHeaderStatus("robot", false, "Run failed: " + data.exec_error);
  }

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

function sendWS(msg) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(msg));
}

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
/* Four fixed viewports: Depth | RGB (top) and Skeleton | Mask (bottom). Each has
   a live MJPEG image and a captured/preview image; we show one set at a time. */
const vpDepth      = document.getElementById("vp-depth");
const cropBox      = document.getElementById("crop-box");
const editPanel    = document.getElementById("edit-panel");
const feedDepth    = document.getElementById("feed-depth");
const feedRgb      = document.getElementById("feed-rgb");
const feedSkeleton = document.getElementById("feed-skeleton");
const feedMask     = document.getElementById("feed-mask");
const stillDepth   = document.getElementById("still-depth");
const stillRgb     = document.getElementById("still-rgb");
const prevSkeleton = document.getElementById("prev-skeleton");
const prevMask     = document.getElementById("prev-mask");

const liveFeeds = [feedDepth, feedRgb, feedSkeleton, feedMask];
const stillImgs = [stillDepth, stillRgb, prevSkeleton, prevMask];

let stillLoaded = false;
let crop        = { x: 0, y: 0, w: 1, h: 1 };   // normalized to displayed image

/* True while the captured stills are showing (vs. the live feeds). */
function editingActive() {
  return !stillDepth.classList.contains("hidden");
}

/* The Depth image currently visible — the crop overlay's drag math uses it. */
function activeDepthImg() {
  return editingActive() ? stillDepth : feedDepth;
}

/* Show/hide the live feeds vs. the captured stills. Idempotent — safe to call
   every state broadcast. The Detect Grooves panel and crop overlay show during
   live preview too, so all four views can be tuned before capturing. */
function showLive(showPanel) {
  liveFeeds.forEach(f => f.classList.remove("hidden"));
  stillImgs.forEach(s => s.classList.add("hidden"));
  editPanel.classList.toggle("hidden", !showPanel);
  cropBox.classList.toggle("hidden", !showPanel);
  renderCrop();
}

function showEditing(phase) {
  const executing = phase === "executing";
  liveFeeds.forEach(f => f.classList.add("hidden"));
  stillImgs.forEach(s => s.classList.remove("hidden"));
  editPanel.classList.toggle("hidden", executing);
  cropBox.classList.toggle("hidden", executing);
  renderCrop();
}

function syncEditUI(phase) {
  if (!stillLoaded || phase === "previewing" || phase === "idle") {
    showLive(phase === "previewing");   // show the Detect panel + crop while previewing live
    return;
  }
  showEditing(phase);
}

/* ── Projection window (projector output) ──────────────────────────────── */
/* Opens /projection in its own window: drag it onto the projector display and
   press F11. The full-frame mask is only composed while this window is open,
   so there is zero projection overhead otherwise. */
/* Chrome allows only ~6 concurrent HTTP/1.1 connections per host, and MJPEG
   streams hold theirs open forever (this app tab alone uses 4). Opening the
   projection windows on 127.0.0.1 instead of localhost gives them their own
   pool, so their mask stream and corner requests never starve. */
function projOrigin() {
  return location.hostname === "localhost"
    ? location.protocol + "//127.0.0.1:" + location.port
    : "";
}

function openProjWindow(btn, url, name, feature, message) {
  let win = window.open(projOrigin() + url, name, feature);
  if (!win) {
    setHeaderStatus("robot", false, "Pop-up blocked — allow pop-ups for this site.");
    return null;
  }
  btn.classList.add("active");
  const watch = setInterval(() => {
    if (!win || win.closed) {
      clearInterval(watch);
      btn.classList.remove("active");
    }
  }, 1000);
  setHeaderStatus("robot", true, message);
  return win;
}

let projWin = null;
let projCalWin = null;
const btnProject = document.getElementById("btn-project");
const btnProjectCal = document.getElementById("btn-project-cal");

btnProject.addEventListener("click", () => {
  if (projWin && !projWin.closed) { projWin.focus(); return; }
  projWin = openProjWindow(btnProject, "/projection", "sandProjection",
    "width=1920,height=1080",
    "Projection window opened — move it to the projector display and press F11 " +
    "(output resolution follows the display: set the projector to 4K in Windows for 4K).");
});

btnProjectCal.addEventListener("click", () => {
  if (projCalWin && !projCalWin.closed) { projCalWin.focus(); return; }
  projCalWin = openProjWindow(btnProjectCal, "/projection?cal", "sandProjectionCal",
    "width=1100,height=760",
    "Calibration window opened — keep it on this screen, drag handles 1–4 while " +
    "watching the sand; the projector output follows live.");
});

/* ── Swap the left/right position of a row's two viewports ──────────────── */
document.getElementById("swap-depth").addEventListener("click", () => {
  document.getElementById("row-depth").classList.toggle("swapped");
  renderCrop();
});
document.getElementById("swap-grooves").addEventListener("click", () => {
  document.getElementById("row-grooves").classList.toggle("swapped");
});

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
  prevSkeleton.src = "";
  prevMask.src = "";
  stillRgb.src = data.rgb || "";
  // Keep the crop the user drew on the live view — it carries into capture.

  // onload re-lays-out the crop box; harmless if it re-fires on later updates.
  stillDepth.onload = renderCrop;
  stillDepth.src = data.depth || "";
  showEditing("editing");
  requestAdjust(true);                  // …then fetch the cropped grooves + mask
  setHeaderStatus("robot", true, "Crop and adjust, then Generate Path.");
}

function handlePreview(data) {
  // Bottom row: cropped skeleton + thick mask, shown side by side.
  if (data.grooves) prevSkeleton.src = data.grooves;
  if (data.mask)    prevMask.src     = data.mask;
  // RGB view shows only the cropped region (Depth stays full so it's croppable).
  if (data.rgb)     stillRgb.src     = data.rgb;
  // The colorized depth can change with the view-range sliders; keep it current.
  if (data.depth && stillLoaded) stillDepth.src = data.depth;
}

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

/* ── Target surface (3D projection) ─────────────────────────────────────── */
function srfRows() { return document.querySelectorAll(".adj-row[data-skey]"); }

function initSurfaceControls() {
  srfRows().forEach(row => {
    const input = row.querySelector("input[type=range]");
    input.min   = row.dataset.min;
    input.max   = row.dataset.max;
    input.step  = row.dataset.step;
    input.value = row.dataset.default;
    updateAdjVal(row);
    input.addEventListener("input", () => {
      updateAdjVal(row);
      applySurfacePose(readSurfacePose());   // instant client-side placement
      sendSurfacePose();                     // debounced push to the server
    });
  });

  document.getElementById("btn-surface-load").addEventListener("click", () =>
    document.getElementById("surface-file").click());
  // Same file input, reachable from the workspace-setup overlay: loading a
  // surface replaces the flat-workspace calibration entirely.
  document.getElementById("btn-ws-load-surface").addEventListener("click", () =>
    document.getElementById("surface-file").click());
  document.getElementById("surface-file").addEventListener("change", uploadSurface);
  document.getElementById("btn-surface-clear").addEventListener("click", () => {
    sendWS({ type: "clear_surface" });
  });
}

function readSurfacePose() {
  const p = { tx: 0.4, ty: 0, tz: 0, rx: 0, ry: 0, rz: 0, offset_mm: 0 };
  srfRows().forEach(row => {
    p[row.dataset.skey] = parseFloat(row.querySelector("input[type=range]").value);
  });
  return p;
}

let surfacePoseTimer = null;
function sendSurfacePose() {
  if (surfacePoseTimer) clearTimeout(surfacePoseTimer);
  surfacePoseTimer = setTimeout(() => {
    const p = readSurfacePose();
    sendWS({ type: "set_surface_pose",
             params: { pose: p, offset_mm: p.offset_mm } });
  }, 150);
}

async function uploadSurface(e) {
  const file = e.target.files && e.target.files[0];
  if (!file) return;
  setHeaderStatus("robot", true, `Uploading ${file.name}…`);
  const form = new FormData();
  form.append("file", file);
  try {
    const res = await fetch("/surface/upload", { method: "POST", body: form });
    const out = await res.json();
    if (!out.ok) throw new Error(out.error || "upload failed");
    // The server broadcasts surface_status (with the mesh) to all clients.
  } catch (err) {
    setHeaderStatus("robot", false, "Surface upload failed: " + err.message);
  }
  e.target.value = "";   // allow re-selecting the same file
}

function handleSurfaceStatus(data) {
  const status = document.getElementById("surface-status");
  if (data.loaded) {
    if (data.mesh) buildSurfaceViz(data.mesh);
    if (data.pose) setSurfaceSliders(data.pose, data.offset_mm);
    const i = data.info || {};
    const size = i.bbox ? `${i.bbox.size[0]}×${i.bbox.size[1]}×${i.bbox.size[2]} m` : "";
    status.textContent = `Loaded: ${i.name || "surface"} — ${i.faces || "?"} faces, ${size}`;
    showOverlay(false);   // surface replaces the P0/Px/Py calibration
  } else {
    if (surfaceGroup) { scene.remove(surfaceGroup); surfaceGroup = null; }
    status.textContent = "No surface loaded.";
  }
  if (data.message) setHeaderStatus("robot", true, data.message);
  applySurfacePose(readSurfacePose());
}

function setSurfaceSliders(pose, offsetMm) {
  srfRows().forEach(row => {
    const key = row.dataset.skey;
    const val = key === "offset_mm" ? offsetMm : pose[key];
    if (val === undefined || val === null) return;
    const input = row.querySelector("input[type=range]");
    input.value = val;
    updateAdjVal(row);
  });
}

/* ── Crop overlay (drag to draw / move / resize) ───────────────────────── */
/* The overlay lives on the Depth viewport and tracks its active image (live
   feed or captured still), so a region can be cropped before *or* after capture. */
function imgContentRect() {
  const cr = vpDepth.getBoundingClientRect();
  const ir = activeDepthImg().getBoundingClientRect();
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
  const ir = activeDepthImg().getBoundingClientRect();
  return {
    x: clamp01((e.clientX - ir.left) / ir.width),
    y: clamp01((e.clientY - ir.top)  / ir.height),
  };
}

let dragMode = null, dragStart = null, cropStart = null;

vpDepth.addEventListener("mousedown", (e) => {
  // No cropping when the overlay is hidden (idle).
  if (cropBox.classList.contains("hidden")) return;
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

new ResizeObserver(renderCrop).observe(vpDepth);

/* ── Column splitter: drag to resize the 4-view block vs. path preview ──── */
(function () {
  const splitter = document.getElementById("col-splitter");
  const mainEl = document.querySelector("main");
  splitter.addEventListener("mousedown", (e) => {
    e.preventDefault();
    splitter.classList.add("dragging");
    document.body.style.userSelect = "none";
    const onMove = (ev) => {
      const rect = mainEl.getBoundingClientRect();
      const min = 240, max = rect.width - 320;              // keep both sides usable
      const w = Math.max(min, Math.min(ev.clientX - rect.left, max));
      mainEl.style.setProperty("--cam-w", w + "px");
      // 3D canvas + crop overlay re-fit automatically via their ResizeObservers.
    };
    const onUp = () => {
      splitter.classList.remove("dragging");
      document.body.style.userSelect = "";
      window.removeEventListener("mousemove", onMove);
      window.removeEventListener("mouseup", onUp);
    };
    window.addEventListener("mousemove", onMove);
    window.addEventListener("mouseup", onUp);
  });
})();
initAdjustControls();
initSurfaceControls();

/* ── Pop-out path preview ───────────────────────────────────────────────── */
/* Moves the whole #panel-3d (canvas + toggles + legend) into its own browser
   window for a bigger view. Same JS context, so the scene keeps updating. */
let popupWin = null;
const panelHome = panel.parentElement;   // <main>, where the panel lives normally
// Hold the node reference: once the panel (and this button with it) moves into
// the popup document, document.getElementById in the MAIN window returns null.
const btnPopout = document.getElementById("btn-popout");

btnPopout.addEventListener("click", () => {
  if (popupWin && !popupWin.closed) { popupWin.close(); popIn(); return; }
  popupWin = window.open("", "pathPreview", "width=1000,height=760");
  if (!popupWin) {
    setHeaderStatus("robot", false, "Pop-up blocked — allow pop-ups for this site.");
    return;
  }
  const d = popupWin.document;
  d.title = "Path Preview";
  const link = d.createElement("link");
  link.rel = "stylesheet";
  link.href = location.origin + "/static/style.css";
  d.head.appendChild(link);
  d.body.style.cssText = "margin:0;background:#0e0e0e;overflow:hidden;";
  d.body.appendChild(panel);
  panel.style.cssText = "position:absolute;inset:0;";
  btnPopout.textContent = "⇱ Pop in";
  popupWin.addEventListener("resize", resizeRenderer);
  popupWin.addEventListener("pagehide", popIn);   // fires when the popup closes
  setTimeout(resizeRenderer, 50);
});

function popIn() {
  if (panel.ownerDocument === document) return;   // already home
  panelHome.appendChild(panel);
  panel.style.cssText = "";                       // back to the CSS grid placement
  btnPopout.textContent = "⧉ Pop out";
  popupWin = null;
  setTimeout(resizeRenderer, 50);
}

window.addEventListener("beforeunload", () => {
  if (popupWin && !popupWin.closed) popupWin.close();
});

/* ── Run / Cancel buttons ──────────────────────────────────────────────── */
document.getElementById("btn-run").addEventListener("click", () => {
  sendWS({ type: "run", params: {
    speed_pct: parseFloat(document.getElementById("exec-speed").value) || 5,
    offset_mm: parseFloat(document.getElementById("exec-offset").value) || 0,
    safety_mm: parseFloat(document.getElementById("exec-safety").value) || 50,
  }});
  setButtonsForPhase("executing");
  setProgress(0);
  document.getElementById("val-phase").textContent = "executing";
});

document.getElementById("exec-speed").addEventListener("input", (e) => {
  document.getElementById("exec-speed-val").textContent = e.target.value + "%";
});

/* ── Save toolpath (URScript + JSON + preview image) ────────────────────── */
document.getElementById("btn-save-path").addEventListener("click", () => {
  // Capture the 3D preview as a PNG. Render first so the buffer is current.
  renderer.render(scene, camera);
  let image = null;
  try { image = renderer.domElement.toDataURL("image/png"); } catch (e) {}
  sendWS({ type: "save_path", params: {
    speed_pct: parseFloat(document.getElementById("exec-speed").value) || 5,
    offset_mm: parseFloat(document.getElementById("exec-offset").value) || 0,
    safety_mm: parseFloat(document.getElementById("exec-safety").value) || 50,
    image,
  }});
  setHeaderStatus("robot", true, "Saving toolpath…");
});

function handleSaveResult(data) {
  if (data.success) {
    setHeaderStatus("robot", true, "✓ Toolpath saved to " + data.folder);
  } else {
    setHeaderStatus("robot", false, "Save failed: " + (data.error || "unknown error"));
  }
}

document.getElementById("btn-cancel").addEventListener("click", () => {
  sendWS({ type: "cancel" });
  setHeaderStatus("robot", true, "Cancelling…");
});
