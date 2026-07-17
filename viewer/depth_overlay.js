/* Participant-Mode popup logic (loaded by depth_view.html, the ⧉ Participant
   Mode window opened from the Depth viewport in Developer Mode).

   Required ids: #stage-wrap #stage #feed #overlay
                 #interval #interval-val #textsize #textsize-val
                 #auto-toggle #trigger #status-chip #status-msg

   What it does:
   - keeps a 4:3 stage fitted to the window, draws the server's depth-number
     labels ([[u, v, mm], ...], full-frame 640x480 coords) over the live feed;
   - Region-interval slider → `depth_overlay_params`; Text-size is client-side;
   - Auto toggle → `set_automation` — ON arms the automated pipeline and locks
     the manual Capture/Generate/Run buttons in Developer Mode;
   - Trigger box (mm) → `set_trigger` (empty = none) — the distance that arms
     the trigger; both are server-side state shared by every open window;
   - shows the automation status (Auto Off/Auto On/Alerted/Sensing/Generating
     Paths/Actuating) big in the stage's top-right corner, from `state`. */

(function () {
  const SRC_W = 640, SRC_H = 480;
  const stage   = document.getElementById("stage");
  const canvas  = document.getElementById("overlay");
  const ctx     = canvas.getContext("2d");
  let labels = [];

  /* ── Layout: keep a 4:3 stage that fits the window ──────────────────────── */
  function layout() {
    const wrap = document.getElementById("stage-wrap");
    const aw = wrap.clientWidth, ah = wrap.clientHeight;
    let w = aw, h = aw * (SRC_H / SRC_W);
    if (h > ah) { h = ah; w = ah * (SRC_W / SRC_H); }
    stage.style.width = w + "px";
    stage.style.height = h + "px";
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
    draw();
  }
  window.addEventListener("resize", layout);

  /* ── Depth-number drawing ───────────────────────────────────────────────── */
  function draw() {
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.width, h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    if (!labels.length) return;
    const sx = w / SRC_W, sy = h / SRC_H;
    const px = parseFloat(document.getElementById("textsize").value) * dpr;
    ctx.font = `${px}px "SF Mono", "Fira Code", monospace`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.lineWidth = Math.max(2, px / 6);
    ctx.strokeStyle = "rgba(0,0,0,0.85)";
    ctx.fillStyle = "#ffffff";
    for (const [u, v, mm] of labels) {
      const t = String(Math.round(mm));
      const x = u * sx, y = v * sy;
      ctx.strokeText(t, x, y);   // dark outline keeps numbers readable on any colour
      ctx.fillText(t, x, y);
    }
  }

  /* ── Sliders ────────────────────────────────────────────────────────────── */
  const intervalEl = document.getElementById("interval");
  const textEl     = document.getElementById("textsize");

  let sendTimer = null;
  intervalEl.addEventListener("input", () => {
    document.getElementById("interval-val").textContent = intervalEl.value + " mm";
    if (sendTimer) clearTimeout(sendTimer);
    sendTimer = setTimeout(sendParams, 150);
  });
  textEl.addEventListener("input", () => {
    document.getElementById("textsize-val").textContent = textEl.value + " px";
    draw();                       // text size is client-side only
  });

  function sendParams() {
    send({ type: "depth_overlay_params",
           params: { interval_mm: parseFloat(intervalEl.value) } });
  }

  /* ── Trigger box (Participant-Mode automation threshold) ────────────────── */
  const triggerEl = document.getElementById("trigger");
  let trigTimer = null;
  triggerEl.addEventListener("input", () => {
    if (trigTimer) clearTimeout(trigTimer);
    trigTimer = setTimeout(() => {
      const v = parseFloat(triggerEl.value);
      send({ type: "set_trigger",
             params: { threshold_mm: Number.isFinite(v) && v > 0 ? v : null } });
    }, 400);
  });

  function syncTrigger(mm) {
    // Adopt the server's threshold (set here or in another window) — but never
    // fight the user while they are typing in this box.
    if (document.activeElement === triggerEl) return;
    const cur = parseFloat(triggerEl.value);
    if (mm == null && triggerEl.value !== "") triggerEl.value = "";
    else if (mm != null && cur !== mm) triggerEl.value = mm;
  }

  /* ── Auto toggle ────────────────────────────────────────────────────────── */
  const autoEl = document.getElementById("auto-toggle");
  let autoOn = false;
  autoEl.addEventListener("click", () => {
    send({ type: "set_automation", params: { on: !autoOn } });
    // The button reflects the server's answer (next state tick), not the click.
  });

  function syncAuto(on) {
    autoOn = !!on;
    autoEl.textContent = autoOn ? "Auto: ON" : "Auto: OFF";
    autoEl.classList.toggle("on", autoOn);
  }

  /* ── Status chip (top-right, big) ───────────────────────────────────────── */
  const chipEl = document.getElementById("status-chip");
  const msgEl  = document.getElementById("status-msg");
  const CHIP_CLASS = {
    "Auto Off": "off", "Auto On": "watching", "Alerted": "alerted",
    "Sensing": "sensing", "Generating Paths": "generating", "Actuating": "actuating",
  };

  function updateParticipant(p) {
    if (!p) return;
    chipEl.textContent = p.status || "Auto Off";
    chipEl.className = "chip-" + (CHIP_CLASS[p.status] || "off");
    msgEl.textContent = p.message || "";
    syncAuto(p.auto);
    syncTrigger(p.trigger_mm);
  }

  /* ── WebSocket: register as an overlay client; receive labels + state ───── */
  let ws = null;
  function send(obj) {
    if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
  }
  function connectWS() {
    const proto = location.protocol === "https:" ? "wss:" : "ws:";
    ws = new WebSocket(`${proto}//${location.host}/ws`);
    ws.onopen = () => {
      send({ type: "depth_overlay_hello" });
      sendParams();               // push the current interval on (re)connect
    };
    ws.onclose = () => setTimeout(connectWS, 2000);
    ws.onmessage = (ev) => {
      const data = JSON.parse(ev.data);
      if (data.type === "depth_labels") {
        labels = data.labels || [];
        draw();
      } else if (data.type === "state" || data.type === "init") {
        updateParticipant(data.participant);
      }
    };
  }

  connectWS();
  layout();
})();
