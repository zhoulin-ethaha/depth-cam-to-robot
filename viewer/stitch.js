/* Dual-camera stitching prototype UI (contained from viewer.js). */
"use strict";

const $ = (id) => document.getElementById(id);

const CAL_INPUTS = {
  tx_mm: $("cal-tx"), ty_mm: $("cal-ty"), tz_mm: $("cal-tz"), yaw_deg: $("cal-yaw"),
};
const PARAM_INPUTS = {
  detect: $("p-detect"),
  smooth_sigma_px: $("p-smooth"),
  detrend_sigma_px: $("p-detrend"),
  groove_depth_mm: $("p-depth"),
  min_blob_px: $("p-blob"),
  min_mean_depth_mm: $("p-meand"),
  min_width_mm: $("p-minw"),
  max_width_mm: $("p-maxw"),
  min_length_mm: $("p-minl"),
};

let ws = null;

function send(type, params) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type, params: params || {} }));
  }
}

function debounce(fn, ms) {
  let t = null;
  return (...a) => { clearTimeout(t); t = setTimeout(() => fn(...a), ms); };
}

/* ── outgoing ── */
function calibFromInputs() {
  const c = {};
  for (const [k, el] of Object.entries(CAL_INPUTS)) {
    const v = parseFloat(el.value);
    if (Number.isFinite(v)) c[k] = v;
  }
  c.swap = $("cal-swap").checked;
  return c;
}
const sendCalib = debounce(() => send("set_calib", calibFromInputs()), 250);

function paramsFromInputs() {
  const p = {};
  for (const [k, el] of Object.entries(PARAM_INPUTS)) {
    p[k] = el.tagName === "SELECT" ? el.value : parseFloat(el.value);
  }
  return p;
}
const sendParams = debounce(() => send("set_params", paramsFromInputs()), 200);

function refreshOutputs() {
  for (const el of Object.values(PARAM_INPUTS)) {
    if (el.tagName !== "SELECT") {
      const out = document.querySelector(`output[for="${el.id}"]`);
      if (out) out.textContent = el.value;
    }
  }
}

for (const el of Object.values(CAL_INPUTS)) el.addEventListener("input", sendCalib);
$("cal-swap").addEventListener("change", sendCalib);
for (const el of Object.values(PARAM_INPUTS)) {
  el.addEventListener("input", () => { refreshOutputs(); sendParams(); });
}
$("btn-refine").addEventListener("click", () => {
  showMsg("Refining from the overlap band…", false);
  send("auto_refine");
});
$("btn-save").addEventListener("click", () => send("save_calib"));

/* ── incoming ── */
function applyCalib(c) {
  if (!c) return;
  for (const [k, el] of Object.entries(CAL_INPUTS)) {
    if (document.activeElement !== el && c[k] !== undefined) {
      el.value = (Math.round(c[k] * 10) / 10).toString();
    }
  }
  if (document.activeElement !== $("cal-swap")) $("cal-swap").checked = !!c.swap;
}

function applyParams(p) {
  if (!p) return;
  for (const [k, el] of Object.entries(PARAM_INPUTS)) {
    if (p[k] !== undefined && document.activeElement !== el) el.value = p[k];
  }
  refreshOutputs();
}

function showMsg(text, isError) {
  const m = $("msg");
  m.textContent = text;
  m.classList.toggle("err", !!isError);
}

function handle(data) {
  if (data.type === "init") {
    applyCalib(data.calib);
    applyParams(data.params);
  } else if (data.type === "state") {
    applyCalib(data.calib);
    const i = data.info;
    $("info").textContent = i
      ? `${i.serials.join(" + ")} · ${i.size[0]}×${i.size[1]} px · ` +
        `${i.mm_per_px} mm/px · overlap ${i.overlap_pct}%`
      : "waiting for frames…";
    $("note").textContent = data.note || (i && i.synthetic ? "SYNTHETIC scene" : "");
    if (data.refine) showMsg(data.refine.message, !data.refine.success);
  } else if (data.type === "save_result") {
    showMsg(data.message, !data.success);
  }
}

/* ── connection ── */
function startStreams() {
  // Chrome caps HTTP/1.1 connections per host NAME and MJPEG streams hold
  // theirs forever (same gotcha as the projection window). Spread the four
  // streams across the localhost / 127.0.0.1 buckets so none gets starved.
  const alt = location.hostname === "127.0.0.1" ? "localhost" : "127.0.0.1";
  const imgs = [...document.querySelectorAll(".vp img[data-src]")];
  imgs.forEach((img, i) => {
    const host = i < 2 ? location.host : `${alt}:${location.port}`;
    img.src = `http://${host}${img.dataset.src}`;
    img.removeAttribute("data-src");
  });
}

function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  // Watchdog: if the handshake hangs (e.g. queued behind other requests),
  // abort and retry instead of sitting in CONNECTING forever.
  const watchdog = setTimeout(() => {
    if (ws.readyState !== WebSocket.OPEN) ws.close();
  }, 4000);
  ws.onopen = () => {
    clearTimeout(watchdog);
    startStreams();   // only claim connections for MJPEG once the WS is up
  };
  ws.onmessage = (ev) => { try { handle(JSON.parse(ev.data)); } catch (_) {} };
  ws.onclose = () => {
    clearTimeout(watchdog);
    $("info").textContent = "disconnected — retrying…";
    setTimeout(connect, 1500);
  };
}
connect();
