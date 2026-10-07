// BOLT controller app - talks to companion/robot_brain/app_server.py over a WebSocket.
"use strict";
const $ = (s) => document.querySelector(s);
const $$ = (s) => [...document.querySelectorAll(s)];
const store = {
  get(k, d) { try { const v = localStorage.getItem("bolt." + k); return v === null ? d : JSON.parse(v); } catch { return d; } },
  set(k, v) { try { localStorage.setItem("bolt." + k, JSON.stringify(v)); } catch {} },
};

// ------------------------------------------------------------------ link
let ws = null, state = null, lastMsg = 0;
function host() { return ($("#host").value || store.get("host", "") || location.host || "localhost:8080").trim(); }
function connect() {
  try { ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${host()}/ws`); }
  catch (e) { setTimeout(connect, 1500); return; }
  ws.onopen = () => { $("#conn").className = "dot on"; toast("connected"); setCam(store.get("cam", "front")); };
  ws.onclose = () => { $("#conn").className = "dot off"; setTimeout(connect, 1000); };
  ws.onmessage = (ev) => { const m = JSON.parse(ev.data); if (m.type === "state") { state = m; lastMsg = Date.now(); render(); } };
}
function send(obj) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj)); }

// ------------------------------------------------------------------ joystick
const stick = $("#stick"), sctx = stick.getContext("2d");
let joy = { x: 0, y: 0, active: false };
function drawStick() {
  const w = stick.width, r = w / 2;
  sctx.clearRect(0, 0, w, w);
  sctx.fillStyle = getComputedStyle(document.body).getPropertyValue("--panel2");
  sctx.beginPath(); sctx.arc(r, r, r - 4, 0, 7); sctx.fill();
  sctx.strokeStyle = "#ffffff22"; sctx.lineWidth = 2;
  sctx.beginPath(); sctx.moveTo(r, 12); sctx.lineTo(r, w - 12); sctx.moveTo(12, r); sctx.lineTo(w - 12, r); sctx.stroke();
  sctx.fillStyle = joy.active ? "#ffd23f" : "#5fd4ff";
  sctx.beginPath(); sctx.arc(r + joy.x * (r - 40), r - joy.y * (r - 40), 34, 0, 7); sctx.fill();
}
function stickPos(e) {
  const b = stick.getBoundingClientRect();
  const x = ((e.clientX - b.left) / b.width) * 2 - 1, y = -(((e.clientY - b.top) / b.height) * 2 - 1);
  const n = Math.hypot(x, y), k = n > 1 ? 1 / n : 1;
  joy.x = x * k; joy.y = y * k;
}
stick.addEventListener("pointerdown", (e) => { stick.setPointerCapture(e.pointerId); joy.active = true; stickPos(e); drawStick(); });
stick.addEventListener("pointermove", (e) => { if (joy.active) { stickPos(e); drawStick(); } });
const release = () => { joy = { x: 0, y: 0, active: false }; drawStick(); send({ type: "drive", v: 0, w: 0 }); };
stick.addEventListener("pointerup", release); stick.addEventListener("pointercancel", release);

// keyboard (desktop)
const keys = new Set();
addEventListener("keydown", (e) => {
  if (e.target.tagName === "INPUT") return;
  keys.add(e.key.toLowerCase());
  if (e.key === " ") { action("jump"); e.preventDefault(); }
  if (e.key.toLowerCase() === "c") $("#crawl").click();
  if (e.key.toLowerCase() === "x") estop();
});
addEventListener("keyup", (e) => keys.delete(e.key.toLowerCase()));

const expo = (v) => Math.sign(v) * v * v;    // finer control around the centre
setInterval(() => {
  let x = joy.x, y = joy.y, kb = false;
  if (keys.has("w")) { y = 1; kb = true; } if (keys.has("s")) { y = -1; kb = true; }
  if (keys.has("a")) { x = -1; kb = true; } if (keys.has("d")) { x = 1; kb = true; }
  if (joy.active || kb) {
    const vmax = +$("#vmax").value;
    send({ type: "drive", v: expo(y) * vmax, w: -expo(x) * 2.5 });
    if (state && state.mode !== "MANUAL" && state.mode !== "IDLE" && (Math.abs(x) + Math.abs(y)) > 0.3) send({ type: "mode", mode: "MANUAL" });
  }
}, 100);

// ------------------------------------------------------------------ buttons
function toast(text, cls = "") {
  const t = $("#toast"); t.textContent = text; t.className = "toast " + cls;
  clearTimeout(toast.t); toast.t = setTimeout(() => t.classList.add("hidden"), 2500);
}
function estop() { send({ type: "estop" }); toast("EMERGENCY STOP", "error"); }
$("#estop").onclick = estop;
$("#release").onclick = () => send({ type: "estop_release" });
$("#sit").onclick = () => action("sit");

function action(name) {
  const h = +$("#jumph").value;
  const extra = { jump: { height: h }, hop: { height: h }, step_up: { height: h }, stairs: { rise: h } }[name] || {};
  send({ type: "action", name, ...extra });
}
$$(".act[data-act]").forEach((b) => b.onclick = () => action(b.dataset.act));
$("#crawl").onclick = (e) => { e.target.classList.toggle("on"); send({ type: "posture", crawl: e.target.classList.contains("on") }); };

const bindRange = (id, out, fmt, fn) => {
  const el = $(id), o = $(out);
  const upd = () => { o.textContent = fmt(+el.value); fn && fn(+el.value); store.set(id, el.value); };
  el.value = store.get(id, el.value); o.textContent = fmt(+el.value);
  el.addEventListener("input", upd);
};
bindRange("#vmax", "#vmaxv", (v) => v.toFixed(1));
bindRange("#height", "#hv", (v) => v.toFixed(3), (v) => send({ type: "posture", height: v }));
bindRange("#pitch", "#pv", (v) => v, (v) => send({ type: "posture", pitch: v * Math.PI / 180 }));
bindRange("#roll", "#rv", (v) => v, (v) => send({ type: "posture", roll: v * Math.PI / 180 }));
bindRange("#jumph", "#jv", (v) => v.toFixed(2));
bindRange("#lspeed", "#lsv", (v) => v.toFixed(2), (v) => send({ type: "settings", line_speed: v }));
$("#level").onclick = () => { for (const id of ["#pitch", "#roll"]) { $(id).value = 0; $(id).dispatchEvent(new Event("input")); } };

// tabs
$$(".tabs button").forEach((b) => b.onclick = () => {
  $$(".tabs button").forEach((x) => x.classList.toggle("on", x === b));
  $$(".tab").forEach((t) => t.classList.toggle("hidden", t.id !== "tab-" + b.dataset.tab));
});
// modes
$$("[data-mode]").forEach((b) => b.onclick = () => send({ type: "mode", mode: b.dataset.mode }));
let lineColor = store.get("color", "red");
$$("#colors button").forEach((b) => {
  b.classList.toggle("on", b.dataset.color === lineColor);
  b.onclick = () => { lineColor = b.dataset.color; store.set("color", lineColor); $$("#colors button").forEach((x) => x.classList.toggle("on", x === b)); };
});
$("#linego").onclick = () => send({ type: "mode", mode: "LINE", color: lineColor });

// lights / eyes
const lights = () => send({ type: "lights", auto: $("#autolights").checked, headlight: +$("#headlight").value, ir: +$("#irlight").value });
["#autolights", "#headlight", "#irlight"].forEach((id) => $(id).addEventListener("input", lights));
$$("#eyes button").forEach((b) => b.onclick = () => send({ type: "eyes", expression: b.dataset.e }));
$("#host").value = store.get("host", "");
$("#host").addEventListener("change", () => { store.set("host", $("#host").value); ws && ws.close(); });

// camera
function setCam(c) {
  store.set("cam", c);
  $$(".camsel button").forEach((b) => b.classList.toggle("on", b.dataset.cam === c));
  $("#video").src = c === "off" ? "" : `${location.protocol}//${host()}/video/${c}.mjpg`;
}
$$(".camsel button").forEach((b) => b.onclick = () => setCam(b.dataset.cam));

// ------------------------------------------------------------------ mission builder
let wps = store.get("wps", []);
function renderWps() {
  store.set("wps", wps);
  $("#wplist").innerHTML = "";
  wps.forEach((w, i) => {
    const li = document.createElement("li");
    li.innerHTML = `<span>${w.action ? "jump " + w.height + " m" : w.lat.toFixed(6) + ", " + w.lon.toFixed(6)}</span>`;
    const x = document.createElement("button"); x.textContent = "x"; x.onclick = () => { wps.splice(i, 1); renderWps(); };
    li.appendChild(x); $("#wplist").appendChild(li);
  });
  drawMap();
}
$("#addhere").onclick = () => { if (state && state.gps) { wps.push({ lat: state.gps.lat, lon: state.gps.lon }); renderWps(); } else toast("no GPS fix yet"); };
$("#addjump").onclick = () => { wps.push({ action: "jump", height: +$("#jumph").value }); renderWps(); };
$("#clearwp").onclick = () => { wps = []; renderWps(); };
$("#addll").onclick = () => {
  const p = $("#latlon").value.split(/[ ,;]+/).map(Number);
  if (p.length >= 2 && isFinite(p[0]) && isFinite(p[1])) { wps.push({ lat: p[0], lon: p[1] }); $("#latlon").value = ""; renderWps(); }
  else toast("format: lat, lon");
};
$("#missiongo").onclick = () => { if (!wps.length) return toast("add waypoints first"); send({ type: "mission", waypoints: wps, start: true }); toast("mission started"); };

const map = $("#map"), mctx = map.getContext("2d");
function drawMap() {
  const pts = wps.filter((w) => !w.action).map((w) => [w.lat, w.lon]);
  if (state && state.gps) pts.push([state.gps.lat, state.gps.lon]);
  if (state && state.route) state.route.forEach((p) => pts.push(p));
  mctx.clearRect(0, 0, map.width, map.height);
  if (!pts.length) { mctx.fillStyle = "#9aa0c3"; mctx.fillText("no positions yet", 10, 20); return; }
  const la = pts.map((p) => p[0]), lo = pts.map((p) => p[1]);
  const c = Math.cos((Math.min(...la) + Math.max(...la)) / 2 * Math.PI / 180);
  const minx = Math.min(...lo) * c, maxx = Math.max(...lo) * c, miny = Math.min(...la), maxy = Math.max(...la);
  const s = Math.min((map.width - 30) / Math.max(maxx - minx, 1e-5), (map.height - 30) / Math.max(maxy - miny, 1e-5));
  const P = (lat, lon) => [15 + (lon * c - minx) * s, map.height - 15 - (lat - miny) * s];
  if (state && state.route && state.route.length > 1) {
    mctx.strokeStyle = "#5fd4ff"; mctx.lineWidth = 3; mctx.beginPath();
    state.route.forEach((p, i) => { const [x, y] = P(p[0], p[1]); i ? mctx.lineTo(x, y) : mctx.moveTo(x, y); }); mctx.stroke();
  }
  mctx.fillStyle = "#ffd23f"; mctx.font = "12px system-ui";
  wps.filter((w) => !w.action).forEach((w, i) => { const [x, y] = P(w.lat, w.lon); mctx.beginPath(); mctx.arc(x, y, 6, 0, 7); mctx.fill(); mctx.fillText(i + 1, x + 8, y - 6); });
  if (state && state.gps) { const [x, y] = P(state.gps.lat, state.gps.lon); mctx.fillStyle = "#3ddc84"; mctx.beginPath(); mctx.arc(x, y, 7, 0, 7); mctx.fill(); }
}

// ------------------------------------------------------------------ HUD + telemetry
const hud = $("#hud"), hctx = hud.getContext("2d");
function render() {
  const t = state.telemetry;
  $("#state").textContent = t.state_name; $("#mode").textContent = state.mode + (state.action ? " · " + state.action : "");
  const b = $("#batt"); b.textContent = t.battery_v.toFixed(1) + " V"; b.className = "pill" + (t.battery_low ? " bad" : t.battery_v < 22.2 ? " warn" : "");
  $("#state").className = "pill" + (t.state_name === "FALLEN" || t.estop ? " bad" : "");
  $("#status").textContent = `${state.status}  ·  ${(t.v).toFixed(2)} m/s` + (state.gps ? `  ·  ${state.gps.lat.toFixed(6)}, ${state.gps.lon.toFixed(6)}` : "");
  $("#standsit").dataset.act = state.mode === "IDLE" ? "stand" : "sit";
  $("#standsit").textContent = state.mode === "IDLE" ? "Stand" : "Sit";
  $("#crawl").classList.toggle("on", !!state.crawl);
  for (const a of state.alerts || []) if (a.t > (render.lastAlert || -1)) { render.lastAlert = a.t; toast(a.text, a.level === "error" ? "error" : ""); }
  // artificial horizon
  const w = hud.width, h = hud.height;
  hctx.clearRect(0, 0, w, h);
  hctx.save(); hctx.translate(w / 2, h / 2); hctx.rotate(-t.roll);
  hctx.strokeStyle = "#ffd23fcc"; hctx.lineWidth = 3;
  const py = t.pitch * 300;
  hctx.beginPath(); hctx.moveTo(-120, py); hctx.lineTo(-40, py); hctx.moveTo(40, py); hctx.lineTo(120, py); hctx.stroke();
  hctx.restore();
  hctx.fillStyle = "#fff"; hctx.font = "16px system-ui";
  hctx.fillText(`pitch ${(t.pitch * 57.3).toFixed(1)}°  roll ${(t.roll * 57.3).toFixed(1)}°  legs ${(t.L[0] * 100).toFixed(0)} cm`, 12, 24);
  drawRadar(); drawMap();
}
function drawRadar() {
  const c = $("#radar"), x = c.getContext("2d"), s = state.sonar || {};
  x.clearRect(0, 0, c.width, c.height);
  const cx = c.width / 2, cy = c.height - 10, R = 130;
  const sect = { fl: Math.PI / 4, f: 0, fr: -Math.PI / 4 };
  for (const [k, a] of Object.entries(sect)) {
    const r = Math.min(s[k] ?? 4, 4) / 4 * R, col = r < 0.6 / 4 * R ? "#ff4d5e" : r < 1.2 / 4 * R ? "#ffd23f" : "#3ddc84";
    x.fillStyle = col + "88"; x.beginPath(); x.moveTo(cx, cy);
    x.arc(cx, cy, r, -Math.PI / 2 - a - 0.22, -Math.PI / 2 - a + 0.22); x.closePath(); x.fill();
  }
  x.fillStyle = "#e8eaf6"; x.fillRect(cx - 8, cy - 6, 16, 12);
  if (state.cliff) { x.fillStyle = "#ff4d5e"; x.fillText("DROP-OFF!", 8, 16); }
}
setInterval(() => { if (Date.now() - lastMsg > 1500) { $("#conn").className = "dot off"; } }, 500);

drawStick(); renderWps(); connect();
if ("serviceWorker" in navigator && location.protocol.startsWith("http")) navigator.serviceWorker.register("sw.js").catch(() => {});
