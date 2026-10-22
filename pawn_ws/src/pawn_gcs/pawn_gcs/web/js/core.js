// Core: REST helpers, WebSocket client, global store & tiny event emitter.
// Every UI module imports from here; nothing else talks to the server directly.

export const $ = (s, el = document) => el.querySelector(s);
export const $$ = (s, el = document) => [...el.querySelectorAll(s)];
export const h = (tag, attrs = {}, ...kids) => {
  const el = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs || {})) {
    if (k === 'class') el.className = v;
    else if (k === 'style' && typeof v === 'object') Object.assign(el.style, v);
    else if (k.startsWith('on') && typeof v === 'function') el.addEventListener(k.slice(2), v);
    else if (v === true) el.setAttribute(k, '');
    else if (v !== false && v != null) el.setAttribute(k, v);
  }
  for (const c of kids.flat()) if (c != null && c !== false) el.append(c instanceof Node ? c : document.createTextNode(String(c)));
  return el;
};
export const fmt = (v, d = 1) => (v == null || Number.isNaN(v) ? '–' : Number(v).toFixed(d));
export const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
export const hhmmss = (t) => { const d = new Date(t * 1000); return d.toTimeString().slice(0, 8); };
export const mmss = (s) => `${String(Math.floor(s / 60)).padStart(2, '0')}:${String(Math.floor(s % 60)).padStart(2, '0')}`;

// ---------------------------------------------------------------- emitter
const listeners = {};
export const on = (topic, fn) => { (listeners[topic] ||= []).push(fn); return () => listeners[topic] = listeners[topic].filter(f => f !== fn); };
export const emit = (topic, data) => { for (const fn of listeners[topic] || []) { try { fn(data); } catch (e) { console.error(topic, e); } } };

// ---------------------------------------------------------------- REST
export async function api(path, body, method) {
  const opt = { method: method || (body !== undefined ? 'POST' : 'GET'), headers: {} };
  if (body !== undefined) { opt.headers['content-type'] = 'application/json'; opt.body = JSON.stringify(body); }
  const r = await fetch('/api' + path, opt);
  const ct = r.headers.get('content-type') || '';
  const data = ct.includes('json') ? await r.json() : await r.text();
  if (!r.ok) throw new Error((data && data.detail) || r.statusText);
  return data;
}

// ---------------------------------------------------------------- store
const HIST = 600; // ~60 s at 10 Hz
export const store = {
  drones: {},         // id -> status (from server)
  tel: {},            // id -> latest {state, odom, t}
  hist: {},           // id -> {t:[], alt:[], bat:[], spd:[], temp:[], vz:[]}
  det: {},            // id -> {plugin -> detection}
  selected: null,
  settings: {},
  meta: {},
  sim: false,
  wsLatency: null,
  map: { markers: [], clouds: {} },
};
export const sel = () => store.drones[store.selected] || null;
export const selId = () => store.selected;

export function select(id) {
  if (!store.drones[id]) return;
  store.selected = id;
  try { localStorage.setItem('pawn.selected', id); } catch {}
  emit('select', id);
}

function pushHist(id, st, od) {
  const hh = (store.hist[id] ||= { t: [], alt: [], bat: [], spd: [], temp: [], vz: [], tof: [] });
  const now = performance.now() / 1000;
  const vals = { t: now, alt: od.z, bat: st.bat, spd: od.speed, temp: st.temph, vz: -(st.vgz || 0) / 10, tof: (st.tof || 0) / 100 };
  for (const k in vals) { hh[k].push(vals[k]); if (hh[k].length > HIST) hh[k].shift(); }
}

// ---------------------------------------------------------------- websocket
let ws = null, wsOpen = false, pingT = 0;
export const wsSend = (obj) => { if (wsOpen) ws.send(JSON.stringify(obj)); };
export const isOnline = () => wsOpen;

function handle(topic, d) {
  switch (topic) {
    case 'telemetry':
      store.tel[d.drone] = d;
      if (store.drones[d.drone]) Object.assign(store.drones[d.drone], d.status);
      pushHist(d.drone, d.state, d.odom);
      break;
    case 'link':
      store.drones[d.drone] = { ...(store.drones[d.drone] || {}), ...d };
      break;
    case 'mode':
      if (store.drones[d.drone]) store.drones[d.drone].mode = d.mode;
      break;
    case 'detections':
      (store.det[d.drone] ||= {})[d.plugin] = d;
      break;
    case 'map':
      if (d.type === 'marker') {
        const i = store.map.markers.findIndex(m => m.id === d.marker.id);
        if (i >= 0) store.map.markers[i] = d.marker; else store.map.markers.push(d.marker);
      } else if (d.type === 'marker_removed') store.map.markers = store.map.markers.filter(m => m.id !== d.id);
      else if (d.type === 'cleared') store.map = { markers: [], clouds: {} };
      else if (d.type === 'cloud') store.map.clouds[d.name] = { points: d.points, color: d.color };
      break;
  }
  emit(topic, d);
}

export function connectWS() {
  const url = (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws';
  ws = new WebSocket(url);
  ws.onopen = () => { wsOpen = true; emit('ws', true); };
  ws.onclose = () => { wsOpen = false; emit('ws', false); setTimeout(connectWS, 1000); };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    if (m.topic === 'batch') { for (const x of m.data) handle(x.topic, x.data); return; }
    if (m.topic === 'pong') { store.wsLatency = performance.now() - m.data.t; emit('latency', store.wsLatency); return; }
    if (m.topic === 'hello') {
      const d = m.data;
      store.sim = d.sim; store.settings = d.settings;
      for (const dr of d.drones) store.drones[dr.id] = dr;
      let want = null; try { want = localStorage.getItem('pawn.selected'); } catch {}
      if (!store.selected || !store.drones[store.selected]) store.selected = store.drones[want] ? want : d.drones[0]?.id;
      emit('hello', d);
      emit('select', store.selected);
      return;
    }
    handle(m.topic, m.data);
  };
}
setInterval(() => { if (wsOpen) { pingT = performance.now(); wsSend({ type: 'ping', t: pingT }); } }, 2000);

// ---------------------------------------------------------------- commands
export async function sendCmd(text, opts = {}) {
  const id = opts.drone || store.selected;
  if (!id) return;
  try {
    if (opts.fleet) {
      const r = await api('/fleet/command', { text });
      return r;
    }
    return await api(`/drones/${id}/command`, { text, source: 'ui' });
  } catch (e) { toast(`${text}: ${e.message}`, 'error'); }
}

// ---------------------------------------------------------------- toasts
export function toast(msg, level = 'info', ms = 3500) {
  const box = document.getElementById('toasts');
  const t = h('div', { class: `toast ${level}` }, msg);
  box.append(t);
  while (box.children.length > 5) box.firstChild.remove();
  setTimeout(() => t.remove(), ms);
}

// device-pixel-ratio aware canvas sizing
export function fitCanvas(cv) {
  const r = cv.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const w = Math.max(1, Math.round(r.width * dpr)), hh = Math.max(1, Math.round(r.height * dpr));
  if (cv.width !== w || cv.height !== hh) { cv.width = w; cv.height = hh; }
  return { w, h: hh, dpr };
}
