// MapView: dependency-free 2D (top-down) / 3D (orbit) canvas map.
// World frame (metres): x = forward at takeoff, y = left, z = up.
// Layers: coverage heat map, tracks, drones, target markers (with 1-sigma uncertainty), bookmarks,
// point clouds (SLAM), extra paths (backtrack / RTH), mission plan, geofence, cursor.
import { fitCanvas } from './core.js';

export class MapView {
  constructor(canvas, { mode = '2d', interactive = true } = {}) {
    this.cv = canvas;
    this.mode = mode;
    this.scale = 40;            // px per metre (2D, css px)
    this.center = [0, 0];       // world x,y at screen centre (2D)
    this.az = -135; this.el = 35; this.dist = 14; this.target = [0, 0, 0.5];
    this.layers = { tracks: {}, drones: {}, markers: [], clouds: {}, plan: null, planIndex: -1, fence: null, cursor: null, home: true,
      coverage: null, paths: {}, selected: null };
    this.onPick = null;          // (marker) => void, called on click near a marker
    this.follow = null;
    if (interactive) this._bind();
    const ro = new ResizeObserver(() => this.draw());
    ro.observe(canvas);
  }

  setMode(m) { this.mode = m; this.draw(); }

  fit(points) {
    const pts = points || [
      ...Object.values(this.layers.tracks).flatMap(t => t.pts),
      ...(this.layers.plan || []),
      ...this.layers.markers.map(m => [m.x, m.y, m.z]),
      [0, 0, 0],
    ];
    if (!pts.length) return;
    const xs = pts.map(p => p[0]), ys = pts.map(p => p[1]);
    const minx = Math.min(...xs), maxx = Math.max(...xs), miny = Math.min(...ys), maxy = Math.max(...ys);
    this.center = [(minx + maxx) / 2, (miny + maxy) / 2];
    this.target = [this.center[0], this.center[1], 0.5];
    const r = this.cv.getBoundingClientRect();
    const span = Math.max(maxx - minx, maxy - miny, 2) * 1.3;
    if (r.width > 20 && r.height > 20) this.scale = Math.max(4, Math.min(600, Math.min(r.width, r.height) / span));
    this.dist = span * 1.4;
    this.draw();
  }

  // ------------------------------------------------------------ interaction
  _bind() {
    const cv = this.cv;
    let drag = null;
    const pts = new Map();
    cv.addEventListener('pointerdown', (e) => { cv.setPointerCapture(e.pointerId); pts.set(e.pointerId, [e.clientX, e.clientY]); drag = { x: e.clientX, y: e.clientY, btn: e.button, shift: e.shiftKey }; });
    cv.addEventListener('pointermove', (e) => {
      if (!drag) return;
      const prev = pts.get(e.pointerId);
      pts.set(e.pointerId, [e.clientX, e.clientY]);
      if (pts.size === 2 && prev) { // pinch
        const [a, b] = [...pts.values()];
        const d1 = Math.hypot(a[0] - b[0], a[1] - b[1]);
        const others = [...pts.entries()].filter(([k]) => k !== e.pointerId)[0][1];
        const d0 = Math.hypot(prev[0] - others[0], prev[1] - others[1]);
        if (d0 > 0) this._zoom(d1 / d0);
        return;
      }
      const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
      drag.x = e.clientX; drag.y = e.clientY;
      this.follow = null;
      if (this.mode === '2d') { this.center[0] += dy / this.scale; this.center[1] += dx / this.scale; }
      else if (drag.btn === 2 || drag.shift) {
        const a = this.az * Math.PI / 180, k = this.dist / 400;
        this.target[0] += (-Math.sin(a) * dx + Math.cos(a) * dy) * k * -1;
        this.target[1] += (Math.cos(a) * dx + Math.sin(a) * dy) * k * -1;
      } else { this.az -= dx * 0.4; this.el = Math.max(5, Math.min(89, this.el + dy * 0.3)); }
      this.draw();
    });
    let down = null;
    cv.addEventListener('pointerdown', (e) => { down = [e.clientX, e.clientY]; });
    cv.addEventListener('click', (e) => {
      if (!this.onPick || !down || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 4) return;
      const r = cv.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
      const P = this._proj(cv.width, cv.height, dpr);
      let best = null, bd = 14 * dpr;
      for (const m of this.layers.markers) {
        const q = P([m.x, m.y, m.z || 0]); if (!q) continue;
        const d = Math.hypot(q[0] - (e.clientX - r.left) * dpr, q[1] - (e.clientY - r.top) * dpr);
        if (d < bd) { bd = d; best = m; }
      }
      this.onPick(best);
    });
    const up = (e) => { pts.delete(e.pointerId); if (!pts.size) drag = null; };
    cv.addEventListener('pointerup', up); cv.addEventListener('pointercancel', up);
    cv.addEventListener('contextmenu', (e) => e.preventDefault());
    cv.addEventListener('wheel', (e) => { e.preventDefault(); this._zoom(e.deltaY < 0 ? 1.15 : 1 / 1.15); }, { passive: false });
  }
  _zoom(f) {
    if (this.mode === '2d') this.scale = Math.max(4, Math.min(600, this.scale * f));
    else this.dist = Math.max(1.5, Math.min(200, this.dist / f));
    this.draw();
  }

  // ------------------------------------------------------------ projection
  _proj(w, h, dpr) {
    if (this.mode === '2d') {
      const s = this.scale * dpr, cx = w / 2, cy = h / 2, [c0, c1] = this.center;
      return (p) => [cx - (p[1] - c1) * s, cy - (p[0] - c0) * s, 1];
    }
    const a = this.az * Math.PI / 180, e = this.el * Math.PI / 180;
    const [tx, ty, tz] = this.target;
    const eye = [tx + this.dist * Math.cos(e) * Math.cos(a), ty + this.dist * Math.cos(e) * Math.sin(a), tz + this.dist * Math.sin(e)];
    const f = norm(sub([tx, ty, tz], eye)), r = norm(cross(f, [0, 0, 1])), u = cross(r, f);
    const foc = h * 0.9;
    return (p) => {
      const d = sub(p, eye), z = dot(d, f);
      if (z < 0.1) return null;
      return [w / 2 + dot(d, r) / z * foc, h / 2 - dot(d, u) / z * foc, z];
    };
  }

  // ------------------------------------------------------------ draw
  draw() {
    if (this._raf) return;
    this._raf = requestAnimationFrame(() => { this._raf = null; this._draw(); });
  }

  _draw() {
    const { w, h, dpr } = fitCanvas(this.cv);
    if (w < 4 || h < 4) return; // hidden pane
    const c = this.cv.getContext('2d');
    c.clearRect(0, 0, w, h);
    if (this.follow && this.layers.drones[this.follow]) {
      const p = this.layers.drones[this.follow].pose;
      if (this.mode === '2d') this.center = [p.x, p.y]; else this.target = [p.x, p.y, p.z];
    }
    const P = this._proj(w, h, dpr);
    const line = (pts, color, width = 1, dash = null) => {
      c.strokeStyle = color; c.lineWidth = width * dpr; c.setLineDash(dash ? dash.map(x => x * dpr) : []);
      c.beginPath(); let pen = false;
      for (const p of pts) { const q = P(p); if (!q) { pen = false; continue; } pen ? c.lineTo(q[0], q[1]) : c.moveTo(q[0], q[1]); pen = true; }
      c.stroke(); c.setLineDash([]);
    };
    // grid
    const span = this.mode === '2d' ? Math.min(400, Math.ceil(Math.max(w, h) / (this.scale * dpr) / 2) + 2) : 15;
    const step = this.mode === '2d' && this.scale < 12 ? 5 : 1;
    const gx0 = Math.floor((this.mode === '2d' ? this.center[0] : this.target[0]) - span), gy0 = Math.floor((this.mode === '2d' ? this.center[1] : this.target[1]) - span);
    for (let i = 0; i <= span * 2; i += step) {
      const gx = Math.round((gx0 + i) / step) * step, gy = Math.round((gy0 + i) / step) * step;
      const major = gx % 5 === 0;
      line([[gx, gy0, 0], [gx, gy0 + span * 2, 0]], major ? '#1d2a37' : '#121a23');
      line([[gx0, gy, 0], [gx0 + span * 2, gy, 0]], gy % 5 === 0 ? '#1d2a37' : '#121a23');
    }
    // axes at home
    line([[0, 0, 0], [1, 0, 0]], '#ef4444', 2); line([[0, 0, 0], [0, 1, 0]], '#22c55e', 2);
    if (this.mode === '3d') line([[0, 0, 0], [0, 0, 1]], '#3b82f6', 2);
    const home = P([0, 0, 0]);
    if (home) { c.fillStyle = '#94a3b8'; c.font = `${10 * dpr}px ui-monospace, monospace`; c.fillText('H', home[0] + 4 * dpr, home[1] + 12 * dpr); }

    // coverage heat map (cells the camera has swept)
    const cov = this.layers.coverage;
    if (cov && cov.cells?.size) {
      const r = cov.res;
      for (const [k, n] of cov.cells) {
        const [i, j] = k.split(',').map(Number);
        const a = Math.min(0.55, 0.12 + n * 0.035);
        c.fillStyle = `rgba(34,197,94,${a})`;
        const q = [P([i * r, j * r, 0]), P([(i + 1) * r, j * r, 0]), P([(i + 1) * r, (j + 1) * r, 0]), P([i * r, (j + 1) * r, 0])];
        if (q.some(x => !x)) continue;
        c.beginPath(); c.moveTo(q[0][0], q[0][1]); for (let t = 1; t < 4; t++) c.lineTo(q[t][0], q[t][1]); c.closePath(); c.fill();
      }
    }
    // geofence
    const f = this.layers.fence;
    if (f) {
      const ring = (z) => Array.from({ length: 73 }, (_, i) => [(f.cx || 0) + f.r * Math.cos(i * Math.PI / 36), (f.cy || 0) + f.r * Math.sin(i * Math.PI / 36), z]);
      line(ring(0), 'rgba(245,158,11,.6)', 1.5, [6, 4]);
      if (this.mode === '3d') { line(ring(f.alt), 'rgba(245,158,11,.35)', 1, [4, 4]); }
    }
    // clouds
    for (const cl of Object.values(this.layers.clouds)) {
      c.fillStyle = cl.color || '#94a3b8';
      for (const p of cl.points) { const q = P(p); if (q) c.fillRect(q[0] - dpr, q[1] - dpr, 2 * dpr, 2 * dpr); }
    }
    // plan
    if (this.layers.plan?.length) {
      line(this.layers.plan, '#a78bfa', 2, [6, 5]);
      this.layers.plan.forEach((p, i) => {
        const q = P(p); if (!q) return;
        c.fillStyle = i === this.layers.planIndex ? '#fbbf24' : '#a78bfa';
        c.beginPath(); c.arc(q[0], q[1], 3.5 * dpr, 0, 7); c.fill();
      });
    }
    // tracks
    for (const t of Object.values(this.layers.tracks)) {
      if (t.pts.length > 1) {
        if (this.mode === '3d') line(t.pts.map(p => [p[0], p[1], 0]), 'rgba(148,163,184,.25)', 1); // ground shadow
        line(t.pts, t.color, 2);
      }
    }
    // extra paths (backtrack route, RTH preview ...)
    for (const p of Object.values(this.layers.paths || {})) {
      if (!p?.pts?.length) continue;
      line(p.pts, p.color || '#fbbf24', p.width || 3, p.dash || null);
      const end = P(p.pts[p.pts.length - 1]);
      if (end && p.arrow !== false) { c.fillStyle = p.color || '#fbbf24'; c.beginPath(); c.arc(end[0], end[1], 4 * dpr, 0, 7); c.fill(); }
    }
    // markers: localised targets (triangle, dashed 1-sigma circle), bookmarks (flag), api/SLAM markers
    for (const m of this.layers.markers) {
      const base = P([m.x, m.y, 0]), top = P([m.x, m.y, m.z || 0.01]);
      if (!top) continue;
      const isBm = m.source === 'bookmark';
      const conf = m.status === 'confirmed', sel = this.layers.selected === m.id;
      const col = isBm ? '#60a5fa' : m.source === 'manual' ? '#fbbf24' : '#f43f5e';
      if (!isBm && m.sigma > 0.05) {
        const ring = Array.from({ length: 37 }, (_, i) => [m.x + m.sigma * Math.cos(i * Math.PI / 18), m.y + m.sigma * Math.sin(i * Math.PI / 18), this.mode === '3d' ? 0 : m.z]);
        line(ring, col + 'aa', 1.2, [3, 3]);
      }
      if (!isBm && m.method === 'bearing' && m.seen_from && m.bearing != null) { // bearing-only: draw the ray
        const b = m.bearing * Math.PI / 180, [sx, sy, sz] = m.seen_from;
        line([[sx, sy, sz], [sx + 6 * Math.cos(b), sy + 6 * Math.sin(b), sz]], col + '99', 1.5, [3, 3]);
      }
      if (base && this.mode === '3d') line([[m.x, m.y, 0], [m.x, m.y, m.z]], col + '88', 1);
      c.lineWidth = (sel ? 2.5 : 1) * dpr; c.strokeStyle = sel ? '#fff' : '#000';
      if (isBm) {
        c.fillStyle = col; c.beginPath(); c.moveTo(top[0], top[1]); c.lineTo(top[0], top[1] - 14 * dpr); c.lineTo(top[0] + 9 * dpr, top[1] - 10 * dpr); c.lineTo(top[0], top[1] - 6 * dpr); c.closePath(); c.fill(); c.stroke();
      } else {
        c.fillStyle = conf ? col : 'rgba(0,0,0,.4)';
        c.beginPath(); c.moveTo(top[0], top[1] - 9 * dpr); c.lineTo(top[0] - 6 * dpr, top[1] + 3 * dpr); c.lineTo(top[0] + 6 * dpr, top[1] + 3 * dpr); c.closePath(); c.fill();
        c.strokeStyle = sel ? '#fff' : col; c.stroke();
      }
      c.fillStyle = isBm ? '#bfdbfe' : '#fecdd3'; c.font = `${conf || sel ? '600 ' : ''}${10 * dpr}px system-ui, sans-serif`;
      c.fillText(`${m.label}${m.hits > 1 ? ' ×' + m.hits : ''}${m.status === 'tentative' ? ' ?' : ''}`, top[0] + 8 * dpr, top[1]);
    }
    // cursor (review)
    if (this.layers.cursor) {
      const q = P(this.layers.cursor);
      if (q) { c.strokeStyle = '#fbbf24'; c.lineWidth = 2 * dpr; c.beginPath(); c.arc(q[0], q[1], 7 * dpr, 0, 7); c.stroke(); }
    }
    // drones
    for (const [id, d] of Object.entries(this.layers.drones)) {
      const p = d.pose; if (!p) continue;
      const q = P([p.x, p.y, p.z]); if (!q) continue;
      const yaw = p.yaw * Math.PI / 180, L = 0.45;
      const nose = P([p.x + L * Math.cos(yaw), p.y + L * Math.sin(yaw), p.z]);
      const l = P([p.x + L * 0.6 * Math.cos(yaw + 2.5), p.y + L * 0.6 * Math.sin(yaw + 2.5), p.z]);
      const r = P([p.x + L * 0.6 * Math.cos(yaw - 2.5), p.y + L * 0.6 * Math.sin(yaw - 2.5), p.z]);
      if (this.mode === '3d') line([[p.x, p.y, 0], [p.x, p.y, p.z]], d.color + '88', 1, [2, 3]);
      // camera FOV wedge
      const fov = 35 * Math.PI / 180, FL = 2.2;
      const f1 = P([p.x + FL * Math.cos(yaw + fov), p.y + FL * Math.sin(yaw + fov), p.z]);
      const f2 = P([p.x + FL * Math.cos(yaw - fov), p.y + FL * Math.sin(yaw - fov), p.z]);
      if (f1 && f2) {
        c.fillStyle = d.color + '22'; c.beginPath(); c.moveTo(q[0], q[1]); c.lineTo(f1[0], f1[1]); c.lineTo(f2[0], f2[1]); c.closePath(); c.fill();
      }
      if (nose && l && r) {
        c.fillStyle = d.color; c.strokeStyle = '#000'; c.lineWidth = dpr;
        c.beginPath(); c.moveTo(nose[0], nose[1]); c.lineTo(l[0], l[1]); c.lineTo(q[0], q[1]); c.lineTo(r[0], r[1]); c.closePath(); c.fill(); c.stroke();
      }
      c.fillStyle = d.color; c.font = `600 ${10 * dpr}px ui-monospace, monospace`;
      c.fillText(d.label || id, q[0] + 10 * dpr, q[1] - 8 * dpr);
    }
    // scale bar (2D)
    if (this.mode === '2d') {
      const m = this.scale >= 60 ? 1 : this.scale >= 15 ? 5 : 10;
      const L = m * this.scale * dpr;
      c.strokeStyle = '#94a3b8'; c.lineWidth = 2 * dpr;
      c.beginPath(); c.moveTo(10 * dpr, h - 12 * dpr); c.lineTo(10 * dpr + L, h - 12 * dpr); c.stroke();
      c.fillStyle = '#94a3b8'; c.font = `${10 * dpr}px ui-monospace, monospace`; c.fillText(`${m} m`, 12 * dpr, h - 18 * dpr);
    }
  }
}

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (a) => { const l = Math.hypot(...a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };
