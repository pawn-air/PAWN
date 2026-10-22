// Minimal dependency-free strip chart (canvas). Good enough for 10 Hz telemetry.
import { h, fitCanvas, fmt } from './core.js';

export class StripChart {
  constructor(parent, { title, color = '#22d3ee', unit = '', digits = 1, min = null, max = null, warn = null } = {}) {
    this.opt = { title, color, unit, digits, min, max, warn };
    this.cv = h('canvas');
    this.cur = h('span', { class: 'cur' });
    this.el = h('div', { class: 'chart' }, this.cv, h('span', { class: 'ttl' }, title), this.cur);
    parent.append(this.el);
  }

  draw(values, cursor = null) {
    const { w, h: H, dpr } = fitCanvas(this.cv);
    const c = this.cv.getContext('2d');
    c.clearRect(0, 0, w, H);
    const vals = (values || []).filter(v => v != null && !Number.isNaN(v));
    const last = values?.length ? values[values.length - 1] : null;
    this.cur.textContent = last == null ? '–' : `${fmt(last, this.opt.digits)} ${this.opt.unit}`;
    this.cur.style.color = this.opt.color;
    if (vals.length < 2) return;
    let lo = this.opt.min ?? Math.min(...vals), hi = this.opt.max ?? Math.max(...vals);
    if (hi - lo < 1e-6) { hi += 0.5; lo -= 0.5; }
    const pad = (hi - lo) * 0.1; lo -= this.opt.min == null ? pad : 0; hi += this.opt.max == null ? pad : 0;
    const top = 16 * dpr, bot = H - 4 * dpr;
    const y = v => bot - (v - lo) / (hi - lo) * (bot - top);
    // grid
    c.strokeStyle = '#16202b'; c.lineWidth = 1;
    for (let i = 0; i <= 3; i++) { const yy = top + (bot - top) * i / 3; c.beginPath(); c.moveTo(0, yy); c.lineTo(w, yy); c.stroke(); }
    if (this.opt.warn != null && this.opt.warn > lo && this.opt.warn < hi) {
      c.strokeStyle = '#7c2d12'; c.setLineDash([4, 4]); c.beginPath(); c.moveTo(0, y(this.opt.warn)); c.lineTo(w, y(this.opt.warn)); c.stroke(); c.setLineDash([]);
    }
    // area + line
    const n = values.length, dx = w / Math.max(1, n - 1);
    c.beginPath();
    let started = false;
    values.forEach((v, i) => { if (v == null) return; const X = i * dx, Y = y(v); if (!started) { c.moveTo(X, Y); started = true; } else c.lineTo(X, Y); });
    c.strokeStyle = this.opt.color; c.lineWidth = 1.5 * dpr; c.stroke();
    c.lineTo(w, bot); c.lineTo(0, bot); c.closePath();
    const g = c.createLinearGradient(0, top, 0, bot); g.addColorStop(0, this.opt.color + '40'); g.addColorStop(1, this.opt.color + '00');
    c.fillStyle = g; c.fill();
    c.fillStyle = '#5b6b7c'; c.font = `${9 * dpr}px ui-monospace, monospace`;
    c.fillText(fmt(hi, this.opt.digits), 4 * dpr, top + 9 * dpr); c.fillText(fmt(lo, this.opt.digits), 4 * dpr, bot - 2 * dpr);
    if (cursor != null) { c.strokeStyle = '#fbbf24'; c.lineWidth = dpr; c.beginPath(); c.moveTo(cursor * w, top); c.lineTo(cursor * w, bot); c.stroke(); }
  }
}
