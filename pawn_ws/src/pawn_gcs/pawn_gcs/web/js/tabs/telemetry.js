// Telemetry tab: one chart per state field (values as received, drone units) + the raw packet stream.
import { $, store, on, h, hhmmss, selId } from '../core.js';
import { StripChart } from '../chart.js';

const COLORS = ['#22d3ee', '#a3e635', '#fbbf24', '#f472b6', '#c084fc', '#38bdf8', '#f97316', '#4ade80'];
let charts = {};

// one chart per field the drone actually sends (emitted by main.js when the field set changes)
on('fields', (keys) => {
  const meta = store.fieldMeta;
  $('#teleCharts').replaceChildren();
  charts = {};
  keys.forEach((k, i) => {
    charts[k] = new StripChart($('#teleCharts'), { title: `${k}  ${meta[k]?.unit ?? ''}`, color: COLORS[i % COLORS.length], digits: 2 });
  });
});

const RAW_LINES = 200;
const rawLines = [];
let lastDraw = 0;
on('telemetry', (d) => {
  if (d.drone !== selId()) return;
  if (!$('#rawPause').checked) {
    rawLines.unshift(`${hhmmss(d.t)}.${String(Math.floor((d.t % 1) * 1000)).padStart(3, '0')}  ${d.raw}`);
    if (rawLines.length > RAW_LINES) rawLines.pop();
  }
  const now = performance.now();
  if (now - lastDraw < 100 || !$('#pane-telemetry').classList.contains('active')) return;
  lastDraw = now;
  const hh = store.hist[d.drone] || {};
  for (const k in charts) charts[k].draw(hh[k] || []);
  if (!$('#rawPause').checked) $('#rawStream').textContent = rawLines.join('\n');
});
on('select', () => { rawLines.length = 0; $('#rawStream').textContent = ''; });
