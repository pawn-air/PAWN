// SDK console: raw command entry with history, completion and inline reference.
import { $, api, on, h, hhmmss, selId, store, toast } from '../core.js';

export const SDK = [
  ['command', 'enter SDK mode'], ['takeoff', 'auto takeoff'], ['land', 'auto land'], ['emergency', 'stop motors immediately'],
  ['streamon', 'enable video'], ['streamoff', 'disable video'], ['up x', 'x=20-500 cm'], ['down x', 'x=20-500 cm'],
  ['left x', 'x=20-500 cm'], ['right x', 'x=20-500 cm'], ['forward x', 'x=20-500 cm'], ['back x', 'x=20-500 cm'],
  ['cw x', 'rotate clockwise x=1-360°'], ['ccw x', 'rotate counter-clockwise x=1-360°'], ['flip x', 'l/r/f/b'],
  ['go x y z speed', 'fly to body-frame offset (cm), speed 10-100'], ['curve x1 y1 z1 x2 y2 z2 speed', 'arc through 2 points'],
  ['stop', 'hover (SDK 2.0)'], ['speed x', 'set speed 10-100 cm/s'], ['rc a b c d', 'sticks -100..100 (lr fb ud yaw)'],
  ['wifi ssid pass', 'set Wi-Fi name/password'], ['setbitrate x', '0 auto, 1-5 Mbps (fw ≥ 2.0)'], ['setresolution high|low', '720p/480p'],
  ['setfps high|middle|low', '30/15/5 fps'], ['speed?', 'current speed'], ['battery?', 'battery %'], ['time?', 'flight time'],
  ['wifi?', 'Wi-Fi SNR'], ['sdk?', 'SDK version'], ['sn?', 'serial number'], ['height?', 'height dm'], ['temp?', 'temperature'],
  ['attitude?', 'IMU attitude'], ['baro?', 'barometer m'], ['tof?', 'ToF distance mm'], ['acceleration?', 'IMU accel'],
];
$('#sdkList').replaceChildren(...SDK.map(([c, d]) => h('option', { value: c.split(' ')[0] + (c.includes(' ') ? ' ' : '') }, d)));
$('#conHelp').replaceChildren(...SDK.map(([c, d]) => h('span', { title: d, style: { marginRight: '10px', cursor: 'pointer' }, onclick: () => { $('#conIn').value = c.replace(/ (x|a b c d|x y z speed|.+)$/, ' '); $('#conIn').focus(); } }, h('code', {}, c))));

const hist = []; let hi = -1;
const log = $('#conLog');
function line(cls, ...parts) {
  log.append(h('div', { class: cls }, h('span', { class: 't' }, hhmmss(Date.now() / 1000)), ...parts));
  while (log.children.length > 800) log.firstChild.remove();
  log.scrollTop = log.scrollHeight;
}

async function send() {
  const text = $('#conIn').value.trim();
  if (!text) return;
  hist.unshift(text); hi = -1;
  $('#conIn').value = '';
  if (text.startsWith('rc ')) toast('rc from the console bypasses the dead-man — prefer the sticks', 'warn');
  try {
    if ($('#conFleet').checked) await api('/fleet/command', { text });
    else await api(`/drones/${selId()}/command`, { text, source: 'console' });
  } catch (e) { line('err', `! ${e.message}`); }
}
$('#conSend').onclick = send;
$('#conIn').addEventListener('keydown', (e) => {
  if (e.key === 'Enter') send();
  else if (e.key === 'ArrowUp') { hi = Math.min(hist.length - 1, hi + 1); if (hist[hi]) $('#conIn').value = hist[hi]; e.preventDefault(); }
  else if (e.key === 'ArrowDown') { hi = Math.max(-1, hi - 1); $('#conIn').value = hist[hi] || ''; e.preventDefault(); }
  else if (e.key === 'Tab') {
    e.preventDefault();
    const v = $('#conIn').value, m = SDK.map(s => s[0].split(' ')[0]).filter(c => c.startsWith(v));
    if (m.length === 1) $('#conIn').value = m[0] + ' ';
    else if (m.length) line('t', m.join('  '));
  }
});

on('command', (d) => {
  const dr = Object.keys(store.drones).length > 1 ? `[${d.drone}] ` : '';
  line(d.ok ? 'ok' : 'err', h('span', { class: 'tx' }, `${dr}> ${d.cmd}`), `  ${d.resp ?? 'timeout'}  `, h('span', { class: 't' }, `${Math.round((d.dt || 0) * 1000)} ms`));
});
