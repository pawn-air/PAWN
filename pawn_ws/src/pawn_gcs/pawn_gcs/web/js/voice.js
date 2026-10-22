// Voice call-outs (Web Speech API, works offline in Chrome/Edge/Safari): the pilot keeps eyes on the video
// while battery, altitude milestones, targets, failsafes and mode changes are announced - like QGroundControl.
// Toggle: 🔊 button or V. Right-click the button to switch English / Türkçe.
import { $, on, store, selId, toast } from './core.js';

const P = {
  en: {
    on: 'Voice on', bat: (b) => `Battery ${b} percent`, takeoff: 'Takeoff', landed: 'Landed', lost: 'Telemetry lost',
    restored: 'Telemetry restored', target: (l) => `Target ${l} confirmed`, collision: 'Brake. Obstacle ahead',
    fence: 'Geofence', rth: 'Returning home', rthWarn: 'Battery low for return. Return home now', mission: (n) => `Mission ${n} complete`,
    mode: (m) => `${m} mode`, emergency: 'Emergency. Motors stopped', alt: (a) => `${a} meters`, candidate: (l) => `Possible ${l}`,
  },
  tr: {
    on: 'Sesli uyarı açık', bat: (b) => `Batarya yüzde ${b}`, takeoff: 'Kalkış', landed: 'İniş tamam', lost: 'Telemetri kayboldu',
    restored: 'Telemetri geri geldi', target: (l) => `Hedef ${l} doğrulandı`, collision: 'Fren. Önde engel',
    fence: 'Sanal çit sınırı', rth: 'Eve dönülüyor', rthWarn: 'Dönüş için batarya azalıyor. Şimdi eve dön', mission: (n) => `Görev ${n} tamamlandı`,
    mode: (m) => `${m} modu`, emergency: 'Acil durum. Motorlar durdu', alt: (a) => `${a} metre`, candidate: (l) => `Olası ${l}`,
  },
};
const LBL = { en: { red_triangle: 'red triangle', person: 'person', victim: 'victim' }, tr: { red_triangle: 'kırmızı üçgen', person: 'insan', victim: 'kazazede' } };

let enabled = false, lang = (navigator.language || 'en').startsWith('tr') ? 'tr' : 'en';
try { enabled = localStorage.getItem('pawn.voice') === '1'; lang = localStorage.getItem('pawn.voiceLang') || lang; } catch {}
const last = new Map();
const btn = $('#btnVoice');
const label = (l) => LBL[lang][l] || String(l).replace(/_/g, ' ');

function render() { btn.textContent = enabled ? '🔊' : '🔇'; btn.classList.toggle('on', enabled); btn.title = `Voice call-outs (V) · ${lang === 'tr' ? 'Türkçe' : 'English'} — right-click to switch language`; }

export function say(text, { key = text, every = 6000, urgent = false } = {}) {
  if (!enabled || !('speechSynthesis' in window) || !text) return;
  const now = performance.now();
  if (now - (last.get(key) || -1e9) < every) return;
  last.set(key, now);
  const ss = window.speechSynthesis;
  if (urgent || ss.pending) ss.cancel();
  const u = new SpeechSynthesisUtterance(text);
  u.lang = lang === 'tr' ? 'tr-TR' : 'en-US';
  const v = ss.getVoices().find(x => x.lang?.toLowerCase().startsWith(lang));
  if (v) u.voice = v;
  u.rate = 1.08;
  ss.speak(u);
}

export function toggleVoice() {
  enabled = !enabled;
  try { localStorage.setItem('pawn.voice', enabled ? '1' : '0'); } catch {}
  render();
  if (enabled) { say(P[lang].on, { every: 0 }); } else window.speechSynthesis?.cancel();
}
btn.onclick = toggleVoice;
btn.oncontextmenu = (e) => {
  e.preventDefault(); lang = lang === 'tr' ? 'en' : 'tr';
  try { localStorage.setItem('pawn.voiceLang', lang); } catch {}
  render(); toast(`Voice: ${lang === 'tr' ? 'Türkçe' : 'English'}`); say(P[lang].on, { every: 0 });
};
render();

// ------------------------------------------------------------------ rules
on('voice', (t) => say(t));
on('target', (d) => say(P[lang].target(label(d.marker.label)), { key: 'target' + d.marker.id, every: 1e9, urgent: true }));
on('collision', () => say(P[lang].collision, { key: 'col', every: 3000, urgent: true }));
on('event', (e) => {
  if (e.drone && e.drone !== selId() && Object.keys(store.drones).length > 1 && e.level !== 'error') return;
  const m = e.msg || '';
  if (/^Takeoff complete/.test(m)) say(P[lang].takeoff);
  else if (/^Landed/.test(m)) say(P[lang].landed);
  else if (/EMERGENCY/.test(m)) say(P[lang].emergency, { urgent: true });
  else if (/Telemetry link LOST/.test(m)) say(P[lang].lost, { urgent: true });
  else if (/Telemetry link restored/.test(m)) say(P[lang].restored);
  else if (/GEOFENCE/.test(m)) say(P[lang].fence, { urgent: true });
  else if (/^RTH:/.test(m)) say(P[lang].rth, { urgent: true });
  else if (/^Smart RTH/.test(m)) say(P[lang].rthWarn, { urgent: true, every: 20000 });
  else if (/^Mission '(.*)' completed/.test(m)) say(P[lang].mission(m.match(/'(.*)'/)[1].replace(/_/g, ' ')));
  else if (/^Target candidate '(.*)'/.test(m)) say(P[lang].candidate(label(m.match(/'(.*)'/)[1])), { key: 'cand', every: 8000 });
  else if (/^Battery CRITICAL/.test(m)) say(m.replace(/ - .*/, ''), { urgent: true });
});
on('mode', (d) => { if (d.drone === selId()) say(P[lang].mode(d.mode.toLowerCase())); });

let lastBat = {}, lastAlt = {};
on('telemetry', (d) => {
  if (d.drone !== selId()) return;
  const b = d.state?.bat;
  if (b != null) {
    const step = Math.floor(b / 10) * 10;
    if (lastBat[d.drone] != null && step < lastBat[d.drone] && step <= 50) say(P[lang].bat(b), { key: 'bat' + step, every: 1e9 });
    lastBat[d.drone] = step;
  }
  const a = Math.round(d.odom?.z || 0);
  if (d.status?.flying && lastAlt[d.drone] != null && a !== lastAlt[d.drone] && a >= 1) say(P[lang].alt(a), { key: 'alt', every: 4000 });
  lastAlt[d.drone] = a;
});
