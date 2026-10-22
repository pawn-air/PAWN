// Vision tab: enable plugins per drone, tune parameters live, see per-plugin timing/results.
// Autonomy tab: pick a controller, set its params, engage AUTO, watch its debug output.
import { $, api, store, on, h, selId, toast } from '../core.js';
import { paramForm } from '../form.js';

let plugins = [];
const live = {};

async function load() {
  plugins = await api('/vision');
  render();
}

function render() {
  const id = selId();
  $('#visionList').replaceChildren(...plugins.map(p => {
    const enabled = !!p.enabled?.[id];
    const liveEl = h('div', { class: 'live' });
    live[p.name] = liveEl;
    const card = h('div', { class: `plugin${p.available ? '' : ' na'}`, style: { '--pc': p.color } },
      h('h3', {}, h('span', {}, `${p.is_target ? '★ ' : ''}${p.title}`),
        h('label', { class: 'switch' }, h('input', {
          type: 'checkbox', checked: enabled, disabled: !p.available,
          onchange: async (e) => {
            try { await api(`/vision/${p.name}/enable`, { drone: id, on: e.target.checked }); p.enabled = { ...(p.enabled || {}), [id]: e.target.checked }; }
            catch (err) { toast(err.message, 'error'); e.target.checked = !e.target.checked; }
          },
        }), h('span', {}, 'on'))),
      h('p', {}, p.description),
      p.available ? null : h('p', { style: { color: 'var(--warn)' } }, `Unavailable: ${p.unavailable_reason}`),
      paramForm(p.params, p.values, async (k, v) => {
        p.values[k] = v;
        try { await api(`/vision/${p.name}/params`, { [k]: v }); } catch (e) { toast(e.message, 'error'); }
      }),
      h('div', { class: 'small muted' }, `max ${p.rate_hz} Hz · ${p.name}`),
      liveEl);
    return card;
  }));
}

on('detections', (d) => {
  if (d.drone !== selId()) return;
  const el = live[d.plugin];
  if (!el) return;
  const items = d.items || [];
  const summary = items.slice(0, 4).map(it => it.info ? `${it.label} ${JSON.stringify(it.info)}` : `${it.label} ${Math.round((it.conf ?? 1) * 100)}%`).join('\n');
  el.textContent = `${d.ms} ms · ${items.length} result(s)${summary ? '\n' + summary : ''}`;
});
on('hello', load);
on('select', render);
on('tab', (t) => { if (t === 'vision') load(); });

// ======================================================= autonomy
let au = { controllers: [], active: {} };
let auValues = {};

async function loadAu() {
  au = await api('/autonomy');
  const cur = au.active[selId()] || au.controllers[0]?.name;
  $('#auSel').replaceChildren(...au.controllers.map(c => h('option', { value: c.name, selected: c.name === cur }, c.title)));
  renderAu();
}
function renderAu() {
  const c = au.controllers.find(x => x.name === $('#auSel').value);
  if (!c) return;
  auValues = { ...c.values };
  $('#auDesc').textContent = c.description + (au.active[selId()] === c.name ? '  [selected for this drone]' : '');
  $('#auParams').replaceChildren(paramForm(c.params, c.values, (k, v) => { auValues[k] = v; }));
}
$('#auSel').onchange = renderAu;
$('#auApply').onclick = async () => {
  try { au = await api(`/drones/${selId()}/autonomy`, { name: $('#auSel').value, values: auValues }); toast('Controller applied', 'success'); renderAu(); }
  catch (e) { toast(e.message, 'error'); }
};
$('#auEngage').onclick = async () => {
  try {
    au = await api(`/drones/${selId()}/autonomy`, { name: $('#auSel').value, values: auValues, engage: true });
    const d = store.drones[selId()];
    if (!d?.flying) toast('Controller armed — AUTO acts only while flying', 'warn');
  } catch (e) { toast(e.message, 'error'); }
};
$('#auStop').onclick = () => api(`/drones/${selId()}/mode`, { mode: 'MANUAL' });
on('autonomy', (d) => {
  if (d.drone !== selId()) return;
  $('#auDebug').textContent = `controller: ${d.controller}\nsticks [lr fb ud yaw]: ${JSON.stringify(d.out)}\n${JSON.stringify(d.debug, null, 2)}`;
});
on('hello', loadAu);
on('select', loadAu);
