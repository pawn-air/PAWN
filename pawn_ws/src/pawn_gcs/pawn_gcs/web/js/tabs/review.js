// Flight review: recorded sessions - files to download, video, snapshots, and the logged commands / drone messages / events.
import { $, api, on, h, fmt, hhmmss, toast } from '../core.js';

let cur = null;

async function load() {
  const sessions = await api('/recordings');
  $('#sessions').replaceChildren(...(sessions.length ? sessions.map(s => h('button', {
    class: `sess${cur?.id === s.id ? ' active' : ''}`, onclick: () => open(s),
  }, h('b', {}, s.id), `${s.open ? 'recording…' : fmt(s.meta.duration_s, 0) + ' s'} · ${s.files.includes('video.mp4') ? '🎞 ' : ''}${s.snapshots.length ? '📷' + s.snapshots.length + ' ' : ''}· ${(s.size / 1e6).toFixed(1)} MB`))
    : [h('div', { class: 'small muted' }, 'No sessions yet. A log runs while a drone is connected.')]));
}

function evText(e) {
  if (e.type === 'command') return `> ${e.cmd}  <  ${e.resp ?? 'no reply (timeout)'}  ${Math.round((e.dt || 0) * 1000)} ms`;
  if (e.type === 'tello') return `drone: ${e.msg}`;
  return e.msg ?? JSON.stringify(e);
}

async function open(s) {
  cur = s; load();
  const base = `/api/recordings/${encodeURIComponent(s.id)}/`;
  const body = $('#reviewBody'); body.classList.remove('muted');
  let events = [];
  try { events = (await (await fetch(base + 'events.jsonl')).text()).trim().split('\n').filter(Boolean).map(l => JSON.parse(l)); } catch {}
  const t0 = s.meta.start || events[0]?.t || 0;
  body.replaceChildren(...[
    h('div', { class: 'row gap wrap small', style: { marginBottom: '6px' } },
      h('b', {}, s.id), h('span', { class: 'muted' }, `start ${s.meta.start ? hhmmss(s.meta.start) : '?'} · ${s.open ? 'recording…' : s.meta.duration_s + ' s'} · sdk ${s.meta.info?.sdk ?? '?'} · sn ${s.meta.info?.sn ?? '?'}`),
      h('span', { class: 'grow' }),
      ...s.files.map(f => h('a', { class: 'btn xs ghost', href: base + f, download: f }, '⇩ ' + f)),
      h('button', { class: 'btn xs ghost', onclick: async () => { if (confirm(`Delete ${s.id}?`)) { await api('/recordings/' + encodeURIComponent(s.id), undefined, 'DELETE'); cur = null; body.replaceChildren(); load(); } } }, 'delete')),
    s.files.includes('video.mp4') ? h('video', { class: 'rev-video', src: base + 'video.mp4', controls: true, muted: true, preload: 'metadata' }) : null,
    s.snapshots.length ? h('div', { class: 'snaps' }, ...s.snapshots.map(f => h('a', { href: base + 'snapshots/' + f, target: '_blank' }, h('img', { src: base + 'snapshots/' + f, loading: 'lazy', title: f })))) : null,
    h('div', { class: 'sub' }, `Log (${events.length} entries)`),
    h('div', { class: 'events rev-events' }, ...events.map(e => h('div', { class: `ev ${e.type === 'command' ? (e.ok ? 'cmd' : 'cmd bad') : e.type === 'tello' ? 'tello' : e.level || ''}` },
      h('span', { class: 't' }, `+${fmt(e.t - t0, 1)}s`), evText(e))))].filter(Boolean));
}

on('tab', (t) => { if (t === 'review') load().catch(e => toast(e.message, 'error')); });
on('recording', () => { if ($('#pane-review').classList.contains('active')) load(); });
