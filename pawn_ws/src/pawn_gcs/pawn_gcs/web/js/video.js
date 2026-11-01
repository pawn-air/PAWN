// Live FPV video (JPEG over WebSocket). No overlay: the picture as decoded.
import { $, on } from './core.js';

const vcv = $('#video'), msg = $('#videoMsg');
const vctx = vcv.getContext('2d');
let vws = null, vid = null, lastFrameT = 0, frames = [], latency = null;

export const videoStats = () => ({ fps: frames.length, latency });

function connect(id) {
  if (vws) { vws.onclose = null; vws.close(); }
  vid = id;
  if (!id) return;
  const url = (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws/video/' + id;
  vws = new WebSocket(url);
  vws.binaryType = 'arraybuffer';
  vws.onmessage = async (ev) => {
    const buf = ev.data;
    const tMs = new DataView(buf).getFloat64(4, true);
    const bmp = await createImageBitmap(new Blob([buf.slice(12)], { type: 'image/jpeg' }));
    if (vcv.width !== bmp.width || vcv.height !== bmp.height) { vcv.width = bmp.width; vcv.height = bmp.height; }
    vctx.drawImage(bmp, 0, 0);
    bmp.close?.();
    const now = performance.now();
    lastFrameT = now;
    frames.push(now); while (frames.length && now - frames[0] > 1000) frames.shift();
    latency = Date.now() - tMs;
    msg.classList.add('hidden');
  };
  vws.onclose = () => setTimeout(() => { if (vid === id) connect(id); }, 1000);
}

on('select', connect);
setInterval(() => {
  const now = performance.now();
  while (frames.length && now - frames[0] > 1000) frames.shift();
  if (now - lastFrameT > 2000) msg.classList.remove('hidden');
}, 500);

$('#btnFull').onclick = () => {
  const el = $('#videoWrap');
  if (document.fullscreenElement) document.exitFullscreen(); else el.requestFullscreen?.();
};
