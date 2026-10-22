"""Self-contained post-flight report (HTML, print-to-PDF friendly) for one recorded session.

Served at ``/api/recordings/<session>/report.html``; relative links resolve to the
session's own files (snapshots, video, dataset).
"""
from __future__ import annotations

import csv
import html
import json
import math
from datetime import datetime
from pathlib import Path


def _f(v, d=0.0):
    try:
        return float(v)
    except (TypeError, ValueError):
        return d


def _load(folder: Path) -> dict:
    meta = json.loads((folder / "meta.json").read_text("utf-8")) if (folder / "meta.json").exists() else {}
    rows = []
    if (folder / "telemetry.csv").exists():
        with (folder / "telemetry.csv").open(encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
    events = []
    if (folder / "events.jsonl").exists():
        for line in (folder / "events.jsonl").read_text("utf-8").splitlines():
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    targets = json.loads((folder / "targets.json").read_text("utf-8")) if (folder / "targets.json").exists() else []
    snaps = sorted(p.name for p in (folder / "snapshots").glob("*.jpg")) if (folder / "snapshots").exists() else []
    kfs = len(list((folder / "keyframes").glob("*.jpg"))) if (folder / "keyframes").exists() else 0
    return {"meta": meta, "rows": rows, "events": events, "targets": targets, "snaps": snaps, "kfs": kfs}


def _svg_track(rows, targets, W=520, H=420) -> str:
    pts = [(_f(r["x"]), _f(r["y"])) for r in rows if r.get("x") not in (None, "")]
    allp = pts + [(t["x"], t["y"]) for t in targets] + [(0.0, 0.0)]
    xs, ys = [p[0] for p in allp], [p[1] for p in allp]
    span = max(max(xs) - min(xs), max(ys) - min(ys), 2.0) * 1.15
    cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
    k = min(W, H) / span

    def P(x, y):  # x forward = up, y left = left
        return W / 2 - (y - cy) * k, H / 2 - (x - cx) * k

    grid = []
    step = 1 if span < 15 else 5
    for g in range(math.floor(cx - span), math.ceil(cx + span) + 1, step):
        a, b = P(g, cy - span), P(g, cy + span)
        grid.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" class="g"/>')
    for g in range(math.floor(cy - span), math.ceil(cy + span) + 1, step):
        a, b = P(cx - span, g), P(cx + span, g)
        grid.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" class="g"/>')
    path = " ".join(f"{P(x, y)[0]:.1f},{P(x, y)[1]:.1f}" for x, y in pts[::max(1, len(pts) // 1500)])
    hx, hy = P(0, 0)
    tg = []
    for t in targets:
        x, y = P(t["x"], t["y"])
        r = max(4.0, t.get("sigma", 0) * k)
        tg.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{r:.1f}" class="unc"/>'
                  f'<path d="M{x:.1f},{y - 8:.1f} L{x - 6:.1f},{y + 4:.1f} L{x + 6:.1f},{y + 4:.1f}Z" class="tg"/>'
                  f'<text x="{x + 9:.1f}" y="{y:.1f}" class="lbl">{html.escape(str(t.get("label")))} #{t.get("id")}</text>')
    return (f'<svg viewBox="0 0 {W} {H}" class="map" role="img" aria-label="Trajectory top view">'
            f'{"".join(grid)}<polyline points="{path}" class="trk"/>'
            f'<rect x="{hx - 5:.1f}" y="{hy - 5:.1f}" width="10" height="10" class="home"/>'
            f'<text x="{hx + 8:.1f}" y="{hy + 14:.1f}" class="lbl">H</text>{"".join(tg)}'
            f'<text x="8" y="{H - 8}" class="lbl">grid {step} m · up = forward at takeoff</text></svg>')


def _svg_series(rows, key, scale=1.0, label="", unit="", W=520, H=120) -> str:
    data = [(_f(r.get("t_rel")), _f(r.get(key)) * scale) for r in rows if r.get(key) not in (None, "")]
    if len(data) < 2:
        return ""
    data = data[::max(1, len(data) // 800)]
    t0, t1 = data[0][0], data[-1][0] or 1
    lo, hi = min(v for _, v in data), max(v for _, v in data)
    if hi - lo < 1e-6:
        hi = lo + 1
    pts = " ".join(f"{(t - t0) / max(t1 - t0, 1e-6) * (W - 50) + 44:.1f},{H - 18 - (v - lo) / (hi - lo) * (H - 30):.1f}"
                   for t, v in data)
    return (f'<figure><figcaption>{html.escape(label)}</figcaption><svg viewBox="0 0 {W} {H}" class="ser">'
            f'<text x="2" y="16" class="lbl">{hi:.1f}{unit}</text><text x="2" y="{H - 18}" class="lbl">{lo:.1f}{unit}</text>'
            f'<line x1="44" y1="{H - 18}" x2="{W - 6}" y2="{H - 18}" class="g"/>'
            f'<polyline points="{pts}" class="line"/><text x="{W - 60}" y="{H - 4}" class="lbl">{t1 - t0:.0f} s</text></svg></figure>')


def build_report(folder: Path) -> str:
    d = _load(Path(folder))
    m, rows = d["meta"], d["rows"]
    dur = m.get("duration_s") or (_f(rows[-1]["t_rel"]) if rows else 0)
    flying = [r for r in rows if r.get("flying") == "1"]
    max_alt = max((_f(r.get("z")) for r in rows), default=0)
    max_rng = max((math.hypot(_f(r.get("x")), _f(r.get("y"))) for r in rows), default=0)
    temps = [_f(r.get("temph")) for r in rows if r.get("temph")]
    warn = [e for e in d["events"] if e.get("type") == "event" and e.get("level") in ("warn", "error", "success")]
    modes = sorted({r.get("mode") for r in rows if r.get("mode")})
    start = datetime.fromtimestamp(m["start"]).strftime("%Y-%m-%d %H:%M:%S") if m.get("start") else "?"
    stats = [("Drone", f'{m.get("name", "")} ({m.get("drone", "")})'), ("Start", start),
             ("Duration", f"{dur // 60:.0f} min {dur % 60:.0f} s"), ("Airborne samples", f"{len(flying)} / {len(rows)}"),
             ("Distance flown", f'{_f(m.get("dist_m")):.1f} m'), ("Max altitude", f"{max_alt:.2f} m"),
             ("Max range from home", f"{max_rng:.1f} m"),
             ("Battery", f'{m.get("bat_start", "?")}% → {m.get("bat_end", "?")}%'),
             ("Max temperature", f"{max(temps):.0f} °C" if temps else "–"), ("Modes", ", ".join(modes) or "–"),
             ("Targets", str(len([t for t in d["targets"] if t.get("status") == "confirmed"])) + " confirmed / "
              + str(len(d["targets"])) + " total"),
             ("3D dataset", f'{d["kfs"]} keyframes' + (" · transforms.json + COLMAP" if d["kfs"] else "")),
             ("SDK / serial", f'{(m.get("info") or {}).get("sdk", "?")} / {(m.get("info") or {}).get("sn", "?")}')]
    trows = "".join(
        f'<tr><td>#{t.get("id")}</td><td>{html.escape(str(t.get("label")))}</td><td>{t.get("status", "")}</td>'
        f'<td>{t["x"]:.2f}, {t["y"]:.2f}, {t["z"]:.2f}</td><td>±{_f(t.get("sigma")):.2f} m</td>'
        f'<td>{html.escape(str(t.get("method", "")))}</td><td>{t.get("hits", 1)}</td></tr>' for t in d["targets"])
    t0 = m.get("start", 0)
    evs = "".join(f'<tr class="{e.get("level")}"><td>{_f(e.get("t")) - t0:7.1f} s</td><td>{e.get("level")}</td>'
                  f'<td>{html.escape(str(e.get("msg", "")))}</td></tr>' for e in warn[:400])
    snaps = "".join(f'<a href="snapshots/{s}"><img src="snapshots/{s}" alt="{s}" loading="lazy"></a>' for s in d["snaps"][:40])
    video = '<p><a href="video.mp4">▶ video.mp4</a></p>' if (Path(folder) / "video.mp4").exists() else ""
    stat_html = "".join(f"<div><span>{k}</span><b>{html.escape(str(v))}</b></div>" for k, v in stats)
    return f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>PAWN flight report</title>
<style>
:root{{--bg:#fff;--fg:#111827;--mut:#6b7280;--line:#e5e7eb;--acc:#0891b2;--tg:#e11d48;--card:#f9fafb}}
@media (prefers-color-scheme:dark){{:root{{--bg:#0b0f14;--fg:#e5e7eb;--mut:#94a3b8;--line:#1f2a37;--acc:#22d3ee;--tg:#fb7185;--card:#111821}}}}
body{{background:var(--bg);color:var(--fg);font:14px/1.5 system-ui,sans-serif;margin:0 auto;max-width:1100px;padding:24px 16px}}
h1{{font-size:22px;margin:0 0 4px}} h2{{font-size:15px;margin:28px 0 8px;text-transform:uppercase;letter-spacing:.06em;color:var(--mut)}}
.stats{{display:grid;grid-template-columns:repeat(auto-fill,minmax(220px,1fr));gap:8px}}
.stats div{{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:8px 10px}}
.stats span{{display:block;color:var(--mut);font-size:12px}} .grid2{{display:grid;grid-template-columns:1fr 1fr;gap:16px}}
@media (max-width:760px){{.grid2{{grid-template-columns:1fr}}}}
svg{{width:100%;height:auto;background:var(--card);border:1px solid var(--line);border-radius:8px}}
.g{{stroke:var(--line);stroke-width:1}} .trk,.line{{fill:none;stroke:var(--acc);stroke-width:2}}
.home{{fill:var(--mut)}} .tg{{fill:var(--tg)}} .unc{{fill:none;stroke:var(--tg);stroke-dasharray:3 3;opacity:.7}}
.lbl{{fill:var(--mut);font:11px ui-monospace,monospace}} figure{{margin:0 0 10px}} figcaption{{font-size:12px;color:var(--mut)}}
table{{border-collapse:collapse;width:100%;font-size:13px}} td,th{{border-bottom:1px solid var(--line);padding:4px 6px;text-align:left}}
tr.error td{{color:var(--tg)}} tr.success td{{color:var(--acc)}} .snaps{{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:6px}}
.snaps img{{width:100%;border-radius:6px}} .muted{{color:var(--mut)}} @media print{{body{{max-width:none}}}}
</style></head><body>
<h1>PAWN flight report</h1><div class="muted">{html.escape(Path(folder).name)} · generated {datetime.now():%Y-%m-%d %H:%M}</div>
<h2>Summary</h2><div class="stats">{stat_html}</div>
<h2>Trajectory &amp; targets</h2><div class="grid2"><div>{_svg_track(rows, d["targets"])}</div><div>
{_svg_series(rows, "z", 1, "Altitude (estimated)", " m")}{_svg_series(rows, "bat", 1, "Battery", "%")}
{_svg_series(rows, "speed", 1, "Ground speed", " m/s")}{_svg_series(rows, "temph", 1, "Temperature", "°")}</div></div>
<h2>Targets</h2>{"<table><tr><th>ID</th><th>Label</th><th>Status</th><th>Position (m)</th><th>1σ</th><th>Method</th><th>Hits</th></tr>" + trows + "</table>" if trows else '<p class="muted">No targets in this flight.</p>'}
<h2>Events (warnings, errors, successes)</h2>{"<table>" + evs + "</table>" if evs else '<p class="muted">None.</p>'}
<h2>Media</h2>{video}<div class="snaps">{snaps or '<p class="muted">No snapshots.</p>'}</div>
<p class="muted">Positions are GCS estimates (dead reckoning unless an external SLAM pose was posted). x = forward at takeoff, y = left, z = up.</p>
</body></html>"""
