"""Drive the GCS from a script (REST + WebSocket) - e.g. for experiments or CI.

    python -m pawn_gcs --sim            # in another terminal
    python examples/api_client.py --drone sim1

Only uses the standard library + `websockets` (installed with uvicorn[standard]).
"""
import argparse
import asyncio
import json
import urllib.request

import websockets


def call(base, path, body=None):
    req = urllib.request.Request(base + path, method="POST" if body is not None else "GET",
                                 data=json.dumps(body).encode() if body is not None else None,
                                 headers={"content-type": "application/json"})
    with urllib.request.urlopen(req, timeout=60) as r:
        return json.loads(r.read())


async def watch_events(ws_url):
    async with websockets.connect(ws_url) as ws:
        async for raw in ws:
            msg = json.loads(raw)
            items = msg["data"] if msg["topic"] == "batch" else [msg]
            for it in items:
                if it.get("topic") == "event":
                    d = it["data"]
                    print(f"  [{d.get('level')}] {d.get('drone') or '-'}: {d['msg']}")


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:8080")
    ap.add_argument("--drone", default="sim1")
    a = ap.parse_args()
    base = a.url + "/api"
    watcher = asyncio.create_task(watch_events(a.url.replace("http", "ws") + "/ws"))

    print("status:", [d["id"] for d in call(base, "/status")["drones"]])
    print("preflight blockers:", call(base, f"/drones/{a.drone}/preflight")["blockers"])
    mission = {"name": "api_demo", "steps": [
        {"type": "takeoff"}, {"type": "move", "dir": "forward", "cm": 100},
        {"type": "rotate", "dir": "ccw", "deg": 180}, {"type": "move", "dir": "forward", "cm": 100},
        {"type": "snapshot"}, {"type": "land"}]}
    print("preview:", {k: v for k, v in call(base, "/missions/preview", mission).items() if k != "points"})
    call(base, f"/drones/{a.drone}/mission/start", mission)
    while True:
        await asyncio.sleep(1)
        m = [d for d in call(base, "/status")["drones"] if d["id"] == a.drone][0]["mission"]
        print(f"mission: {m['state']} step {m['step'] + 1}/{m['total']}")
        if m["state"] not in ("running", "paused"):
            break
    watcher.cancel()


if __name__ == "__main__":
    asyncio.run(main())
