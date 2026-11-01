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


async def watch(ws_url):
    """Print every command reply, drone message and event the GCS publishes."""
    async with websockets.connect(ws_url) as ws:
        async for raw in ws:
            msg = json.loads(raw)
            for it in msg["data"] if msg["topic"] == "batch" else [msg]:
                d = it.get("data", {})
                if it.get("topic") == "command":
                    print(f"  > {d['cmd']}  <  {d['resp']}  ({d['dt'] * 1000:.0f} ms)")
                elif it.get("topic") in ("event", "tello"):
                    print(f"  [{it['topic']}] {d.get('drone') or '-'}: {d['msg']}")


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:8080")
    ap.add_argument("--drone", default="sim1")
    a = ap.parse_args()
    base = a.url + "/api"
    watcher = asyncio.create_task(watch(a.url.replace("http", "ws") + "/ws"))
    print("drones:", [d["id"] for d in call(base, "/status")["drones"]])
    for text in ("takeoff", "forward 100", "ccw 180", "forward 100", "land"):
        await asyncio.to_thread(call, base, f"/drones/{a.drone}/command", {"text": text})
    await asyncio.sleep(1)
    watcher.cancel()


if __name__ == "__main__":
    asyncio.run(main())
