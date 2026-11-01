"""Bare Tello SDK takeoff test - no GCS involved. Close the GCS first (it uses the same port).

    python tools/raw_takeoff.py            # command -> takeoff -> hover 5 s -> land

Prints every datagram sent and received. If this takes off, a takeoff refused in the GCS
comes from something the GCS sends; if this is refused too, SDK mode itself is refused.
"""
import socket
import time

TELLO = ("192.168.10.1", 8889)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("", 8889))
s.settimeout(0.2)


def send(cmd, wait=10.0):
    t0 = time.time()
    s.sendto(cmd.encode(), TELLO)
    print(f"{t0:.3f} TX {cmd}")
    while time.time() - t0 < wait:
        try:
            data, _ = s.recvfrom(1024)
        except socket.timeout:
            continue
        txt = data.decode(errors="replace").strip()
        print(f"{time.time():.3f} RX {txt}   ({(time.time() - t0) * 1000:.0f} ms)")
        return txt
    print(f"{time.time():.3f} -- no reply")
    return None


if send("command", 5) != "ok":
    raise SystemExit("no 'ok' to 'command' - on the Tello Wi-Fi? GCS closed?")
time.sleep(1.0)
r = send("takeoff", 20)
if r == "ok":
    for _ in range(10):                       # hover 5 s, keep the 15 s no-command timer reset
        s.sendto(b"rc 0 0 0 0", TELLO)
        time.sleep(0.5)
    send("land", 20)
