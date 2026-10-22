# Tello networking notes

## Single normal Tello

* The Tello is a Wi-Fi access point (`TELLO-XXXXXX`) at **192.168.10.1**; your PC gets 192.168.10.x by DHCP.
* GCS → drone: UDP `192.168.10.1:8889` (text commands, replies come back to the sending port).
* Drone → GCS: telemetry to UDP **8890**, H.264 video to UDP **11111** (fixed on non-EDU Tellos).
* The PC has **no internet** while on the Tello Wi-Fi. The GCS UI has no CDN dependencies, so it keeps working.
* If you need internet at the same time: use Ethernet / a second adapter for internet and keep the Tello on Wi-Fi.

### Firewall

| OS | What to do |
|---|---|
| Windows | On the first start allow *Python* for **Private** networks. If you clicked away: *Windows Defender Firewall → Allow an app* → python.exe → Private + Public. The Tello network is often classified as *Public*. |
| macOS | *System Settings → Network → Firewall* → allow incoming for Python (or disable the firewall while flying). |
| Linux | `sudo ufw allow 8890/udp && sudo ufw allow 11111/udp` (if ufw is active). |

Symptoms: "SDK mode entered" but *Telemetry LOST* / no video → inbound UDP blocked.

### Other apps

Only one program can bind UDP 8890/11111. Close the Tello app / other scripts. The GCS reports "UDP bind failed" in the Link card.

## Two normal Tellos (tandem)

Normal Tellos cannot join your router (that is EDU-only station mode), and both are 192.168.10.1. Use **one Wi-Fi adapter per drone** (cheap USB dongles work):

1. Connect adapter A to `TELLO-AAAA`, adapter B to `TELLO-BBBB`.
2. Note each adapter's IP (`ipconfig` / `ifconfig` / `ip a`), e.g. 192.168.10.2 and 192.168.10.3.
3. In `config.toml` add two `[[drones]]` with `bind_ip` set to those addresses (example in the file).
4. Start the GCS; each drone has its own sockets bound to its adapter, so packets go out of the right interface and replies/state/video come back to the right socket.

Linux may still route both `192.168.10.1` destinations through one interface. Binding to the source IP is usually enough; if not, add source-based routing:

```bash
sudo ip rule add from 192.168.10.2 table 101
sudo ip route add 192.168.10.0/24 dev wlan0 src 192.168.10.2 table 101
sudo ip rule add from 192.168.10.3 table 102
sudo ip route add 192.168.10.0/24 dev wlan1 src 192.168.10.3 table 102
```

On Windows, if both adapters receive the same subnet, set the interface metric (adapter → IPv4 → Advanced) and rely on `bind_ip`; test with the Console tab (`battery?` to each drone).

Set `home_x / home_y / home_yaw` per drone (where it takes off relative to the first one) so both appear correctly on the shared map.

## Video quality

Newer firmware accepts `setbitrate 0..5`, `setresolution high|low`, `setfps high|middle|low` (try from the Console tab). Interference shows as grey smears / decode errors (Link card → decode errors). Fewer 2.4 GHz neighbours = better video.

## Keep-alive

The Tello lands by itself ~15 s after the last command. The GCS keeps it alive automatically (rc packets while flying, `command` when idle) — closing the browser does *not* stop this; the client-loss failsafe decides what happens (Settings → safety).
