# Claude conclusion: Windows Docker `localhost` UI access (port 8000)

> **Situation briefing for Claude Code.** This document exists to bring the agent fully
> up to speed on a debugging session for the `claude-visualizer-ws` project. It is
> context, not a task list. **Do not make any repository changes from this document
> alone** — read it to understand the situation, then await the user's specific commands
> for the next tasks. Sections 5 and 6 are reusable operating procedures; sections 3, 4,
> 7, 8, and 9 describe what was found, what currently exists, and what remains open.

---

## 1. Summary

The project is a Dockerized ROS 2 visualizer (ROS 2 nodes + `pylsl` + a JS/HTML/CSS
front-end served on port `8000`, WebSocket on `9090`). It was built on Ubuntu 24.04,
packaged with Docker, and run on Windows via WSL2 + Docker Desktop. On Windows the web
UI appeared unreachable at `0.0.0.0:8000` and `localhost:8000`.

Investigation found **two unrelated problems, one per run mode**. Neither was a bug in
the app itself — the server was serving correctly the whole time.

---

## 2. Outcome (TL;DR)

- **Mock mode** was already working. The real issue was the browser address:
  `localhost` resolved to IPv6 `::1` and missed the IPv4-only server. Using
  `http://127.0.0.1:8000` loads the UI. (Confirmed working by the user.)
- **Hardware mode** uses `network_mode: host`, which does not expose the port to
  Windows under Docker Desktop. A bridge network with published ports
  (`docker-compose.windows.yml`) was drafted to address this, mirroring the working mock
  file. (Drafted, not yet tested with hardware — see §7.)

---

## 3. Root causes

### Mock mode — `localhost` vs `127.0.0.1` (IPv6/IPv4)
- The container published ports correctly (`docker ps` showed `0.0.0.0:8000->8000/tcp`).
- `curl` inside WSL and `Invoke-WebRequest` from Windows both returned `200 OK` with the
  full page — so forwarding worked.
- The server binds `0.0.0.0`, which is **IPv4 only**. Browsers resolve `localhost` to
  IPv6 `::1` first; that path reached nothing, so the page looked blank.
- `127.0.0.1` forces IPv4 → works.

### Hardware mode — host networking under Docker Desktop
- Docker Desktop runs containers inside its own hidden Linux VM, not on Windows and not
  in the user's Ubuntu WSL2 distro.
- With `network_mode: host` the server binds inside that hidden VM and **no port is
  published**, so Docker Desktop never forwards anything to Windows `localhost`.
- The intended remedy: stop relying on host networking for the web service; put services
  on a bridge network and publish the ports (Docker Desktop then forwards them).

---

## 4. Key concepts (reference)

- `0.0.0.0` is a **bind** address meaning "listen on all interfaces." It is **not** a
  browsing address. The log line `Uvicorn running on http://0.0.0.0:8000` only confirms
  the server started *inside the container*; it says nothing about host reachability.
- `127.0.0.1` / `localhost` mean "this same machine." The trap on Windows is *which*
  machine — the browser is on Windows, the container is in Docker Desktop's hidden VM.
- **Host networking**: container shares the host's network. Simple on Linux; under
  Docker Desktop the "host" is the hidden VM, so the browser can't reach it.
- **Published ports** (`ports: ["8000:8000"]`): Docker Desktop forwards Windows
  `localhost:8000` → container `:8000`. This is the only path that bridges Windows to the
  container.
- **IPv6/IPv4**: an IPv4-only bind (`0.0.0.0`) is missed by an IPv6-first `localhost`
  lookup. Prefer `127.0.0.1` on Windows, or bind the server to IPv4+IPv6.

---

## 5. Operating procedure: check the USB connection (Teensy 4.1, Windows + WSL2)

Docker can only pass through a device that already exists inside WSL. On Windows, USB is
forwarded into WSL with **usbipd-win**. This must be done before starting hardware mode.

### 5.1 Attach the Teensy to WSL
In **Windows PowerShell as Administrator**:
```powershell
usbipd list
```
Find the Teensy in the list (often shown as "USB Serial Device" / PJRC) and note its
`BUSID` (e.g. `1-4`). Then:
```powershell
usbipd bind --busid <BUSID>          # one-time per device; persists across reboots
usbipd attach --wsl --busid <BUSID>  # must be repeated after each unplug or reboot
```
`bind` marks the device shareable (run once). `attach` hands it to WSL and must be redone
whenever the device is replugged or the machine reboots.

### 5.2 Confirm the device inside WSL
In the **Ubuntu terminal**:
```bash
ls -l /dev/ttyACM*          # expect /dev/ttyACM0 (or your SERIAL_PORT)
lsusb                       # Teensy should appear in the list
dmesg | tail -n 20          # shows the device registering, e.g. "cdc_acm ... ttyACM0"
```
- If `/dev/ttyACM*` is missing, the attach did not work — re-run §5.1. The container has
  nothing to talk to until this device exists.
- Make sure the path matches `SERIAL_PORT` in `.env` (default `/dev/ttyACM0`).

### 5.3 Detach when finished (optional)
```powershell
usbipd detach --busid <BUSID>
```

---

## 6. Operating procedure: run the stack

### 6.1 One-time prep
```bash
cd ~/Project/claude-visualizer-ws/docker
cp .env.example .env        # then edit if needed
```
`.env` keys:
- `SERIAL_PORT` — Teensy device path (default `/dev/ttyACM0`).
- `ROS_DOMAIN_ID` — DDS domain (default in example: `156`). **Both services must use the
  same value**, or the agent and nodes won't discover each other. They read it from the
  same `.env`, so keep one value.

### 6.2 What happens on build/run
- `colcon build` runs **at image build time** (in the Dockerfile), so `--build`
  recompiles the workspace. Omit `--build` for faster restarts when code hasn't changed.
- The entrypoint sources `/opt/ros/jazzy/setup.bash` and `/ros2_ws/install/setup.bash`,
  exports `RMW_IMPLEMENTATION` (default `rmw_cyclonedds_cpp`), activates `CYCLONEDDS_URI`
  if set, then launches `bringup.launch.py`.
- Nodes launched in **hardware** mode: `encoder_reader` (`Kalman_filter.py`),
  `web_visualizer.py`, `experiment_evaluator.py`. The `mock_encoder.py` node only runs
  when `use_mock_encoder:=true` (mock mode); in hardware mode the encoder data comes from
  the Teensy via `micro_ros_agent`.

### 6.3 Start (pick the matching mode)
```bash
cd ~/Project/claude-visualizer-ws/docker

# Always clear stale stacks first (restart: unless-stopped makes them linger)
docker compose down
docker compose -f docker-compose.mock.yml down
docker compose -f docker-compose.windows.yml down

# --- Hardware on Windows (Docker Desktop + WSL2): bridge + published ports ---
docker compose -f docker-compose.windows.yml up --build

# --- Hardware on Linux: original host-networking file ---
docker compose up --build

# --- Mock (any OS), no hardware needed ---
docker compose -f docker-compose.mock.yml up --build
```
Expected logs: the launch line, then the nodes starting, then `web_visualizer` reporting
the HTTP server on `:8000` and WebSocket on `:9090`.

### 6.4 Open the UI
- **Windows:** `http://127.0.0.1:8000`  (use `127.0.0.1`, not `localhost`)
- **Linux:** `http://localhost:8000` or `http://127.0.0.1:8000`

**Both ports matter, and they travel together.** Port `8000` serves the HTML page;
port `9090` is the WebSocket that carries live data (the "Connected/Disconnected" status
and all plotting). In `app.js` the WebSocket URL is built from the page's address bar:
`ws://${location.hostname}:9090`. So whatever host you load the page with, the WebSocket
reuses it — open `127.0.0.1:8000` and the socket goes to `127.0.0.1:9090` automatically.
Consequence: the same `127.0.0.1` rule fixes both ports. If the page is opened via
`localhost` on Windows, the WebSocket hits the same IPv6 dead-end and the UI loads but
stays "Disconnected" with empty charts — which looks like a data bug but is the same
localhost/IPv6 issue. `docker-compose.windows.yml` publishes both `8000` and `9090`.

### 6.5 Verify the Teensy data is reaching the nodes
In a second Ubuntu terminal, while the stack runs:
```bash
cd ~/Project/claude-visualizer-ws/docker
docker compose -f docker-compose.windows.yml exec ros_nodes \
  bash -lc "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic echo /encoder_raw --once"
```
- A message prints → agent ↔ nodes DDS link is alive; charts will populate.
- Hangs with no message → UI works but the DDS link did not form (see §8).

Useful extra checks: `ros2 topic list` (expect `/encoder_raw`, `/estimated_states`,
`/euler_velocity`, `/eval_live`, `/eval_summary`).

### 6.6 Stop
```bash
docker compose -f docker-compose.windows.yml down
```

---

## 7. Current repo state and open items

This records what exists and what was decided, so the agent has accurate context. None of
these are instructions to execute now — await the user's command.

- **Mock mode (`docker/docker-compose.mock.yml`)**: works as-is. Bridge network +
  published ports + `CYCLONEDDS_URI`. No change intended.
- **Linux hardware mode (`docker/docker-compose.yml`)**: works on native Linux with host
  networking. Correct for Linux; no change intended.
- **Windows hardware mode (`docker/docker-compose.windows.yml`)**: a draft was produced
  during the session (content below). Status: **drafted; may or may not exist in the repo
  yet; not verified against real Teensy hardware.** Its one unverified risk is the
  cross-engine DDS link (see §8).
- **Documentation**: README and in-repo docs currently tell users to open
  `http://localhost:8000`. On Windows this can show a blank page (IPv6 issue, §3), and
  even when the page loads, a `localhost` address makes the port-`9090` WebSocket fail
  the same way (UI stuck on "Disconnected", §6.4). A decision was reached that
  `127.0.0.1:8000` should be used/documented on Windows — this single rule covers both
  the page (`8000`) and the live-data socket (`9090`). **Not yet applied.**
- **Optional permanent `localhost` fix**: binding `web_visualizer.py` to IPv4+IPv6 would
  let `localhost` work on Windows. Note this would need to apply to **both** the HTTP
  server (`8000`) and the WebSocket server (`9090`) to be complete. **Considered, not
  implemented.**

Drafted `docker/docker-compose.windows.yml` content:
```yaml
name: claude_visualizer

# Hardware mode for Windows (Docker Desktop + WSL2).
# Bridge network + published ports so the UI reaches the Windows browser.
# Run: docker compose -f docker-compose.windows.yml up --build

services:
  micro_ros_agent:
    image: microros/micro-ros-agent:jazzy
    command: serial --dev ${SERIAL_PORT:-/dev/ttyACM0} -b 115200 -v4
    devices:
      - "${SERIAL_PORT:-/dev/ttyACM0}:${SERIAL_PORT:-/dev/ttyACM0}"
    networks:
      - ros_net
    environment:
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
    restart: unless-stopped

  ros_nodes:
    build:
      context: ..
      dockerfile: docker/Dockerfile
    image: claude_visualizer:latest
    networks:
      - ros_net
    environment:
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
      - CYCLONEDDS_URI=file:///ros2_ws/docker/cyclone_bridge.xml
    ports:
      - "8000:8000"
      - "9090:9090"
    restart: unless-stopped

networks:
  ros_net:
    driver: bridge
```

---

## 8. Known caveat and fallback

The `micro_ros_agent` image uses **Fast DDS**, while `ros_nodes` uses **CycloneDDS**.
On Linux host networking they interoperate fine. Over a Docker bridge, cross-engine DDS
discovery is the one part that can fail to connect. The §6.5 `/encoder_raw` check is the
test.

If the UI loads but `/encoder_raw` produces no message (DDS link did not form), the known
fallback is:

- Install **Docker Engine directly inside the Ubuntu WSL2 distro** (not Docker Desktop).
- Then the original host-networking `docker-compose.yml` behaves exactly as on Linux:
  agent ↔ nodes DDS works, the Teensy is available via usbipd, and the UI is reachable
  from Windows at `http://127.0.0.1:8000` (WSL2 forwards localhost to Windows).
- Note: native Docker Engine and Docker Desktop both provide `docker`; use one or the
  other to avoid conflicts.

---

## 9. Loose end (not investigated this session)

In mock mode the log showed:
```
mock_controller-1 | No executable found
mock_controller-1 exited with code 1
```
This is the `mock_controller` service's command not resolving to a built executable. It
is unrelated to the UI/networking issue and did not block the web visualizer. Open item
for later: verify the `mock_robot_controller` (or equivalent) executable name and that it
is installed by `colcon build` / declared in the package's entry points.
