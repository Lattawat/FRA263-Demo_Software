# Claude Visualizer — Docker Guide

Run the full encoder kinematics visualization pipeline on any machine without installing ROS 2.

| OS | Mode | Browser address |
|---|---|---|
| Linux | Hardware (Teensy) | `http://localhost:8000` |
| Windows WSL2 | Hardware (Teensy) | `http://127.0.0.1:8000` |
| Windows WSL2 | Mock (no Teensy) | `http://127.0.0.1:8000` |
| macOS | Mock (no Teensy) | `http://localhost:8000` |

> **Windows note:** Always use `http://127.0.0.1:8000` — never `http://localhost:8000`.
> On Windows, browsers resolve `localhost` to IPv6 (`::1`) first, which misses the
> IPv4-only server. This same rule fixes both the page (port `8000`) and the live-data
> WebSocket (port `9090`) — so one address change covers everything.

---

## 1. Install Docker (one time per machine)

### Linux (Ubuntu)

```bash
# Install Docker Engine
curl -fsSL https://get.docker.com | sh

# Allow your user to run Docker without sudo
sudo usermod -aG docker $USER

# Allow your user to access USB devices (needed for Teensy)
sudo usermod -aG dialout $USER
```

Apply the group changes — either log out and back in, or run this to apply immediately in the current terminal:
```bash
newgrp docker
```

Install Docker Compose plugin:
```bash
sudo apt install docker-compose-plugin
```

If that package is not found, install the binary directly:
```bash
mkdir -p ~/.docker/cli-plugins
curl -SL https://github.com/docker/compose/releases/download/v2.36.2/docker-compose-linux-x86_64 \
  -o ~/.docker/cli-plugins/docker-compose
chmod +x ~/.docker/cli-plugins/docker-compose
```

Verify:
```bash
docker --version
docker compose version
```

---

### Windows

1. Download and install **Docker Desktop for Windows** from https://docs.docker.com/desktop/install/windows-install/
   - During install, choose the **WSL 2** backend when prompted
2. Download and install **usbipd-win** from https://github.com/dorssel/usbipd-win/releases
   - This is needed to pass the Teensy USB into the WSL2 environment
3. Open **Docker Desktop** → Settings → Resources → WSL Integration → enable your Ubuntu distro → Apply & Restart
4. Download Ubuntu application from Microsoft Store, this will act as a VM with a Linux file system. 

All commands in this guide are run inside the **Ubuntu Ubuntu WSL2 terminal (Open from Ubuntu app, downloaded in step 4, in Windows not a powershell)**, unless stated otherwise.

Verify (inside Ubuntu Ubuntu WSL2 terminal):
```bash
docker --version
docker compose version
```

---

### macOS

1. Download and install **Docker Desktop for Mac** from https://docs.docker.com/desktop/install/mac-install/
2. Open Docker Desktop and wait until it shows "Engine running"

> **Note:** macOS does not support USB serial device passthrough into Docker containers.
> The real Teensy cannot be used in Docker on Mac. Use **mock mode** instead (see Section 6).

Verify:
```bash
docker --version
docker compose version
```

---

## 2. First-Time Setup (all platforms)

Clone or copy the project onto the machine, then:

```bash
cd claude_visualizer_ws/docker

# Create your local config file
cp .env.example .env
```

Open `.env` and set `SERIAL_PORT` to match your Teensy's device path (Linux/Windows only).
On Linux/WSL2, find it with:
```bash
ls /dev/ttyACM*
```

Then build the Docker image (takes a few minutes the first time):
```bash
docker compose build
```

---

## 3. USB Setup for Teensy — Windows WSL2 Only

Every time you plug in the Teensy, open **PowerShell as Administrator** and run:

```powershell
# List connected USB devices and find the Teensy (shows as "USB Serial Device")
usbipd list

# First time only — bind the device (replace 2-3 with your BUSID from the list above)
usbipd bind --busid 2-3

# Attach it to WSL2 every time you plug it in
usbipd attach --wsl --busid 2-3
```

Then in Ubuntu WSL2 terminal, confirm it's visible:
```bash
ls /dev/ttyACM*
# Should show: /dev/ttyACM0
```

---

## 4. Running — Hardware Mode (Regular mode, with Teensy)
### On Linux

Make sure the Teensy is plugged in and visible at the device path in your `.env` file.

```bash
cd claude_visualizer_ws/docker

# Start all services in the background
docker compose up -d

# Watch all logs live (Ctrl+C to stop watching — containers keep running)
docker compose logs -f

# Watch only the ROS nodes
docker compose logs -f ros_nodes

# Watch only the micro-ROS agent (Teensy bridge)
docker compose logs -f micro_ros_agent
```

Open your browser at **http://localhost:8000**

### Send experiment commands

In a separate terminal:
```bash
docker compose run --rm -it ros_nodes ros2 run claude_visualizer mock_robot_controller.py
```
Then type commands at the prompt:
```
ptp 5 index                      # point-to-point move
pp 3 0,36,72 CW,CCW,CW true     # pick and place
perf 1.0 2.0                     # performance test
prec 0 36 10 index               # precision test
stop                             # stop experiment
quit                             # exit
```

### Stop everything
```bash
docker compose down
```

---

### On Windows WSL2

Uses a separate compose file (`docker-compose.windows.yml`) with bridge networking and published ports — required because Docker Desktop does not support `network_mode: host` on Windows.

Make sure the Teensy is attached to WSL2 first (see Section 3).

```bash
cd claude_visualizer_ws/docker

# Start all services in the background
docker compose -f docker-compose.windows.yml up -d

# Watch all logs live
docker compose -f docker-compose.windows.yml logs -f

# Watch only the ROS nodes
docker compose -f docker-compose.windows.yml logs -f ros_nodes
```

Open your browser at **http://127.0.0.1:8000** (not `localhost`)

### Send experiment commands

In a separate Ubuntu WSL2 terminal:
```bash
docker compose -f docker-compose.windows.yml run --rm -it ros_nodes ros2 run claude_visualizer mock_robot_controller.py
```

### Stop everything
```bash
docker compose -f docker-compose.windows.yml down
```

### If charts stay empty after UI loads

The `micro_ros_agent` (FastDDS) and `ros_nodes` (CycloneDDS) may fail to exchange data over a bridge network. Verify with:
```bash
docker compose -f docker-compose.windows.yml exec ros_nodes \
  bash -lc "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic echo /encoder_raw --once"
```
- A message prints → data is flowing normally
- Hangs with no output → DDS cross-engine link failed

**Fallback:** Install Docker Engine natively inside WSL2 (instead of using Docker Desktop) and use the Linux compose file (`docker-compose.yml`). With native Docker Engine, host networking works exactly as on Linux and the DDS link is reliable.

---

## 6. Running — Mock Mode (macOS and any machine without a Teensy)

Mock mode replaces the Teensy with software-simulated encoder data so the full pipeline runs without any hardware. Works on macOS, Windows, and Linux.

```bash
cd claude_visualizer_ws/docker

# Build if you haven't yet
docker compose -f docker-compose.mock.yml build

# Start all services (logs appear in this terminal)
docker compose -f docker-compose.mock.yml up
```

Open your browser:
- **macOS / Linux:** `http://localhost:8000`
- **Windows:** `http://127.0.0.1:8000`

### Send experiment commands (mock_controller)

In a second terminal, attach to the mock_controller:
```bash
docker attach claude_visualizer_mock-mock_controller-1
```
Type commands at the prompt (same as above: `ptp 5 index`, `stop`, etc.)

To detach without stopping the container: press **Ctrl+P then Ctrl+Q**

### Stop everything
```bash
# Ctrl+C in the terminal where you ran docker compose up
# Or from another terminal:
docker compose -f docker-compose.mock.yml down
```

---

## 7. Rebuild After Code Changes

If you edit any Python scripts, config files, or the web frontend:

```bash
# Linux hardware mode
docker compose up -d --build

# Windows hardware mode
docker compose -f docker-compose.windows.yml up -d --build

# Mock mode
docker compose -f docker-compose.mock.yml up --build
```

---

## 8. Configuration

Edit `docker/.env` to change settings:

| Variable | Default | Description |
|---|---|---|
| `SERIAL_PORT` | `/dev/ttyACM0` | USB device path for the Teensy (Linux/WSL2 only) |
| `ROS_DOMAIN_ID` | `0` | ROS 2 DDS domain. Change if multiple ROS 2 systems share the same network |

Experiment pass/fail criteria are configured live from the browser **CRITERIA tab** — no file editing needed.

### How to change ROS_DOMAIN_ID from the Ubuntu Ubuntu WSL2 terminal

`nano` is a simple text editor built into the Ubuntu terminal — no installation needed. It lets you open and edit files directly in the terminal without a graphical interface.

```bash
cd claude_visualizer_ws/docker
nano .env
```

Inside nano, use the arrow keys on the keyboard to navigate to the line `ROS_DOMAIN_ID=0` and change the number (any value 0–232).

Save and exit:
- **Ctrl+O** → press **Enter** (saves the file)
- **Ctrl+X** (exits nano)

Then restart the containers to apply the new value:
```bash
# Linux
docker compose down && docker compose up -d

# Windows
docker compose -f docker-compose.windows.yml down && docker compose -f docker-compose.windows.yml up -d

# Mock mode
docker compose -f docker-compose.mock.yml down && docker compose -f docker-compose.mock.yml up
```

> **Important:** All machines running the system at the same time must use the **same `ROS_DOMAIN_ID`** value, or the nodes on different machines won't discover each other on the network.

---

## 9. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Page blank or "Disconnected" on Windows | `localhost` resolves to IPv6 on Windows | Use `http://127.0.0.1:8000` instead of `localhost` |
| Charts empty after UI loads (Windows hardware) | FastDDS ↔ CycloneDDS bridge link failed | Run the `/encoder_raw` check in §5 — if it hangs, use the Docker Engine fallback |
| `Permission denied: /dev/ttyACM0` | User not in `dialout` group | `sudo usermod -aG dialout $USER` then log out/in |
| `permission denied` on Docker socket | User not in `docker` group yet | Run `newgrp docker` in the current terminal to apply immediately |
| Browser connects but charts are empty | Teensy not sending data | Check `docker compose logs micro_ros_agent` — should show "Opened" and incoming packets |
| `docker compose` command not found | Compose plugin missing | Try `sudo apt install docker-compose-plugin`; if not found, use the binary install in §1 |
| `No such device` on startup | Teensy not plugged in or wrong port | Plug in Teensy, run `ls /dev/ttyACM*`, update `SERIAL_PORT` in `.env` |
| mock_controller exits immediately | Container stdin closed | Use `docker attach` (not `docker exec`) to connect to it |
| Nodes can't see each other (mock mode) | DDS discovery issue in bridge network | `CYCLONEDDS_URI` is already set in the compose file — check it's not been removed |
