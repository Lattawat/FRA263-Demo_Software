# server.py
import argparse
import asyncio
import json
import pylsl
import websockets
from websockets.exceptions import ConnectionClosed
from protocol import Protocol
import time
from pylsl import StreamInfo, StreamOutlet, local_clock

# Modbus Protocol
protocol = Protocol()

# Server state
server_state = {
    "background_tasks": [],
}

# Robot stats (updated by modbus_status_loop; read by stats_broadcast_loop)
robot_state = {
    "position": "--",
    "speed": "--",
    "accel": "--",
    "gripper_z": "Idle",
    "gripper_jaw": "Idle",
    "mode": "Idle",
    "emergency": "Idle",
}

# Link / heartbeat state (updated by heartbeat_loop; read by stats_broadcast_loop)
link_state = {
    "last_ya_time": None,  # perf_counter() when reg 0x00 last equaled YA (22881)
}

# set_home zero reference for the base-system (WS) UI only; snapshot of
# theta_actual_pos at the last set_home. LSL/verification stream stays raw.
home_state = {"pos_offset_deg": 0.0}

# --- Loop rates (each loop is independent) ---
STATS_HZ = 25.0                   # WebSocket + LSL: broadcast cached state
HB_HZ = 5.0                       # Modbus: read 0x00 and reply HI when robot sends YA
MODBUS_STATUS_HZ = 25.0           # Modbus: full read for pos / gripper / mode

STATS_INTERVAL = 1.0 / STATS_HZ
HB_INTERVAL = 1.0 / HB_HZ
MODBUS_STATUS_INTERVAL = 1.0 / MODBUS_STATUS_HZ

# Robot must send YA again within this window or UI shows heartbeat_alive=false
HB_DEAD_TIMEOUT = 0.8

modbus_lock = asyncio.Lock()

# ── Group identity → LSL stream-name suffix ──────────────────────────────────
# Mirrors ROS web_visualizer.py: group 0 = legacy (no suffix); N>=1 suffixes
# every LSL stream name with f"_{N}" so this back-end's ActualStates/EventTrigger
# resolve as the group's streams on the visualizer side. CLI arg only (no env).
_parser = argparse.ArgumentParser(description="Base-System back-end (LSL + WebSocket)")
_parser.add_argument("--group_number", type=int, default=0,
                     help="Integer group number. 0 = legacy single-group (no LSL suffix); "
                          "N>=1 suffixes LSL stream names with _N to match web_visualizer.")
_args, _ = _parser.parse_known_args()          # parse_known_args: tolerate extra flags
group_number = _args.group_number              # 0 = legacy (no suffix)
_suf = (lambda n: f"{n}_{group_number}") if group_number else (lambda n: n)
print(f"[server_111] group_number={group_number} "
      f"lsl_suffix={'_' + str(group_number) if group_number else '(none)'}")

# LSL StreamInfo for robot states (position, speed, accel)
actual_state = pylsl.StreamInfo(
    name=_suf("ActualStates"),
    type="States",
    channel_count=3, # position, speed, accel
    nominal_srate=STATS_HZ,
    channel_format="float32",
    source_id=_suf("mock_robot_controller-actual_states")
)

# LSL StreamInfo for events
event_info = pylsl.StreamInfo(
    name=_suf("EventTrigger"),
    type="Trigger",
    channel_count=1,
    nominal_srate=pylsl.IRREGULAR_RATE,
    channel_format="string",
    source_id=_suf("mock_robot_controller-event_trigger")
)

# Create LSL outlets
states_outlet = pylsl.StreamOutlet(actual_state)
event_outlet = pylsl.StreamOutlet(event_info)


async def _sleep_fixed_rate(next_tick: float, interval: float) -> float:
    """Sleep until the next fixed-rate tick; reset if we fell behind."""
    next_tick += interval
    sleep_time = next_tick - time.perf_counter()
    if sleep_time > 0:
        await asyncio.sleep(sleep_time)
    else:
        next_tick = time.perf_counter()
    return next_tick


async def _wait_stats_tick(next_tick: float) -> float:
    """
    Sleep-first STATS_HZ Hz scheduler: wait for the slot, then emit exactly one sample.
    Keeps one STATS (+ LSL) sample every STATS_INTERVAL (1/STATS_HZ s → STATS_HZ hz).
    """
    delay = next_tick - time.perf_counter()
    if delay > 0:
        await asyncio.sleep(delay)
    return next_tick + STATS_INTERVAL


def _lsl_motion_sample() -> list[float]:
    """Float triplet for LSL outlet (position, speed, accel)."""
    #temp
    # print("Back-end => veri", float(protocol.theta_actual_pos))
    return [
        float(protocol.theta_actual_pos),
        float(protocol.theta_actual_speed),
        float(protocol.theta_actual_accel),
    ]


def _serial_connected() -> bool:
    return bool(protocol.client) and protocol.is_connected()


def _heartbeat_alive() -> bool:
    """True when serial is up and YA was seen within HB_DEAD_TIMEOUT."""
    if not _serial_connected():
        return False
    last_ya = link_state["last_ya_time"]
    if last_ya is None:
        return False
    return (time.perf_counter() - last_ya) <= HB_DEAD_TIMEOUT


def _apply_protocol_to_robot_state() -> None:
    """Copy latest Modbus status fields into robot_state."""
    robot_state["position"] = protocol.theta_actual_pos
    robot_state["speed"] = protocol.theta_actual_speed
    robot_state["accel"] = protocol.theta_actual_accel
    robot_state["emergency"] = protocol.emergency_stop_status
    robot_state["mode"] = protocol.moving_status

    reed1 = protocol.gripper_actual_reed1
    reed2 = protocol.gripper_actual_reed2
    reed3 = protocol.gripper_actual_reed3

    robot_state["gripper_z"] = (
        "Up" if (reed1 and not reed2) else
        "Down" if (reed2 and not reed1) else
        "Idle"
    )
    robot_state["gripper_jaw"] = (
        "Close" if reed3 is True else
        "Open" if reed3 is False else
        "Idle"
    )


def _offseted_position_deg():
    """robot_state position minus the set_home zero offset, for the React UI.
    Passes through the '--' placeholder untouched (before first Modbus read)."""
    raw = robot_state["position"]
    if isinstance(raw, (int, float)):
        return round(raw - home_state["pos_offset_deg"], 4)   # 4 dp: kill float noise
    return raw


def _build_stats_payload() -> dict:
    """
    STATS message for the UI.

    serial_connected — COM port open (Connect succeeded)
    heartbeat_alive  — robot YA handshake OK (YA seen within HB_DEAD_TIMEOUT)
    heartbeat        — same as heartbeat_alive (kept for existing UI)
    """
    serial_ok = _serial_connected()
    hb_alive = _heartbeat_alive()

    #temp
    # print("Back-end => Front-end", robot_state["position"])

    return {
        "type": "STATS",
        # "pos": robot_state["position"],
        "pos": _offseted_position_deg(),
        "speed": robot_state["speed"],
        "accel": robot_state["accel"],
        "gripper": f"{robot_state['gripper_z']} / {robot_state['gripper_jaw']}",
        "mode": robot_state["mode"],
        "emergency": robot_state["emergency"],
        "serial_connected": serial_ok,
        "heartbeat_alive": hb_alive,
        "heartbeat": int(hb_alive),
    }


async def heartbeat_loop() -> None:
    """
    5 Hz Modbus heartbeat: read reg 0x00; if robot sent YA, reply HI.
    Does not block the STATS_HZ Hz STATS broadcast loop.
    """
    next_tick = time.perf_counter()
    try:
        while True:
            if _serial_connected():
                async with modbus_lock:
                    saw_ya, _wrote_hi = await asyncio.to_thread(protocol.heartbeat_tick)
                if saw_ya:
                    link_state["last_ya_time"] = time.perf_counter()

            next_tick = await _sleep_fixed_rate(next_tick, HB_INTERVAL)
    except asyncio.CancelledError:
        pass


async def modbus_status_loop() -> None:
    """
    Poll full register block (0x00–0x31) for position, gripper, mode, etc.
    Heartbeat reply is handled separately by heartbeat_loop at 5 Hz.
    """
    next_tick = time.perf_counter()
    freq_count = 0
    freq_t0 = time.perf_counter()

    try:
        while True:
            if _serial_connected():
                async with modbus_lock:
                    ok = await asyncio.to_thread(protocol.routine)

                if ok:
                    _apply_protocol_to_robot_state()

                freq_count += 1
                now = time.perf_counter()
                if now - freq_t0 >= 1.0:
                    print(f"[FREQ] Modbus status poll: {freq_count / (now - freq_t0):.1f} Hz")
                    freq_count = 0
                    freq_t0 = now

            next_tick = await _sleep_fixed_rate(next_tick, MODBUS_STATUS_INTERVAL)
    except asyncio.CancelledError:
        pass


async def stats_broadcast_loop(websocket) -> None:
    """
    Exactly STATS_HZ samples per second to WebSocket + LSL.

    Sleep-first timing: wait 1/STATS_HZ s → send one sample → repeat (STATS_HZ samples/s).
    LSL timestamps are evenly spaced at 1/STATS_HZ regardless of small send jitter.
    """
    next_tick = time.perf_counter()
    lsl_t0 = None
    sample_idx = 0
    samples_this_second = 0
    second_t0 = time.perf_counter()

    try:
        while True:
            next_tick = await _wait_stats_tick(next_tick)

            # If we fell more than one period behind, realign (no burst, keep STATS_HZ cap)
            now = time.perf_counter()
            if next_tick < now - STATS_INTERVAL:
                next_tick = now + STATS_INTERVAL

            if lsl_t0 is None:
                lsl_t0 = local_clock()

            lsl_timestamp = lsl_t0 + sample_idx / STATS_HZ
            payload = _build_stats_payload()

            states_outlet.push_sample(_lsl_motion_sample(), timestamp=lsl_timestamp)

            try:
                await websocket.send(json.dumps(payload))
            except ConnectionClosed:
                break

            sample_idx += 1
            samples_this_second += 1

            if now - second_t0 >= 1.0:
                print(
                    f"[FREQ] STATS broadcast: {samples_this_second} samples/s "
                    f"(target {int(STATS_HZ)})"
                )
                samples_this_second = 0
                second_t0 = now

    except asyncio.CancelledError:
        pass


async def _cancel_background_tasks() -> None:
    for task in server_state.get("background_tasks", []):
        if task and not task.done():
            task.cancel()
    for task in server_state.get("background_tasks", []):
        if task:
            try:
                await task
            except asyncio.CancelledError:
                pass
    server_state["background_tasks"] = []


def _start_background_loops(websocket) -> list:
    tasks = [
        asyncio.create_task(stats_broadcast_loop(websocket), name="stats_broadcast"),
        asyncio.create_task(modbus_status_loop(), name="modbus_status"),
        asyncio.create_task(heartbeat_loop(), name="heartbeat"),
    ]
    server_state["background_tasks"] = tasks
    return tasks

async def handler(websocket: websockets.WebSocketServerProtocol):
    print("React Client Connected!")

    # Ensure only ONE set of background loops (React dev mode can connect twice)
    await _cancel_background_tasks()
    _start_background_loops(websocket)

    initial_stats = {
        "type": "STATS",
        "message": "Connected to Python Backend",
        # "pos": robot_state["position"],
        "pos": _offseted_position_deg(),
        "speed": robot_state["speed"],
        "accel": robot_state["accel"],
        "gripper": f"{robot_state['gripper_z']} / {robot_state['gripper_jaw']}",
        "mode": robot_state["mode"],
        "emergency": robot_state["emergency"],
        "serial_connected": _serial_connected(),
        "heartbeat_alive": _heartbeat_alive(),
        "connected": _serial_connected(),  # legacy: serial only on first message
    }

    try:
        await websocket.send(json.dumps(initial_stats))
    except ConnectionClosed:
        await _cancel_background_tasks()
        return

    try:
        async for message in websocket:
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                await websocket.send(json.dumps({"mode": "Error", "message": "Invalid JSON"}))
                continue

            # print(f"[RECEIVED] {data}")


            req_mode = data.get("mode")
            action = data.get("action")
            # ---------------- CONNECT / DISCONNECT ----------------
            if req_mode == "Connect" and action == "connect_port":
                port_num = data.get("port")
                # com_port = f"COM{port_num}"
                com_port = f"/dev/ttyACM{port_num}"  # for Linux (COM1 → ttyUSB0)

                slave = int(data.get("slave", 21))
                async with modbus_lock:
                    if protocol.client and protocol.is_connected() and protocol.port == com_port and protocol.slave_address == slave:
                        ok = True
                    else :
                        ok = await asyncio.to_thread(protocol.connect_rtu, com_port, slave)

                link_state["last_ya_time"] = None  # wait for fresh YA after (re)connect
                home_state["pos_offset_deg"] = 0.0  # drop any stale set_home zero on (re)connect

                await websocket.send(json.dumps({
                    "mode": "Connect",
                    "action": "connect_port",
                    "status": "success" if ok else "failed",
                    "message": f"Connected to {com_port} (slave {slave})" if ok else f"Failed to connect to {com_port}",
                }))
                continue

            # ---------------- HOME ----------------
            if req_mode == "Home":
                if action == "go_home":  # WRITE 0x01
                    async with modbus_lock:
                        await asyncio.to_thread(protocol.write_base_system_status, "go_home")
                    continue

                elif action == "set_home":  # WRITE 0x01
                    async with modbus_lock:
                        await asyncio.to_thread(protocol.write_base_system_status, "set_home")
                    # Zero the base-system UI at this instant (WS path only; LSL stays raw).
                    home_state["pos_offset_deg"] = float(protocol.theta_actual_pos)
                    continue

            # ---------------- MANUAL / JOG ----------------
            elif req_mode == "Manual":
                if action == "set_manual":  # WRITE 0x01 — Jog operating mode
                    
                    async with modbus_lock:
                        await asyncio.to_thread(protocol.write_base_system_status, "Jog")
                    continue

                # WRITE 0x02 — gripper open | close | up | down
                elif action == "gripper_up":
                    async with modbus_lock:
                        await asyncio.to_thread(protocol.write_gripper_command, "Up")
                    continue

                elif action == "gripper_down":
                    async with modbus_lock:
                        await asyncio.to_thread(protocol.write_gripper_command, "Down")
                    continue

                elif action == "gripper_open":
                    async with modbus_lock:
                        await asyncio.to_thread(protocol.write_gripper_command, "Open")
                    continue

                elif action == "gripper_close":
                    async with modbus_lock:
                        await asyncio.to_thread(protocol.write_gripper_command, "Close")
                    continue

                # WRITE 0x03 — gripper pick | place
                elif action == "gripper_pick":
                    async with modbus_lock:
                        await asyncio.to_thread(protocol.write_gripper_movement, "Pick")
                    continue

                elif action == "gripper_place":
                    async with modbus_lock:
                        await asyncio.to_thread(protocol.write_gripper_movement, "Place")
                    continue

                # WRITE 0x05 — jog (degree)
                elif action == 'jog':
                    value = data.get('value')
                    direction = '+' if data.get('direction') == 'CCW' else '-'
                    jog_value = int(str(direction) + str(value))

                    payload = {
                        "mode": req_mode,
                        "action": action,
                        "value": value,
                        "direction": data.get('direction'),
                        "timestamp": time.time()
                    }
                    event_outlet.push_sample([json.dumps(payload)])

                    async with modbus_lock:
                        await asyncio.to_thread(protocol.write_jog, jog_value)
                    continue

            # ---------------- AUTO ----------------
            elif req_mode == "Auto":
                if action == 'set_auto':
                    async with modbus_lock:
                        await asyncio.to_thread(protocol.write_base_system_status, "Auto")  # WRITE 0x01
                    continue

                if action == "pick_place":
                    order_sequence = data.get('sequence')
                    direction_sequence = data.get('directions')
                    gripper_enable = (data.get('use_gripper'))   
                    n_pair = len(order_sequence) // 2

                    # encode signed sequence
                    encode = [order_sequence[0]] + [
                        -pos if d == "CW" else pos
                        for pos, d in zip(order_sequence[1:], direction_sequence)
                    ]
                    # print("encode:", encode)

                    payload = {
                        "mode": req_mode,
                        "action": action,
                        "order_sequence": order_sequence,
                        "direction_sequence": direction_sequence,
                        "use_gripper": gripper_enable,
                        "num": n_pair,
                        "timestamp": time.time()
                    }
                    event_outlet.push_sample([json.dumps(payload)])

                    async with modbus_lock:
                        # WRITE 0x12–0x21 — pick/place per hole + direction
                        addresses = [0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x20,0x21]

                        for i, addr in enumerate(addresses):
                            value = encode[i] if i < len(encode) else 0

                            await asyncio.to_thread(
                                protocol.write_pick_place_hole,
                                addr,
                                value
                            )

                        # WRITE 0x22 — n pair pick-place
                        await asyncio.to_thread(
                            protocol.write_n_pair, n_pair                           
                        )
                    
                    # WRITE 0x04 — gripper enable (AUTO)
                    if gripper_enable:
                        async with modbus_lock:
                            await asyncio.to_thread(protocol.write_gripper_checkbox, 'Enable')
                        continue

                    else:
                        async with modbus_lock:
                            await asyncio.to_thread(protocol.write_gripper_checkbox, 'Disable')
                        continue
                    
                    
                elif action == 'point_to_point':    
                    p2p_unit = data.get('unit')
                    p2p_value = data.get('value')

                    payload = {
                        "mode": req_mode,
                        "action": action,
                        "unit": p2p_unit,
                        "value": p2p_value,
                        "timestamp": time.time()
                    }
                    event_outlet.push_sample([json.dumps(payload)])

                    async with modbus_lock:
                        await asyncio.to_thread(protocol.write_p2p_unit, p2p_unit)      # WRITE 0x23 — P2P unit
                        await asyncio.to_thread(protocol.write_p2p_value, p2p_value)    # WRITE 0x24 — P2P position
                    continue

            # ---------------- TEST ----------------
            elif req_mode == "Test":
                if action == "set_test":
                    async with modbus_lock:
                        await asyncio.to_thread(protocol.write_base_system_status, "Test")  # WRITE 0x01 — Test mode
                    continue

                elif action == "performance": 
                    speed_test = data.get('speed')
                    accel_test = data.get('accel')

                    event_payload = {
                        "mode": req_mode,
                        "action": action,
                        "speed": speed_test,
                        "accel": accel_test,
                        "timestamp": time.time()
                    }

                    event_outlet.push_sample([json.dumps(event_payload)])

                    async with modbus_lock:
                        await asyncio.to_thread(protocol.write_test_mode, 'Performance')    # WRITE 0x06
                        await asyncio.to_thread(protocol.write_test_speed, speed_test)      # WRITE 0x07
                        await asyncio.to_thread(protocol.write_test_accel, accel_test)      # WRITE 0x08
                    continue

                elif action == "precision":
                    init_pos_test = data.get('init_pos')
                    target_pos_test = data.get('tar_pos')
                    repeat_test = data.get('repeat')
                    unit_test = data.get('unit')
                    
                    unit_sign = '+' if unit_test == 'degree' else '-'
                    repeat_w_unit = int(str(unit_sign) + str(repeat_test))

                    event_payload = {
                        "mode": req_mode,
                        "action": action,
                        "init_pos": init_pos_test,
                        "target_pos": target_pos_test,
                        "repeat": repeat_test,
                        "unit": unit_test,
                        "timestamp": time.time()
                    }
                    event_outlet.push_sample([json.dumps(event_payload)])

                    async with modbus_lock:
                        await asyncio.to_thread(protocol.write_test_mode, 'Precision')      # WRITE 0x06
                        await asyncio.to_thread(protocol.write_test_init_pos, init_pos_test)        # WRITE 0x09
                        await asyncio.to_thread(protocol.write_test_target_pos, target_pos_test)    # WRITE 0x10
                        await asyncio.to_thread(protocol.write_test_repeat, repeat_w_unit)          # WRITE 0x11
                    continue

            # ---------------- STOP ----------------
            elif req_mode == "Stop" and action == 'stop':
                async with modbus_lock:
                    await asyncio.to_thread(protocol.write_stop_process, 'Stop')    # WRITE 0x25 — soft stop
                continue

            elif req_mode == "Stop" and action == 'cancel_stop':
                async with modbus_lock:
                    await asyncio.to_thread(protocol.write_stop_process, 'Normal')    # WRITE 0x25 — cancel stop
                continue

            else : 
                print(f"[ERROR] Can't recog{req_mode}")

            # ---------------- FALLBACK ----------------
            await websocket.send(json.dumps({
                "mode": "Error",
                "message": f"Unsupported command: mode={req_mode}, action={action}"
            }))

    except websockets.exceptions.ConnectionClosed:
        print("Frontend client disconnected.")
    finally:
        await _cancel_background_tasks()


async def main():
    async with websockets.serve(handler, "localhost", 8765):
        print("WebSocket Server running ws://localhost:8765")
        await asyncio.Future()

async def send_ack(websocket, mode, action, status="success", message=""):
    await websocket.send(json.dumps({
        "type": "ACK",
        "mode": mode,
        "action": action,
        "status": status,
        "message": message,
        "timestamp": time.time()
    }))

if __name__ == "__main__":
    asyncio.run(main())