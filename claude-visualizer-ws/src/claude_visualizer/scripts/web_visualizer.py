#!/usr/bin/env python3
"""
web_visualizer — LSL ↔ ROS 2 bridge + WebSocket server (Phase 2)
=================================================================

Data flows:

  Flow 1  (ROS → LSL):
      /estimated_states (ROS)  ──►  LSL outlet "EncoderState"
        lets robot_controller.py (non-ROS) read the Kalman-filtered state

  Flow 2  (LSL → ROS):
      LSL inlet "ActualStates"  ──►  /actual_states (ROS)

  Flow 3  (LSL → ROS):
      LSL inlet "EventTrigger"  ──►  /event_trigger  (ROS)

  Flow 4  (ROS → WebSocket):
      /estimated_states, /actual_states, /event_trigger  ──►  JSON over WS
        the JS frontend (browser) connects to ws://<host>:<ws_port> and
        receives live JSON messages; no rosbridge / foxglove required

JSON wire format  (one message per envelope):
    {"topic": "estimated_states" | "actual_states" | "event_trigger",
     "data": { ...fields from the ROS msg... }}

New clients receive the most recent message from each topic on connect,
then live updates.

LSL channel conventions:

  "ActualStates"  — 3 float channels:
      [actual_position, actual_velocity, actual_acceleration]

  "EventTrigger"  — 1 string channel containing JSON, e.g.:
      {"event": "START", "profile_id": "run_001",
       "label": "ramp_test", "expected_duration": 5.0}

Dependencies (install in your venv when ready):
    pip install pylsl websockets
"""

import asyncio
import functools
import http.server
import json
import math
import os
import threading

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import pylsl
import websockets

from std_msgs.msg import Empty

from claude_visualizer_interface.msg import EncoderState, ActualStates, EventTrigger, ExperimentEval
from claude_visualizer_interface.srv import UpdateCriteria
from claude_visualizer.utils import create_outlet

ENCODER_STATE_CHANNELS = ("position", "velocity", "acceleration")
ACTUAL_STATES_CHANNELS = ("actual_position", "actual_velocity", "actual_acceleration")

_DEG_TO_RAD = math.pi / 180.0


# ── Message → JSON serializers ────────────────────────────────────────────────

def _stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def estimated_states_to_json(msg: EncoderState) -> dict:
    return {
        "topic": "estimated_states",
        "data": {
            "stamp":        _stamp_to_sec(msg.header.stamp),
            "position":     msg.position,
            "velocity":     msg.velocity,
            "acceleration": msg.acceleration,
            "pos_variance": msg.pos_variance,
            "vel_variance": msg.vel_variance,
            "acc_variance": msg.acc_variance,
            "raw_ticks":    msg.raw_ticks,
        },
    }


def actual_states_to_json(msg: ActualStates) -> dict:
    return {
        "topic": "actual_states",
        "data": {
            "stamp":                _stamp_to_sec(msg.header.stamp),
            "actual_position":      msg.actual_position,
            "actual_velocity":      msg.actual_velocity,
            "actual_acceleration":  msg.actual_acceleration,
        },
    }

def event_trigger_to_json(msg: EventTrigger) -> dict:
    try:
        payload = json.loads(msg.event)
    except (TypeError, ValueError):
        payload = {"event": msg.event}
    return {
        "topic": "event_trigger",
        "data": {
            "stamp": _stamp_to_sec(msg.header.stamp),
            **payload,
        },
    }


def _eval_to_json(topic: str, msg: ExperimentEval) -> dict:
    try:
        metrics = json.loads(msg.data)
    except (TypeError, ValueError):
        metrics = {}
    return {
        "topic": topic,
        "data": {
            "stamp":  _stamp_to_sec(msg.header.stamp),
            "action": msg.action,
            **metrics,
        },
    }


class _SilentHTTPHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *args): pass

    # Serve the runtime WebSocket port to the browser so app.js no longer needs a
    # hardcoded port. ws_port is attached to the server object in _run_http_server.
    def do_GET(self):
        if self.path == "/config.json":
            body = json.dumps({"ws_port": self.server.ws_port}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        return super().do_GET()


# ── Node ──────────────────────────────────────────────────────────────────────

class WebVisualizerNode(Node):

    def __init__(self) -> None:
        super().__init__("web_visualizer")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("ws_port", 9090)
        self.declare_parameter("ws_host", "0.0.0.0")
        self.declare_parameter("http_port", 8000)
        # LSL stream suffix for multi-pair LAN isolation ("" = legacy single-pair).
        self.declare_parameter("session", "")
        self.declare_parameter("lsl_params.actual_states_stream.name",   "ActualStates")
        self.declare_parameter("lsl_params.event_trigger_stream.name",    "EventTrigger")
        self.declare_parameter("lsl_params.resolve_timeout_s",            5.0)
        self.declare_parameter("topics.actual_states",                    "/actual_states")
        self.declare_parameter("topics.event_trigger",                    "/event_trigger")
        self.declare_parameter("topics.encoder",                          "/estimated_states")
        self.declare_parameter("lsl_params.actual_states_stream.channel", [])
        self.declare_parameter("lsl_params.event_trigger_stream.channel", [])

        self.declare_parameter("lsl_params.estimated_states_stream.name", "EstimatedStates")
        self.declare_parameter("lsl_params.estimated_states_stream.type", "States")
        self.declare_parameter("lsl_params.estimated_states_stream.channel", ["estimated_position", "estimated_velocity", "estimated_acceleration"])
        self.declare_parameter("lsl_params.estimated_states_stream.outlet_type", "REGULAR")
        self.declare_parameter("lsl_params.estimated_states_stream.sampling_rate_hz", 100.0)
        self.declare_parameter("lsl_params.estimated_states_stream.source_id", "web_visualizer")
        
        self._ws_port         = self.get_parameter("ws_port").value
        self._ws_host         = self.get_parameter("ws_host").value
        self._http_port       = self.get_parameter("http_port").value
        # actual_states_stream  = self.get_parameter("lsl_params.actual_states_stream.name").value
        # event_trigger_stream  = self.get_parameter("lsl_params.event_trigger_stream.name").value
        # Apply the per-pair LSL suffix so this verifier resolves only its own robot's streams.
        session = self.get_parameter("session").value or ""
        _suf = (lambda n: f"{n}_{session}") if session else (lambda n: n)
        actual_states_stream  = _suf(self.get_parameter("lsl_params.actual_states_stream.name").value)
        event_trigger_stream  = _suf(self.get_parameter("lsl_params.event_trigger_stream.name").value)
        
        resolve_timeout       = self.get_parameter("lsl_params.resolve_timeout_s").value
        actual_states_topic   = self.get_parameter("topics.actual_states").value
        event_trigger_topic   = self.get_parameter("topics.event_trigger").value
        topic_encoder         = self.get_parameter("topics.encoder").value

        self.actual_states_channel  = self.get_parameter("lsl_params.actual_states_stream.channel").value
        self.event_trigger_channel  = self.get_parameter("lsl_params.event_trigger_stream.channel").value
        
        # estimated_states_lsl_stream_info = dict(
        #     {
        #         "name": self.get_parameter("lsl_params.estimated_states_stream.name").value,
        #         "type": self.get_parameter("lsl_params.estimated_states_stream.type").value,
        #         "channel": self.get_parameter("lsl_params.estimated_states_stream.channel").value,
        #         "outlet_type": self.get_parameter("lsl_params.estimated_states_stream.outlet_type").value,
        #         "sampling_rate_hz": self.get_parameter("lsl_params.estimated_states_stream.sampling_rate_hz").value,
        #         "source_id": self.get_parameter("lsl_params.estimated_states_stream.source_id").value
        #     }
        # )
        # Suffix the outlet name + source_id too, so a robot-side consumer resolves the right pair.
        estimated_states_lsl_stream_info = dict(
            {
                "name": _suf(self.get_parameter("lsl_params.estimated_states_stream.name").value),
                "type": self.get_parameter("lsl_params.estimated_states_stream.type").value,
                "channel": self.get_parameter("lsl_params.estimated_states_stream.channel").value,
                "outlet_type": self.get_parameter("lsl_params.estimated_states_stream.outlet_type").value,
                "sampling_rate_hz": self.get_parameter("lsl_params.estimated_states_stream.sampling_rate_hz").value,
                "source_id": _suf(self.get_parameter("lsl_params.estimated_states_stream.source_id").value)
            }
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── WebSocket server state ──────────────────────────────────────────
        self._ws_clients = set()                 # touched only by ws thread's loop
        self._ws_last_messages = {}              # topic → last serialized JSON str
        
        self._ws_loop_scheduler = asyncio.new_event_loop()
        #functionally, the AbstractEventLoop acts as a scheduler for asynchronous tasks.
        #However, it is called a "loop" because its internal mechanism is literally 
        #a continuous while loop that keeps the program alive.

        self._ws_thread_exec = threading.Event()
        self._ws_thread = threading.Thread(
            target=self._run_ws_loop, daemon=True, name="ws-server",
        )
        self._ws_thread.start()
        self._ws_thread_exec.wait()

        self._http_thread = threading.Thread(
            target=self._run_http_server, daemon=True, name="http-server",
        )
        self._http_thread.start()

        # ── Estimated States data flow: ROS → LSL (subscribe to /estimated_states, push to LSL) ────
        # self._estimated_states_outlet = self._make_estimated_states_outlet()
        self._estimated_states_outlet = create_outlet(estimated_states_lsl_stream_info, pylsl.cf_float32)
        self.create_subscription(EncoderState, topic_encoder, self._estimated_states_cb, qos)

        # ── Actual States and Trigger data flow: LSL → ROS (pull from LSL inlet, publish to ROS) ─────
        self._actual_states_pub = self.create_publisher(ActualStates, actual_states_topic, qos)
        self._event_trigger_pub = self.create_publisher(EventTrigger, event_trigger_topic,  qos)
        # Tells the encoder_reader (Kalman) node to re-zero /estimated_states at the
        # source, so the evaluator + LSL outlet + WS all share one zeroed frame.
        self._zero_estimated_pub = self.create_publisher(Empty, "/zero_estimated_states", qos)

        # Also subscribe to these topics so that ANY publisher (our LSL worker
        # OR mock_encoder OR a future robot_controller ROS node) gets
        # broadcast to the WebSocket clients. LSL workers therefore do NOT
        # broadcast directly — the subscription callbacks do.
        self.create_subscription(ActualStates,    actual_states_topic,  self._actual_states_cb,  qos)
        self.create_subscription(EventTrigger,    event_trigger_topic,  self._event_trigger_cb,  qos)
        self.create_subscription(ExperimentEval,  "/eval_live",         self._eval_live_cb,      qos)
        self.create_subscription(ExperimentEval,  "/eval_summary",      self._eval_summary_cb,   qos)

        self._criteria_client = self.create_client(UpdateCriteria, "/update_criteria")
        self._last_criteria_json = "{}"

        # Actual-side zero offset stays here because /actual_states is produced
        # locally (LSL worker). The estimated-side zero now lives in the Kalman
        # node so it applies upstream of every consumer.
        self._actual_pos_offset_rad = 0.0

        self._lsl_threads_exec = threading.Event()
        self._lsl_threads = [
            threading.Thread(
                target=self._actual_states_worker,
                args=(actual_states_stream, resolve_timeout),
                daemon=True, name="lsl-actual-states",
            ),
            threading.Thread(
                target=self._event_trigger_worker,
                args=(event_trigger_stream, resolve_timeout),
                daemon=True, name="lsl-event-trigger",
            ),
        ]
        for t in self._lsl_threads: #activate all _lsl_threads to receive the stream
            t.start()

        for ip, port in self._ws_bound_addresses:
            self.get_logger().info(f"Open browser at      http://{ip}:{self._http_port}")
            self.get_logger().info(f"WebSocket server on  ws://{ip}:{port}")

    # ── HTTP file server (blocking thread) ──────────────────────────────────

    def _run_http_server(self) -> None:
        web_dir = os.path.join(
            get_package_share_directory("claude_visualizer"), "web"
        )
        handler = functools.partial(_SilentHTTPHandler, directory=web_dir)
        self._http_server = http.server.HTTPServer(("0.0.0.0", self._http_port), handler)
        self._http_server.ws_port = self._ws_port   # exposed to the browser via GET /config.json
        self._http_server.serve_forever()

    # ── WebSocket server (asyncio thread) ───────────────────────────────────

    def _run_ws_loop(self) -> None:
        asyncio.set_event_loop(self._ws_loop_scheduler) #tell the asyncio that self._ws_loop_scheduler is the asyncio event loop
        self._ws_loop_scheduler.run_until_complete(self._start_ws_server())
        self._ws_thread_exec.set()
        self._ws_loop_scheduler.run_forever()

    async def _start_ws_server(self) -> None:
        self._ws_server = await websockets.serve(
            self._handle_client, self._ws_host, self._ws_port,
        )
        self._ws_bound_addresses = [
            sock.getsockname() for sock in self._ws_server.sockets
        ]

    async def _handle_client(self, websocket) -> None:
        # _handle_client is a callback that the websockets lib invokes once per connection
        
        # Send the last known value for each topic so the browser has an
        # immediate picture instead of waiting for the next tick.
        for cached in self._ws_last_messages.values():
            await websocket.send(cached)
        
        snapshot = await self._get_criteria_snapshot()
        if snapshot:
            await websocket.send(snapshot)

        self._ws_clients.add(websocket)
        try:
            async for raw in websocket:
                try:
                    await self._handle_command(json.loads(raw))
                except Exception:
                    pass
        except websockets.ConnectionClosed:
            pass
        finally:
            self._ws_clients.discard(websocket)

    def _broadcast(self, payload: dict) -> None:
        """Thread-safe: called from ROS/LSL threads. Pushes JSON to all clients."""
        message = json.dumps(payload)
        self._ws_last_messages[payload["topic"]] = message
        if self._ws_clients:
            asyncio.run_coroutine_threadsafe(
                self._broadcast_async(message), self._ws_loop_scheduler,
            )

    async def _get_criteria_snapshot(self) -> str | None:
        if not self._criteria_client.wait_for_service(timeout_sec=0.5):
            return None
        req = UpdateCriteria.Request()
        req.criteria_json = "{}"
        future = self._criteria_client.call_async(req)
        while not future.done():
            await asyncio.sleep(0.05)

        result = future.result()
        if result and result.success:
            self._last_criteria_json = result.current_criteria_json
            return json.dumps({
                "topic": "criteria_snapshot",
                "data": json.loads(result.current_criteria_json)
            })
        
        return None

    async def _handle_command(self, cmd: dict) -> None:
        command = cmd.get("command")
        if command == "time_sync":
            ref_sec = self.get_clock().now().nanoseconds / 1e9
            payload = json.dumps({"topic": "time_sync", "data": {"ref_stamp": ref_sec}})
            self._ws_last_messages["time_sync"] = payload
            await self._broadcast_async(payload)
        elif command == "stop_experiment":
            msg = EventTrigger()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "browser"
            msg.event = json.dumps({"mode": "STOP", "action": "stop"})
            self._event_trigger_pub.publish(msg)
        elif command == "skip_iteration":
            msg = EventTrigger()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "browser"
            msg.event = json.dumps({"action": "skip_iteration"})
            self._event_trigger_pub.publish(msg)
        elif command == "criteria_update":
            data = cmd.get("data", {})
            if not self._criteria_client.wait_for_service(timeout_sec=0.5):
                return
            req = UpdateCriteria.Request()
            req.criteria_json = json.dumps(data)
            future = self._criteria_client.call_async(req)
            while not future.done():
                await asyncio.sleep(0.05)
            result = future.result()
            acknowledgement = {"topic": "criteria_ack", "data": {
                           "success": result.success,
                           "message": result.message,
                        }}
            if result.success:
                self._last_criteria_json = result.current_criteria_json
                acknowledgement["data"]["criteria"] = json.loads(result.current_criteria_json)

            await self._broadcast_async(json.dumps(acknowledgement))
        elif command == "pos_sync":
            delta = float(cmd.get("data", {}).get("delta_rad", 0.0))
            self._actual_pos_offset_rad += delta
        elif command == "zero_set":
            act_now = float(cmd.get("data", {}).get("act_rad", 0.0))
            # Estimated side: ask the Kalman node to re-zero at the source so the
            # evaluator and LSL outlet stay in the same frame as the display.
            self._zero_estimated_pub.publish(Empty())
            # Actual side: /actual_states is produced here, so offset it locally.
            self._actual_pos_offset_rad += act_now

    async def _broadcast_async(self, message: str) -> None:
        for client in list(self._ws_clients):
            try:
                await client.send(message)
            except Exception as e:
                self.get_logger().warn(f"[_broadcast_async] send failed: {type(e).__name__}: {e}")
                self._ws_clients.discard(client)

    # ── Estimated States data flow: /estimated_states → LSL outlet + WS broadcast ──────────────────

    def _estimated_states_cb(self, msg: EncoderState) -> None:
        # msg.position already arrives zeroed from the Kalman node, so the LSL
        # outlet and the WS broadcast both carry the same frame, no patching here.
        self._estimated_states_outlet.push_sample(
            [float(msg.position), float(msg.velocity), float(msg.acceleration)],
            timestamp=_stamp_to_sec(msg.header.stamp),
        )
        self._broadcast(estimated_states_to_json(msg))
        

    # ── Actual States and Trigger data flow: LSL inlet "ActualStates" → /actual_states + WS ──────────────

    def _actual_states_cb(self, msg: ActualStates) -> None:
        self._broadcast(actual_states_to_json(msg))

    def _event_trigger_cb(self, msg: EventTrigger) -> None:
        self._broadcast(event_trigger_to_json(msg))

    def _eval_live_cb(self, msg: ExperimentEval) -> None:
        self._broadcast(_eval_to_json("eval_live", msg))

    def _eval_summary_cb(self, msg: ExperimentEval) -> None:
        self._broadcast(_eval_to_json("eval_summary", msg))


    def extract_sample_from_lsl_inlet(self, inlet, name: str, timeout: float):
        """Pull one sample. inlet persists across calls via the caller's local variable.

        Returns (sample, inlet):
          - sample is None if nothing arrived or an error occurred
          - inlet is None when the stream needs to be re-resolved next call
        """
        # self.get_logger().info(f"[{name}] Attempting to pull sample from LSL inlet...")
        if inlet is None:
            inlet = self._resolve_stream(name, timeout)
            if inlet is None:
                return None, None
        try:
            # self.get_logger().info(f"[{name}] Pulling sample from LSL inlet...")
            sample, _ = inlet.pull_sample(timeout=1.0)
        except Exception as e:
            self.get_logger().warn(f"[{name}] pull_sample error: {e}")
            return None, None  # inlet=None triggers re-resolve next iteration

        if sample is None:
            return None, inlet

        if type(sample[0]) == float:
            if len(sample) < len(self.actual_states_channel):
                self.get_logger().warn(
                    f"[actual_states] expected {len(self.actual_states_channel)} channels, got {len(sample)}"
                )
                return None, inlet
            return sample, inlet

        elif type(sample[0]) == str:
            try:
                return json.loads(sample[0]), inlet
            except (TypeError, ValueError):
                return {"event": "LABEL", "label": str(sample[0])}, inlet

        else:
            self.get_logger().info("Sample type not match with the condition")
            return None, inlet

    def _actual_states_worker(self, name: str, timeout: float) -> None:
        inlet = None
        while not self._lsl_threads_exec.is_set():
            sample, inlet = self.extract_sample_from_lsl_inlet(inlet, name, timeout)
            # self.get_logger().info(f"[{name}] Sample received: {sample}")
            if sample is not None:
                msg = ActualStates()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "robot_controller"
                msg.actual_position     = float(sample[0]) * _DEG_TO_RAD - self._actual_pos_offset_rad
                msg.actual_velocity     = float(sample[1])
                msg.actual_acceleration = float(sample[2])
                self._actual_states_pub.publish(msg)
                # No _broadcast here — the ROS subscription callback handles it.
                # self.get_logger().info(
                #     f"[ActualStates]: position={msg.actual_position} | velocity={msg.actual_velocity} | acceleration={msg.actual_acceleration}"
                # )

    # ── Flow 3: LSL inlet "EventTrigger" → /event_trigger + WS ──────────────

    def _event_trigger_worker(self, name: str, timeout: float) -> None:
        inlet = None
        while not self._lsl_threads_exec.is_set():
            json_payload, inlet = self.extract_sample_from_lsl_inlet(inlet, name, timeout)
            
            if json_payload is not None:
                msg = EventTrigger()
                msg.header.stamp    = self.get_clock().now().to_msg()
                msg.header.frame_id = "robot_controller"
                msg.event = json.dumps(json_payload)
                self.get_logger().info(
                    f"[EventTrigger] event={json_payload.get('event')} payload={msg.event}"
                )
                self._event_trigger_pub.publish(msg)
                # No _broadcast here — the ROS subscription callback handles it.

    # ── Helpers ─────────────────────────────────────────────────────────────

    def _resolve_stream(self, name: str, timeout: float):
        self.get_logger().info(f"Resolving LSL stream '{name}' (timeout={timeout}s)…")
        streams = pylsl.resolve_byprop("name", name, timeout=timeout)
        if not streams:
            self.get_logger().warn(
                f"LSL stream '{name}' not found — will keep retrying"
            )
            return None
        self.get_logger().info(f"LSL stream '{name}' resolved.")
        return pylsl.StreamInlet(streams[0])

    # ── Shutdown ────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._lsl_threads_exec.set()
        for t in self._lsl_threads:
            t.join(timeout=1.5)
        try:
            self._ws_loop_scheduler.call_soon_threadsafe(self._ws_loop_scheduler.stop)
        except Exception:
            pass
        try:
            self._http_server.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
