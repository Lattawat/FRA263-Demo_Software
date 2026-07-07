"""
bringup launch file — full pipeline (Phase 1 + Phase 2)

Nodes launched:
  1. mock_encoder    — synthetic position data (replaces Teensy)
  2. encoder_reader  — constant-jerk Kalman filter → /encoder_state
  3. web_visualizer  — LSL ↔ ROS 2 bridge + WebSocket server  (Phase 2)

The WebSocket server is built into web_visualizer.py itself (via the
`websockets` library), so there is NO separate rosbridge / foxglove node.
The JS frontend connects to ws://<host>:<rosbridge_port>.

Usage:
  ros2 launch claude_visualizer bringup.launch.py
  ros2 launch claude_visualizer bringup.launch.py waveform:=sine

──────────────────────────────────────────────────────────────────────────────
 PHASE-1 DEBUG ROLLBACK
──────────────────────────────────────────────────────────────────────────────
If you want to go back to verifying just the mock_encoder + Kalman filter
pipeline (e.g. to re-check KF tuning in PlotJuggler), temporarily comment out
the Phase-2 node in the returned LaunchDescription at the bottom of this file:

        # web_visualizer_node,    ← comment to disable Phase 2

Leave `mock_encoder_node` and `encoder_reader_node` active. Nothing else needs
to change — params.yaml, messages, and topics stay the same.
──────────────────────────────────────────────────────────────────────────────
"""

import os

from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, LogInfo
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _derive_pair(pair_id: int):
    """Map one pair id N → (ws_port, http_port, session suffix).

    N == 0 keeps the legacy single-pair defaults (9090/8000, no LSL suffix).
    N >= 1 derives unique ports and an LSL stream suffix so multiple pairs can
    coexist on one LAN (and even co-located on one host without port clashes).
    """
    if pair_id:
        return 9000 + pair_id, 8000 + pair_id, str(pair_id)
    return 9090, 8000, ""


def generate_launch_description():
    pkg = FindPackageShare("claude_visualizer")
    params_file    = PathJoinSubstitution([pkg, "config", "params.yaml"])
    criteria_file  = PathJoinSubstitution([pkg, "config", "criteria.yaml"])

    # ── Declare overridable arguments ────────────────────────────────────────
    waveform_arg = DeclareLaunchArgument(
        "waveform",
        default_value="trapezoid",
        description="Mock encoder waveform: sine | trapezoid | step",
    )

    # rosbridge_port_arg = DeclareLaunchArgument(
    #     "rosbridge_port",
    #     default_value="9090",
    #     description="WebSocket port served by web_visualizer (legacy name kept "
    #                 "to match params.yaml; no actual rosbridge process runs).",
    # )

    # Pair identity drives ROS domain isolation (via the sourced env file's
    # ROS_DOMAIN_ID), the LSL stream suffix, and the web ports. Defaults to the
    # CV_PAIR_ID env var set by pairs/pair<N>.env; overridable on the CLI.
    pair_id_arg = DeclareLaunchArgument(
        "pair_id",
        default_value=os.environ.get("CV_PAIR_ID", "0"),
        description="Integer pair id 0-101. Derives ws_port (9000+N), http_port "
                    "(8000+N) and the LSL session suffix. 0 = legacy defaults "
                    "(9090/8000, no suffix).",
    )

    # robot_id_arg = DeclareLaunchArgument(
    #     "robot_id",
    #     default_value="default",
    #     description="Robot ID used to look up evaluation criteria in criteria.yaml.",
    # )
    # robot_id removed: pair_id is now the single setup identifier and also selects
    # the criteria row (criteria.yaml is keyed by pair_id). See pair_id_arg above.

    use_mock_encoder_arg = DeclareLaunchArgument(
        "use_mock_encoder",
        default_value="false",
        description="Start mock_encoder instead of waiting for Teensy hardware. "
                    "Set to true for hardware-free mode (macOS Docker, CI).",
    )

    # ── Phase 1 nodes ────────────────────────────────────────────────────────
    mock_encoder_node = Node(
        package="claude_visualizer",
        executable="mock_encoder.py",
        name="mock_encoder",
        output="screen",
        parameters=[
            params_file,
            {"waveform": LaunchConfiguration("waveform")},
        ],
        condition=IfCondition(LaunchConfiguration("use_mock_encoder")),
    )

    encoder_reader_node = Node(
        package="claude_visualizer",
        executable="Kalman_filter.py",
        name="encoder_reader",
        output="screen",
        parameters=[params_file],
    )

    # ── Phase 2 node ─────────────────────────────────────────────────────────
    # web_visualizer bridges LSL ↔ ROS 2 and also serves the WebSocket endpoint
    # consumed by the JS frontend. Safe to skip during Phase-1 debugging — see
    # ROLLBACK note at top of file.
    # web_visualizer_node = Node(
    #     package="claude_visualizer",
    #     executable="web_visualizer.py",
    #     name="web_visualizer",
    #     output="screen",
    #     parameters=[
    #         params_file,
    #         {"rosbridge_port": LaunchConfiguration("rosbridge_port")},
    #     ],
    # )

    # Built inside an OpaqueFunction so ws_port/http_port/session can be derived
    # from the resolved pair_id at launch time (substitutions can't do arithmetic).
    def _web_visualizer_setup(context, *_args, **_kwargs):
        pair_id = int(LaunchConfiguration("pair_id").perform(context) or "0")
        ws_port, http_port, session = _derive_pair(pair_id)
        node = Node(
            package="claude_visualizer",
            executable="web_visualizer.py",
            name="web_visualizer",
            output="screen",
            parameters=[
                params_file,
                {"ws_port": ws_port, "http_port": http_port, "session": session},
            ],
        )
        return [
            LogInfo(msg=f"[claude_visualizer] pair_id={pair_id} → "
                        f"ws_port={ws_port} http_port={http_port} "
                        f"lsl_suffix={'_' + session if session else '(none)'}"),
            node,
        ]

    experiment_evaluator_node = Node(
        package="claude_visualizer",
        executable="experiment_evaluator.py",
        name="experiment_evaluator",
        output="screen",
        parameters=[
            {
                # "robot_id":         LaunchConfiguration("robot_id"),
                # Pass-through string param (no arithmetic) → plain Node, no OpaqueFunction.
                "pair_id":            LaunchConfiguration("pair_id"),
                "criteria_file_path": criteria_file,
            }
        ],
    )

    return LaunchDescription([
        waveform_arg,
        # rosbridge_port_arg,
        pair_id_arg,
        # robot_id_arg,                 # ← removed: pair_id selects criteria now
        use_mock_encoder_arg,
        LogInfo(msg="[claude_visualizer] Full pipeline bringup (Phase 1 + Phase 2)"),
        mock_encoder_node,
        encoder_reader_node,
        # web_visualizer_node,          # ← comment to disable Phase 2
        OpaqueFunction(function=_web_visualizer_setup),   # ← Phase 2 (derives ports/session)
        experiment_evaluator_node,    # ← comment to disable evaluation subsystem
    ])
