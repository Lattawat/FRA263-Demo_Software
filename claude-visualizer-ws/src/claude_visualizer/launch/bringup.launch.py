"""
bringup launch file — full pipeline (Phase 1 + Phase 2)

Nodes launched (all under the ROS namespace /G<group_number>):
  1. mock_encoder    — synthetic position data (replaces Teensy)
  2. encoder_reader  — constant-jerk Kalman filter → /G<N>/estimated_states
  3. web_visualizer  — LSL ↔ ROS 2 bridge + WebSocket server  (Phase 2)
  4. experiment_evaluator — pass/fail scoring

Group isolation is by ROS **namespace** (not ROS_DOMAIN_ID): a single arg
`group_number:=N` puts the whole ROS graph under /G<N>/ and selects the LSL
stream suffix (_N) + criteria row. Default N=0 → /G0/ (there is always a
namespace). Web ports are fixed (9090 / 8000) — different machines have
different IPs, so per-group ports are unnecessary. The removed per-group port
scheme is archived in docs/Per-Group Port Configuration (archived).md.

──────────────────────────────────────────────────────────────────────────────
 WHY GroupAction + PushRosNamespace (instead of `namespace=` on each Node)
──────────────────────────────────────────────────────────────────────────────
The "clean" alternative is to pass `namespace=["G", LaunchConfiguration(...)]`
to every Node() and list them flat. It works — but the group boundary is
deliberately preferred here because:

  1. CORRECTNESS BY CONSTRUCTION — the namespace is declared ONCE at the group
     boundary, so every node inside is guaranteed to inherit /G<N>/. With a
     per-node `namespace=`, a node added later WITHOUT that kwarg silently lands
     in the ROOT namespace — a cross-group isolation bug with no error message.
  2. SINGLE SOURCE OF TRUTH — one place sets/changes the group namespace,
     instead of repeating the same substitution on all four nodes.
  3. UNIFORM SCOPE — PushRosNamespace composes cleanly with any future
     group-scoped settings (remaps, params, nested namespaces).

The trade-off (one GroupAction wrapper vs. flat list) is tiny; the isolation
guarantee in (1) is worth it for a multi-group system.

Usage:
  ros2 launch claude_visualizer bringup.launch.py                 # group 0 → /G0/
  ros2 launch claude_visualizer bringup.launch.py group_number:=5 # group 5 → /G5/
  ros2 launch claude_visualizer bringup.launch.py waveform:=sine

──────────────────────────────────────────────────────────────────────────────
 PHASE-1 DEBUG ROLLBACK
──────────────────────────────────────────────────────────────────────────────
To verify just the mock_encoder + Kalman filter pipeline, comment out the
web_visualizer_node line inside the GroupAction at the bottom of this file.
──────────────────────────────────────────────────────────────────────────────
"""

# import os  # no longer needed (CV_PAIR_ID env default removed)

from launch import LaunchDescription
# OpaqueFunction removed — no launch-time arithmetic left (ports are fixed), so the
# namespace/session are passed as plain substitutions instead of resolved in Python.
# from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, GroupAction
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# _derive_group() is gone: the namespace is now built directly as a substitution
# (["G", LaunchConfiguration("group_number")]) and the "group 0 → no LSL suffix"
# rule moved into web_visualizer (_suf treats "0"/"" as no-suffix). Nothing left
# to compute in Python at launch time.


def generate_launch_description():
    pkg = FindPackageShare("claude_visualizer")
    params_file    = PathJoinSubstitution([pkg, "config", "params.yaml"])
    criteria_file  = PathJoinSubstitution([pkg, "config", "criteria.yaml"])

    group_number = LaunchConfiguration("group_number")
    # ROS namespace token, e.g. "G5" (or "G0" by default). Always present.
    namespace = ["G", group_number]

    # ── Declare overridable arguments ────────────────────────────────────────
    waveform_arg = DeclareLaunchArgument(
        "waveform",
        default_value="trapezoid",
        description="Mock encoder waveform: sine | trapezoid | step",
    )

    # group_number drives the ROS namespace (/G<N>), the LSL suffix (_N) and the
    # criteria row. CLI-only (env file removed); default 0 → /G0/.
    group_number_arg = DeclareLaunchArgument(
        "group_number",
        default_value="0",
        description="Group number N. Puts the ROS graph under /G<N>/, sets the "
                    "LSL suffix (_N, none for 0) and selects the criteria row. "
                    "Default 0 → /G0/.",
    )

    use_mock_encoder_arg = DeclareLaunchArgument(
        "use_mock_encoder",
        default_value="false",
        description="Start mock_encoder instead of waiting for Teensy hardware. "
                    "Set to true for hardware-free mode (macOS Docker, CI).",
    )

    # ── Nodes (namespace applied by the GroupAction below, not per-node) ──────
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

    web_visualizer_node = Node(
        package="claude_visualizer",
        executable="web_visualizer.py",
        name="web_visualizer",
        output="screen",
        parameters=[
            params_file,
            # ws_port/http_port fixed via params.yaml (9090/8000). group_number as
            # a STRING (ParameterValue forces str so "5" is not type-inferred to
            # int); web_visualizer treats "0"/"" as no LSL suffix.
            {"group_number": ParameterValue(group_number, value_type=str)},
        ],
    )

    experiment_evaluator_node = Node(
        package="claude_visualizer",
        executable="experiment_evaluator.py",
        name="experiment_evaluator",
        output="screen",
        parameters=[
            {
                # group_number selects the criteria row (param is dynamic_typing,
                # so the int that launch infers from "5" is accepted).
                "group_number":       group_number,
                "criteria_file_path": criteria_file,
            }
        ],
    )

    # PushRosNamespace applies /G<N> to EVERY node in this group — see the
    # "WHY GroupAction" note at the top of the file.
    namespaced_group = GroupAction([
        PushRosNamespace(namespace),
        mock_encoder_node,
        encoder_reader_node,
        web_visualizer_node,          # ← comment to disable Phase 2
        experiment_evaluator_node,    # ← comment to disable evaluation subsystem
    ])

    return LaunchDescription([
        waveform_arg,
        group_number_arg,
        use_mock_encoder_arg,
        LogInfo(msg=["[claude_visualizer] group_number=", group_number,
                     " → namespace /G", group_number, ", ports 9090/8000 (fixed)"]),
        namespaced_group,
    ])
