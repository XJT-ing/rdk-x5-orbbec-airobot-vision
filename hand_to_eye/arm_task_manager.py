#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch the requested visual detector and grasp chain from /arm/grasp_command.

Expected primary command payload:
  msg.data = "苹果"

Legacy debug payload on /command is still accepted:
  [{"actuator":"机械臂","action":"抓取","params":{"target":"苹果"}}]
The node is intended to run on the vision/arm RDK X5 after:
  1. airbot_server is running on can1
  2. the arm has been moved to the lower home pose
  3. the Orbbec camera is already running
"""

import json
import os
import shlex
import signal
import subprocess
import time
from pathlib import Path
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def shell_join(parts):
    return " && ".join(parts)


@dataclass(frozen=True)
class TargetSpec:
    label: str
    mode: str
    object_name: str
    topic: str
    yolo_class: str = ""
    detector_node: str = ""


TARGETS: Dict[str, TargetSpec] = {
    "苹果": TargetSpec(
        label="苹果",
        mode="yolo",
        object_name="apple",
        topic="/detect_yolo/apple_position",
        yolo_class="apple",
    ),
    "香蕉": TargetSpec(
        label="香蕉",
        mode="yolo",
        object_name="banana",
        topic="/detect_yolo/banana_position",
        yolo_class="banana",
    ),
    "瓶子": TargetSpec(
        label="瓶子",
        mode="yolo",
        object_name="bottle",
        topic="/detect_yolo/bottle_position",
        yolo_class="bottle",
    ),
    "蛋糕": TargetSpec(
        label="蛋糕",
        mode="yolo",
        object_name="cake",
        topic="/detect_yolo/cake_position",
        yolo_class="cake",
    ),
    "小黄鸭": TargetSpec(
        label="小黄鸭",
        mode="detector",
        object_name="duck",
        topic="/duck_position",
        detector_node="duck_detector_node",
    ),
    "绿色药盒": TargetSpec(
        label="绿色药盒",
        mode="detector",
        object_name="box",
        topic="/box_position",
        detector_node="box_detector_node",
    ),
    "大樱桃": TargetSpec(
        label="大樱桃",
        mode="detector",
        object_name="red_circle",
        topic="/red_circle_position",
        detector_node="red_circle_detector_node",
    ),
}

TARGET_ALIASES: Dict[str, str] = {
    "apple": "\u82f9\u679c",
    "banana": "\u9999\u8549",
    "bottle": "\u74f6\u5b50",
    "\u6c34\u676f": "\u74f6\u5b50",
    "\u676f\u5b50": "\u74f6\u5b50",
    "\u6c34\u74f6": "\u74f6\u5b50",
    "\u996e\u6599\u74f6": "\u74f6\u5b50",
}

POSE_COMMAND_ALIASES: Dict[str, str] = {
    "idle": "idle",
    "\u7a7a\u95f2": "idle",
    "\u60c5\u7eea\u8bc6\u522b": "idle",
    "\u60c5\u7eea\u68c0\u6d4b": "idle",
    "near_grasp": "near_grasp",
    "\u8fd1\u8ddd\u79bb\u6293\u53d6": "near_grasp",
    "\u8fd1\u8ddd\u79bb\u6293\u53d6\u59ff\u6001": "near_grasp",
    "pre_grasp": "pre_grasp",
    "\u8fdc\u8ddd\u79bb\u67e5\u770b": "pre_grasp",
    "\u8fdc\u8ddd\u79bb\u6293\u53d6": "pre_grasp",
    "\u8fdc\u8ddd\u79bb\u67e5\u770b\u59ff\u6001": "pre_grasp",
}


class ArmTaskManager(Node):
    def __init__(self):
        super().__init__("arm_task_manager")

        self.declare_parameter("robot_root", "/home/sunrise/robot")
        self.declare_parameter("task_timeout_sec", 60.0)
        self.declare_parameter("cleanup_detector_after_done", True)
        self.declare_parameter("launch_yolo_for_grasp", False)
        self.declare_parameter("disable_executor_init_for_auto_grasp", True)
        self.declare_parameter("open_loop_config_file", "")
        self.declare_parameter("command_topic", "/arm/grasp_command")
        self.declare_parameter("pose_command_topic", "/arm/pose_command")
        self.declare_parameter("legacy_command_topic", "/command")
        self.declare_parameter("next_target_delay_sec", 5.0)
        self.declare_parameter("start_open_loop_on_startup", False)
        self.declare_parameter("auto_start_open_loop_for_grasp", False)
        self.declare_parameter("pose_switch_start_delay_sec", 2.0)
        self.declare_parameter("plan_status_period_sec", 2.0)
        self.declare_parameter("post_task_pose_command", "pre_grasp")
        self.declare_parameter("post_task_pose_repeat_count", 3)
        self.declare_parameter("post_task_pose_repeat_period_sec", 1.0)

        self.robot_root = str(self.get_parameter("robot_root").value)
        self.task_timeout_sec = float(self.get_parameter("task_timeout_sec").value)
        self.cleanup_detector_after_done = bool(
            self.get_parameter("cleanup_detector_after_done").value)
        self.launch_yolo_for_grasp = bool(
            self.get_parameter("launch_yolo_for_grasp").value)
        self.disable_executor_init_for_auto_grasp = bool(
            self.get_parameter("disable_executor_init_for_auto_grasp").value)
        self.open_loop_config_file = str(
            self.get_parameter("open_loop_config_file").value or "")
        self.next_target_delay_sec = float(
            self.get_parameter("next_target_delay_sec").value)
        self.start_open_loop_on_startup = bool(
            self.get_parameter("start_open_loop_on_startup").value)
        self.auto_start_open_loop_for_grasp = bool(
            self.get_parameter("auto_start_open_loop_for_grasp").value)
        self.pose_switch_start_delay_sec = float(
            self.get_parameter("pose_switch_start_delay_sec").value)
        self.plan_status_period_sec = float(
            self.get_parameter("plan_status_period_sec").value)
        self.post_task_pose_command = str(
            self.get_parameter("post_task_pose_command").value or "")
        self.post_task_pose_repeat_count = int(
            self.get_parameter("post_task_pose_repeat_count").value)
        self.post_task_pose_repeat_period_sec = float(
            self.get_parameter("post_task_pose_repeat_period_sec").value)

        command_topic = str(self.get_parameter("command_topic").value)
        pose_command_topic = str(self.get_parameter("pose_command_topic").value)
        legacy_command_topic = str(self.get_parameter("legacy_command_topic").value)
        self.create_subscription(String, command_topic, self.grasp_command_callback, 10)
        self.create_subscription(String, pose_command_topic, self.pose_command_callback, 10)
        if legacy_command_topic:
            self.create_subscription(String, legacy_command_topic, self.command_callback, 10)
        self.create_subscription(
            String,
            "/robot_arm/executor_status",
            self.executor_status_callback,
            10,
        )
        self.create_subscription(
            String,
            "/robot_arm/gripper_cmd",
            self.gripper_cmd_callback,
            10,
        )
        self.create_subscription(
            String,
            "/robot_arm/grasp_task_status",
            self.grasp_task_status_callback,
            10,
        )
        self.status_pub = self.create_publisher(String, "/arm_task/status", 10)
        self.plan_status_pub = self.create_publisher(
            String, "/arm_task/grasp_plan_status", 10)
        self.pose_switch_pub = self.create_publisher(
            String, "/robot_arm/pose_switch_cmd", 10)
        self.grasp_enable_pub = self.create_publisher(
            String,
            "/arm_task/grasp_enable",
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self.active_object_pub = self.create_publisher(
            String,
            "/arm_task/active_object",
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self.timer = self.create_timer(1.0, self.watchdog)

        self.open_loop_proc: Optional[subprocess.Popen] = None
        self.transform_proc: Optional[subprocess.Popen] = None
        self.detector_proc: Optional[subprocess.Popen] = None
        self.active_spec: Optional[TargetSpec] = None
        self.active_since: Optional[float] = None
        self.task_queue: List[TargetSpec] = []
        self.current_plan_id = 0
        self.current_target_index = 0
        self.pending_next_start_time: Optional[float] = None
        self.pending_pose_command: Optional[str] = None
        self.pending_pose_publish_time: Optional[float] = None
        self.post_task_pose_repeat_remaining = 0
        self.post_task_pose_next_time: Optional[float] = None
        self.last_plan_status_publish_time = 0.0
        self.executor_seen_busy = False
        self.gripper_close_seen = False
        self.executor_busy_after_gripper_close = False
        self.latest_grasp_task_status = "UNKNOWN"
        self.executor_status = "UNKNOWN"
        self.publish_grasp_enable(False)

        self.get_logger().info(
            f"arm_task_manager ready. Listening on {command_topic}; "
            f"pose command topic={pose_command_topic}; legacy JSON topic={legacy_command_topic}. "
            f"Known targets: {', '.join(TARGETS.keys())}")
        if self.start_open_loop_on_startup:
            self.ensure_open_loop()

    def grasp_command_callback(self, msg: String):
        command_text = msg.data.strip()
        if not command_text:
            self.get_logger().warning("Ignore empty /arm/grasp_command target.")
            return
        self.start_grasp_plan(command_text)

    def pose_command_callback(self, msg: String):
        pose_command = self.resolve_pose_command(msg.data.strip())
        if pose_command is None:
            self.publish_status(f"POSE_REJECTED unknown pose={msg.data.strip()}")
            self.get_logger().warning(
                f"Unknown pose command {msg.data!r}; known={list(POSE_COMMAND_ALIASES.keys())}")
            return

        if self.active_spec is not None and not self.task_finished():
            self.publish_status(
                f"POSE_REJECTED busy active={self.active_spec.label} pose={pose_command}")
            self.get_logger().warning(
                f"Reject pose command {pose_command}: grasp task {self.active_spec.label} is active.")
            return

        self.post_task_pose_repeat_remaining = 0
        self.post_task_pose_next_time = None
        self.pending_pose_command = pose_command
        self.pending_pose_publish_time = time.monotonic()
        self.publish_status(f"POSE_PENDING pose={pose_command} delay=0.0s")
        self.get_logger().info(
            f"Queued pose switch command: pose={pose_command}. "
            "Manual open_loop_grasp.launch.py must already be running.")

    @staticmethod
    def resolve_pose_command(command_text: str) -> Optional[str]:
        text = command_text.strip()
        if not text:
            return None
        lowered = text.lower()
        if lowered in POSE_COMMAND_ALIASES:
            return POSE_COMMAND_ALIASES[lowered]
        for key, pose in POSE_COMMAND_ALIASES.items():
            if key and key in text:
                return pose
        return None

    def command_callback(self, msg: String):
        try:
            commands = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Ignore malformed legacy /command JSON: {exc}")
            return

        if not isinstance(commands, list):
            self.get_logger().warning("Ignore legacy /command because payload is not a JSON array.")
            return

        for command in commands:
            if not isinstance(command, dict):
                continue
            if command.get("actuator") != "机械臂" or command.get("action") != "抓取":
                continue
            target = command.get("params", {}).get("target")
            self.start_grasp_plan(str(target))
            return

    def start_grasp_plan(self, command_text: str):
        specs = self.parse_target_sequence(command_text)
        if not specs:
            self.publish_status(f"REJECTED unknown target={command_text}")
            self.publish_plan_status("rejected", None, "unknown_target")
            self.get_logger().warning(
                f"Unknown grasp command {command_text!r}; known={list(TARGETS.keys())}")
            return

        if self.active_spec is not None and not self.task_finished():
            self.publish_status(
                f"BUSY active={self.active_spec.label} rejected_command={command_text}")
            self.publish_plan_status("busy", self.active_spec, "arm_busy")
            self.get_logger().warning(
                f"Reject command {command_text!r}: current task {self.active_spec.label} is active.")
            return

        self.current_plan_id += 1
        self.current_target_index = 0
        self.task_queue = list(specs)
        self.pending_next_start_time = None
        self.publish_plan_status("plan_started", None, "command_accepted")
        self.start_next_queued_target()

    def parse_target_sequence(self, command_text: str) -> List[TargetSpec]:
        normalized = command_text.strip()
        matches = []

        names = list(TARGETS.keys()) + list(TARGET_ALIASES.keys())
        for name in names:
            if not name:
                continue
            index = normalized.find(name)
            if index < 0:
                continue
            canonical = TARGET_ALIASES.get(name, name)
            spec = TARGETS.get(canonical)
            if spec is not None:
                matches.append((index, spec))

        if not matches:
            canonical = TARGET_ALIASES.get(normalized, normalized)
            spec = TARGETS.get(canonical)
            return [spec] if spec is not None else []

        ordered = []
        seen_objects = set()
        for _, spec in sorted(matches, key=lambda item: item[0]):
            if spec.object_name in seen_objects:
                continue
            seen_objects.add(spec.object_name)
            ordered.append(spec)
        return ordered

    def start_next_queued_target(self):
        if not self.task_queue:
            self.publish_active_object("")
            self.publish_plan_status("plan_finished", None, "all_targets_done")
            return

        spec = self.task_queue.pop(0)
        self.current_target_index += 1
        self.start_grasp_for_target(spec)

    def start_grasp_for_target(self, target):
        if isinstance(target, TargetSpec):
            spec = target
            target_label = target.label
        else:
            target_label = str(target)
            canonical = TARGET_ALIASES.get(target_label, target_label)
            spec = TARGETS.get(canonical)
        if spec is None:
            self.publish_status(f"REJECTED unknown target={target_label}")
            self.publish_plan_status("rejected", None, "unknown_target")
            self.get_logger().warning(
                f"Unknown grasp target {target_label!r}; known={list(TARGETS.keys())}")
            return

        if self.active_spec is not None and not self.task_finished():
            self.publish_status(
                f"BUSY active={self.active_spec.label} rejected={spec.label}")
            self.get_logger().warning(
                f"Reject {spec.label}: current task {self.active_spec.label} is still active.")
            return

        self.active_spec = spec
        self.active_since = time.monotonic()
        self.executor_seen_busy = False
        self.gripper_close_seen = False
        self.executor_busy_after_gripper_close = False
        self.latest_grasp_task_status = "WAITING_FOR_TARGET"
        self.publish_grasp_enable(False)
        self.publish_active_object(spec.object_name)

        if self.auto_start_open_loop_for_grasp:
            self.ensure_open_loop()
        else:
            self.get_logger().info(
                "Skip auto-start open_loop_grasp.launch.py; "
                "manual open-loop terminal is expected to be running.")
        self.ensure_transform()
        self.restart_detector(spec)

        self.publish_status(f"GRASPING target={spec.label} topic={spec.topic}")
        self.publish_plan_status("target_started", spec, "target_started")
        self.get_logger().info(
            f"Started grasp task: label={spec.label}, mode={spec.mode}, "
            f"object_name={spec.object_name}, topic={spec.topic}")

    def ensure_open_loop(self):
        if self.process_alive(self.open_loop_proc):
            return
        config_file = self.prepare_open_loop_config()
        launch_args = "task_log_level:=info executor_log_level:=warn"
        if config_file:
            launch_args = f"config_file:={shlex.quote(config_file)} " + launch_args
        cmd = shell_join([
            "source /opt/ros/humble/setup.bash",
            f"source {shlex.quote(self.robot_root)}/robot_ws/install/setup.bash",
            "ros2 launch robot_bringup open_loop_grasp.launch.py " + launch_args,
        ])
        self.open_loop_proc = self.start_shell_process("open_loop_grasp", cmd)

    def prepare_open_loop_config(self) -> str:
        if self.open_loop_config_file:
            return self.open_loop_config_file
        if not self.disable_executor_init_for_auto_grasp:
            return ""

        candidates = [
            Path(self.robot_root) / "robot_ws/install/robot_bringup/share/robot_bringup/config/open_loop_grasp.yaml",
            Path(self.robot_root) / "robot_ws/src/robot_bringup/config/open_loop_grasp.yaml",
        ]
        source_path = next((p for p in candidates if p.exists()), None)
        if source_path is None:
            self.get_logger().warning(
                "Cannot find open_loop_grasp.yaml; launch will use package default config.")
            return ""

        text = source_path.read_text(encoding="utf-8")
        if "do_init: true" not in text:
            return str(source_path)
        text = text.replace("do_init: true", "do_init: false", 1)
        target_path = Path("/tmp/arm_task_open_loop_grasp_no_init.yaml")
        target_path.write_text(text, encoding="utf-8")
        self.get_logger().info(
            f"Use auto-grasp config {target_path} based on {source_path}; "
            "arm_executor_node.do_init=false.")
        return str(target_path)

    def ensure_transform(self):
        if self.process_alive(self.transform_proc):
            return
        script = f"{self.robot_root}/hand_to_eye/camera_to_base_transform.py"
        cmd = shell_join([
            "source /opt/ros/humble/setup.bash",
            f"source {shlex.quote(self.robot_root)}/Orbbec_ws/install/setup.bash",
            f"source {shlex.quote(self.robot_root)}/robot_ws/install/setup.bash",
            f"python3 {shlex.quote(script)} "
            "--ros-args -p active_object_filter_enabled:=true "
            "-p require_grasp_enable_for_output:=true "
            "-p target_hold_sec:=8.0",
        ])
        self.transform_proc = self.start_shell_process("camera_to_base_transform", cmd)

    def restart_detector(self, spec: TargetSpec):
        self.stop_process(self.detector_proc, "detector")
        self.detector_proc = None

        if spec.mode == "yolo":
            if not self.launch_yolo_for_grasp:
                self.get_logger().info(
                    f"Reuse resident YOLO for {spec.label}; "
                    "set launch_yolo_for_grasp:=true to let arm_task_manager start it.")
                return
            classes = f"['{spec.yolo_class}']"
            cmd = shell_join([
                "source /opt/ros/humble/setup.bash",
                "source /opt/tros/humble/setup.bash",
                f"source {shlex.quote(self.robot_root)}/Orbbec_ws/install/setup.bash",
                "ros2 run detect_yolo detect_yolo_node "
                f"--ros-args -p forward_classes:={shlex.quote(classes)}",
            ])
            self.detector_proc = self.start_shell_process(
                f"detect_yolo_{spec.yolo_class}", cmd)
            return

        if spec.mode == "detector":
            cmd = shell_join([
                "source /opt/ros/humble/setup.bash",
                f"source {shlex.quote(self.robot_root)}/Orbbec_ws/install/setup.bash",
                f"ros2 run detector {shlex.quote(spec.detector_node)}",
            ])
            self.detector_proc = self.start_shell_process(spec.detector_node, cmd)
            return

        raise ValueError(f"Unsupported target mode: {spec.mode}")

    def start_shell_process(self, name: str, command: str):
        self.get_logger().info(f"Start {name}: {command}")
        return subprocess.Popen(
            ["bash", "-lc", command],
            start_new_session=True,
            stdout=None,
            stderr=None,
        )

    def executor_status_callback(self, msg: String):
        self.executor_status = msg.data.strip().upper()
        if self.active_spec is None:
            return

        if self.executor_status == "BUSY":
            self.executor_seen_busy = True
            if self.gripper_close_seen:
                self.executor_busy_after_gripper_close = True
            return

        if self.executor_status in ("ERROR", "TIMEOUT", "REJECTED_INVALID_JOINT_LIMIT"):
            spec = self.active_spec
            self.publish_status(f"ERROR target={spec.label} executor={self.executor_status}")
            self.publish_plan_status("target_error", spec, self.executor_status)
            self.get_logger().error(
                f"Grasp task failed: {spec.label}, executor_status={self.executor_status}")
            self.finish_active_task()

    def gripper_cmd_callback(self, msg: String):
        if self.active_spec is None:
            return
        command = msg.data.strip().lower()
        if command != "close":
            return
        if self.gripper_close_seen:
            return
        self.gripper_close_seen = True
        self.executor_busy_after_gripper_close = False
        self.publish_plan_status("gripper_close_sent", self.active_spec, "grasp_close_started")
        self.get_logger().info(
            f"Gripper close command observed for active target: {self.active_spec.label}. "
            "Waiting for /robot_arm/grasp_task_status DONE before reporting completion.")

    def grasp_task_status_callback(self, msg: String):
        status = msg.data.strip()
        if not status:
            return
        self.latest_grasp_task_status = status
        if self.active_spec is None:
            return

        normalized = status.strip().upper()
        if normalized == "DONE" or normalized.startswith("DONE "):
            spec = self.active_spec
            self.publish_status(
                f"DONE target={spec.label} remaining={len(self.task_queue)}")
            self.publish_plan_status("target_done", spec, "grasp_task_done")
            self.get_logger().info(f"Grasp task done: {spec.label}")
            self.finish_active_task()
            return

        if normalized in ("FAILED", "ERROR", "TIMEOUT") or normalized.startswith(("FAILED ", "ERROR ", "TIMEOUT ")):
            spec = self.active_spec
            self.publish_status(f"ERROR target={spec.label} grasp_task={status}")
            self.publish_plan_status("target_error", spec, status)
            self.get_logger().error(
                f"Grasp task failed: {spec.label}, grasp_task_status={status}")
            self.finish_active_task()

    def watchdog(self):
        now = time.monotonic()
        if self.pending_pose_command is not None and self.pending_pose_publish_time is not None:
            if now >= self.pending_pose_publish_time:
                self.publish_pose_switch_command(self.pending_pose_command)
                self.pending_pose_command = None
                self.pending_pose_publish_time = None

        if (
            self.post_task_pose_repeat_remaining > 0
            and self.post_task_pose_next_time is not None
            and now >= self.post_task_pose_next_time
        ):
            self.publish_pose_switch_command(self.post_task_pose_command)
            self.post_task_pose_repeat_remaining -= 1
            if self.post_task_pose_repeat_remaining > 0:
                self.post_task_pose_next_time = (
                    now + max(0.1, self.post_task_pose_repeat_period_sec)
                )
            else:
                self.post_task_pose_next_time = None

        if self.active_spec is None and self.pending_next_start_time is not None:
            if now >= self.pending_next_start_time:
                self.pending_next_start_time = None
                self.start_next_queued_target()
            return

        if self.active_spec is None or self.active_since is None:
            return
        self.publish_active_object(self.active_spec.object_name)
        now = time.monotonic()
        if now - self.last_plan_status_publish_time >= self.plan_status_period_sec:
            self.publish_plan_status("target_active", self.active_spec, "task_running")
            self.last_plan_status_publish_time = now
        age = time.monotonic() - self.active_since
        if age <= self.task_timeout_sec:
            return
        spec = self.active_spec
        self.publish_status(f"TIMEOUT target={spec.label}")
        self.publish_plan_status("target_timeout", spec, "task_timeout")
        self.get_logger().error(f"Grasp task timeout: {spec.label}, age={age:.1f}s")
        self.finish_active_task()

    def finish_active_task(self):
        self.schedule_post_task_pose_switch()
        if self.cleanup_detector_after_done:
            self.stop_process(self.detector_proc, "detector")
            self.detector_proc = None
        self.active_spec = None
        self.active_since = None
        self.executor_seen_busy = False
        self.gripper_close_seen = False
        self.executor_busy_after_gripper_close = False
        self.publish_grasp_enable(False)
        self.publish_active_object("")
        if self.task_queue:
            self.publish_plan_status("waiting_next_target", None, "more_targets_queued")
            self.pending_next_start_time = time.monotonic() + self.next_target_delay_sec
        else:
            self.publish_plan_status("plan_finished", None, "all_targets_done")

    def schedule_post_task_pose_switch(self):
        pose_command = self.post_task_pose_command.strip()
        if not pose_command:
            return
        self.publish_pose_switch_command(pose_command)
        repeat_total = max(1, self.post_task_pose_repeat_count)
        self.post_task_pose_repeat_remaining = repeat_total - 1
        if self.post_task_pose_repeat_remaining > 0:
            self.post_task_pose_next_time = (
                time.monotonic() + max(0.1, self.post_task_pose_repeat_period_sec)
            )
        else:
            self.post_task_pose_next_time = None

    def task_finished(self) -> bool:
        return self.active_spec is None

    def publish_status(self, text: str):
        self.status_pub.publish(String(data=text))

    def publish_pose_switch_command(self, pose_command: str):
        self.pose_switch_pub.publish(String(data=pose_command))
        self.publish_status(f"POSE_SENT pose={pose_command}")
        self.get_logger().info(
            f"Published pose switch command to /robot_arm/pose_switch_cmd: {pose_command}")

    def publish_grasp_enable(self, enabled: bool):
        self.grasp_enable_pub.publish(String(data="true" if enabled else "false"))

    def publish_plan_status(self, event: str, spec: Optional[TargetSpec], reason: str):
        current = self.target_to_dict(spec) if spec is not None else None
        remaining = [self.target_to_dict(item) for item in self.task_queue]
        payload = {
            "version": 1,
            "plan_id": self.current_plan_id,
            "event": event,
            "reason": reason,
            "current_target_index": self.current_target_index,
            "current_target": current,
            "remaining_count": len(remaining),
            "remaining_targets": remaining,
            "next_target": remaining[0] if remaining else None,
            "continue_grasp": len(remaining) > 0,
            "task_finished": event == "plan_finished" and not remaining,
        }
        self.plan_status_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":"))))

    def publish_active_object(self, object_name: str):
        self.active_object_pub.publish(String(data=object_name))

    @staticmethod
    def target_to_dict(spec: TargetSpec) -> dict:
        return {
            "label": spec.label,
            "object_name": spec.object_name,
            "topic": spec.topic,
            "mode": spec.mode,
            "yolo_class": spec.yolo_class,
        }

    @staticmethod
    def process_alive(proc: Optional[subprocess.Popen]) -> bool:
        return proc is not None and proc.poll() is None

    def stop_process(self, proc: Optional[subprocess.Popen], name: str):
        if proc is None or proc.poll() is not None:
            return
        self.get_logger().info(f"Stop {name} pid={proc.pid}")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=5.0)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass

    def destroy_node(self):
        self.stop_process(self.detector_proc, "detector")
        self.stop_process(self.transform_proc, "camera_to_base_transform")
        self.stop_process(self.open_loop_proc, "open_loop_grasp")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmTaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
