#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish one chassis coordination topic from arm-side visual state.

Inputs:
  /visual_target_base_raw (robot_msgs/msg/VisualTarget, target in base_link)
  /arm_task/status (std_msgs/msg/String, optional high-level task status)
  /arm_task/grasp_plan_status (std_msgs/msg/String, JSON task queue status)
  /robot_arm/executor_status (std_msgs/msg/String, low-level arm status)
  Graspable detector PointStamped topics

Output:
  /chassis/approach_request (std_msgs/msg/String, JSON)

The chassis only needs to subscribe to one topic. The JSON payload tells it
whether to approach, stop approaching, continue after a completed grasp, or end
the current grasp-transfer cycle.
"""

import json
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from robot_msgs.msg import VisualTarget


@dataclass
class TargetSnapshot:
    msg: VisualTarget
    seen_time_sec: float


@dataclass
class CameraPointSnapshot:
    msg: PointStamped
    topic: str
    object_name: str
    seen_time_sec: float


class ChassisApproachPublisher(Node):
    """Publish continuous chassis state and grasp-cycle completion events."""

    def __init__(self):
        super().__init__('chassis_approach_publisher')

        self._declare_parameters()
        self._load_parameters()

        self.sequence = 0
        self.grasp_event_sequence = 0
        self.latest_target: Optional[TargetSnapshot] = None
        self.last_near_grasp_publish_time_sec = 0.0
        self.last_near_grasp_object_name = ''
        self.last_post_grasp_pose_publish_time_sec = 0.0
        self.current_target_key = ''
        self.ready_since_sec: Optional[float] = None
        self.near_grasp_requested = False
        self.near_grasp_request_time_sec: Optional[float] = None
        self.grasp_enable_state: Optional[bool] = None
        self.last_graspable_seen_time_by_object: Dict[str, float] = {}
        self.latest_camera_point_by_object: Dict[str, CameraPointSnapshot] = {}
        self.camera_topic_by_object: Dict[str, str] = {}
        self.executor_seen_busy = False

        self.pending_grasp_done_time_sec: Optional[float] = None
        self.pending_grasp_done_source = ''
        self.pending_grasp_done_status = ''
        self.latest_plan_status: Optional[dict] = None
        self.latest_plan_status_time_sec: Optional[float] = None
        self.active_grasp_event_payload: Optional[dict] = None
        self.active_grasp_event_until_sec = 0.0
        self.grasp_execution_started = False
        self.grasp_enable_ready_latched = False
        self.last_state_log_key = ''
        self.last_state_log_time_sec = 0.0

        self.target_sub = self.create_subscription(
            VisualTarget,
            self.input_topic,
            self.target_callback,
            10,
        )
        self.arm_task_status_sub = self.create_subscription(
            String,
            self.arm_task_status_topic,
            self.arm_task_status_callback,
            10,
        )
        self.plan_status_sub = self.create_subscription(
            String,
            self.plan_status_topic,
            self.plan_status_callback,
            10,
        )
        self.executor_status_sub = self.create_subscription(
            String,
            self.executor_status_topic,
            self.executor_status_callback,
            10,
        )
        self._create_graspable_detector_subscriptions()

        self.request_pub = self.create_publisher(String, self.output_topic, 10)
        self.pose_switch_pub = self.create_publisher(String, self.pose_switch_cmd_topic, 10)
        self.grasp_enable_pub = self.create_publisher(
            String,
            self.grasp_enable_topic,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self.publish_timer = self.create_timer(
            1.0 / max(self.state_publish_rate_hz, 0.1),
            self.publish_current_state,
        )
        self.publish_grasp_enable(False)

        self.get_logger().info(f'Listening: {self.input_topic}')
        self.get_logger().info(f'Listening: {self.arm_task_status_topic}')
        self.get_logger().info(f'Listening: {self.plan_status_topic}')
        self.get_logger().info(f'Listening: {self.executor_status_topic}')
        self.get_logger().info(f'Publishing continuously: {self.output_topic}')
        self.get_logger().info(f'Publishing grasp enable gate: {self.grasp_enable_topic}')
        self.get_logger().info(
            f'Auto near_grasp switch={self.auto_switch_near_grasp_when_ready}; '
            f'command_topic={self.pose_switch_cmd_topic}')
        self.get_logger().info(
            f'Auto post-grasp pose switch={self.auto_switch_pre_grasp_after_done}; '
            f'post_grasp_command={self.post_grasp_pose_command}')
        self.get_logger().info(
            f'Require active grasp plan for chassis/grasp trigger='
            f'{self.require_active_grasp_plan}')
        self.get_logger().info(
            'Reach config: '
            f'workspace_x=[{self.workspace_x_min_m:.3f}, {self.workspace_x_max_m:.3f}], '
            f'near_grasp_switch_x_m={self.near_grasp_switch_x_m:.3f}, '
            f'chassis_stop_x_m={self.chassis_stop_x_m:.3f}, '
            f'grasp_ready_x_m={self.grasp_ready_x_m:.3f}, '
            f'workspace_y_abs_m={self.workspace_y_abs_max_m:.3f}, '
            f'desired_target_x_m={self.desired_target_x_m:.3f}'
        )

    def _declare_parameters(self):
        self.declare_parameter('input_topic', '/visual_target_base_raw')
        self.declare_parameter('output_topic', '/chassis/approach_request')
        self.declare_parameter('arm_task_status_topic', '/arm_task/status')
        self.declare_parameter('plan_status_topic', '/arm_task/grasp_plan_status')
        self.declare_parameter('executor_status_topic', '/robot_arm/executor_status')
        self.declare_parameter('pose_switch_cmd_topic', '/robot_arm/pose_switch_cmd')
        self.declare_parameter('grasp_enable_topic', '/arm_task/grasp_enable')

        # Match the current open_loop_grasp.yaml defaults on the robot.
        self.declare_parameter('workspace_x_min_m', 0.10)
        self.declare_parameter('workspace_x_max_m', 0.68)
        self.declare_parameter('workspace_y_abs_max_m', 0.38)
        self.declare_parameter('workspace_z_min_m', 0.02)
        self.declare_parameter('workspace_z_max_m', 0.75)

        self.declare_parameter('desired_target_x_m', 0.45)
        self.declare_parameter('near_grasp_switch_x_m', 0.90)
        self.declare_parameter('chassis_stop_x_m', 0.78)
        self.declare_parameter('grasp_ready_x_m', 0.68)
        self.declare_parameter('min_move_forward_m', 0.03)
        self.declare_parameter('max_move_forward_m', 0.50)
        self.declare_parameter('max_move_left_m', 0.30)

        self.declare_parameter('min_confidence', 0.70)
        self.declare_parameter('require_stable_target', True)
        self.declare_parameter('include_lateral_request', True)
        self.declare_parameter('target_timeout_sec', 1.5)
        self.declare_parameter('detector_target_timeout_sec', 1.5)

        self.declare_parameter('state_publish_rate_hz', 2.0)
        self.declare_parameter('post_grasp_observe_delay_sec', 2.0)
        self.declare_parameter('grasp_event_repeat_sec', 3.0)
        self.declare_parameter('auto_switch_near_grasp_when_ready', True)
        self.declare_parameter('near_grasp_command', 'near_grasp')
        self.declare_parameter('near_grasp_publish_cooldown_sec', 3.0)
        self.declare_parameter('near_grasp_to_grasp_enable_delay_sec', 2.5)
        self.declare_parameter('ready_hold_sec', 0.5)
        self.declare_parameter('require_active_grasp_plan', True)
        self.declare_parameter('auto_switch_pre_grasp_after_done', True)
        self.declare_parameter('post_grasp_pose_command', 'pre_grasp')
        self.declare_parameter('graspable_detector_topics', [
            '/duck_position:duck',
            '/box_position:box',
            '/red_circle_position:red_circle',
            '/detect_yolo/apple_position:apple',
            '/detect_yolo/banana_position:banana',
            '/detect_yolo/bottle_position:bottle',
            '/detect_yolo/cake_position:cake',
        ])

    def _load_parameters(self):
        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.arm_task_status_topic = str(self.get_parameter('arm_task_status_topic').value)
        self.plan_status_topic = str(self.get_parameter('plan_status_topic').value)
        self.executor_status_topic = str(self.get_parameter('executor_status_topic').value)
        self.pose_switch_cmd_topic = str(self.get_parameter('pose_switch_cmd_topic').value)
        self.grasp_enable_topic = str(self.get_parameter('grasp_enable_topic').value)

        self.workspace_x_min_m = float(self.get_parameter('workspace_x_min_m').value)
        self.workspace_x_max_m = float(self.get_parameter('workspace_x_max_m').value)
        self.workspace_y_abs_max_m = float(self.get_parameter('workspace_y_abs_max_m').value)
        self.workspace_z_min_m = float(self.get_parameter('workspace_z_min_m').value)
        self.workspace_z_max_m = float(self.get_parameter('workspace_z_max_m').value)

        self.desired_target_x_m = float(self.get_parameter('desired_target_x_m').value)
        self.near_grasp_switch_x_m = float(
            self.get_parameter('near_grasp_switch_x_m').value)
        self.chassis_stop_x_m = float(self.get_parameter('chassis_stop_x_m').value)
        self.grasp_ready_x_m = float(self.get_parameter('grasp_ready_x_m').value)
        self.min_move_forward_m = float(self.get_parameter('min_move_forward_m').value)
        self.max_move_forward_m = float(self.get_parameter('max_move_forward_m').value)
        self.max_move_left_m = float(self.get_parameter('max_move_left_m').value)

        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.require_stable_target = bool(self.get_parameter('require_stable_target').value)
        self.include_lateral_request = bool(self.get_parameter('include_lateral_request').value)
        self.target_timeout_sec = float(self.get_parameter('target_timeout_sec').value)
        self.detector_target_timeout_sec = float(
            self.get_parameter('detector_target_timeout_sec').value)

        self.state_publish_rate_hz = float(self.get_parameter('state_publish_rate_hz').value)
        self.post_grasp_observe_delay_sec = float(
            self.get_parameter('post_grasp_observe_delay_sec').value)
        self.grasp_event_repeat_sec = float(self.get_parameter('grasp_event_repeat_sec').value)
        self.auto_switch_near_grasp_when_ready = bool(
            self.get_parameter('auto_switch_near_grasp_when_ready').value)
        self.near_grasp_command = str(self.get_parameter('near_grasp_command').value)
        self.near_grasp_publish_cooldown_sec = float(
            self.get_parameter('near_grasp_publish_cooldown_sec').value)
        self.near_grasp_to_grasp_enable_delay_sec = float(
            self.get_parameter('near_grasp_to_grasp_enable_delay_sec').value)
        self.ready_hold_sec = float(self.get_parameter('ready_hold_sec').value)
        self.require_active_grasp_plan = bool(
            self.get_parameter('require_active_grasp_plan').value)
        self.auto_switch_pre_grasp_after_done = bool(
            self.get_parameter('auto_switch_pre_grasp_after_done').value)
        self.post_grasp_pose_command = str(self.get_parameter('post_grasp_pose_command').value)
        raw_topics = self.get_parameter('graspable_detector_topics').value
        self.graspable_detector_topics = self._parse_graspable_detector_topics(raw_topics)

    def _create_graspable_detector_subscriptions(self):
        self.detector_subs = []
        for topic, object_name in self.graspable_detector_topics:
            self.camera_topic_by_object[object_name] = topic
            self.detector_subs.append(
                self.create_subscription(
                    PointStamped,
                    topic,
                    lambda msg, name=object_name, topic_name=topic:
                    self.graspable_detector_callback(msg, name, topic_name),
                    10,
                )
            )

    @staticmethod
    def _parse_graspable_detector_topics(raw_topics):
        parsed = []
        for item in raw_topics:
            text = str(item).strip()
            if not text:
                continue
            if ':' in text:
                topic, object_name = text.split(':', 1)
            else:
                topic = text
                object_name = text.rsplit('/', 1)[-1].replace('_position', '')
            parsed.append((topic.strip(), object_name.strip()))
        return parsed

    def target_callback(self, msg: VisualTarget):
        if msg.object_name:
            self.last_graspable_seen_time_by_object[msg.object_name] = self.now_sec()

        if msg.confidence < self.min_confidence:
            return
        if self.require_stable_target and not bool(msg.is_stable):
            return

        self.latest_target = TargetSnapshot(msg=msg, seen_time_sec=self.now_sec())

    def graspable_detector_callback(self, msg: PointStamped, object_name: str, topic: str):
        now = self.now_sec()
        self.last_graspable_seen_time_by_object[object_name] = now
        self.latest_camera_point_by_object[object_name] = CameraPointSnapshot(
            msg=msg,
            topic=topic,
            object_name=object_name,
            seen_time_sec=now,
        )

    def arm_task_status_callback(self, msg: String):
        status = msg.data.strip()
        if status.startswith((
            'DONE target=',
            'TIMEOUT target=',
            'ERROR target=',
        )):
            self.mark_arm_done('/arm_task/status', status)

    def plan_status_callback(self, msg: String):
        try:
            self.latest_plan_status = json.loads(msg.data)
            self.latest_plan_status_time_sec = self.now_sec()
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f'Ignore malformed plan status JSON: {exc}')
            return

        event = str(self.latest_plan_status.get('event', '')).strip()
        if event == 'target_started':
            self.grasp_execution_started = False
        elif event == 'gripper_close_sent':
            self.grasp_execution_started = True
            self.publish_grasp_enable(False)
        elif self.plan_status_is_terminal_or_waiting(self.latest_plan_status):
            self.grasp_execution_started = False
            self.reset_grasp_gate('', publish_disable=True)

    def executor_status_callback(self, msg: String):
        status = msg.data.strip().upper()
        if status == 'BUSY':
            self.executor_seen_busy = True
            return

        if self.executor_seen_busy and status in ('DONE', 'IDLE'):
            self.executor_seen_busy = False
            self.get_logger().debug(
                f'Ignore executor {status} for grasp completion; waiting for /arm_task/status.')

    def mark_arm_done(self, source_topic: str, status: str):
        now = self.now_sec()
        self.publish_post_grasp_pose(now)
        self.pending_grasp_done_time_sec = now
        self.pending_grasp_done_source = source_topic
        self.pending_grasp_done_status = status
        self.get_logger().info(
            f'Arm grasp terminal status. Wait {self.post_grasp_observe_delay_sec:.1f}s '
            f'then report whether more graspable objects remain. '
            f'source={source_topic}, status={status!r}'
        )

    def publish_current_state(self):
        now = self.now_sec()
        self.maybe_build_grasp_done_event(now)

        if self.active_grasp_event_payload and now <= self.active_grasp_event_until_sec:
            payload = dict(self.active_grasp_event_payload)
        else:
            payload = self.build_approach_state(now)

        self.sequence += 1
        payload['sequence'] = self.sequence
        payload['stamp_sec'] = round(now, 3)

        out = String()
        out.data = json.dumps(payload, separators=(',', ':'))
        self.request_pub.publish(out)
        self.log_published_state(payload, now)

    def maybe_build_grasp_done_event(self, now: float):
        if self.pending_grasp_done_time_sec is None:
            return
        if now - self.pending_grasp_done_time_sec < self.post_grasp_observe_delay_sec:
            return

        plan_status = self.current_plan_status(now)
        if plan_status is not None:
            remaining_targets = plan_status.get('remaining_targets') or []
            remaining_objects = [
                str(item.get('object_name', ''))
                for item in remaining_targets
                if isinstance(item, dict) and item.get('object_name')
            ]
            continue_grasp = bool(plan_status.get('continue_grasp', len(remaining_objects) > 0))
            current_target = plan_status.get('current_target')
            next_target = plan_status.get('next_target')
            plan_id = plan_status.get('plan_id')
        else:
            remaining_objects = self.recent_graspable_objects(now)
            continue_grasp = len(remaining_objects) > 0
            current_target = None
            next_target = None
            plan_id = None

        camera_payload = self.camera_payload_for_done_event(
            now,
            next_target,
            current_target,
        )
        self.grasp_event_sequence += 1
        self.active_grasp_event_payload = {
            'version': 1,
            'plan_id': plan_id,
            'event': 'grasp_cycle_done',
            'request_id': f'grasp_cycle_done_{self.grasp_event_sequence}',
            'chassis_action': 'wait_next_grasp' if continue_grasp else 'grasp_task_finished',
            'approach_cmd': 2,
            'need_approach': False,
            'stop_chassis': True,
            'grasp_cycle_done': True,
            'continue_grasp': continue_grasp,
            'task_finished': not continue_grasp,
            'all_grasp_done': not continue_grasp,
            'need_return_for_next_grasp': continue_grasp,
            'remaining_graspable_count': len(remaining_objects),
            'remaining_graspable_objects': remaining_objects,
            'current_target': current_target,
            'next_target': next_target,
            'remaining_requested_targets': remaining_targets if plan_status is not None else [],
            **camera_payload,
            'arm_status_source': self.pending_grasp_done_source,
            'arm_status': self.pending_grasp_done_status,
            'post_grasp_observe_delay_sec': self.post_grasp_observe_delay_sec,
            'reason': 'more_graspable_objects_after_grasp'
            if continue_grasp else 'no_graspable_objects_after_grasp',
        }
        self.active_grasp_event_until_sec = now + self.grasp_event_repeat_sec
        self.pending_grasp_done_time_sec = None
        self.reset_grasp_gate('', publish_disable=True)
        self.get_logger().info(
            'Grasp done event: '
            f'continue_grasp={continue_grasp}, remaining={remaining_objects}'
        )

    def build_approach_state(self, now: float):
        recent_objects = self.recent_graspable_objects(now)
        plan_status = self.current_plan_status(now)
        current_plan_target = plan_status.get('current_target') if plan_status else None
        next_plan_target = plan_status.get('next_target') if plan_status else None
        remaining_requested_targets = (
            plan_status.get('remaining_targets') if plan_status else []
        ) or []
        target = self.current_target(now)
        if not self.plan_status_allows_chassis_control(plan_status):
            self.reset_grasp_gate('', publish_disable=True)
            object_name = (
                self.object_name_from_plan_target(current_plan_target)
                or self.object_name_from_plan_target(next_plan_target)
            )
            camera_payload = self.camera_payload_for_object(object_name, now)
            return {
                'version': 1,
                'plan_id': plan_status.get('plan_id') if plan_status else None,
                'event': 'approach_state',
                'request_id': 'approach_state',
                'chassis_action': 'wait_for_active_grasp_plan',
                'approach_cmd': 2,
                'need_approach': False,
                'stop_chassis': True,
                'ready_for_arm_grasp': False,
                'all_grasp_done': bool(
                    plan_status.get('task_finished', False)) if plan_status else False,
                'need_return_for_next_grasp': False,
                'scene_has_graspable': len(recent_objects) > 0,
                'remaining_graspable_count': len(recent_objects),
                'remaining_graspable_objects': recent_objects,
                'current_target': current_plan_target,
                'next_target': next_plan_target,
                'remaining_requested_targets': remaining_requested_targets,
                **camera_payload,
                'move_forward_m': 0.0,
                'move_left_m': 0.0,
                'grasp_trigger_enabled': self.grasp_enable_state,
                'require_active_grasp_plan': self.require_active_grasp_plan,
                'reason': self.inactive_plan_reason(plan_status),
            }

        if target is None:
            object_name = (
                self.last_near_grasp_object_name
                or self.object_name_from_plan_target(current_plan_target)
            )
            if object_name:
                target_key = self.make_target_key(plan_status, object_name)
                if target_key != self.current_target_key:
                    self.reset_grasp_gate(target_key, publish_disable=True)
                self.maybe_publish_near_grasp_and_enable_grasp(
                    object_name,
                    now,
                    allow_grasp_enable=self.grasp_enable_ready_latched,
                )

            if self.near_grasp_requested:
                self.maybe_publish_near_grasp_and_enable_grasp(
                    object_name,
                    now,
                    allow_grasp_enable=self.grasp_enable_ready_latched,
                )
                camera_payload = self.camera_payload_for_object(object_name, now)
                return {
                    'version': 1,
                    'plan_id': plan_status.get('plan_id') if plan_status else None,
                    'event': 'approach_state',
                    'request_id': 'approach_state',
                    'chassis_action': 'stop_approach',
                    'approach_cmd': 2,
                    'need_approach': False,
                    'stop_chassis': True,
                    'ready_for_arm_grasp': self.grasp_enable_ready_latched,
                    'all_grasp_done': False,
                    'need_return_for_next_grasp': False,
                    'scene_has_graspable': len(recent_objects) > 0,
                    'remaining_graspable_count': len(recent_objects),
                    'remaining_graspable_objects': recent_objects,
                    'current_target': current_plan_target,
                    'next_target': next_plan_target,
                    'remaining_requested_targets': remaining_requested_targets,
                    **camera_payload,
                    'move_forward_m': 0.0,
                    'move_left_m': 0.0,
                    'grasp_trigger_enabled': self.grasp_enable_state,
                    'near_grasp_latched': True,
                    'grasp_enable_ready_latched': self.grasp_enable_ready_latched,
                    'reason': 'visual_target_lost_after_chassis_stop_keep_grasp_flow'
                    if self.grasp_enable_ready_latched
                    else 'visual_target_lost_after_near_grasp_before_chassis_stop',
                }

            camera_payload = self.camera_payload_for_object(object_name, now)
            return {
                'version': 1,
                'plan_id': plan_status.get('plan_id') if plan_status else None,
                'event': 'approach_state',
                'request_id': 'approach_state',
                'chassis_action': 'stop_approach',
                'approach_cmd': 2,
                'need_approach': False,
                'stop_chassis': True,
                'ready_for_arm_grasp': False,
                'all_grasp_done': False,
                'need_return_for_next_grasp': False,
                'scene_has_graspable': len(recent_objects) > 0,
                'remaining_graspable_count': len(recent_objects),
                'remaining_graspable_objects': recent_objects,
                'current_target': current_plan_target,
                'next_target': next_plan_target,
                'remaining_requested_targets': remaining_requested_targets,
                **camera_payload,
                'move_forward_m': 0.0,
                'move_left_m': 0.0,
                'grasp_trigger_enabled': self.grasp_enable_state,
                'near_grasp_latched': self.near_grasp_requested,
                'grasp_enable_ready_latched': self.grasp_enable_ready_latched,
                'reason': 'no_recent_visual_target_base_stop_and_force_near_grasp',
            }

        msg = target.msg
        x = float(msg.x)
        y = float(msg.y)
        z = float(msg.z)
        target_key = self.make_target_key(plan_status, msg.object_name)
        if target_key != self.current_target_key:
            self.reset_grasp_gate(target_key, publish_disable=True)

        in_workspace = self._target_is_in_arm_workspace(x, y, z)
        ready_for_grasp = self._target_is_ready_for_grasp(x, y, z)
        target_too_far = x > self.chassis_stop_x_m
        switch_ready_for_near_grasp = self._target_is_ready_for_near_grasp_switch(x, y, z)
        stop_ready_for_grasp_enable = self._target_is_ready_for_grasp_enable(x, y, z)
        lateral_needs_adjust = (
            target_too_far
            and self.include_lateral_request
            and abs(y) > self.workspace_y_abs_max_m
        )

        move_forward_m = 0.0
        move_left_m = 0.0
        if target_too_far:
            move_forward_m = self._clamp(
                x - self.desired_target_x_m,
                0.0,
                self.max_move_forward_m,
            )
            if move_forward_m < self.min_move_forward_m:
                move_forward_m = 0.0
        if self.include_lateral_request and (target_too_far or lateral_needs_adjust):
            move_left_m = self._clamp(y, -self.max_move_left_m, self.max_move_left_m)

        need_approach = target_too_far
        camera_payload = self.camera_payload_for_object(msg.object_name, now)
        if stop_ready_for_grasp_enable:
            self.grasp_enable_ready_latched = True

        if not need_approach:
            self.ready_since_sec = self.ready_since_sec or now
            ready_hold_ok = now - self.ready_since_sec >= self.ready_hold_sec
            if ready_hold_ok:
                self.maybe_publish_near_grasp_and_enable_grasp(
                    msg.object_name,
                    now,
                    allow_grasp_enable=self.grasp_enable_ready_latched,
                )
        else:
            self.ready_since_sec = None
            self.publish_grasp_enable(False)
        if stop_ready_for_grasp_enable:
            reason = 'target_reached_chassis_stop_ready_for_grasp_enable'
        elif switch_ready_for_near_grasp:
            reason = 'target_reached_near_grasp_switch_x_keep_approach'
        elif need_approach:
            reason = 'target_before_chassis_stop_x'
        else:
            reason = 'target_stopped_but_outside_yz_workspace'
        return {
            'version': 1,
            'plan_id': plan_status.get('plan_id') if plan_status else None,
            'event': 'approach_state',
            'request_id': 'approach_state',
            'chassis_action': 'approach_target' if need_approach else 'stop_approach',
            'approach_cmd': 1 if need_approach else 2,
            'need_approach': need_approach,
            'stop_chassis': not need_approach,
            'ready_for_arm_grasp': stop_ready_for_grasp_enable,
            'all_grasp_done': False,
            'need_return_for_next_grasp': False,
            'scene_has_graspable': len(recent_objects) > 0,
            'remaining_graspable_count': len(recent_objects),
            'remaining_graspable_objects': recent_objects,
            'current_target': current_plan_target,
            'next_target': next_plan_target,
            'remaining_requested_targets': remaining_requested_targets,
            'target_id': msg.target_id,
            'object_name': msg.object_name,
            **camera_payload,
            'frame_id': msg.header.frame_id,
            'target_x_m': x,
            'target_y_m': y,
            'target_z_m': z,
            'target_age_sec': round(now - target.seen_time_sec, 3),
            'workspace_x_max_m': self.workspace_x_max_m,
            'workspace_y_abs_max_m': self.workspace_y_abs_max_m,
            'desired_target_x_m': self.desired_target_x_m,
            'near_grasp_switch_x_m': self.near_grasp_switch_x_m,
            'chassis_stop_x_m': self.chassis_stop_x_m,
            'grasp_ready_x_m': self.grasp_ready_x_m,
            'ready_hold_sec': self.ready_hold_sec,
            'near_grasp_to_grasp_enable_delay_sec':
            self.near_grasp_to_grasp_enable_delay_sec,
            'target_in_arm_workspace': in_workspace,
            'target_in_strict_grasp_range': ready_for_grasp,
            'near_grasp_switch_ready': switch_ready_for_near_grasp,
            'grasp_enable_ready_latched': self.grasp_enable_ready_latched,
            'near_grasp_latched': self.near_grasp_requested,
            'target_too_far': target_too_far,
            'lateral_needs_adjust': lateral_needs_adjust,
            'grasp_trigger_enabled': self.grasp_enable_state,
            'move_forward_m': move_forward_m,
            'move_left_m': move_left_m,
            'confidence': float(msg.confidence),
            'reason': reason,
        }

    def maybe_publish_near_grasp_and_enable_grasp(
        self,
        object_name: str,
        now: float,
        allow_grasp_enable: bool,
    ):
        if not self.auto_switch_near_grasp_when_ready:
            return
        if not object_name:
            return

        if not self.near_grasp_requested:
            self.publish_near_grasp(object_name, now)
            self.near_grasp_requested = True
            self.near_grasp_request_time_sec = now
            self.publish_grasp_enable(False)
            return

        if (
            allow_grasp_enable
            and
            not self.grasp_enable_state
            and self.near_grasp_request_time_sec is not None
            and now - self.near_grasp_request_time_sec
            >= self.near_grasp_to_grasp_enable_delay_sec
        ):
            self.publish_grasp_enable(True)
            self.get_logger().info(
                'Near_grasp settle delay elapsed. Enabled /visual_target_base '
                'for open_loop grasp.')
            return

        cooldown_elapsed = now - self.last_near_grasp_publish_time_sec
        if (
            not self.grasp_enable_state
            and cooldown_elapsed >= self.near_grasp_publish_cooldown_sec
        ):
            self.publish_near_grasp(object_name, now)

    def publish_near_grasp(self, object_name: str, now: float):
        out = String()
        out.data = self.near_grasp_command
        self.pose_switch_pub.publish(out)
        self.last_near_grasp_publish_time_sec = now
        self.last_near_grasp_object_name = object_name
        self.get_logger().info(
            f'Visual target is ready for grasp. Published pose switch: {self.near_grasp_command}'
        )

    def publish_grasp_enable(self, enabled: bool):
        if enabled == self.grasp_enable_state:
            return
        self.grasp_enable_state = enabled
        self.grasp_enable_pub.publish(String(data='true' if enabled else 'false'))
        self.get_logger().info(f'Published grasp enable: {self.grasp_enable_state}')

    def reset_grasp_gate(self, target_key: str, publish_disable: bool):
        if target_key == self.current_target_key and not publish_disable:
            return
        self.current_target_key = target_key
        self.ready_since_sec = None
        self.near_grasp_requested = False
        self.near_grasp_request_time_sec = None
        self.last_near_grasp_object_name = ''
        self.grasp_enable_ready_latched = False
        if publish_disable:
            self.publish_grasp_enable(False)

    @staticmethod
    def make_target_key(plan_status: Optional[dict], object_name: str) -> str:
        if not plan_status:
            return object_name
        plan_id = plan_status.get('plan_id')
        target_index = plan_status.get('current_target_index')
        return f'{plan_id}:{target_index}:{object_name}'

    def publish_post_grasp_pose(self, now: float):
        if not self.auto_switch_pre_grasp_after_done:
            return
        if not self.post_grasp_pose_command:
            return
        if now - self.last_post_grasp_pose_publish_time_sec < 1.0:
            return

        out = String()
        out.data = self.post_grasp_pose_command
        self.pose_switch_pub.publish(out)
        self.last_post_grasp_pose_publish_time_sec = now
        self.last_near_grasp_object_name = ''
        self.get_logger().info(
            f'Published post-grasp pose switch: {self.post_grasp_pose_command}'
        )

    def current_target(self, now: float) -> Optional[TargetSnapshot]:
        if self.latest_target is None:
            return None
        if now - self.latest_target.seen_time_sec > self.target_timeout_sec:
            return None
        return self.latest_target

    def current_plan_status(self, now: float) -> Optional[dict]:
        if self.latest_plan_status is None or self.latest_plan_status_time_sec is None:
            return None
        if now - self.latest_plan_status_time_sec > 30.0:
            return None
        return self.latest_plan_status

    def plan_status_allows_chassis_control(self, plan_status: Optional[dict]) -> bool:
        if not self.require_active_grasp_plan:
            return True
        if not isinstance(plan_status, dict):
            return False
        if self.grasp_execution_started:
            return False
        if bool(plan_status.get('task_finished', False)):
            return False

        event = str(plan_status.get('event', '')).strip()
        if event not in ('target_started', 'target_active'):
            return False
        return isinstance(plan_status.get('current_target'), dict)

    @staticmethod
    def plan_status_is_terminal_or_waiting(plan_status: Optional[dict]) -> bool:
        if not isinstance(plan_status, dict):
            return False
        if bool(plan_status.get('task_finished', False)):
            return True
        event = str(plan_status.get('event', '')).strip()
        return event in (
            'target_done',
            'target_timeout',
            'target_error',
            'plan_finished',
            'waiting_next_target',
            'rejected',
        )

    def inactive_plan_reason(self, plan_status: Optional[dict]) -> str:
        if not self.require_active_grasp_plan:
            return 'active_grasp_plan_not_required'
        if not isinstance(plan_status, dict):
            return 'no_active_grasp_plan'
        if self.grasp_execution_started:
            return 'arm_grasp_execution_started'
        if bool(plan_status.get('task_finished', False)):
            return 'grasp_plan_finished'
        event = str(plan_status.get('event', '')).strip()
        if event:
            return f'grasp_plan_event_{event}_not_active'
        return 'grasp_plan_not_active'

    def camera_payload_for_done_event(
        self,
        now: float,
        next_target: Optional[dict],
        current_target: Optional[dict],
    ) -> dict:
        object_name = self.object_name_from_plan_target(next_target)
        if object_name is None:
            object_name = self.object_name_from_plan_target(current_target)
        return self.camera_payload_for_object(object_name, now)

    def camera_payload_for_object(self, object_name: Optional[str], now: float) -> dict:
        payload = {
            'camera_topic': self.camera_topic_by_object.get(object_name or '', ''),
            'camera_frame_id': '',
            'camera_x_m': None,
            'camera_y_m': None,
            'camera_z_m': None,
            'camera_point_age_sec': None,
            'point_stamped': None,
        }
        if not object_name:
            return payload

        snapshot = self.latest_camera_point_by_object.get(object_name)
        if snapshot is None:
            return payload
        if now - snapshot.seen_time_sec > self.detector_target_timeout_sec:
            return payload

        msg = snapshot.msg
        stamp = msg.header.stamp
        camera_x = float(msg.point.x)
        camera_y = float(msg.point.y)
        camera_z = float(msg.point.z)
        payload.update({
            'camera_topic': snapshot.topic,
            'camera_frame_id': msg.header.frame_id,
            'camera_x_m': camera_x,
            'camera_y_m': camera_y,
            'camera_z_m': camera_z,
            'camera_point_age_sec': round(now - snapshot.seen_time_sec, 3),
            'point_stamped': {
                'header': {
                    'stamp': {
                        'sec': int(stamp.sec),
                        'nanosec': int(stamp.nanosec),
                    },
                    'frame_id': msg.header.frame_id,
                },
                'point': {
                    'x': camera_x,
                    'y': camera_y,
                    'z': camera_z,
                },
            },
        })
        return payload

    @staticmethod
    def object_name_from_plan_target(target: Optional[dict]) -> Optional[str]:
        if not isinstance(target, dict):
            return None
        object_name = target.get('object_name')
        return str(object_name) if object_name else None

    def recent_graspable_objects(self, now: float) -> List[str]:
        return sorted(
            object_name
            for object_name, seen_time in self.last_graspable_seen_time_by_object.items()
            if now - seen_time <= self.detector_target_timeout_sec
        )

    def _target_is_in_arm_workspace(self, x: float, y: float, z: float) -> bool:
        if x < self.workspace_x_min_m or x > self.workspace_x_max_m:
            return False
        if abs(y) > self.workspace_y_abs_max_m:
            return False
        if z < self.workspace_z_min_m or z > self.workspace_z_max_m:
            return False
        return True

    def _target_is_ready_for_grasp(self, x: float, y: float, z: float) -> bool:
        if x < self.workspace_x_min_m or x > min(self.workspace_x_max_m, self.grasp_ready_x_m):
            return False
        if abs(y) > self.workspace_y_abs_max_m:
            return False
        if z < self.workspace_z_min_m or z > self.workspace_z_max_m:
            return False
        return True

    def _target_is_ready_for_near_grasp_switch(
        self,
        x: float,
        y: float,
        z: float,
    ) -> bool:
        if x < self.workspace_x_min_m or x > self.near_grasp_switch_x_m:
            return False
        if abs(y) > self.workspace_y_abs_max_m:
            return False
        if z < self.workspace_z_min_m or z > self.workspace_z_max_m:
            return False
        return True

    def _target_is_ready_for_grasp_enable(self, x: float, y: float, z: float) -> bool:
        if x < self.workspace_x_min_m or x > self.chassis_stop_x_m:
            return False
        if abs(y) > self.workspace_y_abs_max_m:
            return False
        if z < self.workspace_z_min_m or z > self.workspace_z_max_m:
            return False
        return True

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(value, high))

    def log_published_state(self, payload: dict, now: float):
        key = ':'.join(str(payload.get(name, '')) for name in (
            'event',
            'chassis_action',
            'approach_cmd',
            'object_name',
            'reason',
            'plan_id',
        ))
        periodic = (
            payload.get('event') == 'approach_state'
            and now - self.last_state_log_time_sec >= 5.0
        )
        if key == self.last_state_log_key and not periodic:
            return

        self.last_state_log_key = key
        self.last_state_log_time_sec = now
        self.get_logger().info(
            'Chassis request: '
            f"cmd={payload.get('approach_cmd')} "
            f"action={payload.get('chassis_action')} "
            f"object={payload.get('object_name') or self.object_name_from_plan_target(payload.get('current_target')) or ''} "
            f"base={self.format_payload_xyz(payload, ('target_x_m', 'target_y_m', 'target_z_m'))} "
            f"camera={self.format_payload_xyz(payload, ('camera_x_m', 'camera_y_m', 'camera_z_m'))} "
            f"reason={payload.get('reason')}"
        )

    @staticmethod
    def format_payload_xyz(payload: dict, keys) -> str:
        values = [payload.get(key) for key in keys]
        if any(value is None for value in values):
            return 'None'
        try:
            return '(' + ','.join(f'{float(value):.3f}' for value in values) + ')'
        except (TypeError, ValueError):
            return 'None'

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = ChassisApproachPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
