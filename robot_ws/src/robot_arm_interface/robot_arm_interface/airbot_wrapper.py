# -*- coding: utf-8 -*-
import math
import time
from airbot_py.arm import AIRBOTPlay, RobotMode, SpeedProfile


class AirbotWrapper:
    def __init__(self, url="localhost", port=50001):
        self.url = url
        self.port = port
        self.robot = None

    def connect(self, speed_profile="default"):
        if self.robot is None:
            self.robot = AIRBOTPlay(url=self.url, port=self.port)
            self.robot.connect()
        self.set_speed_profile(speed_profile)

    def disconnect(self):
        if self.robot is not None:
            self.robot.disconnect()
            self.robot = None

    def set_speed_profile(self, speed_profile="default"):
        if self.robot is None:
            raise RuntimeError("Robot is not connected.")

        if speed_profile == "slow":
            self.robot.set_speed_profile(SpeedProfile.SLOW)
        elif speed_profile == "fast":
            self.robot.set_speed_profile(SpeedProfile.FAST)
        else:
            self.robot.set_speed_profile(SpeedProfile.DEFAULT)

    def get_state(self):
        return self.robot.get_state()

    def get_joint_pos(self):
        return self.robot.get_joint_pos()

    def get_joint_vel(self):
        return self.robot.get_joint_vel()

    def get_end_pose(self):
        return self.robot.get_end_pose()

    def get_end_position(self):
        pose = self.get_end_pose()
        if pose is None or len(pose) < 2:
            raise RuntimeError("Failed to read current end pose.")
        return list(pose[0])

    def get_end_orientation(self):
        pose = self.get_end_pose()
        if pose is None or len(pose) < 2:
            raise RuntimeError("Failed to read current end pose.")
        return list(pose[1])

    def move_joints(self, joint_target):
        self.robot.switch_mode(RobotMode.PLANNING_POS)
        self.robot.move_to_joint_pos(joint_target)

    def move_cart_waypoints(self, waypoints):
        self.robot.switch_mode(RobotMode.PLANNING_WAYPOINTS)
        self.robot.move_with_cart_waypoints(waypoints)

    def move_to_cart_target_with_current_orientation(self, target_xyz):
        current_quat = self.get_end_orientation()
        self.move_cart_waypoints([
            [list(target_xyz), current_quat],
        ])

    def servo_joints(self, joint_target):
        self.robot.switch_mode(RobotMode.SERVO_JOINT_POS)
        self.robot.servo_joint_pos(joint_target)

    def go_home(self):
        home_joint = [0.0, -0.785398, 0.785398, 0.0, 0.0, 0.0]
        self.robot.switch_mode(RobotMode.PLANNING_POS)
        self.robot.move_to_joint_pos(home_joint)

    def open_gripper(self):
        self.robot.switch_mode(RobotMode.SERVO_JOINT_POS)
        for _ in range(50):
            self.robot.servo_eef_pos([0.07])
            time.sleep(0.02)

    def close_gripper(self):
        self.robot.switch_mode(RobotMode.SERVO_JOINT_POS)
        for _ in range(50):
            self.robot.servo_eef_pos([0.0])
            time.sleep(0.02)

    def _joint_max_abs_error(self, target_joint, current_joint):
        return max(abs(t - c) for t, c in zip(target_joint, current_joint))

    def _position_distance(self, target_pos, current_pos):
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        dz = target_pos[2] - current_pos[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def wait_until_joint_reached(self, target_joint, timeout_sec=5.0, tolerance_rad=0.03, poll_interval=0.05):
        start_time = time.time()

        while time.time() - start_time < timeout_sec:
            current_joint = self.get_joint_pos()
            if current_joint is None or len(current_joint) < len(target_joint):
                time.sleep(poll_interval)
                continue

            err = self._joint_max_abs_error(target_joint, current_joint)
            if err <= tolerance_rad:
                return True

            time.sleep(poll_interval)

        return False

    def wait_until_pose_reached(self, target_pos, timeout_sec=5.0, tolerance_m=0.01, poll_interval=0.05):
        start_time = time.time()

        while time.time() - start_time < timeout_sec:
            current_pos = self.get_end_position()
            err = self._position_distance(target_pos, current_pos)
            if err <= tolerance_m:
                return True

            time.sleep(poll_interval)

        return False

    def move_joints_and_wait(self, joint_target, timeout_sec=8.0, tolerance_rad=0.03):
        self.move_joints(joint_target)
        return self.wait_until_joint_reached(
            target_joint=joint_target,
            timeout_sec=timeout_sec,
            tolerance_rad=tolerance_rad,
        )

    def move_cart_and_wait(self, target_xyz, keep_current_orientation=True, orientation=None, timeout_sec=8.0, tolerance_m=0.01):
        if keep_current_orientation:
            quat = self.get_end_orientation()
        else:
            if orientation is None:
                raise ValueError("orientation must be provided when keep_current_orientation is False")
            quat = list(orientation)

        self.move_cart_waypoints([
            [list(target_xyz), quat],
        ])

        return self.wait_until_pose_reached(
            target_pos=target_xyz,
            timeout_sec=timeout_sec,
            tolerance_m=tolerance_m,
        )
