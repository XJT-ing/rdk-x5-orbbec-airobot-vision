#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from robot_msgs.msg import ArmCommand, ArmState
from robot_arm_interface.airbot_wrapper import AirbotWrapper


class ArmExecutorNode(Node):
    def __init__(self):
        super().__init__('arm_executor_node')

        self.robot = AirbotWrapper(url='localhost', port=50001)
        self.connected = False

        self.busy = False
        self.last_command = ''
        self.last_success = True
        self.last_error = ''

        self.home_joint = [0.0, -0.785398, 0.785398, 0.0, 0.0, 0.0]
        self.sleep_joint = [0.0, -0.7853981633974483, 1.8325957145940461, -1.3089969389957472, 1.3089969389957472, 1.5707963267948966]

        self.state_pub = self.create_publisher(
            ArmState,
            '/robot_arm/state',
            10
        )
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/robot_arm/end_pose',
            10
        )

        self.cmd_sub = self.create_subscription(
            ArmCommand,
            '/robot_arm/cmd',
            self.command_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.publish_feedback)

        try:
            self.robot.connect(speed_profile='default')
            self.connected = True
            self.get_logger().info('ArmExecutorNode connected to robot server.')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to robot server: {e}')

        self.get_logger().info('ArmExecutorNode started.')
        self.get_logger().info('Subscribed topic: /robot_arm/cmd')
        self.get_logger().info('Publishing topics: /robot_arm/state, /robot_arm/end_pose')

    def command_callback(self, msg: ArmCommand):
        if not self.connected:
            self.get_logger().error('Robot is not connected.')
            return

        if self.busy:
            self.get_logger().warning('Executor is busy, skip this command.')
            return

        threading.Thread(
            target=self.execute_command,
            args=(msg,),
            daemon=True
        ).start()

    def execute_command(self, msg: ArmCommand):
        self.busy = True
        self.last_command = msg.command_type
        self.last_success = False
        self.last_error = ''

        try:
            command = msg.command_type.strip().upper()

            if command == 'MOVE_JOINT':
                if len(msg.joint_target) < 6:
                    raise ValueError('MOVE_JOINT requires 6 joint values.')
                ok = self.robot.move_joints_and_wait(
                    joint_target=list(msg.joint_target[:6]),
                    timeout_sec=8.0,
                    tolerance_rad=0.03
                )

            elif command == 'MOVE_CART_KEEP_ORI':
                if len(msg.cartesian_position) < 3:
                    raise ValueError('MOVE_CART_KEEP_ORI requires 3 position values.')
                ok = self.robot.move_cart_and_wait(
                    target_xyz=list(msg.cartesian_position[:3]),
                    keep_current_orientation=True,
                    timeout_sec=8.0,
                    tolerance_m=0.01
                )

            elif command == 'MOVE_CART':
                if len(msg.cartesian_position) < 3:
                    raise ValueError('MOVE_CART requires 3 position values.')
                if len(msg.cartesian_orientation) < 4:
                    raise ValueError('MOVE_CART requires 4 orientation values.')
                ok = self.robot.move_cart_and_wait(
                    target_xyz=list(msg.cartesian_position[:3]),
                    keep_current_orientation=False,
                    orientation=list(msg.cartesian_orientation[:4]),
                    timeout_sec=8.0,
                    tolerance_m=0.01
                )

            elif command == 'OPEN_GRIPPER':
                self.robot.open_gripper()
                ok = True

            elif command == 'CLOSE_GRIPPER':
                self.robot.close_gripper()
                ok = True

            elif command == 'GO_HOME':
                ok = self.robot.move_joints_and_wait(
                    joint_target=self.home_joint,
                    timeout_sec=8.0,
                    tolerance_rad=0.03
                )

            elif command == 'GO_SLEEP':
                ok = self.robot.move_joints_and_wait(
                    joint_target=self.sleep_joint,
                    timeout_sec=8.0,
                    tolerance_rad=0.03
                )

            else:
                raise ValueError(f'Unsupported command_type: {msg.command_type}')

            self.last_success = bool(ok)
            if ok:
                self.get_logger().info(f'Command succeeded: {msg.command_type}')
            else:
                self.last_error = f'Command timeout or target not reached: {msg.command_type}'
                self.get_logger().error(self.last_error)

        except Exception as e:
            self.last_error = str(e)
            self.last_success = False
            self.get_logger().error(f'Command failed: {e}')
        finally:
            self.busy = False

    def publish_feedback(self):
        if not self.connected:
            return

        try:
            joint_pos = self.robot.get_joint_pos()
            pose = self.robot.get_end_pose()
            if joint_pos is None or pose is None or len(pose) < 2:
                return

            position = pose[0]
            orientation = pose[1]

            state_msg = ArmState()
            state_msg.header.stamp = self.get_clock().now().to_msg()
            state_msg.header.frame_id = 'base_link'
            state_msg.arm_state = 'BUSY' if self.busy else 'IDLE'
            state_msg.busy = self.busy
            state_msg.success = self.last_success
            state_msg.last_command = self.last_command
            state_msg.error_message = self.last_error
            state_msg.joint_pos = list(joint_pos)
            state_msg.end_position = list(position)
            state_msg.end_orientation = list(orientation)
            self.state_pub.publish(state_msg)

            pose_msg = PoseStamped()
            pose_msg.header.stamp = state_msg.header.stamp
            pose_msg.header.frame_id = 'base_link'
            pose_msg.pose.position.x = float(position[0])
            pose_msg.pose.position.y = float(position[1])
            pose_msg.pose.position.z = float(position[2])
            pose_msg.pose.orientation.x = float(orientation[0])
            pose_msg.pose.orientation.y = float(orientation[1])
            pose_msg.pose.orientation.z = float(orientation[2])
            pose_msg.pose.orientation.w = float(orientation[3])
            self.pose_pub.publish(pose_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to publish feedback: {e}')

    def destroy_node(self):
        if self.connected:
            try:
                self.robot.disconnect()
            except Exception as e:
                self.get_logger().warning(f'Failed to disconnect cleanly: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmExecutorNode()
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
