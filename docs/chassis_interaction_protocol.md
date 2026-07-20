# 底盘与上主控交互协议

本文档说明底盘端如何订阅上主控 X5 发布的机械臂/相机协作话题。

## 1. 总体数据流

```text
上主控 X5
  Orbbec 相机
    -> YOLO / detector
    -> /detect_yolo/{object}_position 或 /duck_position 等相机原始坐标
    -> camera_to_base_transform.py
    -> /visual_target_base_raw，机械臂 base_link 坐标，供底盘靠近/停止判断
    -> /visual_target_base，只有进入可抓范围后才打开，供 open_loop 抓取触发
    -> chassis_approach_publisher.py
    -> /chassis/approach_request

底盘端
  订阅 /chassis/approach_request
  解析 JSON
  根据 approach_cmd、camera_x_m、camera_z_m、all_grasp_done 等字段控制底盘
```

底盘端只需要订阅一个核心话题：

```text
/chassis/approach_request
```

## 2. 话题格式

```text
Topic: /chassis/approach_request
Type:  std_msgs/msg/String
Data:  JSON 字符串
```

也就是 ROS2 消息本身是：

```text
std_msgs/msg/String
string data
```

`data` 字段里是 JSON。

## 3. 底盘端最需要关注的字段

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `point_stamped` | object/null | 当前阶段要抓取物品的相机原始坐标，等价于 `geometry_msgs/msg/PointStamped` 的 JSON 表达 |
| `camera_x_m` | number/null | 物品在相机坐标系下的 x，单位 m |
| `camera_y_m` | number/null | 物品在相机坐标系下的 y，单位 m |
| `camera_z_m` | number/null | 物品在相机坐标系下的 z/depth，单位 m |
| `camera_frame_id` | string | 相机坐标系，一般是 `camera_color_optical_frame` |
| `camera_topic` | string | 当前目标原始坐标来源 topic，例如 `/detect_yolo/apple_position` |
| `object_name` | string | 当前目标英文名，例如 `apple`、`bottle` |
| `target_x_m` | number/null | 当前目标在机械臂 `base_link` 坐标系下的 x，机械臂/相机端用它判断是否需要底盘靠近 |
| `target_y_m` | number/null | 当前目标在机械臂 `base_link` 坐标系下的 y |
| `target_z_m` | number/null | 当前目标在机械臂 `base_link` 坐标系下的 z |
| `chassis_action` | string | 上主控给底盘的动作语义 |
| `approach_cmd` | int | 简化控制字段：`1=继续靠近/前进`，`2=停止` |
| `need_approach` | bool | 是否需要继续靠近当前物品 |
| `all_grasp_done` | bool | 是否全部物品都抓取完成 |
| `need_return_for_next_grasp` | bool | 本次抓完但还有下一个物品时为 true，底盘需要返回继续配合抓取 |
| `ready_for_arm_grasp` | bool | 机械臂/相机端判断目标已经进入可抓范围 |
| `grasp_trigger_enabled` | bool | 机械臂/相机端是否已经打开抓取触发门，true 后 open_loop 才会收到 `/visual_target_base` 并执行抓取 |

## 4. `approach_cmd` 定义

```text
approach_cmd = 1
  底盘继续靠近当前目标。
  可使用 camera_x_m 做角速度 PID，使用 camera_z_m 做线速度控制。

approach_cmd = 2
  底盘停止。
  可能是机械臂/相机端提前停止等待进入抓取范围，也可能是一次抓取动作完成。
```

建议底盘端优先使用：

```text
approach_cmd
all_grasp_done
need_return_for_next_grasp
camera_x_m
camera_z_m
```

## 5. 相机坐标字段

`point_stamped` 示例：

```json
{
  "header": {
    "stamp": {
      "sec": 123,
      "nanosec": 456000000
    },
    "frame_id": "camera_color_optical_frame"
  },
  "point": {
    "x": 0.05,
    "y": -0.02,
    "z": 0.85
  }
}
```

同一份坐标也会以扁平字段重复发布，方便底盘端解析：

```json
{
  "camera_x_m": 0.05,
  "camera_y_m": -0.02,
  "camera_z_m": 0.85
}
```

控制建议：

```text
camera_x_m
  物体在相机坐标系水平偏移。
  可用于 PID 控制 angular.z，让物体保持居中。

camera_y_m
  物体在相机坐标系垂直方向位置。
  当前可暂不用于底盘控制。

camera_z_m
  物体相对相机的深度/距离。
  可用于映射 linear.x，距离较近时减速。
```

## 6. 机械臂/相机端的距离判断规则

底盘端不要自己判断机械臂能不能抓，由上主控判断。上主控不会使用半径/radius 作为底盘停止依据，避免不同物品尺寸导致判断不稳定。

```text
target_x_m > 0.80
  /chassis/approach_request 发布 approach_cmd=1，底盘继续靠近。

target_x_m <= 0.80
  /chassis/approach_request 发布 approach_cmd=2，底盘提前停止，防止底盘响应延迟造成过冲。

target_x_m <= 0.68 且 target_y_m / target_z_m 在机械臂工作空间内
  上主控自动发布 near_grasp 姿态切换。
  等待 2.5 秒后发布 /arm_task/grasp_enable=true。
  camera_to_base_transform.py 才会把目标发布到 /visual_target_base，触发 open_loop 真正执行抓取动作。

抓取动作真正完成后
  open_loop 发布 /robot_arm/grasp_task_status=DONE。
  arm_task_manager.py 发布 /arm_task/status: DONE target=...
  chassis_approach_publisher.py 再向 /chassis/approach_request 发布 grasp_cycle_done。
  同时机械臂自动切回 pre_grasp，准备下一次抓取。
```

## 7. 典型 JSON 示例

### 7.1 需要继续靠近当前物品

```json
{
  "version": 1,
  "event": "approach_state",
  "chassis_action": "approach_target",
  "object_name": "apple",
  "camera_topic": "/detect_yolo/apple_position",
  "camera_frame_id": "camera_color_optical_frame",
  "camera_x_m": 0.05,
  "camera_y_m": -0.02,
  "camera_z_m": 0.85,
  "point_stamped": {
    "header": {
      "stamp": {
        "sec": 123,
        "nanosec": 456000000
      },
      "frame_id": "camera_color_optical_frame"
    },
    "point": {
      "x": 0.05,
      "y": -0.02,
      "z": 0.85
    }
  },
  "approach_cmd": 1,
  "need_approach": true,
  "stop_chassis": false,
  "ready_for_arm_grasp": false,
  "grasp_trigger_enabled": false,
  "target_x_m": 1.10,
  "target_y_m": 0.08,
  "target_z_m": 0.45,
  "chassis_stop_x_m": 0.80,
  "grasp_ready_x_m": 0.68,
  "all_grasp_done": false,
  "need_return_for_next_grasp": false,
  "reason": "target_before_chassis_stop_x"
}
```

底盘动作：

```text
继续靠近 apple。
使用 camera_x_m 控制角速度，camera_z_m 控制线速度。
```

### 7.2 已提前停止，等待目标进入机械臂可抓范围

```json
{
  "version": 1,
  "event": "approach_state",
  "chassis_action": "stop_approach",
  "object_name": "apple",
  "camera_topic": "/detect_yolo/apple_position",
  "camera_frame_id": "camera_color_optical_frame",
  "camera_x_m": 0.01,
  "camera_y_m": 0.0,
  "camera_z_m": 0.45,
  "approach_cmd": 2,
  "need_approach": false,
  "stop_chassis": true,
  "ready_for_arm_grasp": false,
  "grasp_trigger_enabled": false,
  "target_x_m": 0.76,
  "target_y_m": 0.03,
  "target_z_m": 0.48,
  "chassis_stop_x_m": 0.80,
  "grasp_ready_x_m": 0.68,
  "all_grasp_done": false,
  "need_return_for_next_grasp": false,
  "reason": "target_stopped_wait_grasp_range"
}
```

底盘动作：

```text
立即停止运动，等待机械臂/相机端继续判断是否进入可抓范围。
```

### 7.3 进入可抓范围，机械臂准备抓取

```json
{
  "version": 1,
  "event": "approach_state",
  "chassis_action": "stop_approach",
  "object_name": "apple",
  "camera_topic": "/detect_yolo/apple_position",
  "camera_frame_id": "camera_color_optical_frame",
  "camera_x_m": 0.01,
  "camera_y_m": 0.0,
  "camera_z_m": 0.45,
  "approach_cmd": 2,
  "need_approach": false,
  "stop_chassis": true,
  "ready_for_arm_grasp": true,
  "grasp_trigger_enabled": true,
  "target_x_m": 0.66,
  "target_y_m": 0.03,
  "target_z_m": 0.48,
  "chassis_stop_x_m": 0.80,
  "grasp_ready_x_m": 0.68,
  "all_grasp_done": false,
  "need_return_for_next_grasp": false,
  "reason": "target_ready_for_grasp"
}
```

底盘动作：

```text
保持停止。此时机械臂端已经切到 near_grasp，并打开抓取触发门，open_loop 会执行抓取动作。
```

### 7.4 本次抓完，但还有下一个物品

```json
{
  "version": 1,
  "event": "grasp_cycle_done",
  "chassis_action": "wait_next_grasp",
  "approach_cmd": 2,
  "need_approach": false,
  "stop_chassis": true,
  "grasp_cycle_done": true,
  "continue_grasp": true,
  "task_finished": false,
  "all_grasp_done": false,
  "need_return_for_next_grasp": true,
  "next_target": {
    "label": "水杯",
    "object_name": "bottle",
    "topic": "/detect_yolo/bottle_position",
    "mode": "yolo",
    "yolo_class": "bottle"
  },
  "remaining_requested_targets": [
    {
      "label": "水杯",
      "object_name": "bottle",
      "topic": "/detect_yolo/bottle_position",
      "mode": "yolo",
      "yolo_class": "bottle"
    }
  ],
  "reason": "more_graspable_objects_after_grasp"
}
```

底盘动作：

```text
当前物品已抓取完成。
还有下一个物品需要抓取，底盘完成搬运后需要返回继续配合抓取。
```

### 7.5 全部物品都抓完

```json
{
  "version": 1,
  "event": "grasp_cycle_done",
  "chassis_action": "grasp_task_finished",
  "approach_cmd": 2,
  "need_approach": false,
  "stop_chassis": true,
  "grasp_cycle_done": true,
  "continue_grasp": false,
  "task_finished": true,
  "all_grasp_done": true,
  "need_return_for_next_grasp": false,
  "remaining_requested_targets": [],
  "reason": "no_graspable_objects_after_grasp"
}
```

底盘动作：

```text
全部抓取任务结束。
底盘不需要再返回抓取点。
```

## 8. 底盘端订阅示例

```python
#!/usr/bin/env python3
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ChassisCommandSubscriber(Node):
    def __init__(self):
        super().__init__('chassis_command_subscriber')
        self.create_subscription(
            String,
            '/chassis/approach_request',
            self.callback,
            10,
        )

    def callback(self, msg: String):
        data = json.loads(msg.data)

        approach_cmd = int(data.get('approach_cmd', 2))
        all_grasp_done = bool(data.get('all_grasp_done', False))
        need_return = bool(data.get('need_return_for_next_grasp', False))

        if approach_cmd == 1:
            camera_x = data.get('camera_x_m')
            camera_z = data.get('camera_z_m')
            if camera_x is None or camera_z is None:
                self.get_logger().warning('No camera point in approach command.')
                return

            camera_x = float(camera_x)
            camera_z = float(camera_z)

            # TODO:
            # angular.z = PID(camera_x)
            # linear.x = speed_from_depth(camera_z)
            self.get_logger().info(
                f'approach target: camera_x={camera_x:.3f}, camera_z={camera_z:.3f}'
            )
            return

        if approach_cmd == 2:
            # TODO: publish zero /cmd_vel
            self.get_logger().info('stop chassis')

        if all_grasp_done:
            self.get_logger().info('all grasp tasks done; no need to return.')
            return

        if need_return:
            self.get_logger().info('current grasp done; return for next target.')
            return


def main(args=None):
    rclpy.init(args=args)
    node = ChassisCommandSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 9. 调试命令

底盘端查看上主控发布内容：

```bash
ros2 topic echo /chassis/approach_request
```

上主控手动测试发布：

```bash
ros2 topic pub --once /chassis/approach_request std_msgs/msg/String \
"{data: '{\"approach_cmd\":1,\"camera_x_m\":0.05,\"camera_z_m\":0.85,\"all_grasp_done\":false,\"need_return_for_next_grasp\":false}'}"
```

## 10. 物品名称和来源 topic

| 物品 | `object_name` | 相机原始坐标来源 |
| --- | --- | --- |
| 苹果 | `apple` | `/detect_yolo/apple_position` |
| 香蕉 | `banana` | `/detect_yolo/banana_position` |
| 瓶子 | `bottle` | `/detect_yolo/bottle_position` |
| 水杯/杯子/瓶子 | `bottle` | `/detect_yolo/bottle_position` |
| 蛋糕 | `cake` | `/detect_yolo/cake_position` |
| 小黄鸭 | `duck` | `/duck_position` |
| 绿色药盒 | `box` | `/box_position` |
| 大樱桃/红色圆 | `red_circle` | `/red_circle_position` |

## 11. 通信环境要求

两块 X5 需要在同一个 ROS2 Domain：

```bash
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
```

如果底盘端收不到话题，先检查：

```bash
ros2 topic list | grep chassis
ros2 topic echo /chassis/approach_request
```

