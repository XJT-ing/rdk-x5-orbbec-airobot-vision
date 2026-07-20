# 底盘联调测试说明

本文档用于测试底盘端对 `/chassis/approach_request` 的订阅、运动控制、停止、返回和任务结束逻辑。

## 1. 底盘端功能摘要

| 模块 | 实现 |
| --- | --- |
| 订阅 | `/chassis/approach_request`，类型 `std_msgs/msg/String`，内容为 JSON |
| 发布 | `/cmd_vel`，类型 `geometry_msgs/msg/Twist` |
| 前进控制 | `camera_x_m` -> PID 算 `angular.z`；`camera_z_m` -> 深度-速度映射 |
| 减速策略 | `camera_z_m < 0.5m` 时二次减速，最小速度 `0.1` |
| 路径记录 | 每 `0.5s` 保存 `(v, w, dt)` |
| 返回逻辑 | 抓取后逆序回放路径返回 |
| 状态机 | `IDLE -> MOVING -> STOPPED -> 转180度 -> 回放 -> 根据是否还有任务继续/退出` |
| 语音 | `edge-tts` 晓晓音色播放“抓完了” |

## 2. 底盘端启动

```bash
source /opt/ros/humble/setup.bash
source ~/wheeltec_ros2/install/setup.bash
ros2 launch wheeltec_grap wheeltec_grap.launch.py
```

## 3. 上主控话题

底盘端订阅：

```text
/chassis/approach_request
std_msgs/msg/String
```

`data` 字段是 JSON 字符串。

底盘端重点读取：

| 字段 | 含义 |
| --- | --- |
| `approach_cmd` | `1=继续靠近/前进`，`2=停止` |
| `camera_x_m` | 目标在相机坐标系下的水平偏移，用于角速度 PID |
| `camera_z_m` | 目标深度/距离，用于线速度映射 |
| `all_grasp_done` | 是否全部物品抓取完成 |
| `need_return_for_next_grasp` | 本次抓完但还有下一个目标，是否需要返回继续抓 |
| `next_target` | 下一个要抓取的目标信息，可选 |

## 4. 测试项

### 4.1 测试开始靠近

上主控或任意 ROS2 终端发布：

```bash
ros2 topic pub --once /chassis/approach_request std_msgs/msg/String \
"{data: '{\"approach_cmd\":1,\"camera_x_m\":0.10,\"camera_y_m\":0.0,\"camera_z_m\":1.20,\"all_grasp_done\":false,\"need_return_for_next_grasp\":false}'}"
```

预期：

```text
底盘开始移动。
camera_x_m=0.10 会产生角速度修正。
camera_z_m=1.20 会映射前进速度。
```

### 4.2 测试停止

```bash
ros2 topic pub --once /chassis/approach_request std_msgs/msg/String \
"{data: '{\"approach_cmd\":2,\"camera_x_m\":0.02,\"camera_y_m\":0.0,\"camera_z_m\":0.45,\"all_grasp_done\":false,\"need_return_for_next_grasp\":false}'}"
```

预期：

```text
底盘停止。
进入 STOPPED 或等待机械臂抓取状态。
```

### 4.3 测试本次抓完但还有下一个目标

```bash
ros2 topic pub --once /chassis/approach_request std_msgs/msg/String \
"{data: '{\"event\":\"grasp_cycle_done\",\"approach_cmd\":2,\"all_grasp_done\":false,\"need_return_for_next_grasp\":true,\"next_target\":{\"object_name\":\"bottle\",\"topic\":\"/detect_yolo/bottle_position\"}}'}"
```

预期：

```text
底盘认为当前物品抓取完成。
底盘执行返回逻辑。
返回后再转 180 度。
准备继续下一轮靠近/抓取。
```

### 4.4 测试全部抓完

```bash
ros2 topic pub --once /chassis/approach_request std_msgs/msg/String \
"{data: '{\"event\":\"grasp_cycle_done\",\"approach_cmd\":2,\"all_grasp_done\":true,\"need_return_for_next_grasp\":false}'}"
```

预期：

```text
底盘执行最终返回/退出流程。
播放“抓完了”。
不再回到抓取点。
```

## 5. 真实联调时上主控会自动发布的字段

真实运行时，上主控 `chassis_approach_publisher.py` 会持续发布：

```text
approach_cmd
camera_x_m
camera_y_m
camera_z_m
all_grasp_done
need_return_for_next_grasp
point_stamped
object_name
camera_topic
```

底盘端不需要再单独订阅 `/detect_yolo/xxx_position`，除非需要额外调试。

## 6. 上主控相关启动

上主控启动底盘协调发布节点：

```bash
cd ~

source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash

python3 /home/sunrise/robot/hand_to_eye/chassis_approach_publisher.py
```

## 7. 调试命令

查看底盘端是否收到指令：

```bash
ros2 topic echo /chassis/approach_request
```

查看底盘是否发布速度：

```bash
ros2 topic echo /cmd_vel
```

查看 topic 是否存在：

```bash
ros2 topic list | grep chassis
ros2 topic list | grep cmd_vel
```

## 8. 通过标准

| 测试 | 通过标准 |
| --- | --- |
| `approach_cmd=1` | 底盘开始靠近，`/cmd_vel.linear.x` 有正速度 |
| `approach_cmd=2` | 底盘停止，`/cmd_vel` 归零 |
| `need_return_for_next_grasp=true` | 底盘执行返回，并准备下一轮抓取 |
| `all_grasp_done=true` | 底盘完成最终退出流程并播放“抓完了” |

