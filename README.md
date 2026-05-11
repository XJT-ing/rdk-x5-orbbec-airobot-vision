# airbot-vision-grasping

这是 AIRBOT Play 机械臂视觉抓取项目，包含两个主要工作区：

- `robot_ws/`：机械臂侧 ROS 2 包，包括执行器、任务状态机、消息和启动文件。
- `Orbbec_ws/`：Orbbec 相机和检测侧工作区。
- `hand_to_eye/`：手眼标定脚本和数据。

当前默认开环抓取流程要求视觉侧直接发布机器人 `base_link` 坐标系下的 `/visual_target_base`。旧的机器人侧 camera-to-base 桥接节点已经删除。

## 快速开始

## 推荐实机启动顺序

终端 0：启动 AIRBOT 服务。

```bash
sudo airbot_server -i can1 -p 50001
```

终端 1：启动 Orbbec 相机。

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
ros2 launch orbbec_camera gemini2.launch.py
```

终端 2：启动检测节点。

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
ros2 run detector apple_detector_node
ros2 run detector box_detector_node
ros2 run detector duck_detector_node
```

终端 3：启动机械臂执行器和开环抓取任务。

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/airbot-vision-grasping/robot_ws/install/setup.bash
ros2 launch robot_bringup open_loop_grasp.launch.py
```

终端 4：启动相机坐标到 base_link 的转换桥。

注意：这里必须 source `robot_ws/install/setup.bash`，否则 `robot_msgs/msg/VisualTarget` 不可见。

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
source /home/sunrise/robot/airbot-vision-grasping/robot_ws/install/setup.bash
python3 /home/sunrise/robot/airbot-vision-grasping/hand_to_eye/camera_to_base_transform.py
```

```text
/visual_target_base  robot_msgs/msg/VisualTarget  frame_id=base_link
```

更多 topic 契约和测试命令见 [robot_ws/README.md](robot_ws/README.md) 与 [Orbbec_ws/README.md](Orbbec_ws/README.md)。
