# RDK X5 × AIRBOT 视觉抓取与情绪识别系统

> 基于 RDK X5、Orbbec Gemini2 深度相机和 AIRBOT Play 机械臂的物体识别抓取系统，支持 YOLO 通用检测、颜色专用检测器以及面部情绪识别。

## 项目简介

本项目是 RDK X5 机器人平台上的视觉抓取子模块，实现以下核心功能：

- **YOLOv8/v11 通用目标检测** — 支持 COCO 80 类物体识别并输出三维坐标
- **颜色专用检测器** — 针对苹果（红）、小黄鸭（黄）、绿色药盒的精准检测与定位
- **手眼协调抓取** — 相机坐标系 → 机械臂基座坐标系转换，open-loop 状态机抓取
- **面部情绪识别** — 5 种情绪分类（happy / neutral / surprise / low_mood / negative_distress），支持主动交互

## 硬件依赖

| 硬件 | 型号 | 用途 |
|------|------|------|
| 主控板 | RDK X5 | 运行 ROS2 Humble |
| 深度相机 | Orbbec Gemini2 | RGB-D 图像采集 |
| 机械臂 | AIRBOT Play | 物体抓取执行 |
| 通信接口 | CAN (can1) | 主控与机械臂通信 |

## 目录结构

```
.
├── Orbbec_ws/                       # Orbbec 相机 ROS2 工作空间
│   ├── src/
│   │   ├── detect_yolo/             # YOLO 通用检测（C++），发布 3D 坐标
│   │   ├── detector/                # 专用颜色检测（apple / duck / box）
│   │   ├── emotion/                 # 情绪识别原型（Python + C++）
│   │   ├── emotion_landmark_cpp/    # 人脸关键点 + 情绪分类（C++ 版）
│   │   └── emotion_local/           # 情绪融合节点（Python，主用版本）
│   └── Log/                         # 相机运行日志
│
├── robot_ws/                        # 机械臂 ROS2 工作空间
│   └── src/
│       ├── robot_arm_driver/        # 机械臂驱动（唯一 AIRBOT SDK 调用者）
│       ├── robot_arm_interface/     # AIRBOT SDK Python 封装层
│       ├── robot_bringup/           # Launch 文件 + 抓取参数配置
│       ├── robot_msgs/              # 自定义 ROS2 消息定义
│       └── robot_tasks/             # 抓取任务状态机（主入口）
│
├── hand_to_eye/                     # 手眼标定 + 坐标转换
│   ├── camera_to_base_transform.py  # 相机 → base_link 转换（主链路）
│   ├── solve_handeye.py             # 手眼标定求解
│   ├── visual_target_bridge.py      # 视觉目标桥接
│   └── auto_pick_from_base.py       # 旧版自动抓取（已废弃，仅供参考）
│
├── docs/
│   ├── grasp_startup_commands.md    # 话题/节点/数据流参考
│   └── service_robot_interface.md   # 外部模块调用说明
│
├── start_auto_grasp.sh              # 全链路一键启动
└── start_airbot_can0.sh             # AIRBOT 服务启动
```

## 软件依赖

- **OS**: Ubuntu 22.04 (RDK X5 默认)
- **ROS2**: Humble Hawksbill
- **Python**: 3.10+
- **Orbbec SDK**: Gemini2 相机驱动
- **AIRBOT SDK**: 机械臂 Python SDK
- **OpenCV**: 图像处理
- **YOLOv8/v11**: 通过 ONNX / OpenCV DNN 推理

## 环境配置

### 1. 编译 Orbbec 工作空间

```bash
cd /home/sunrise/robot/Orbbec_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. 编译机械臂工作空间

```bash
cd /home/sunrise/robot/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. 确认 TROS 环境

YOLO 检测节点依赖地瓜机器人 TROS 环境：

```bash
source /opt/tros/humble/setup.bash
```

## 启动步骤

**终端 0 — AIRBOT 服务**

```bash
sudo airbot_server -i can1 -p 50001
# 验证服务已监听
sudo ss -lntp | grep 50001
```

**终端 1 — Orbbec 相机**

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
ros2 launch orbbec_camera gemini2.launch.py
```

**终端 2 — 目标检测**

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
# 选择以下之一：
ros2 run detector duck_detector_node          # 小黄鸭检测
ros2 run detector apple_detector_node         # 苹果检测
ros2 run detector box_detector_node           # 绿色药盒检测
# 或使用 YOLO 通用检测：
ros2 run detect_yolo detect_yolo_node
```

**终端 3 — 主抓取链路**

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash
ros2 launch robot_bringup open_loop_grasp.launch.py
```

**终端 4 — 坐标转换桥**

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
source /home/sunrise/robot/robot_ws/install/setup.bash
python3 /home/sunrise/robot/hand_to_eye/camera_to_base_transform.py
```

**终端 5 — 情绪识别（可选）**

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source /home/sunrise/robot/Orbbec_ws/install/setup.bash
python3 /home/sunrise/robot/Orbbec_ws/src/emotion_local/emotion_local/emotion_fusion_node.py
```

## 抓取数据流

```
┌──────────────┐     ┌──────────────────┐     ┌──────────────────────┐
│ Orbbec 相机   │────>│ 检测节点          │────>│ /duck_position 等     │
│ RGB-D 图像    │     │ detector / YOLO   │     │ (相机坐标系 Point)    │
└──────────────┘     └──────────────────┘     └──────────┬───────────┘
                                                         │
                                              ┌──────────▼───────────┐
                                              │ camera_to_base_       │
                                              │ transform.py          │
                                              │ (相机坐标 → base_link) │
                                              └──────────┬───────────┘
                                                         │
                                              ┌──────────▼───────────┐
                                              │ /visual_target_base   │
                                              │ (RobotMsgs/VisualTarget)│
                                              └──────────┬───────────┘
                                                         │
                                              ┌──────────▼───────────┐
                                              │ grasp_task_open_loop  │
                                              │ (抓取状态机)           │
                                              └──────────┬───────────┘
                                                         │
                                              ┌──────────▼───────────┐
                                              │ arm_executor_node     │
                                              │ (AIRBOT SDK 唯一入口) │
                                              └──────────┬───────────┘
                                                         │
                                              ┌──────────▼───────────┐
                                              │ AIRBOT Play 机械臂    │
                                              └──────────────────────┘
```

## 核心话题速查

| 话题 | 消息类型 | 方向 | 说明 |
|------|----------|------|------|
| `/camera/color/image_raw` | `Image` | 相机发布 | 彩色图像 |
| `/camera/depth/image_raw` | `Image` | 相机发布 | 深度图像 |
| `/camera/color/camera_info` | `CameraInfo` | 相机发布 | 相机内参 |
| `/duck_position` | `PointStamped` | 检测器发布 | 小黄鸭 3D 位置 |
| `/apple_position` | `PointStamped` | 检测器发布 | 苹果 3D 位置 |
| `/box_position` | `PointStamped` | 检测器发布 | 药盒 3D 位置 |
| `/yolo_detections` | 自定义 | YOLO 发布 | YOLO 检测结果 |
| `/visual_target_base` | `VisualTarget` | 转换桥发布 | base_link 系目标坐标 |
| `/robot_arm/end_pose` | `PoseStamped` | 执行器发布 | 末端位姿 |
| `/robot_arm/joint_state` | `ArmJointState` | 执行器发布 | 关节状态 |
| `/robot_arm/executor_status` | `String` | 执行器发布 | 状态 (IDLE/BUSY/DONE/ERROR) |
| `/robot_arm/gripper_cmd` | `String` | 状态机发布 | 夹爪控制 (`open` / `close`) |
| `/emotion/result` | 自定义 | 情绪节点发布 | 情绪识别结果 |

## 抓取状态机

```text
IDLE → WAIT_PRE_TARGET → SET_GRIPPER_ORIENTATION
  → MOVE_PRE_GRASP → MOVE_GRASP → CLOSE_GRIPPER
  → MOVE_LIFT → MOVE_RETREAT → IDLE

异常 → RECOVER → clear_error → 抬升 → safe_pose → IDLE
```

## 关键参数配置

机械臂抓取参数配置文件位于 `robot_ws/src/robot_bringup/config/open_loop_grasp.yaml`，主要包括：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `approach_mode` | `top_down` | 抓取方向（上方抓取） |
| `max_cartesian_step` | `0.08` m | 笛卡尔运动单步最大值 |
| `last_seen_target_max_age_sec` | `5.0` s | 目标丢失后继续执行的最长时间 |
| `table_z` | 桌面高度 | 防止碰撞桌面 |
| `use_last_seen_target_on_loss` | `true` | 目标短暂丢失时使用历史位置 |

## 调试技巧

```bash
# 手动发送假目标测试抓取
ros2 topic pub -r 10 /visual_target_base robot_msgs/msg/VisualTarget \
  "{header: {frame_id: 'base_link'}, x: 0.35, y: 0.0, z: 0.12, confidence: 0.90, depth: 0.12, image_width: 640, image_height: 480}"

# 控制夹爪
ros2 topic pub --once /robot_arm/gripper_cmd std_msgs/msg/String "{data: 'open'}"
ros2 topic pub --once /robot_arm/gripper_cmd std_msgs/msg/String "{data: 'close'}"

# 清除错误
ros2 topic pub --once /robot_arm/reset_executor std_msgs/msg/String "{data: 'clear_error'}"

# 查看执行器状态
ros2 topic echo /robot_arm/executor_status

# 检查话题是否正常
ros2 topic list | grep -E "(duck|visual|robot_arm|emotion)"
```

## 常见问题

| 问题 | 排查步骤 |
|------|----------|
| 机械臂不动 | 1. 检查 `airbot_server` 是否监听 50001 2. 查看 `/robot_arm/executor_status` 是否为 ERROR |
| 坐标明显偏移 | 确认深度图与彩色图已对齐，检查 camera_info 是否正确 |
| 夹爪不动作 | 确认 `/robot_arm/executor_status` 为 BUSY/IDLE，不在 ERROR 状态 |
| 抓取到空中 | 检查手眼标定参数（`hand_to_eye/camera_to_base_transform.py` 中的变换矩阵） |

## 维护说明

- **`arm_executor_node.py`** 是唯一允许直接调用 AIRBOT SDK 的节点，不要在其他地方直接调用 SDK
- **`grasp_task_open_loop.py`** 是当前主抓取入口，旧的 `auto_pick_from_base.py` 仅供调试参考
- **`end_position_publisher.py`** 直接连接 SDK，不要和主链路同时运行
- 不在主链路中使用 ROS2 Action 或 MoveIt

## 仓库信息

- 主控板：RDK X5
- 实机路径：`/home/sunrise/robot/`
- ROS2 版本：Humble
- 主语言：Python / C++
