# airbot-vision-grasping

AIRBOT Play + Orbbec 视觉抓取项目。

  完整抓取指令流程                                                                                                      
  终端 0（前置）— 启动 CAN 总线连接                                                                                     
  sudo airbot_server -i can1 -p 50001

  保持运行，不要关闭。

  ---
  终端 1 — 机械臂归位

  python3 /home/sunrise/robot/hand_to_eye/move_to_lower_home.py

  等待执行完成（打印 "Moved to lower, more tolerant home pose." 后关闭即可）。

  ---
  终端 2 — 启动相机

  source /opt/ros/humble/setup.bash
  source /home/sunrise/robot/Orbbec_ws/install/setup.bash
  ros2 launch orbbec_camera gemini2.launch.py \
      enable_depth:=true \
      enable_ir:=false \
      enable_accel:=false \
      enable_gyro:=false \
      enable_point_cloud:=false \
      enable_colored_point_cloud:=false \
      enable_d2c_viewer:=false \
      color_width:=640 \
      color_height:=480 \
      color_fps:=30

  ---
  终端 3 — 检测节点

  source /opt/ros/humble/setup.bash
  source /home/sunrise/robot/Orbbec_ws/install/setup.bash
  ros2 run detector duck_detector_node

  ▎ 其他物体：把 duck 换成 apple 或 box

  查看 YOLO 检测结果（如果用 YOLO 替代独立检测器）：

  source /opt/ros/humble/setup.bash
  source /home/sunrise/robot/Orbbec_ws/install/setup.bash
  ros2 topic echo /detect_yolo/bottle_position   # 瓶子

  ---
  终端 4 — 自动抓取

  source /opt/ros/humble/setup.bash
  source /home/sunrise/robot/robot_ws/install/setup.bash
  ros2 launch robot_bringup open_loop_grasp.launch.py

  ---
  终端 5 — 相机坐标到基座变换

  source /opt/ros/humble/setup.bash
  source /home/sunrise/robot/Orbbec_ws/install/setup.bash
  source /home/sunrise/robot/robot_ws/install/setup.bash
  python3 /home/sunrise/robot/hand_to_eye/camera_to_base_transform.py


  ---
  验证用的辅助命令

  查看相机检测到的目标在 base_link 下的坐标：

  source /opt/ros/humble/setup.bash
  ros2 topic echo /visual_target_base

=================================================
五类情绪识别（happy、neutral、surprise、low_mood、negative_distress）
=================================================
 -------------------------------------------------
终端1 — 启动相机                                                                                                      
source /opt/ros/humble/setup.bash
source ~/robot/Orbbec_ws/install/setup.bash
ros2 launch orbbec_camera gemini2.launch.py

终端2 — 启动情绪识别
source /opt/ros/humble/setup.bash                                                                                       
source /opt/tros/humble/setup.bash

可视化：
python3 ~/robot/Orbbec_ws/src/emotion_local/emotion_local/emotion_fusion_node.py --ros-args -p show_image:=true 
无窗口：
python3 ~/robot/Orbbec_ws/src/emotion_local/emotion_local/emotion_fusion_node.py

终端3 — 看 JSON 结果
source /opt/ros/humble/setup.bash
ros2 topic echo /emotion/result

