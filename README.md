# rdk-x5-orbbec-airobot-vision

The project uses ROS2 Humble as the main software framework. It integrates YOLOv8-based object detection, RGB-D depth sensing, robotic arm grasping control, facial emotion recognition, and human pose detection. The system is designed for a companion robot scenario, where the robot can perceive objects and humans in the environment and perform basic interaction tasks.

## Project Overview

The main goal of this project is to build a visual perception and robotic manipulation system on the RDK X5 platform. The Orbbec Gemini2 RGB-D camera is used to obtain color and depth images. YOLOv8 is used to detect target objects, and depth information is used to estimate the 3D position of the object. The estimated coordinates are then provided to the AIRobot robotic arm for grasping.

In addition to object grasping, the project also includes facial emotion recognition and human pose detection using the camera input, which can support human-robot interaction and elderly-care companion robot applications.

## Main Functions

- Orbbec Gemini2 RGB-D camera configuration on RDK X5
- ROS2 Humble camera topic acquisition
- YOLOv8-based object detection
- RGB-D depth-based target localization
- AIRobot robotic arm grasping control
- Facial emotion recognition
- Human pose detection
- Robot vision system integration and testing

## Hardware Platform

- RDK X5
- Orbbec Gemini2 RGB-D camera
- AIRobot robotic arm
- Robot chassis / companion robot platform

## Software Environment

- Ubuntu 22.04
- ROS2 Humble
- Python / C++
- OpenCV
- YOLOv8
- Orbbec SDK / Orbbec ROS2 driver

## My Contributions

- Configured and tested the Orbbec Gemini2 RGB-D camera on RDK X5.
- Built ROS2 nodes for camera data acquisition and object detection.
- Used YOLOv8 to detect target objects for robotic grasping.
- Used depth images to estimate target object coordinates.
- Integrated visual perception results with the AIRobot robotic arm.
- Implemented and tested facial emotion recognition using camera images.
- Implemented and tested human pose detection for human-robot interaction.
- Participated in system debugging, testing, and project integration.

## Current Status

The current version focuses on RGB-D perception, object detection, robotic grasping, facial emotion recognition, and human pose detection. Further work will improve grasping stability, multi-object interaction, and real-time deployment performance on RDK X5.
