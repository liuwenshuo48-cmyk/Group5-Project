# 🤖 Experiment 3 · Part 3 --- Real Vision & Workspace Calibration

> Desktop Object Sorting System · Real Camera Vision Module\
> Group 5 · Robotics Integration Project

------------------------------------------------------------------------

## 1. Overview

Part 3 implements the real-world vision perception and workspace
calibration module of Experiment 3.

This module transfers the vision system from simulation to the physical
robot environment using a real RGB camera.

Main responsibilities:

-   Real RGB camera acquisition
-   ArUco marker detection and camera alignment verification
-   Image rectification
-   YOLOv8 real-object detection deployment
-   Six-grid workspace calibration
-   Pixel coordinate to Grid coordinate mapping
-   ROS 2 vision information publishing

The output is provided to:

    Part 4 — Real mechArm270 Manipulation Integration

------------------------------------------------------------------------

## 2. System Pipeline

    Real RGB Camera
            |
            v
    ArUco Detection
            |
            v
    Camera Rectification
            |
            v
    YOLO Object Detection
            |
            v
    Pixel Coordinate
            |
            v
    Grid Calibration
            |
            v
    ROS 2 Vision Output
            |
            v
    Robot Manipulation

------------------------------------------------------------------------

## 3. Main Functions

### 3.1 Real Camera Vision

The real RGB camera provides workspace images for object detection and
robot interaction.

------------------------------------------------------------------------

### 3.2 ArUco Calibration

  File                   Function
  ---------------------- ----------------------------------
  aruco_check.py         Detect ArUco markers
  aruco_rectify.py       Image rectification
  aruco_rectify_pub.py   ROS 2 rectified image publishing

------------------------------------------------------------------------

### 3.3 Workspace Grid Calibration

The physical workspace is divided into six pickup grids.

Relationship:

    Image Pixel Coordinate
              |
              v
    Grid Coordinate
              |
              v
    Robot Target Position

Files:

  File                Function
  ------------------- ---------------------------
  grid_calibrate.py   Generate calibration data
  grid_mapper.py      Pixel/Grid transformation
  grid_check.py       Grid verification
  grid_check_tcp.py   TCP communication test
  grid_publisher.py   Publish Grid information
  send_grid_test.py   Test messages

Calibration file:

    calibration/
    └── real_grid_points.json

------------------------------------------------------------------------

## 4. YOLO Real Vision Detection

Model:

    model/
    └── best.pt

Detection node:

    code/detector_node.py

Functions:

-   Load YOLO model
-   Detect objects
-   Calculate object center
-   Publish detection information

------------------------------------------------------------------------

## 5. Repository Structure

    Part3_Real_Vision/

    ├── code/
    │   ├── detector_node.py
    │   ├── aruco_check.py
    │   ├── aruco_rectify.py
    │   ├── aruco_rectify_pub.py
    │   ├── grid_calibrate.py
    │   ├── grid_check.py
    │   ├── grid_check_tcp.py
    │   ├── grid_mapper.py
    │   ├── grid_publisher.py
    │   └── send_grid_test.py
    │
    ├── model/
    │   └── best.pt
    │
    ├── calibration/
    │   └── real_grid_points.json
    │
    ├── screenshots/
    ├── logs/
    └── backups/

------------------------------------------------------------------------

## 6. Deployment Environment

    Hardware:
    NVIDIA Jetson Orin NX

    OS:
    Ubuntu 22.04

    Middleware:
    ROS 2 Humble

    Vision:
    YOLOv8 + OpenCV

    Camera:
    RGB Camera

------------------------------------------------------------------------

## 7. Running Procedure

### Start ROS 2

``` bash
source /opt/ros/humble/setup.bash
source ~/Exp_3_group_5/experiment3_ws/install/setup.bash
```

### Start Camera

``` bash
start_camera.sh
```

### Start ArUco Rectification

``` bash
python3 aruco_rectify_pub.py
```

### Start YOLO Detector

``` bash
ros2 run experiment3_vision detector_node
```

### Check Grid Mapping

``` bash
python3 grid_check.py
```

------------------------------------------------------------------------

## 8. Final Result

After completing Part 3:

-   Real RGB camera captures the workspace
-   ArUco calibration verifies camera position
-   YOLO detects real objects
-   Six workspace grids are calibrated
-   Pixel coordinates are converted into Grid positions
-   Vision results are transferred to Part 4

------------------------------------------------------------------------

## 9. Notes

-   Part 3 focuses on real vision perception and workspace calibration.
-   Robot control and grasp execution belong to Part 4.
-   Simulation implementation belongs to Part 2.

------------------------------------------------------------------------

Experiment 3 · Group 5 · Part 3 --- Real Vision & Workspace Calibration
