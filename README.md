# Group5-Project

## Experiment 3 — Automatic Classification and Sorting of Desktop Objects

**Robotics Integration Group Project I**  
**Ocean University of China · Group 5 · 2026**

This repository contains the complete implementation of **Experiment 3: Automatic Classification and Sorting of Desktop Objects**.

The project integrates **YOLOv8 object detection, ROS 2 task control, Gazebo simulation, ArUco-based real-camera calibration, fixed-grid localization, and mechArm robotic-arm execution** into a complete desktop sorting system.

The overall workflow follows the course requirement of **simulation-first development followed by physical validation**.

---

## 1. Project Objective

The goal of this experiment is to automatically recognize and sort at least two categories of desktop objects.

In this project:

- **Square** objects are assigned to **Bin1**
- **Circle** objects are assigned to **Bin2**
- The desktop workspace is divided into six fixed grasping grids:
  - `G1`
  - `G2`
  - `G3`
  - `G4`
  - `G5`
  - `G6`

The complete task pipeline is:

```text
Camera
  ↓
YOLO Detection
  ↓
Grid Localization
  ↓
Category Mapping
  ↓
Bin Assignment
  ↓
Pick-and-Place Task
  ↓
mechArm / Gripper
  ↓
DONE / FAIL
  ↓
Next Target
```

---

## 2. Repository Structure

The repository is organized according to the four group modules used during Experiment 3.

```text
Group5-Project/
│
├── Experiment3-part1/
│   └── Vision recognition, dataset preparation, YOLO training,
│       simulation-domain adaptation and real-domain fine-tuning
│
├── Experiment3-Part2/
│   └── Experiment3-Part2/
│       └── Simulation task control, grid decision,
│           ROS 2 Action, state machine and Gazebo closed-loop sorting
│
├── Part3_Real_Vision/
│   └── Real-camera acquisition, ArUco perspective rectification,
│       six-grid calibration and vision-to-grid mapping
│
├── Experiment3-Part4/
│   └── Physical mechArm control, fixed-point grasping,
│       gripper control, safety handling and system integration
│
└── README.md
```

Each module contains its own source code, configuration files, experimental materials, and module-level documentation where applicable.

---

## 3. Group Division

| Member | Main Responsibility |
|---|---|
| **Wenshuo Liu** | Visual dataset, YOLO training, simulation/real-domain fine-tuning, Jetson deployment, ROS 2 detection messages |
| **Jingwen Cao** | Simulation task control, grid determination, state machine, PickAndPlace Action, robotic-arm and gripper simulation |
| **Wanqian Deng** | Real camera, ArUco perspective rectification, six-grid calibration, stable target queue, vision-to-grid mapping |
| **Xiaorui Dong** | Real mechArm, fixed grasping points, gripper control, safe positions, emergency handling, physical-system integration |

---

## 4. System Architecture

The project is divided into three logical layers.

### Perception Layer

- RGB / simulation camera input
- YOLOv8n object detection
- Object category prediction
- Bounding-box center extraction
- Confidence score output
- ArUco-based perspective correction for the physical system

### Task-Control Layer

- Six-grid localization
- Category-to-bin mapping
- Stable-frame confirmation
- Deterministic task queue
- State-machine control
- Duplicate-task prevention
- Exception handling

### Execution Layer

- ROS 2 PickAndPlace interface in simulation
- Fixed-point grasping on the real mechArm
- Gripper open/close control
- Safe-height motion
- Placement in Bin1 / Bin2
- HOME return
- DONE / FAIL feedback

---

## 5. Vision Module

The detector uses **YOLOv8n** for two categories:

```text
class_id = 0 → Square
class_id = 1 → Circle
```

The original dataset contains:

- 474 training images
- 135 validation images
- 67 test images
- 676 images in total

The initial model achieved an overall **mAP50 of 0.991**.

To reduce the domain gap between data collection, Gazebo simulation, and the final physical workspace, additional simulation-domain images and real-camera images were used for fine-tuning.

The final ROS 2 detection interface publishes object information including:

- class
- confidence
- bounding-box center
- bounding-box dimensions

---

## 6. Simulation System

The simulation module contains:

- mechArm270 model
- gripper
- desktop
- six picking grids
- six target objects
- two classification bins
- task-control node
- PickAndPlace interface
- exception handling
- complete ROS 2 closed-loop workflow

A single task follows the sequence:

```text
HOME
→ Target Above
→ Descend
→ Gripper Close
→ Lift
→ Move to Bin
→ Release
→ Retreat
→ HOME
```

The system processes detected targets sequentially and prevents repeated or concurrent grasp requests.

### Simulation Result

```text
6 / 6 targets successfully processed
```

---

## 7. Real-Vision Localization

The physical RGB camera is installed at an oblique angle, so perspective distortion must be corrected before stable grid localization.

Four ArUco markers are used to calculate a planar homography and generate a normalized top-view workspace.

The rectified image is mapped to six calibrated picking positions:

```text
G1  G2  G3
G4  G5  G6
```

The real-vision pipeline includes:

```text
RGB Camera
→ ArUco Detection
→ Perspective Rectification
→ YOLO Detection
→ Bounding-Box Center
→ Grid Mapping
→ Stable Target Queue
```

To reduce repeated triggering caused by detection jitter, a target set must remain stable for multiple consecutive frames before it is added to the execution queue.

---

## 8. Physical Robot Integration

The physical system uses predefined grasping and placement positions.

For each grid, the robot uses:

- safe approach position
- grasping position
- lifting motion
- Bin1 / Bin2 placement position
- safe retreat
- HOME position

The complete physical workflow is:

```text
Camera Capture
→ YOLO Detection
→ ArUco Rectification
→ Grid Mapping
→ Category Determination
→ Bin Assignment
→ Safe Approach
→ Grasp
→ Lift
→ Move to Bin
→ Release
→ HOME
→ DONE
→ Next Target
```

---

## 9. Exception Handling and Safety

The system includes handling strategies for:

- empty grids
- unrecognized objects
- low-confidence detections
- repeated detections
- grasp failure
- dropped objects
- unreachable positions
- unsafe motion conditions

Safety measures include:

- fixed verified grasp points
- safe-height motion
- HOME reset
- gripper-state checks
- joint-limit checks
- low-speed physical debugging
- emergency stopping
- failure-state handling

---

## 10. Final Experimental Results

### Simulation

| Item | Result |
|---|---:|
| Total targets | 6 |
| Successfully processed | 6 |
| Completion rate | 100% |

### Physical Experiment

| Grid | Category | Bin | Grasping | Placement |
|---|---|---|---|---|
| G1 | Square | Bin1 | Success | Success |
| G2 | Square | Bin1 | Success | Success |
| G3 | Square | Bin1 | Success | Success |
| G4 | Circle | Bin2 | Success | Success |
| G5 | Circle | Bin2 | Success | Success |
| G6 | Circle | Bin2 | Failure | Failure |

Final physical sorting result:

```text
5 / 6 successfully sorted
Success rate: 83.3%
```

The result satisfies the course requirement that at least five out of six objects be correctly sorted.

---

## 11. Main Software and Hardware

### Software

- Ubuntu
- ROS 2
- Python
- CMake
- Gazebo
- Ultralytics YOLOv8
- OpenCV
- vision_msgs
- tf2 / tf2_ros

### Hardware

- Jetson platform
- RGB camera
- mechArm robotic arm
- gripper
- desktop sorting workspace
- ArUco markers

---

## 12. How to Use This Repository

Because the complete system is divided into four modules, please refer to the corresponding module directory for detailed setup and execution instructions.

```text
Vision / YOLO:
Experiment3-part1/

Simulation / Task Control:
Experiment3-Part2/Experiment3-Part2/

Real Vision:
Part3_Real_Vision/

Real Robot:
Experiment3-Part4/
```

For physical deployment, confirm all camera parameters, network settings, robot connection parameters, grasping coordinates, and safety positions before execution.

---

## 13. Key Engineering Features

This project uses several engineering strategies to improve stability and repeatability:

- fixed-grid grasping instead of arbitrary-position grasp planning
- YOLO domain adaptation for simulation and real environments
- ArUco homography for stable real-world coordinate mapping
- deterministic multi-target scheduling
- stable-frame confirmation
- processed-target deduplication
- feedback-based task execution
- safe-height robotic-arm motion
- layered debugging from simulation to physical deployment

---

## 14. Authors

**Group 5**

- Jingwen Cao
- Xiaorui Dong
- Wanqian Deng
- Wenshuo Liu

Ocean University of China  
Robotics Integration Group Project I  
2026
