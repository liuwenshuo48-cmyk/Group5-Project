🤖 Experiment 3 · Part 4 

Desktop Object Classification \& Sorting · mechArm Real‑Robot Control Module

Group 5 · Robotics Integration Project



Part 2 subscribes detection outputs from the YOLOv8 vision module (Part 1). It implements grid mapping, task scheduling and exception handling, and controls the physical mechArm robotic arm to complete pick‑and‑place sorting operations. This work finishes sim‑to‑real migration. Task logic developed in simulation is reused, and only the robot execution unit is replaced for real hardware control.



Repository policy: Source code, configuration files, launch scripts and documentation are maintained on GitHub. Raw experiment photos, complete runtime logs and the full lab report are stored on Quark Cloud Drive.



\---



\## System Pipeline

Vision Detection → Grid Localization → Bin Assignment → Task Dispatch → Real Robot Execution → Exception Handling → Status Feedback → Loop



\- Input topic subscribed from Part 1: `vision\_msgs/Detection2DArray`

\- Output: pick‑and‑place task commands passed to the mechArm hardware driver



\### Grid‑Bin Mapping

| Grid | Object Class | Target Bin |

|:-----|:-------------|:-----------|

| G1‑G3 | Square | Bin1 |

| G4‑G6 | Circle | Bin2 |



Notes

\- Class ID remapping is completed inside the vision node. No additional class conversion is required within this task‑control node.

\- Fixed‑point grasping is adopted. Each grid corresponds to pre‑calibrated Cartesian coordinates of the robotic arm.

\- Single operation cycle: Approach → Descend → Close gripper → Lift → Move to target bin → Open gripper → Retract → Return to HOME position



\---



\## Repository Structure

```

Experiment2‑part2/

├── config/                  # YAML files for grasp points, placement points and HOME position

├── real\_robot\_driver/       # mechArm communication interface and gripper control logic

├── grid\_task/               # Grid mapping, bin assignment and task state machine

├── exception\_handler/       # Logic for empty grid, unrecognized object and grasping failure

├── safety\_check/            # Joint limit verification, collision pre‑check and HOME reset

├── launch/

│   └── real\_classification.launch.py  # Main launch file for physical robot experiment

├── logs/                    # Runtime log outputs (large log files are ignored by git)

└── README.md

```



Individual experiment report PDF and original experiment screenshots are not committed to GitHub.



\---



\## Pre‑launch Hardware Debug Checklist

Before starting automatic classification tasks, complete the following debugging work:

1\. Establish communication between ROS 2 and mechArm, verify joint state feedback and gripper opening‑closing actions.

2\. Confirm the safe HOME position and valid joint angle range.

3\. Tune grasping points for six grids and placement points for two bins. Ensure all waypoints lie within the robot’s workspace.

4\. Carry out collision inspection to avoid collision with the table surface, camera bracket and other experimental equipment.

5\. Load calibration data to build the coordinate mapping from image pixel space to desktop grid space.



Hardware configuration:

\- Top‑mount overhead RGB camera (eye‑to‑hand configuration)

\- mechArm robotic arm equipped with electric gripper

\- Six picking grids numbered G1‑G6 and two classification bins



\---



\## Workflow \& Exception Rules

\### Full automatic classification workflow

1\. The overhead camera captures images of the desktop scene, which is processed by the Part 1 vision node.

2\. The vision node publishes detection results including object category and bounding box.

3\. Map the center of detection bounding box to the corresponding grid ID.

4\. Assign target classification bin: Square objects go to Bin1; Circle objects go to Bin2.

5\. Dispatch pick‑and‑place task with grid ID and target bin ID.

6\. The robotic arm executes fixed‑point grasping, transportation and placement operations.

7\. The robot returns to HOME safe position and continues processing the next valid target.



\### Exception handling

\- \*\*Empty grid\*\*: Skip the current grid without sending grasping commands to avoid meaningless empty grasping movement.

\- \*\*Unidentified object\*\*: Skip the target or enter waiting state to prevent mis‑classification.

\- \*\*Grasping failure / object drop\*\*: Mark current task as failed, terminate subsequent operations for this target, and make the robot return to safe status.



\### Safety requirements

\- Verify the validity of grasping points, placement points and HOME position in advance.

\- Prevent collision with table and experimental fixtures during movement.

\- Ensure classification bins are within the robot’s reachable workspace.

\- Restrict joint angles within hardware safety limits.

\- Support emergency stop response. The robot returns to HOME position when tasks finish.



\---



\## Experiment Metrics

Test configuration: 3 square objects placed in G1‑G3, 3 circle objects placed in G4‑G6.



| Grid | Class  | Bin  | Grasp    | Place    | Remarks               |

|------|--------|------|----------|----------|-----------------------|

| G1   | Square | Bin1 | success  | success  |                       |

| G2   | Square | Bin1 | success  | success  |                       |

| G3   | Square | Bin1 | success  | success  |                       |

| G4   | Circle | Bin2 | success  | success  |                       |

| G5   | Circle | Bin2 | success  | success  |                       |

| G6   | Circle | Bin2 | lose     | lose     | Grasp‑drop failure    |



\- Total targets: 6

\- Successful grasp: 5

\- Correct classification: 5

\- Classification success rate: 100%

\- Abnormal tasks: 1

\- Collision count: 1

\- Joint overload: 0



Sim‑to‑real experience: In physical experiments, errors are introduced by camera installation deviation, calibration drift, robot positioning error and gripper instability, which do not appear in simulation. Reusing simulation task interfaces and only modifying robot execution modules can reduce the workload of simulation‑to‑hardware migration.



\---



\## Run Instructions

Running environment: Jetson Orin NX, ROS 2 Humble



Prerequisite: The Part 1 YOLOv8 vision node is active and publishing `/detections` topic.



```bash

ros2 launch experiment2\_real\_robot real\_classification.launch.py

```



Important file paths:

```

\# Fixed‑point coordinate parameters

\~/Exp\_3\_group\_5/experiment3\_ws/src/experiment2\_real\_robot/config/grid\_points.yaml

\# Backup of HOME position parameters

home\_position\_backup.yaml

\# Interface description document

\~/Exp\_3\_group\_5/docs/interface\_part2\_to\_part1.md

```



\---



\## Storage Strategy

| GitHub Contents               | Quark Cloud Drive Assets                          |

|:------------------------------|:--------------------------------------------------|

| Source code, launch files, configuration | Raw experiment photos, terminal screenshots |

| Documentation and README file | Full lab report, large‑size runtime log files |



The repository is kept lightweight for group collaboration. Original experimental materials are stored on cloud drive for experiment reproduction.



\---



\## Environment

\- ROS 2 Humble

\- Hardware: mechArm robotic arm, electric gripper, overhead RGB camera

\- Computing platform: NVIDIA Jetson Orin NX

\- Simulation baseline: Gazebo, task logic migrated from simulation projects

\- Dependency: Experiment3‑part1 YOLOv8 vision recognition node



Part 2 outputs physical robot pick‑and‑place actions, runtime status feedback and exception handling logic. It completes sim‑to‑real deployment for desktop object classification system.

```

