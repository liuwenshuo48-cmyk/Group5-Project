# 🤖 Experiment 3 · Part 1 — Vision Recognition

> **Desktop Object Classification & Sorting · YOLOv8 Vision Module**  
> Group 5 · Robotics Integration Project

Part 1 is responsible for the **vision recognition** stage of the system. A top-view camera and YOLOv8 are used to detect **Square** and **Circle** objects, then publish detections through ROS 2 for downstream positioning and task control.

> **Repository policy:** GitHub keeps the code-facing experiment record, key metrics, plots, and best model weights. Large datasets are stored separately in Quark Cloud Drive.

---

## 🔗 System Pipeline

**Recognition → Decision → Grasping → Classification → Exception Handling**

This module publishes:

```text
vision_msgs/Detection2DArray
```

for **Part 2 — Grid Positioning & Task Control**.

### Class convention

| Group standard | Object | Quantity | Target bin |
|---|---|---:|---|
| Class 0 | Square | 3 | `Square_bin` |
| Class 1 | Circle | 3 | `Circle_bin` |

> [!IMPORTANT]
> The training dataset uses **`0 = Circle, 1 = Square`**, which is the reverse of the group interface standard.  
> Do **not** modify `data.yaml` or the YOLO `.txt` labels. The class IDs are remapped in the Jetson ROS 2 detection node before publishing.

---

## 📁 Repository Layout

```text
Experiment3-part1/
├── train/                   # Initial YOLOv8n training
├── circle_adapt_v1/         # Gazebo / simulation adaptation
├── train_finetune/          # Real-camera fine-tuning
├── README.md
└── E3刘文硕24020036044.pdf  # Individual experiment report
```

Large datasets and automatically generated batch previews are intentionally excluded from GitHub.

---

## ☁️ Dataset Downloads

### ① Initial YOLO Dataset — `dataset/`

**676 images** captured by phone and converted to standard YOLO format.

| Split | Images |
|---|---:|
| Train | 474 |
| Validation | 135 |
| Test | 67 |

**Quark Cloud Drive:**  
https://pan.quark.cn/s/15479040d1c2?pwd=zYMx

**Extraction code:** `zYMx`

Expected structure:

```text
dataset/
├── images/
│   ├── train/
│   ├── val/
│   └── test/
├── labels/
│   ├── train/
│   ├── val/
│   └── test/
└── data.yaml
```

Dataset class IDs remain:

```text
0 = Circle
1 = Square
```

### ② Real-Camera Recapture Dataset — `circle_recapture-dataset/`

During real-robot testing, the Circle object became much harder to recognize because the actual camera angle differed from the original training viewpoint. **48 groups of images** were therefore recaptured using the robot's real camera and annotated with LabelMe.

**Quark Cloud Drive:**  
https://pan.quark.cn/s/3114832e3422?pwd=Cy6D

**Extraction code:** `Cy6D`

Typical structure:

```text
circle_recapture-dataset/
├── circle_001.jpg
├── circle_001.json
├── ...
├── circle_048.jpg
└── circle_048.json
```

Both Circle and Square objects are annotated in the LabelMe `.json` files.

---

## 🧪 Training Stages

### Stage 1 · Initial Training

**Directory:** `train/`  
**Base model:** YOLOv8n  
**Maximum epochs:** 150  
**Actual training:** ~86 epochs (early stopping)

```bash
yolo detect train \
  model=yolov8n.pt \
  data=dataset/data.yaml \
  epochs=150 \
  imgsz=640 \
  batch=16 \
  device=0 \
  workers=4 \
  patience=30 \
  project=. \
  name=train
```

| Metric | Result |
|---|---:|
| mAP50 | **0.991** |
| mAP50-95 | **0.855** |

---

### Stage 2 · Simulation Adaptation

**Directory:** `circle_adapt_v1/`

After deployment in Gazebo, Square remained detectable while Circle recognition degraded under the simulated rendering conditions.

A semi-automatic annotation workflow was used:

```text
Automatic Square detection
        +
Manual Circle selection
        ↓
Simulation ground-truth labels
        ↓
Local fine-tuning (~20 epochs)
```

After adaptation, both Square and Circle could be detected in the same simulation scene with confidence values of approximately **0.86+**.

---

### Stage 3 · Real-Camera Fine-Tuning

**Directory:** `train_finetune/`  
**Starting weights:** `train/weights/best.pt`  
**Fine-tuning:** 50 epochs

The recaptured real-camera data were added to reduce the viewpoint gap between the original dataset and the physical robot camera.

```bash
yolo detect train \
  model=train/weights/best.pt \
  data=dataset/data.yaml \
  epochs=50 \
  imgsz=640 \
  batch=16 \
  device=0 \
  project=. \
  name=train_finetune
```

New real-camera data split:

```text
40 train / 8 validation
```

| Metric | Result |
|---|---:|
| Overall mAP50 | **0.988** |
| Circle mAP50 | **0.995** |
| Circle Precision | **0.999** |
| Square mAP50 | **0.981** |

> [!NOTE]
> The validation set contains 143 images, only 8 of which are newly captured real-camera images. The numerical improvement is therefore expected to be modest; the practical benefit was evaluated mainly through real-robot Circle recognition tests.

---

## 📦 What Is Kept in Each Training Directory?

To keep the repository lightweight, each training stage retains only the most useful experiment artifacts:

```text
<training-directory>/
├── weights/
│   └── best.pt
├── args.yaml
├── results.csv
├── results.png
├── confusion_matrix.png
├── confusion_matrix_normalized.png
├── BoxF1_curve.png
├── BoxP_curve.png
├── BoxR_curve.png
└── BoxPR_curve.png
```

The following generated files are intentionally omitted:

```text
train_batch*.jpg
val_batch*.jpg
weights/last.pt
large intermediate datasets
other non-essential training artifacts
```

---

## 🚀 Deployment

### Final model

```text
train_finetune/weights/best.pt
```

The final model is deployed on **Jetson Orin NX + ROS 2 Humble** at:

```text
~/Exp_3_group_5/experiment3_ws/src/experiment3_vision/weights/best.pt
```

The original Stage 1 model is backed up as:

```text
best_v1_backup.pt
```

### Jetson documentation

```text
~/Exp_3_group_5/docs/interface_part1_to_part2.md
~/Exp_3_group_5/experiment3_ws/src/experiment3_vision/README.md
```

---

## 🗂️ Storage Strategy

| GitHub | Cloud Drive |
|---|---|
| `best.pt` | Full image datasets |
| Training configuration | Real-camera source images |
| `results.csv` | LabelMe source annotations |
| Evaluation plots | Large intermediate data |
| README & report | Other bulky raw assets |

This keeps the repository **small, readable, and useful for collaboration** while preserving the data required to reproduce the experiments.

---

## 🛠️ Environment

- **Model:** YOLOv8n
- **Framework:** Ultralytics YOLO
- **Middleware:** ROS 2 Humble
- **Deployment:** NVIDIA Jetson Orin NX
- **Simulation:** Gazebo
- **Task:** Two-class object detection (`Circle`, `Square`)

---

> **Part 1 output:** object detection results → ROS 2 → Part 2 grid positioning & task control
