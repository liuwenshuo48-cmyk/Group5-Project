# Part 2 Interface Contract

## Vision -> Task Manager

- Topic: `/detections`
- Type: `vision_msgs/msg/Detection2DArray`
- Published class convention:
  - `0 = Square`
  - `1 = Circle`
- Bounding-box center:
  - `bbox.center.position.x`
  - `bbox.center.position.y`

> The YOLO training model internally uses the opposite raw ordering. `experiment3_vision/detector_node.py` performs the conversion before publishing `/detections`; downstream nodes must **not** invert it again.

## Task Manager -> Arm Execution

- Action: `/pick_and_place`
- Type: `experiment3_interfaces/action/PickAndPlace`

```text
# Goal
int32 grid_id
int32 bin_id
---
# Result
bool success
string message
---
# Feedback
string state
```

## Final simulation mapping

| Grid | Object | Bin |
|---|---|---|
| G1 | Square | Bin 1 |
| G2 | Square | Bin 1 |
| G3 | Square | Bin 1 |
| G4 | Circle | Bin 2 |
| G5 | Circle | Bin 2 |
| G6 | Circle | Bin 2 |

Final scheduling order: `G1 -> G2 -> G3 -> G4 -> G5 -> G6`.
