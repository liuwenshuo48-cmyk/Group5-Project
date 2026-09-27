# experiment3_vision

实验3 Part1 视觉识别节点：订阅相机图像，用YOLOv8检测Square(口香糖盒)/Circle(胶带)，发布检测结果供Part2使用。

## 依赖

- ROS 2 Humble
- Python包：`ultralytics`（pip安装，注意需要`numpy<2`，本机opencv-python 4.13.0与numpy1.26.4已验证兼容）
- ROS依赖：`rclpy`, `sensor_msgs`, `vision_msgs`, `cv_bridge`
- 摄像头驱动：`ros-humble-usb-cam`

## 模型文件

权重文件放在 `weights/best.pt`（YOLOv8n，训练指标见 `~/Exp_3_group_5/training_artifacts/`）。

## 编译
cd ~/Exp_3_group_5/experiment3_ws
colcon build --packages-select experiment3_vision
source install/setup.bash
## 运行

第一步，启动摄像头节点：
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0
第二步，另开终端，启动检测节点：
source ~/Exp_3_group_5/experiment3_ws/install/setup.bash
ros2 run experiment3_vision detector_node
启动后会弹出实时检测可视化窗口，同时向 `/detections` 发布 `vision_msgs/Detection2DArray`。

## 可调参数

| 参数名 | 默认值 | 说明 |
|---|---|---|
| weights_path | `~/Exp_3_group_5/experiment3_ws/src/experiment3_vision/weights/best.pt` | 模型权重路径 |
| image_topic | `/image_raw` | 输入图像topic |
| detection_topic | `/detections` | 输出检测结果topic |
| conf_threshold | 0.5 | 置信度过滤阈值 |
| show_window | true | 是否弹出实时可视化窗口 |

## 接口对接

详见 `~/Exp_3_group_5/docs/interface_part1_to_part2.md`，包含class_id映射关系（重要：模型原始输出与组内标准class_id相反，节点内部已转换）。

## 已知注意事项

- 摄像头`brightness`参数默认可能被设为异常值导致画面过曝，正常应为0：
v4l2-ctl -d /dev/video0 --set-ctrl=brightness=0
- 实测检测频率约30Hz（Jetson Orin NX）。
