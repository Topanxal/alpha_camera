# Alfa Robot Vision Module (v2.0)

`alfa_robot_vision` 是 Alfa Robot 物流机器人的视觉处理模块，基于 ROS 2 Humble 开发。
v2.0 版本采用了 **分层架构**，将全局场景分析与局部视觉伺服分离，以支持更复杂的物流任务（如分层抓取、侧拉/顶吸策略）。

## 架构详解 (System Architecture)

### 1. Scene Analyzer Node (`scene_analyzer`)
**定位**: "大脑" (Decision Maker)
*   **输入**: RGB-D 图像流 + 外部触发信号。
*   **处理流程**:
    1.  **YOLO Detection**: 识别所有可见箱子。
    2.  **3D Projection**: 结合深度图计算箱子中心坐标 (Camera Frame)。
    3.  **TF Transform**: 将坐标转换到 `base_link` 或 `map` 坐标系。
    4.  **Strategy Allocation**:
        *   若 `z > layer_height_threshold`: 判定为顶层，策略为 **SIDE_PULL (侧拉)**。
        *   若 `z <= layer_height_threshold`: 判定为底层，策略为 **TOP_PICK (顶吸)**。
*   **输出**: 包含所有目标箱子 ID、策略、位姿的列表。

### 2. Visual Servo Node (`visual_servo`)
**定位**: "小脑" (Action Controller)
*   **输入**: RGB-D 图像流 + 模式切换指令。
*   **模式逻辑**:
    *   **FACE_ALIGN (侧面模式)**:
        *   目标: 距离表面 20cm，且法向量平行于光轴。
        *   算法: ROI 深度均值控制距离，法向量投影控制 Pitch/Yaw。
    *   **TOP_ALIGN (顶面模式)**:
        *   目标: 对准最平整的吸取区域 (Best Patch)。
        *   算法: **Patch-based Grasping** (3x3 网格深度方差搜索)。
*   **输出**: 实时的高频误差反馈 (`error_x`, `error_y`, `error_z`, `yaw_error`)。

---

## ROS 接口规范 (ROS Interfaces)

### Topics
| Topic Name | Type | Publisher/Subscriber | Description |
| :--- | :--- | :--- | :--- |
| `/head_camera/rgb/image_raw` | `sensor_msgs/Image` | Sub | RGB 图像输入 |
| `/head_camera/depth/image_rect_raw` | `sensor_msgs/Image` | Sub | 深度图像输入 |
| `/head_camera/camera_info` | `sensor_msgs/CameraInfo` | Sub | 相机内参 |
| `/scene_debug_image` | `sensor_msgs/Image` | Pub | YOLO 检测与策略可视化 |
| `/servo_debug_image` | `sensor_msgs/Image` | Pub | 伺服准星与 Patch 可视化 |
| `/scene_markers` | `visualization_msgs/MarkerArray` | Pub | **[New]** Rviz 3D 可视化 (绿色=侧拉, 蓝色=顶吸) |
| `/servo_feedback` | `logistics_interfaces/ServoFeedback` | Pub | 实时伺服误差反馈 |

### Services
| Service Name | Type | Description |
| :--- | :--- | :--- |
| `/analyze_scene` | `logistics_interfaces/AnalyzeScene` | 触发全局分析，返回所有箱子策略列表 |
| `/set_servo_mode` | `logistics_interfaces/SetServoMode` | 切换伺服模式 (IDLE / FACE_ALIGN / TOP_ALIGN) |

### TF Frames
*   **`base_link`**: 机器人基座坐标系 (Target Frame)。
*   **`head_camera_optical_frame`**: 相机光学坐标系 (Source Frame)。
*   **`map`** (Optional): 全局地图坐标系 (用于多箱子持久化追踪)。

---

## 环境依赖

*   **System**: Ubuntu 22.04 / ROS 2 Humble / Python 3.10
*   **Python Libraries**:
    *   `ultralytics`
    *   `numpy<2.0` (Critical for cv_bridge compatibility)
    *   `opencv-python`
*   **ROS Packages**:
    *   `realsense2_camera`
    *   `logistics_interfaces` (包含自定义消息定义)

## 快速开始

### 1. 编译
```bash
colcon build --packages-select logistics_interfaces alfa_robot_vision
source install/setup.bash
```

### 2. 启动硬件驱动
```bash
ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=640x480x30 depth_module.profile:=640x480x30 align_depth.enable:=true
```

### 3. 启动视觉系统
```bash
ros2 launch alfa_robot_vision vision_system.launch.py
```

## 参数配置 (`config/vision_params.yaml`)

| Node | Parameter | Default | Description |
| :--- | :--- | :--- | :--- |
| **scene_analyzer** | `model_path` | "box_best.pt" | YOLO模型路径 |
| | `layer_height_threshold` | 1.2 | 分层高度阈值 (米) |
| | `confidence_threshold` | 0.5 | 检测置信度 |
| **visual_servo** | `target_frame` | "base_link" | 目标坐标系 |

## 调试
使用 `rqt_image_view` 查看调试话题：
*   `/scene_debug_image`: 显示 YOLO 检测结果和策略分配。
*   `/servo_debug_image`: 显示伺服对准过程中的特征提取情况。

## 文档索引 (Documentation)

详细的开发文档和规划请参考 `docs/` 目录：

*   **[VLA Roadmap](docs/vla_roadmap.md)**: 未来的视觉-语言-动作模型演进路线图。
*   **[Development Plan](docs/develop_plan.md)**: 当前版本的开发计划与状态。
*   **[Test Plan](docs/vision_test_plan.md)**: 视觉系统的测试用例与验证方法。
*   **[Troubleshooting](docs/TROUBLESHOOTING.md)**: 常见问题排查（如 NumPy 冲突、TF 错误等）。
*   **[TODO](docs/todo.md)**: 待办事项列表。
*   **[Env Setup Notes](docs/env_setup_notes.md)**: 环境配置备忘录。

## 故障排除
请参考 [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) 获取常见问题（如 NumPy 版本冲突、No detections 等）的解决方案。
