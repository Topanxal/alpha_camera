# Alfa Robot Vision Module (v2.0)

`alfa_robot_vision` 是 Alfa Robot 物流机器人的视觉处理模块，基于 ROS 2 Humble 开发。
v2.0 版本采用了 **分层架构**，将全局场景分析与局部视觉伺服分离，以支持更复杂的物流任务（如分层抓取、侧拉/顶吸策略）。

## 架构概览

本模块包含两个核心节点：

1.  **Scene Analyzer Node (`scene_analyzer`)**
    *   **职责**：全局场景理解。
    *   **功能**：
        *   接收外部触发（Service）。
        *   使用 YOLOv8 检测视野内所有箱子。
        *   计算每个箱子的 3D 坐标。
        *   基于高度（Z轴）将箱子分层（Top/Bottom）。
        *   根据层级分配抓取策略（SIDE_PULL / TOP_PICK）。
    *   **接口**：
        *   Service: `analyze_scene` (logistics_interfaces/srv/AnalyzeScene)

2.  **Visual Servo Node (`visual_servo`)**
    *   **职责**：局部精细对准。
    *   **功能**：
        *   接收模式切换指令（Service）。
        *   **FACE_ALIGN (侧面模式)**：计算箱子中心偏差和表面法向量夹角（Pitch/Yaw）。
        *   **TOP_ALIGN (顶面模式)**：计算箱子边缘角度偏差（Yaw）和 XY 位移。
        *   实时发布伺服反馈（Feedback）。
    *   **接口**：
        *   Service: `set_servo_mode` (logistics_interfaces/srv/SetServoMode)
        *   Topic: `servo_feedback` (logistics_interfaces/msg/ServoFeedback)

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

## 故障排除
请参考 `TROUBLESHOOTING.md` 获取常见问题（如 NumPy 版本冲突、No detections 等）的解决方案。
