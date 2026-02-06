# 视觉系统测试规划 (Intel RealSense D455f)

本文档旨在规划 `alfa_robot_vision` 模块与 Intel RealSense D455f 相机的集成测试流程。测试分为环境准备、硬件连接、软件通信和功能验证四个阶段。

## 阶段一：环境准备与驱动安装
**目标**：确保系统具备运行 RealSense 相机所需的驱动和 ROS 2 接口包。

### 1.1 系统检查 (已执行)
- [x] 检查 `librealsense2` (未安装)
- [x] 检查 `ros-humble-realsense2-camera` (未安装)

### 1.2 安装步骤
1.  **安装 Intel RealSense SDK 2.0 (`librealsense2`)**
    *   注册 Intel 服务器公钥。
    *   添加 Intel apt 仓库。
    *   安装 `librealsense2-dkms` 和 `librealsense2-utils`。
2.  **安装 ROS 2 RealSense 包装器**
    *   运行 `sudo apt install ros-humble-realsense2-camera`。
    *   运行 `sudo apt install ros-humble-realsense2-description`。

### 1.3 验证安装
- 运行 `rs-enumerate-devices --version` 确认 SDK 版本。
- 运行 `ros2 pkg list | grep realsense` 确认 ROS 包已就绪。

---

## 阶段二：硬件连接与驱动测试
**目标**：确认相机硬件被系统识别，且 ROS 驱动能正常发布图像数据。

### 2.1 硬件识别
1.  将 D455f 相机连接至 USB 3.0 接口。
2.  运行 `rs-enumerate-devices` 查看是否检测到设备信息（序列号、固件版本）。

### 2.2 ROS 驱动运行
1.  启动相机节点：
    ```bash
    ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=640x480x30 depth_module.profile:=640x480x30 align_depth.enable:=true
    ```
    *(注：根据实际需求调整分辨率和帧率，D455f 建议开启 `align_depth.enable` 以便 RGB-D 对齐)*
2.  检查话题列表：
    - `ros2 topic list`
    - 确认 `/camera/camera/color/image_raw` 和 `/camera/camera/depth/image_rect_raw` (或 `aligned_depth_to_color`) 存在。
3.  检查数据频率：
    - `ros2 topic hz /camera/camera/color/image_raw`

---

## 阶段三：视觉服务节点集成测试
**目标**：验证 `alfa_robot_vision` 节点能否正确连接相机话题并提供服务。

### 3.1 配置确认
- 检查 `alfa_robot_vision/config/vision_params.yaml` 中的话题名称是否与 `realsense2_camera` 发布的一致。
    - 默认 RealSense 发布在 `/camera/...`，而 Vision Node 监听 `/head_camera/...`。
    - **行动项**：需要修改 Launch 文件进行 Remap，或者修改配置文件。建议在 Launch 文件中做 Remap。

### 3.2 节点启动
1.  启动视觉节点（包含 Remap）：
    ```bash
    ros2 launch alfa_robot_vision vision_bringup.launch.py
    ```
2.  查看日志，确认 "Camera Info Received" 和 "Vision Server Initialized"。

### 3.3 服务可用性
- 运行 `ros2 service list` 确认 `/get_object_list` 存在。

---

## 阶段四：识别功能验证
**目标**：验证 YOLO 模型推理及 3D 坐标转换功能。

### 4.1 静态物体识别
1.  在相机前放置已知物体（如盒子）。
2.  手动调用服务：
    ```bash
    ros2 service call /get_object_list logistics_interfaces/srv/QueryObjects "{target_classes: []}"
    ```
3.  观察返回结果：
    - `success`: True
    - `objects`: 列表不为空
    - `pose`: x, y, z 坐标是否合理（例如距离相机 1米左右，z 应接近 1.0）。

### 4.2 坐标验证（粗略）
- 移动物体，观察 x, y, z 变化趋势是否符合预期（左/右移动 x 变，远/近移动 z 变）。

---

## 下一步行动建议
1.  请确认是否开始执行 **阶段一：环境准备** 的安装工作？
