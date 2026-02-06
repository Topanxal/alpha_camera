# 视觉系统仿真测试指南 (Simulation Test Guide)

本文档旨在指导开发者在**没有真实机械臂**和**决策层**的情况下，利用手持相机和可视化工具，对视觉系统的核心功能进行闭环验证。通过 "Human-in-the-Loop"（人作为执行器）的方式，模拟机器人作业流程。

---

## 1. 测试环境搭建 (Setup)

### 硬件准备
*   **工控机**: 已安装 ROS 2 和 `alfa_robot_vision`。
*   **双 Realsense 相机**:
    *   **Global Camera**: 模拟安装在龙门架/高处的全局相机（建议固定在三脚架或高处）。
    *   **Local Camera**: 模拟安装在机械臂末端的手眼相机（手持，用于模拟机械臂运动）。
*   **测试对象**: 若干个标准纸箱（建议贴上标签或胶带模拟真实纹理）。

### 软件准备
*   启动视觉系统：
    ```bash
    ros2 launch alfa_robot_vision vision_system.launch.py
    ```
*   启动可视化工具：
    ```bash
    ros2 run rqt_image_view rqt_image_view
    rviz2
    ```

---

## 2. 测试流程 (Workflow)

### 第一步：静态场景分析验证 (Static Scene Validation)
**目标**：验证 `scene_analyzer` 的检测精度、3D 坐标转换准确性及策略分配逻辑。

1.  **场景布置**：
    *   在 Global Camera 视野内摆放 2-3 个箱子。
    *   设置不同高度（模拟不同层级）和位置。
2.  **触发检测**：
    *   打开终端，手动调用服务：
        ```bash
        ros2 service call /analyze_scene logistics_interfaces/srv/AnalyzeScene "{trigger: true}"
        ```
3.  **验证点 (Checklist)**：
    *   **2D 视觉 (RQT)**：
        *   查看 `/scene_debug_image`。
        *   [ ] 所有箱子是否都被框出？
        *   [ ] 标签（SIDE_PULL/TOP_PICK）是否符合预期高度？
    *   **3D 空间 (Rviz)**：
        *   添加 `PointCloud2` (Global Camera)。
        *   添加 `MarkerArray` (话题 `/scene_markers`，需代码实现)。
        *   [ ] **核心验证**：算法生成的 3D 包围盒（Marker）是否与点云中的真实箱子**严丝合缝**？
        *   *故障征兆*：如果 Marker 漂浮在点云前方或后方，说明深度标定或内参有问题。

### 第二步：手眼伺服模拟 (Hand-Eye Servo Simulation)
**目标**：验证 `visual_servo` 的实时反馈逻辑和误差计算。

1.  **场景布置**：
    *   只保留一个目标箱子。
2.  **模拟操作**：
    *   手持 Local Camera，将其视为机械臂末端。
    *   **Face Align (侧面)**：将相机正对箱子侧面，距离约 30-50cm。
    *   **Top Align (顶面)**：将相机垂直向下对准箱子顶面。
3.  **触发模式**：
    ```bash
    ros2 service call /set_servo_mode logistics_interfaces/srv/SetServoMode "{mode: 1}"  # 1=FACE, 2=TOP
    ```
4.  **验证点 (Checklist)**：
    *   **动态反馈 (RQT)**：
        *   查看 `/servo_debug_image`。
        *   画面中应显示十字准星（Image Center）和目标特征点。
    *   **数据验证 (Terminal)**：
        *   观察 `/servo_feedback` 话题输出的 `error_x`, `error_y`, `error_yaw`。
        *   [ ] **归零测试**：尝试移动手部，使画面对准。当您觉得“对准了”时，检查误差数据是否接近 0。
        *   [ ] **解耦测试**：只平移不旋转，检查是否只有 XYZ 误差变化，角度误差不变（反之亦然）。

### 第三步：全流程“伪”闭环 (Mock Loop)
**目标**：模拟从“看到”到“抓取”的全过程，验证坐标系转换链条。

1.  **Phase 1: 全局定位**
    *   Global Camera 拍照分析。
    *   在 Rviz 中记录目标箱子的 3D 坐标（例如：x=0.5, y=0.2, z=0.8）。
2.  **Phase 2: 粗定位 (Coarse Move)**
    *   **人肉机械臂**：看着 Rviz 的坐标，手持 Local Camera 移动到该坐标附近（模拟机械臂规划路径到达 Pre-grasp 点）。
    *   *验证*：此时 Local Camera 的画面中应该能看到目标箱子。如果看不到，说明 Global -> Base -> Local 的 TF 链条有严重偏差。
3.  **Phase 3: 精定位 (Fine Servo)**
    *   切换到 Servo 模式。
    *   根据反馈微调手部位置，直到误差归零。
    *   *物理验证*：此时用尺子测量相机与箱子表面的距离，看是否符合预期的进刀距离（如 20cm）。

---

## 3. 可视化工具配置建议 (Rviz Setup)

为了达到最佳观察效果，建议在 Rviz 中保存一个 `.rviz` 配置文件，包含以下 Display：

1.  **Global Frame**: `base_link` (或 `map`)。
2.  **PointCloud2**:
    *   Topic: `/camera/global/depth/color/points`
    *   Size: 2mm (Flat Squares)
    *   Decay Time: 0 (实时)
3.  **MarkerArray**:
    *   Topic: `/scene_markers` (需在 `scene_analyzer` 中实现)
    *   Namespace: `box_strategy`
4.  **Image**:
    *   Topic: `/servo_debug_image` (Overlay 模式)
5.  **TF**:
    *   Show Names: True
    *   Scale: 0.5

通过这一套流程，您可以在不需要动用一兵一卒（机械臂）的情况下，把视觉系统的**精度、逻辑和鲁棒性**验证得七七八八。
