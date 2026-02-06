这是一份基于之前文档更新后的 **v2.0 开发文档**。

这份文档整合了我们刚才讨论的“双层堆叠、侧拉+顶吸”混合策略，保留了原文档中关于架构规范、目录结构和调试建议的精华，并重写了接口定义和核心逻辑以适应新的复杂场景。

---

# 📘 ROS 2 功能包开发文档: `logistics_vision` (v2.0)

**适用场景**: 双层箱子堆叠（顶层侧拉、底层顶吸）
**ROS 版本**: ROS 2 Humble
**核心原则**: 无状态感知 (Stateless Perception) —— 视觉负责提供几何事实，状态机负责决策流程。

## 1. 架构概览 (Architecture)

### 1.1 硬件配置

* **Global Camera (头顶)**: 负责全局场景分析，确定箱子层级和进刀策略。
* **Local Camera (手眼)**: 负责精细伺服，根据不同策略对准侧面或顶面。

### 1.2 节点拓扑

* **`scene_analyzer_node` (Service)**:
* **输入**: Global Camera RGB-D 图像。
* **功能**: YOLO 检测 -> 深度投影 -> 分层逻辑 -> 生成策略。
* **输出**: `AnalyzeScene` Response (包含策略和进刀位姿)。


* **`visual_servo_node` (Topic)**:
* **输入**: Local Camera 图像 + 伺服模式指令 (`FACE` 或 `TOP`)。
* **功能**: 实时计算吸盘相对于目标平面的位姿误差。
* **输出**: `ServoFeedback` (给电控闭环)。


* **`tf_broadcaster`**: (基础依赖) 发布相机与机械臂基座的坐标变换。

---

## 2. 接口定义 (Interfaces) - **核心更新**

请在 `logistics_interfaces` 包中定义以下内容。

### 2.1 自定义消息 (`.msg`)

**A. 伺服反馈 (`msg/ServoFeedback.msg`)**
用于局部闭环，增加了角度误差以支持侧面吸附的垂直度校准。

```bash
std_msgs/Header header
string current_mode      # "FACE_ALIGN" (侧面) 或 "TOP_ALIGN" (顶面)

# 1. 偏差量 (用于 PID 控制)
float32 error_x          # 横向偏差 (米)
float32 error_y          # 纵向偏差 (米)

# 2. 角度偏差 (关键!)
# FACE模式: 吸盘法线与箱子侧面法线的夹角 (Pitch/Yaw)
# TOP模式: 吸盘旋转角与箱子边缘的夹角 (Yaw)
float32 error_angle_pitch 
float32 error_angle_yaw

# 3. 距离与状态
float32 distance_to_surface  # 吸盘面距离目标面的垂直距离
bool target_locked           # 是否稳定跟踪中

```

**B. 单个目标策略 (`msg/BoxStrategy.msg`)**
包含了决策层需要的所有几何信息。

```bash
string id             # 箱子ID
int32 layer_index     # 层数: 0=底层, 1=顶层
string strategy       # "SIDE_PULL" (侧拉) 或 "TOP_PICK" (顶吸)

# 进刀位姿 (Approach Pose)
# SIDE_PULL -> 前立面中心点 + 法向量朝向机器人
# TOP_PICK  -> 顶面中心点 + 法向量朝上
geometry_msgs/Pose approach_pose

# 物理尺寸 (用于避障和托盘高度计算)
geometry_msgs/Vector3 dimensions

```

### 2.2 服务定义 (`.srv`)

**C. 全局场景分析 (`srv/AnalyzeScene.srv`)**
替代了旧版的 `QueryObjects`，更强调分析结果。

```bash
# Request
bool trigger  # 是否触发分析
---
# Response
bool success
BoxStrategy[] targets  # 返回所有箱子的策略列表

```

**D. 伺服模式切换 (`srv/SetServoMode.srv`)**
用于控制局部伺服节点的行为。

```bash
# Request
string mode  # "IDLE" (待机), "FACE_ALIGN" (侧面), "TOP_ALIGN" (顶面)
---
# Response
bool success

```

---

## 3. 目录结构 (ROS 2 Python Pkg)

结构保持标准，建议将两个逻辑拆分为独立文件以便维护。

```text
logistics_vision/
├── package.xml
├── setup.py
├── config/
│   └── vision_params.yaml    # 参数：模型路径、分层高度阈值、相机Topic名
├── launch/
│   └── vision_system.launch.py
├── logistics_vision/
│   ├── __init__.py
│   ├── scene_analyzer_node.py # [更新] 全局分析节点
│   ├── visual_servo_node.py   # [更新] 局部伺服节点
│   └── utils/
│       ├── yolo_wrapper.py
│       └── math_tools.py      # 坐标变换、平面拟合算法
└── resource/

```

---

## 4. 核心逻辑详解 (Implementation Logic)

### 4.1 全局分析节点 (`scene_analyzer_node.py`)

**触发流程**:

1. **推理**: 获取图像，运行 YOLO，得到 Bounding Box。
2. **3D 投影 & TF**: 将像素坐标转为 `base_link` 下的  坐标。**务必确保高度 z 是准确的。**
3. **分层判定**:
* 读取参数 `box_height_threshold` (如 0.15m)。
* 如果 : 标记为 `layer_index = 0` (底层)。
* 如果 : 标记为 `layer_index = 1` (顶层)。


4. **策略生成**:
* **Layer 1 (侧拉)**:
* 计算前表面中心： (假设 X 轴朝前)。
* 计算进刀姿态：四元数需使吸盘法线水平指向箱子。
* 填充 `strategy = "SIDE_PULL"`, `approach_pose = P_{face}`。


* **Layer 0 (顶吸)**:
* 计算顶面中心：。
* 计算进刀姿态：吸盘法线垂直向下。
* 填充 `strategy = "TOP_PICK"`, `approach_pose = P_{top}`。





### 4.2 局部伺服节点 (`visual_servo_node.py`)

**循环逻辑 (30Hz)**:
根据 `current_mode` 执行不同算法：

* **Mode: `FACE_ALIGN` (侧面对准)**     1.  **ROI**: 截取图像中心区域。
2.  **法向量估计**: 利用深度图/点云，拟合中心区域的平面。
3.  **误差计算**:
* `error_angle`: 计算相机光轴与平面法向量的夹角。
* `distance`: 平面中心的深度值。
* `error_x, y`: 平面质心与图像中心的像素偏差。
* **Mode: `TOP_ALIGN` (顶面对准)**
1. **矩形检测**: 传统的 Canny + FindContours + MinAreaRect。
2. **误差计算**:
* `error_angle_yaw`: 矩形旋转角。
* `error_x, y`: 矩形中心偏差。





---

## 5. 开发建议与调试 (Tips)

1. **先做 TF 验证**:
在写任何识别代码前，先用 `static_transform_publisher` 发一个假的箱子坐标，看机械臂能不能准确走到那个位置。如果 TF 树不对，视觉算得再准也没用。
2. **侧面反光处理**:
侧吸模式下，箱子侧面的胶带可能导致深度相机数据丢失（出现空洞）。
* **策略**: 在 `visual_servo_node` 里加一个过滤器，如果中心点深度无效，就取周围 5x5 区域的平均值。如果依然无效，发布 `target_locked = False`，让状态机停止前进并报错。


3. **调试工具**:
* 不要用 `cv2.imshow`。
* 创建 `debug/global_image` 和 `debug/local_image` 话题，在 Rviz 里看画了框和法向量箭头的图。
* 使用 `ros2 topic echo /vision/servo_feedback` 实时查看误差数值跳动情况。


4. **第一版简化**:
* 如果双相机数据流太大，可以先只跑 Global Node，Local Node 仅返回 0 误差，验证状态机流程通畅。
* 侧拉时，如果法向量计算太复杂，可以先假设箱子是正对着放的，只通过视觉修正 X/Y 位移，忽略角度修正。