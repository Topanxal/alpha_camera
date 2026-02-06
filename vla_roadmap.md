# Alfa Robot VLA 演进路线图 (Vision-Language-Action Roadmap)

本文档旨在规划 Alfa Robot 从当前的「视觉检测+规则控制」架构向「端到端 VLA 大模型」架构演进的技术路径。基于现有的 **RTX 2000 Ada (16GB)** 算力平台，我们采用 **"Edge-First" (边缘优先)** 策略，确保工业场景下的实时性与稳定性。

---

## 架构总览：云边协同 (Cloud-Edge Hybrid)

我们将构建一个 "快思考 + 慢思考" 的双层脑架构：

*   **☁️ 云端/高层 (慢思考)**: 负责复杂语义理解与长序列规划。
    *   *模型*: GPT-4V, Gemini 1.5 Pro, LLaVA-Next.
    *   *任务*: "把所有红色的易碎品搬到右边的托盘上"。
    *   *频率*: 低频 (事件触发)。
*   **🤖 边缘/底层 (快思考)**: 负责实时感知与动作生成。
    *   *模型*: **OpenVLA (7B-Quant)**, **Octo-Small**, 或微调后的 **RT-1-X**.
    *   *任务*: 输出机械臂末端位姿 (dx, dy, dz, d_yaw, gripper) @ 10-50Hz.
    *   *硬件*: **本机 RTX 2000 Ada**.

---

## Phase 1: 3D 数据底座 (Data Foundation)
**目标**: 让机器人从 "看图片" 进化到 "理解空间"。VLA 模型需要高质量的 3D 表征作为输入或训练数据。

### 1.1 局部实时建图 (Local Real-time Mapping)
*   **技术栈**: ROS 2 + Open3D (CUDA) / TSDF Integration.
*   **功能**:
    *   订阅 Realsense RGB-D 数据流。
    *   利用机械臂的移动，融合多视角深度图。
    *   生成去噪、补全后的 **TSDF Voxel Grid** 或 **高精点云**。
*   **产出**: 解决单帧深度图噪点大、盲区多的问题，为 VLA 提供稳定的空间输入。

### 1.2 3D 语义分割 (3D Semantic Segmentation)
*   **技术栈**: PointGroup / Mask3D (可部署于 RTX 2000).
*   **功能**:
    *   在点云层面对箱子进行实例分割。
    *   解决 "紧密排列箱子无法区分" 的难题。
*   **产出**: 带有 ID 和语义标签的 Object-centric Map。

---

## Phase 2: 本地 VLA 部署 (Onboard VLA Inference)
**目标**: 部署一个能在工控机上实时跑的 VLA 模型，替代目前的 "Visual Servo" 逻辑。

### 2.1 模型选型与压缩
鉴于 RTX 2000 Ada 的 16GB 显存，直接跑 FP16 的 7B 模型略显吃力（显存占用约 14GB，留给系统和图形界面的空间不足）。
*   **推荐模型**: **OpenVLA-7B (4-bit Quantization)**.
    *   *显存占用*: 约 5-6GB。
    *   *推理速度*: Orin/RTX 2000 上可达 5-10Hz。
*   **备选模型**: **Octo-Small (Transformer-based Policy)**.
    *   *特点*: 更轻量，专为机械臂控制设计。

### 2.2 微调 (LoRA Fine-tuning)
通用 VLA 模型泛化性好但精度低，必须针对拆垛场景微调。
*   **数据采集**: 使用 Phase 1 的建图模块，人工遥控机器人拆垛，录制 `(RGB-D, Proprioception, Language_Instruction, Action)` 数据对。
*   **训练策略**: 使用 **LoRA (Low-Rank Adaptation)** 技术，只训练 1%-2% 的参数，单卡 RTX 4090 (开发机) 即可完成训练，然后部署到工控机。

---

## Phase 3: 语义级任务规划 (Semantic Planning)
**目标**: 让机器人听懂人话，处理异常。

### 3.1 VLM 视觉问答 (Visual Question Answering)
*   **场景**: 当 VLA 连续 3 次抓取失败，或遇到未知物体。
*   **流程**:
    1.  机器人拍照上传云端/本地 VLM (如 LLaVA)。
    2.  Prompt: "当前画面中有什么异常？箱子为什么抓不起来？"
    3.  VLM 回复: "箱子表面有透明胶带反光，导致深度缺失。" 或 "箱子被压住了。"

### 3.2 任务链生成
*   **输入**: "拆除所有易爆品"。
*   **VLM 输出**:
    1.  `detect_objects(category='explosive')`
    2.  `plan_grasp_sequence(priority='safety')`
    3.  `execute_action(target_id=...)`

---

## 实施路线建议 (Action Plan)

1.  **Q1**: 完成 **Phase 1.1**。创建一个 `alfa_robot_mapping` 包，跑通 Open3D TSDF 建图。这是性价比最高的一步。
2.  **Q2**: 尝试 **Phase 2.1**。在 RTX 2000 上部署量化版的 OpenVLA，测试其简单的 Pick-and-Place 能力。
3.  **Q3**: 数据采集与微调。构建自己的拆垛数据集，让 VLA 真正可用。

## 硬件升级预留
虽然 RTX 2000 Ada 很强，但如果未来要跑全参数微调或更大模型，建议预留升级接口：
*   **外接显卡坞 (eGPU)**: 通过雷电接口扩展一张 RTX 4090 (仅用于离线训练或重度推理)。
*   **双机互联**: 一台工控机跑 ROS 控制，另一台 Jetson AGX Orin 专门跑 VLA，通过以太网通信。
