# Vision System Troubleshooting Guide

本文档记录了 `alfa_robot_vision` 模块在开发和部署过程中可能遇到的典型问题及其解决方案。

## 1. 致命崩溃：`AttributeError: _ARRAY_API not found`

### 现象
节点启动失败，报错信息如下：
```text
File "/opt/ros/humble/local/lib/python3.10/dist-packages/cv_bridge/core.py", line 194, in imgmsg_to_cv2
    res = cvtColor2(im, img_msg.encoding, desired_encoding)
AttributeError: _ARRAY_API not found
```
或者出现关于 NumPy 版本的警告：
```text
A module that was compiled using NumPy 1.x cannot be run in NumPy 2.2.6 as it may crash.
```

### 原因
ROS 2 Humble 的 `cv_bridge` 是基于 **NumPy 1.x** 编译的 C++ 扩展。
当你安装 `ultralytics` 或其他现代深度学习库时，`pip` 可能会自动将 NumPy 升级到 2.x 最新版（如 2.2.6）。这种版本不匹配会导致二进制兼容性问题，引发崩溃。

### 解决方案
强制降级 NumPy 到 1.x 版本（推荐 1.26.4）：

```bash
pip3 install "numpy<2.0"
```

---

## 2. 静默失败：运行正常但无任何检测结果

### 现象
*   节点正常启动，无报错。
*   话题 `/debug_image` 正常发布，但画面上始终显示 "No detections" 或空列表。
*   即使将置信度降到 0.1 也无济于事。

### 原因
`ultralytics` 库未安装。
在 `utils/yolo_wrapper.py` 中，为了防止节点崩溃，模型加载部分使用了 `try-except` 块。如果缺少 `ultralytics`，它会捕获 `ImportError` 并打印一条 Warning，然后将模型设为 `None`。后续的 `detect` 方法会直接返回空列表。
由于 ROS 2 的日志刷新很快，这条 Warning 很容易被忽略。

### 解决方案
安装 YOLO 库：
```bash
pip3 install ultralytics
```

---

## 3. 数据流中断：Vision Server 收不到图像

### 现象
*   Realsense 相机节点正在运行，`ros2 topic list` 能看到 `/camera/...` 话题。
*   Vision Server 正在运行，但日志中从未出现 "Starting YOLO inference"。
*   `rqt_graph` 显示两个节点之间没有连线。

### 原因
话题名称不匹配。
*   Realsense 默认发布在 `/camera/camera/color/image_raw`。
*   Vision Server 默认订阅 `/head_camera/rgb/image_raw`。

### 解决方案
在 Launch 文件 (`launch/vision_bringup.launch.py`) 中使用 `remappings` 将两者对齐：

```python
Node(
    package='alfa_robot_vision',
    executable='vision_server',
    remappings=[
        ('/head_camera/rgb/image_raw', '/camera/camera/color/image_raw'),
        ('/head_camera/depth/image_rect_raw', '/camera/camera/aligned_depth_to_color/image_raw'),
        ('/head_camera/camera_info', '/camera/camera/color/camera_info')
    ]
)
```

---

## 4. 识别率极低：有检测但框不准

### 现象
*   能够检测到物体，但置信度（Confidence）很低（如 0.4 左右）。
*   即使是很明显的物体也经常漏检。

### 原因
颜色空间不匹配。
*   OpenCV 默认读取图像为 **BGR** 格式。
*   虽然 YOLO/PyTorch 通常训练时使用 RGB，但 `ultralytics` 库在推理时通常能很好地处理 BGR 输入（或者其内部预处理已经考虑了这一点）。
*   **错误的做法**：如果在将 ROS 图像（BGR8）转为 OpenCV 图像后，手动执行 `cv2.cvtColor(img, cv2.COLOR_BGR2RGB)` 再传给模型，反而可能导致识别率下降。

### 解决方案
直接将 `cv_bridge` 转换得到的 BGR 图像传给 YOLO 模型，**不要**手动转换颜色空间。

```python
# 正确做法
cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
detections = self.yolo.detect(cv_image)
```

---

## 5. 参数不生效：Debug 模式无法开启

### 现象
*   `vision_params.yaml` 中设置了 `debug_mode: true`。
*   但 `rqt_image_view` 中 `/debug_image` 话题无数据或显示灰色。

### 原因
ROS 2 参数加载机制问题。
如果 Launch 文件中加载参数文件的方式不正确（例如命名空间嵌套错误），或者节点代码中获取参数的逻辑有误（例如未先 `declare_parameter`），参数将保持默认值（通常是 `False`）。

### 解决方案
1.  确保 Launch 文件正确加载 YAML：
    ```python
    params_file = os.path.join(..., 'config', 'vision_params.yaml')
    Node(..., parameters=[params_file])
    ```
2.  确保 YAML 文件格式正确（注意缩进）：
    ```yaml
    /vision_server:
      ros__parameters:
        debug_mode: true
    ```
