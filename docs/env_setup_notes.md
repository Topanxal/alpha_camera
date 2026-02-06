# 1. 临时将系统路径移到最前，屏蔽 conda
export PATH=/usr/bin:$PATH

# 2. 重新 source ROS 环境
source /opt/ros/humble/setup.bash

# 3. 启动 rqt_image_view
ros2 run rqt_image_view rqt_image_view

ros2 service call /set_servo_mode logistics_interfaces/srv/SetServoMode "{mode: 'TOP_ALIGN'}"