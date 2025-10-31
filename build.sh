
#!/bin/bash

# 切换到工作空间根目录（假设当前脚本在 workspace_root/script/ 中）
cd "$(dirname "$(dirname "$0")")"

# 编译 文件
colcon build --symlink-install --cmake-args   -DROS_EDITION=ROS2   -DHUMBLE_ROS=humble  

