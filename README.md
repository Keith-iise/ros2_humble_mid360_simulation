# Mid360雷达仿真搭建指南


## 1. 安装依赖
（注：依赖可能随环境差异有所不同，建议边运行边补充安装）

### 1.1 安装 Livox SDK2
自行克隆源码并编译安装：
```bash
# 克隆源码（需提前安装git）
git clone https://github.com/Livox-SDK/Livox-SDK2.git  
cd Livox-SDK2

# 编译安装
mkdir build && cd build
cmake .. && make -j$(nproc) 
sudo make install  
```

### 1.2 安装ROS 2及Gazebo相关依赖
```bash
sudo apt install \
  ros-humble-gazebo* \          
  ros-humble-ros2-control* \    
  ros-humble-robot-state-publisher \ 
  ros-humble-joint-state-publisher \ 
  ros-humble-xacro  
```


## 2. 环境配置

### 2.1 （可选）下载Gazebo官方模型库
```bash
#############################################
# 若Gazebo缺少基础模型，可手动克隆官方模型库
#############################################
cd ~/.gazebo/
git clone https://github.com/osrf/gazebo_models.git models


sudo chmod 777 ~/.gazebo/models
sudo chmod 777 ~/.gazebo/models/*
```

### 2.2 配置环境变量
通过修改`.bashrc`文件设置Gazebo和ROS 2相关路径：
```bash

gedit ~/.bashrc
```

在文件末尾添加以下内容（按实际Gazebo版本调整路径中的`11`）：
```bash
# Gazebo资源路径配置
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:$GAZEBO_RESOURCE_PATH
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/usr/share/gazebo/models:/usr/share/gazebo-11/models
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:/usr/share/gazebo:/usr/share/gazebo-11

# 库文件路径配置（ROS 2及系统库）
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib

# （可选）conda环境配置
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH  
# （可选）阻止Gazebo从互联网自动下载模型（加速本地加载）
export GAZEBO_MODEL_DATABASE_URI=""
```

添加完成后，生效配置：
```bash
source ~/.bashrc
```


## 3. 编译项目
```bash
./build.sh  
```


## 4. 运行仿真
```bash
./run.sh  
```
