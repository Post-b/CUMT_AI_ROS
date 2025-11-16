# YDLiDAR 安装配置脚本

## 一键安装脚本

```bash
#!/bin/bash

echo "开始安装 YDLiDAR SDK 和驱动..."

# 安装 YDLiDAR SDK
echo "步骤1: 安装 YDLiDAR SDK"
cd /ydlidar_SDK
mkdir -p build
cd build
cmake ..
make
sudo make install

# 配置串口
echo "步骤2: 配置串口权限"
sudo chmod 777 /dev/ttyUSB0
sudo usermod -a -G dialout $USER

# 编译 ROS 工作空间
echo "步骤3: 编译 ROS 工作空间"
cd /catkin_ws
catkin_make

# 设置环境变量
echo "步骤4: 设置环境变量"
source ./devel/setup.sh

# 验证安装
echo "步骤5: 验证安装"
echo "ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH"

echo "安装完成!"
