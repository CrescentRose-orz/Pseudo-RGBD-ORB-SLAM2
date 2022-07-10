# 简介
## Tello_ROS_ORBSLAM
python2 环境的ROS catkin　工作空间，包含orbslam，octomap寻路
## py3Ros_ws
python3 环境的ROS catkin　工作空间，包含伪RGBD摄像头
## sripts 
用于快速启动系统的命令，使用前需要修改内部的绝对地址

# 环境配置与编译工具 
最好用翻墙代理，并设置命令行代理
.bashrc 中
```
export http_proxy="http://yourProxyAdress:port"
export https_proxy="http://yourProxyAdress:port"
```
## 安装 ROS melodic
官方安装教程链接：
http://wiki.ros.org/melodic/Installation/Ubuntu
### 准备从packages.ros.org.安装ROS
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### 配置apt-key
```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

### 安装ROS
更新软件源运行：
```
sudo apt update
```

### 安装完整的ROS-melodic及常用库，也可以自行安装无附带库版本，但需要不全本课题使用的rviz库。
完整安装运行：
```
sudo aptitude install ros-melodic-desktop ros-melodic-perception ros-melodic-simulators ros-melodic-urdf-sim-tutorial
sudo apt install ros-melodic-desktop-full
```

### 初始化rosdep 以检查相关依赖，便于后续执行ＲＯＳ核心的安装。
运行：
```
sudo rosdep init
rosdep update
```


## 自动配置环境变量
运行：
```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

###　安装常用的命令行工具
安装rosinstall
运行：
``` 
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

## 克隆本仓库
本部分基于源仓库搭建：https://github.com/tau-adl/Tello_ROS_ORBSLAM.git
```
cd ~
mkdir ROS
cd ROS
git clone https://github.com/tau-adl/Tello_ROS_ORBSLAM.git
```
# 安装所需依赖
## Easy Install Prerequisites
### catking tools
首先下载ROS库，其中包含我们所需要的catkin_tools
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
```
安装catkin_tools，该工具用于编译和管理ＲＯＳ的核心以及各节点包
```
sudo apt-get update
sudo apt-get install python-catkin-tools
```
### Eigen3
Eigen3为常用的数学矩阵库，在g2o,ORB-SLAM的各个模块中均有使用。3.3.9版本测试可用。
也可以直接下载二进制包(未测试，不推荐)<br>
~~sudo apt install libeigen3-dev~~

### ffmpeg
```
sudo apt install ffmpeg
```
### Python catkin tools (如果根据本教程安装，应当在ROS安装时已经安装过了)
```
sudo apt-get install python-catkin-tools
```
### 手柄驱动
手柄驱动用于手动控制无人机飞行。该驱动在ROS melodic下，xbox 360手柄测试可用:
```
sudo apt install ros-melodic-joystick-drivers
```
### Python PIL
```
sudo apt-get install python-imaging-tk
```
### ompl
```
sudo apt-get install ros-`rosversion -d`-ompl
```

### octomap 及可视化工具
```
sudo apt-get install ros-melodic-octomap ros-melodic-octomap-mapping ros-melodic-octomap-msgs ros-melodic-octomap-ros ros-melodic-octomap-rviz-plugins ros-melodic-octomap-server
```
## 需要源码编译的部分
### Pangolin (orbslam2中使用)
源仓库： https://github.com/stevenlovegrove/Pangolin
请注意需要**使用0.5 tag版本**
```
cd ~/ROS/Pseudo-RGBD-ORB-SLAM2/

```
~~git clone  https://github.com/stevenlovegrove/Pangolin.git~~
本仓库提供了可用版本存档(未修改)
```
cd ~/ROS/Pseudo-RGBD-ORB-SLAM2/Pangolin-0.5
sudo apt install libgl1-mesa-dev
sudo apt install libglew-dev
sudo apt-get install libxkbcommon-dev
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build
```

### h264decoder
源仓库： https://github.com/DaWelter/h264decoder
请注意需要**使用v1 tag版本**
```
cd ~/ROS/Pseudo-RGBD-ORB-SLAM2/
```
~~git clone https://github.com/DaWelter/h264decoder.git~~ <br>
本仓库提供了可用版本存档(已将
h264decoder.cpp 中将 PIX_FMT_RGB24 全部替换为 AV_PIX_FMT_RGB24）
```
cd h264decoder-1
mkdir build
cd build
cmake ..
make
```
将编译完成的链接库拷贝至python路径(ROS melodic使用python2.7)。建议使用anaconda　环境。不使用anaconda:
```
sudo cp cd ~/ROS/Pseudo-RGBD-ORB-SLAM2/h264decoder/libh264decoder.so /你的/python路径/python2.7/dist-packages
```
使用则dist-packages将换成site-packages，例如：
```
sudo cp cd ~/ROS/Pseudo-RGBD-ORB-SLAM2/h264decoder/libh264decoder.so /你的/anaconda路径/envs/lib/python2.7/site-packages
```
### libccd碰撞库
```
cd ~/ROS/Pseudo-RGBD-ORB-SLAM2/libccd
mkdir build
cd build
cmake  ..
make
sudo make install
```

### fcl碰撞库
经测试0.5.0 版本可用
```
cd ~/ROS/Pseudo-RGBD-ORB-SLAM2/fcl-0.5
mkdir build
cd build
cmake  ..
make
sudo make install
```

### VTK
经测试7.1.1 版本可用
```
cd ~/ROS/Pseudo-RGBD-ORB-SLAM2/VTK-7.1.1
mkdir build
cd build
cmake  ..
make
sudo make install
```

# 安装本仓库的orbslam2节点及路径规划节点

## 安装TelloPy 
TelloPy由tau-adl于源仓库修改： https://github.com/dji-sdk/Tello-Python and https://github.com/hanyazou/TelloPy
```
cd ~/ROS/Pseudo-RGBD-ORB-SLAM2/Tello_ROS_ORBSLAM/TelloPy
sudo python setup.py install
```
## 安装ROS相关依赖
```
cd ~/ROS/Pseudo-RGBD-ORB-SLAM2/Tello_ROS_ORBSLAM/ROS/tello_catkin_ws/
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

# 编译 orbslam2节点及路径规划器节点
基于https://github.com/appliedAI-Initiative/orb_slam_2_ros 和https://github.com/rayvburn/ORB-SLAM2_ROS
## 编译代码:
```
cd ~/ROS/Pseudo-RGBD-ORB-SLAM2/Tello_ROS_ORBSLAM/ROS/tello_catkin_ws/
catkin init
catkin clean
catkin build
```
## 建立环境变量到bashrc
```
echo "source $PWD/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
## orbslam Dbow词典库
从orbslam2源仓库的readme链接中下载词典文件，并置于Vocabulary文件夹下
https://github.com/raulmur/ORB_SLAM2
# 安装本仓库的pseudo-camera
基于https://github.com/JiawangBian/SC-SfMLearner-Release
##　创建python3下的ROS环境
推荐创建虚拟环境
### 安装相关依赖
```
sudo apt-get install python-catkin-tools python3-dev python3-catkin-pkg-modules python3-numpy python3-yaml ros-melodic-cv-bridge
```
### 初始化python3的ROS环境
```
cd ~/ROS/Pseudo-RGBD-ORB-SLAM2/
mkdir python3_ws/src
cd ~/ROS/Pseudo-RGBD-ORB-SLAM2/python3_ws/src
catkin_init_workspace 
cd ..
catkin_make
catkin_make install 
```
下载cv_bridge到src
```
cd src
git clone -b melodic https://github.com/ros-perception/vision_opencv.git
```
编译并配置
```
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/你的/ｐython3执行程序路径/python3 -DPYTHON_INCLUDE_DIR=/你的/ｐython3include路径/python3.6m -DPYTHON_LIBRARY=/你的/ｐython3动态库路径/libpython3版本.so
//例如：catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/home/xiaolin/anaconda3/bin/python -DPYTHON_INCLUDE_DIR=/home/xiaolin/anaconda3/include/python3.8 -DPYTHON_LIBRARY=/home/xiaolin/anaconda3/lib/libpython3.8.so
source ~/ROS/python3_ws/devel/setup.bash
source ~/python3_ws/devel/setup.bash
```
### 安装sc-sfmlearner的相关依赖
```
cd ~/ROS/Pseudo-RGBD-ORB-SLAM2/pseudo-camera
sudo pip3 install requirements.txt
```
将pseudo-camera拷贝至新建空间内
cd ~/ROS/Pseudo-RGBD-ORB-SLAM2/
sudo cppseudo-camera cd ~/ROS/Pseudo-RGBD-ORB-SLAM2/py3Ros_ws/src/pseudo-camera -r
### 预训练模型
源仓库中网盘下载到ckpts文件夹中，修改文件名为rgbd.ckpt，或者修改启动命令中的模型名均可。
https://github.com/JiawangBian/sc_depth_pl
## Notes
我们主要使用了以下的开源库(递归关系不一一展开)
### sc-sfmlearner
几何一致的深度估计https://github.com/JiawangBian/SC-SfMLearner-Release　
https://github.com/JiawangBian/sc_depth_pl
### Tello_ROS_ORBSLAM
基于tello无人机的ORBSLAM2的快速开发框架及GUIhttps://github.com/tau-adl/Tello_ROS_ORBSLAM
### Flock
tello无人机的驱动节点https://github.com/clydemcqueen/flock
