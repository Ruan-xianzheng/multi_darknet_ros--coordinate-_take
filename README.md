# Qingzhou-SMMR-Explore

## Platform
- ubuntu 18.04 (bash)
- ROS melodic
- CUDA-10.2
- cuDNN-7.6.5
- 显卡是2080 对应的算力是75 （需要根据自己电脑修改，修改的地方见后续）
- OpenCV版本是3.3.1 (也可以根据自己的版本修改，修改见后续)

## Dependency

### Cartographer

请到网址 [Cartographer-for-SMMR](https://github.com/efc-robot/Cartographer-for-SMMR) 处安装修改后的 Cartographer 

and

```bash
source /PATH/TO/CARTO_CATKIN_WS/devel_isolated/setup.bash
```

### Pytorch for PointNetVLAD

```bash
pip install torch torchvision
```

### qingzhou_robot（）

```bash
sudo apt-get install ros-melodic-joint-state-publisher-gui 
sudo apt install ros-melodic-controller-manager 
sudo apt install ros-melodic-gazebo-ros-control 
sudo apt install ros-melodic-joint-state-controller 
sudo apt install ros-melodic-velocity-controllers 
sudo apt install ros-melodic-position-controllers 
sudo apt install ros-melodic-gmapping 
sudo apt install ros-melodic-map-server 
sudo apt install ros-melodic-amcl 
sudo apt install ros-melodic-move-base 
sudo apt install ros-melodic-nav-core 
sudo apt install ros-melodic-costmap-* 
sudo apt install ros-melodic-teb-local-planner 
sudo apt install ros-melodic-global-planner 
```

### QT

请到 [ROS Qt Creator Plug-in documentation](https://ros-qtc-plugin.readthedocs.io/en/latest/_source/How-to-Install-Users.html) 下载支持 ros 的 qt 版本

### darknet_ros代码需要根据自己电脑修改的地方

#### (1)darknet功能包下的Makefile文件

```html
(1)ARCH= -gencode arch=compute_计算机算力,code=[sm_计算机算力,compute_计算机算力]  #这句代码中的计算机算力需要根据自己电脑显卡写，可参考https://developer.nvidia.com/cuda-gpus#compute
(2)NVCC=/usr/local/cuda-版本号/bin/nvcc  #代码中的版本号需要根据自己的CUDA版本号修改。如果CUDA的路径不一样的话也要根据自己的修改
(3)COMMON+= -DGPU -I/usr/local/cuda-版本号/include/ #代码中的版本号需要根据自己的CUDA版本号修改 路径不同的话也要修改
(4)LDFLAGS+= -L/usr/local/cuda-11.4/lib64 -lcuda -lcudart -lcublas -lcurand  #代码中的版本号需要根据自己的CUDA版本号修改 路径不同的话也要修改
```
#### (2)darknet_ros功能包中的CMakeLists.txt文件

```cpp
find_package(OpenCV 版本号 REQUIRED) #OpenCV的版本修改，需要根据自己安装的版本修改
```

## Run SMMR-Explore （）Todo

编译工作空间

```bash
catkin_make
```

运行 launch 文件

```bash
source devel/setup.bash
roslaunch turtlebot3sim small_env_three_robots.launch 
roslaunch turtlebot3sim three_robots.launch
roslaunch mmpf three_mmpf_node.launch 
```
## QT（实现多台机器人图像信息、RVIZ信息、坐标信息等数据的显示）
Todo:
## Run darknet_ros（实现多台机器人的目标识别以及目标信息的发送）

编译工作空间

```bash
cd /darknet_ros工作空间/src/darknet_ros/darknet
make
cd darknet_ros工作空间  #进入darknet_ros的工作空间
catkin_make -j1  #编译
```

如果编译出错，显示depth_take/Object.h: 没有那个文件或目录，解决方法是单独编译depth_take功能包,在include/depth_take中生成Object.h

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="depth_take"
```
然后，再编译整体
```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```

运行 launch 文件

```bash
source devel/setup.bash
roslaunch darknet_ros three_yolo.launch #开启三个yolo画面
```

## Run depth_take（实现物体坐标信息的提取、转换与发送）

运行 cpp 文件

```bash
source devel/setup.bash #depth_take功能包和darknet_ros功能包放在同一个工作空间中，因此不需要再catkin_make
rosrun depth_take tf_trans #开启robot1所对应的坐标信息处理
rosrun depth_take tf_trans2 #开启robot2所对应的坐标信息处理
rosrun depth_take tf_trans3 #开启robot3所对应的坐标信息处理
```
