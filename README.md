# UR3-ROS-Control

### Overview

Universal Robot (UR3) moving to detected object with ROS using a realsense camera.

It is for UR3, UR5 and UR10 manipulator(I'm using only UR3 real-hardwrae) with Moveit!. It detects object using darknet-ros and jsk-pcl.

- [darknet_ros (YOLO)](https://github.com/leggedrobotics/darknet_ros) for real-time detection object by making bounding box
- [jsk_pcl](https://github.com/jsk-ros-pkg/jsk_recognition) estimation coordinate detected object by darknet_ros(YOLO)
- [universal_robot](http://wiki.ros.org/action/show/universal_robots?action=show&redirect=universal_robot)
- [ur_modern_driver](http://wiki.ros.org/ur_modern_driver)

They are tested under JetsonTX2, ROS1 melodic and Ubuntu 18.04, OpenCV 3.4.6, CUDA Version 10.2.

### Installation

If you want to use this package, please follow this procedure

#### Dependencies

This package works on the ROS(Robot Operating System).

Please check your linux version, before downloading ROS 

- [Ubuntu 18.04 (melodic install)](http://wiki.ros.org/melodic/Installation/Ubuntu)

YOLO depends on the OpenCV (at least 3.4.x)

- [OpenCV 3.4.6 (on Jetson)](https://jkjung-avt.github.io/opencv-on-nano/)

  ROS1 default python package is `python2.7`. So I change a shell script a little to install OpenCV 3.4.6 on python2. [Here](https://github.com/mywnajsldkf/object-detection/blob/master/doc/install_opencv-3.4.6.sh) is shell script that I changed.

- If you don't have any package. please follow this procedure.

  ```
  $ mkdir -p ~/catkin_ws/src
  $ cd ..
  $ catkin_make	# build, devel, src directory will be made
  $ source ~/catkin_ws/devel/setup.bash	# register workspace
  ```

- darknet_ros

  ```
  $ cd ~/catkin_ws/src
  $ git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
  $ cd ..
  $ catkin_make -DCMAKE_BUILD_TYPE=Release
  $ rospack profile
  ```

- [IntelRealSense](https://github.com/IntelRealSense)

  - Download [Intel Realsense SDK](https://github.com/IntelRealSense/librealsense/releases)

  - [realsense-ros](https://github.com/IntelRealSense/librealsense/releases)

  - Please check the realsense-ros version because it depends on the Intel RealSense SDK

    ( I installed [RealSenseSDK(v2.31.0)](https://github.com/IntelRealSense/librealsense/releases/tag/v2.31.0) matches [realsense-ros(2.2.11)](https://github.com/IntelRealSense/realsense-ros/tree/2.2.11) )

- [jsk_recongnition](https://github.com/jsk-ros-pkg/jsk_recognition)

  ```
  sudo apt-get install ros-melodic-jsk-recognition
  sudo apt-get install ros-melodic-jsk-topic-tools
  
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/jsk-ros-pkg/jsk_common.git
  ```

- packages

  ```
  $ sudo apt-get install ros-melodic-octomap-server
  $ sudo apt-get install ros-melodic-nodelet
  $ sudo apt-get install ros-melodic-depth-image-proc
  $ sudo apt-get install ros-melodic-rtabmap-ros
  $ sudo apt-get install ros-melodic-navigation
  ```

- [Universal Robot](https://github.com/ros-industrial/universal_robot), [ur_modern_driver](https://github.com/ros-industrial/ur_modern_driver)

  ```
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/ros-industrial/universal_robot
  $ git clone https://github.com/ThomasTimm/ur_modern_driver
  $ cd ..
  $ catkin_make
  ```

  You might be see a error during build your project. Refer this [document](https://github.com/ros-industrial/ur_modern_driver/issues/58).

- [Moveit!](https://moveit.ros.org/)

  ```
  $ sudo apt-get install ros-melodic-moveit
  ```

  

---

Todo

- UR3 연결해서 간단한 test해보기
- 실행하기
  - Transform 하는 코드 다루기
  - 간단하게 moveit! 실행하는 것 다루기
  - 목표치까지 가는 코드 작성하기