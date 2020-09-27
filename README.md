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


### Start

1. UR manipulator with moveit test (I'm using UR3 model)

   - Bringup Gazebo

     ```
     $ roslaunch ur_gazebo ur3.launch
     $ roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch sim:=true
     $ roslaunch ur3_moveit_config moveit_rviz.launch config:=true
     ```

   - Bringup Real UR manipulator

     ```
     $ roslaunch ur_modern_driver ur3_bringup.launch robot_ip:=ROBOT_IP
     $ roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch
     ```

     For more details, visit [here](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial).

   - Contolling the UR manipulator

     ```
     $ rosrun ur_modern_driver test_move.py
     ```

2. Copy and paste

   ```
   $ cd UR3-ROS-Control/darknet_ros
   
   in config directory
   $ cp -r ros.yaml ~/catkin_ws/src/darknet_ros/darknet_ros/config
   $ cp -r tomato.yaml ~/catkin_ws/src/darknet_ros/darknet_ros/config
   $ cp -r tomato_yolo_jsk.rviz ~/catkin_ws/src/darknet_ros/darknet_ros/config
   
   in launch directory
   $ cp -r bringup_d435.launch ~/catkin_ws/src/darknet_ros/darknet_ros/launch
   $ cp -r darknet_ros.launch ~/catkin_ws/src/darknet_ros/darknet_ros/launch
   $ cp -r object_detect_rviz.launch ~/catkin_ws/src/darknet_ros/darknet_ros/launch
   $ cp -r object_detection.launch ~/catkin_ws/src/darknet_ros/darknet_ros/launch
   $ cp -r object_jsk_test.launch ~/catkin_ws/src/darknet_ros/darknet_ros/launch
   
   in darknet_ros directory
   $ mv src ~/catkin_ws/src/darknet_ros/darknet_ros/src
   $ mv include ~/catkin_ws/src/darknet_ros/darknet_ros/include
   
   in yolo_network_config/cfg
   $ cp -r tomato.cfg ~/catkin_ws/src/darknet_ros/darknet_ros/cfg
   
   in universal_robot directory
   $ mv ur_description ~/catkin_ws/src/universal_robot/ur_description
   $ mv ur_description ~/catkin_ws/src/universal_robot/ur3_moveit_config
   ```

   Weight file Download [link](https://drive.google.com/file/d/1f615qxgQMaswqy6ZJyNqtfsV400mJbBY/view)

   ```
   $ cp -r tomato_3000.weights ~/catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights
   ```

3. Launch

   - Bringup Gazebo

     ```
     $ roscore
     $ roslaunch ur_gazebo ur3.launch
     $ roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch sim:=true
     ```

   - (Or) Bringup RealRobot

     ```
     $ roscore
     $ roslaunch ur_modern_driver ur3_bringup.launch robot_ip:=ROBOT_IP
     $ roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch
     ```

   - start Rviz

     ```
     $ roslaunch darknet_ros object_detect_rviz.launch
     $ roslaunch darknet_ros object_jsk_test.launch
     $ roslaunch darknet_ros ur3_yolo_move.py 		// Control UR3 with YOLO  
     ```

     

   

