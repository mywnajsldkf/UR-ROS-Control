# UR3-ROS-Control

### Overview

Universal Robot (UR3) moving to detected object with ROS using a realsense camera.

It is for UR3, UR5 and UR10 manipulator(I'm using only UR3 real-hardwrae) with Moveit!. It detects object using darknet-ros and jsk-pcl.

- [darknet_ros_3d](https://github.com/IntelligentRoboticsLabs/gb_visual_detection_3d) for real-time detection object by making 3D bounding box
- [universal_robot](http://wiki.ros.org/action/show/universal_robots?action=show&redirect=universal_robot)
- [ur_modern_driver](http://wiki.ros.org/ur_modern_driver)

They are tested under ROS1 melodic and Ubuntu 18.04, OpenCV 3.4.0, CUDA Version 10.2.

### Installation

If you want to use this package, please follow this procedure

#### Dependencies

This package works on the ROS(Robot Operating System).

1. Please check your linux version, before downloading ROS 

   [Ubuntu 18.04 (melodic install)](http://wiki.ros.org/melodic/Installation/Ubuntu)

2. YOLO depends on the OpenCV (at least 3.4.x)

   ROS1 default python package is `python2.7`. so please keep in mind installing OpenCV 3.4.0 on python2. 

3. Make your own package. Please follow this procedure

   ```
   $ mkdir -p ~/catkin_ws/src
   $ cd ..
   $ catkin_make	# build, devel, src directory will be made
   $ source ~/catkin_ws/devel/setup.bash	# register workspace
   ```

4. darknet_ros_3d

   I used YOLOV4 model to detect the object 

   ```
   $ cd ~/catkin_ws/src
   $ git clone https://github.com/tom13133/darknet_ros			
   $ cd darknet_ros
   $ rm -rf darknet
   $ git clone https://github.com/AlexeyAB/darknet 		// clone darknet
   $ cd ..
   $ catkin_make
   ```

5. [IntelRealSense](https://github.com/IntelRealSense)

   - Download [IntelRealsense SDK](https://github.com/IntelRealSense/librealsense/releases) 

     ```
     // install some packages to build the IntelRealsense SDK
     $ sudo apt install libgtk-3-dev libxcursor-dev libxinerama-dev
     $ tar zxf librealsense-2.41.0			// i installed RealSenseSDK(v2.41.0)
     $ cd librealsense2.41.0
     $ mkdir build
     $ cd build
     $ cmake ..
     
     // start SDK build
     $ make -j$(nproc)
     // After finish build completely, start install
     $ sudo make install
     ```

   - [Realsense-ros](https://github.com/IntelRealSense/realsense-ros)

     ```
     $ cd ~/catkin_ws/src
     $ git clone https://github.com/IntelRealSense/realsense-ros.git
     $ cd ..
     $ catkin_make
     ```

   - Please check the realsense-ros version because it depends on the Intel RealSense SDK

     (I installed RealSenseSDK(v2.41.0) matches realsense-ros(2.2.1))

     For more details, please refer https://github.com/IntelRealSense/realsense-ros/releases

6. Universal Robit, ur_modern_driver

   ```
   $ cd ~/catkin_ws/src
   $ git clone https://github.com/ros-industrial/universal_robot
   $ git clone https://github.com/ThomasTimm/ur_modern_driver
   $ cd ..
   $ catkin_make
   ```

   You might be see a error during building your project. Refer this [document](https://github.com/ros-industrial/ur_modern_driver/issues/58).

7. [Moveit!](https://moveit.ros.org/)

   ```
   $ sudo apt-get install ros-melodic-moveit
   ```



---

**Todo**

- UR3 로봇을 가지고 실험해보는 것 정리하기 -> 새롭게 하나 패키지 파서 거기에 차근차근 정리하는 것도 괜찮을 듯
- object detection을 진행하는것
- 사용한 yolov3, v4 모델에 대해 내용 -> 내 나름대로 이렇게 사용하면 좋겠다 등등을 정리하여 올린다.



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

     

   

