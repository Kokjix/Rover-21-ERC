# Rover-21-Panel-Detection
Panel Detection for ERC Competition for darknet configuration please look https://github.com/leggedrobotics/darknet_ros.git you need to check the compute capability (version) of your GPU. You can find it from here: [CUDA - WIKIPEDIA](https://en.wikipedia.org/wiki/CUDA#Supported_GPUs).

To Run this package you need
1. Realsense ROS package

After install realsense package run these following.
- cd catkin_ws
- catkin_make -DCMAKE_BUILD_TYPE=Release
- source devel/setup.bash

First Terminal:
- roslaunch realsense2_camera rs_camera.launch

Second Terminal:
- roslaunch darknet_ros yolo_v3.launch

Third Terminal:
-roslaunch darknet_ros_msgs yolodistance.launch (also you can use -rosrun dark_net_ros_msgs yolodistance_v2.py)
After run this launch file detected object's 3D world coordinats published /auto_arm_topic topic

This repository also contain Haar Cascade python script for panel detection it use realsense and OpenCV python 3 library and get 3D coordinats for panel components.

In order to run this script run the following line
-python3 erc_buton_switch_socket.py
