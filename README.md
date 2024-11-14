# BLUESEA ROS driver

## Overview
----------
Provide M300 radar with ROS driver, which can run on ROS1 version and has built-in IMU data;

## Get and build the BLUESEA ROS driver package
1.Get the M300 ROS driver from Github and deploy the corresponding location

    mkdir m300_ros   											//create a folder and customize it
    cd m300_ros 												//into this folder
    git clone https://github.com/BlueSeaLidar/m300-ros.git  src //download the driver package and rename it to src
2.Build

    catkin_make
3.Update the current ROS package environment

    source ./devel/setup.sh


4.Using ROS launch to run drivers

    roslaunch Lidar_M300 [launch file]    		//The specific launch file description is as follows

## Driver launch launch file


Main parameter configuration instructionsï¼?

    param name="lidar_ip" value="192.168.1.33"#Lidar_IP
    param name="frame_id_Msg" value="m300_frame_test"#msg frame_id
    param name="frame_id_PointCloud2" value="base_link_test"#pointcloud2 frame_id




## rosbag bag operating instructions

	rostopic list 
Get the topic list, the driver default topic name is /lidar1/scan

	rosbag record topic_name 

Start recording data.

The recorded file is named with a timestamp, to stop recording, CTRL+C in the current terminal 

	rosbag play packet name

Check the recorded packet in the path where the packet is stored, if it prompts failed connect master exception, then ros master first and then rosbag play.

## Business Support

Please contact the technical support (https://pacecat.com/) through the official website for specific usage problems.
