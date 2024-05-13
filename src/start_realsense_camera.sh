#!/bin/bash

ros2 launch realsense2_camera rs_get_params_from_yaml_launch.py config_file:="/home/user/ros2_ws/src/realsense-ros/realsense2_camera/config/config.yaml" camera_name:=D435 camera_namespace:=robot
