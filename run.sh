#!/usr/bin/env bash
cd /home/dano/robot-ros2

source /home/dano/ros2_ws_gospace/install/setup.bash
source /home/dano/robot-ros2/install/setup.bash

ros2 launch robot_bringup robot_launch.py