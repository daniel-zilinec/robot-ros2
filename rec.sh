#!/usr/bin/env bash

BAG_NAME="${HOSTNAME}_$(date +"%Y%m%d_%H%M%S")"
BAG_PATH="/home/dano/robot-ros2/bags/${BAG_NAME}"

cd /home/dano/robot-ros2

source /home/dano/robot-ros2/install/setup.bash

ros2 bag record --topics /image_raw/compressed /camera_info /traction_motor_cmd /steering_motor_cmd /scan_c1 /fix /heading /vel -o "${BAG_PATH}"