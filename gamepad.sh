#!/usr/bin/env bash
cd /home/dano/robot-ros2

source /home/dano/robot-ros2/install/setup.bash

ros2 launch gamepad_teleop gamepad_launch.py topic_steering:=/steering_cmd