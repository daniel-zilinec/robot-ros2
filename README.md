# robot-ros2
ROS2 workspace for a robot with Ackermann steering

## Parts of robot
### Implemented
* Slamtec RPLIDAR C1
* DFRobot DRI0042 motor driver support

### Not implemented
* H-Bridge for steering - DFrobot DRI0042
* Camera

## Running The Robot

Use the top-level bringup launch when you want the full robot stack. It starts the
lidar and motor driver together:

```bash
cd /home/dano/robot-ros2
ros
robot_ros
ros2 launch robot_bringup robot_launch.py
```

You can disable one subsystem while debugging:

```bash
ros2 launch robot_bringup robot_launch.py enable_motor:=false
ros2 launch robot_bringup robot_launch.py enable_lidar:=false
```

## Manual Motor Control

The motor driver listens on the `/motor_cmd` topic and expects a `std_msgs/Float32`
value in the range `-1.0` to `1.0`:

* `1.0` = full forward
* `-1.0` = full reverse
* `0.0` = stop / coast

Make sure the motor driver launch is running first:

```bash
ros2 launch motor_driver motor_launch.py
```

Then publish commands from another terminal:

```bash
# Forward at 30%
ros2 topic pub /motor_cmd std_msgs/msg/Float32 "{data: 0.3}" --once

# Reverse at 30%
ros2 topic pub /motor_cmd std_msgs/msg/Float32 "{data: -0.3}" --once

# Stop
ros2 topic pub /motor_cmd std_msgs/msg/Float32 "{data: 0.0}" --once
```

The node has a 2 second watchdog, so if commands stop arriving it will coast the
motor automatically.
