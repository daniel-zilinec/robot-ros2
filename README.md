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
lidar and drive motor together:

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

To start steering motor as a second motor instance:

```bash
ros2 launch robot_bringup robot_launch.py enable_steering_motor:=true
```

To run steering motor only:

```bash
ros2 launch robot_bringup robot_launch.py enable_lidar:=false enable_motor:=false enable_steering_motor:=true
```

To run drive + steering + USB gamepad teleop:

```bash
ros2 launch robot_bringup robot_launch.py \
	enable_lidar:=false \
	enable_motor:=true \
	enable_steering_motor:=true \
	enable_gamepad:=true
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

Soft start/stop is enabled by default and is controlled per motor instance with
these parameters:

* `soft_start_stop` = `true` or `false`
* `soft_start_stop_rate_per_s` = how fast the command can change, in normalized
	command units per second

That means you can enable it for steering and disable it for drive, or give each
motor its own ramp rate.

## Steering Motor Control

The steering motor uses a second `motor_driver` node instance and listens on:

* `/steering_motor_cmd`

Command format is the same as drive motor (`std_msgs/Float32`, range `-1.0` to `1.0`).

Example steering commands:

```bash
# Turn one direction
ros2 topic pub /steering_motor_cmd std_msgs/msg/Float32 "{data: 0.3}" --once

# Turn opposite direction
ros2 topic pub /steering_motor_cmd std_msgs/msg/Float32 "{data: -0.3}" --once

# Stop steering motor
ros2 topic pub /steering_motor_cmd std_msgs/msg/Float32 "{data: 0.0}" --once
```

Steering soft start/stop can be toggled independently from drive:

```bash
ros2 launch robot_bringup robot_launch.py \
	enable_steering_motor:=true \
	steering_soft_start_stop:=true \
	steering_soft_start_stop_rate_per_s:=0.4
```

## USB Gamepad Teleop

The `gamepad_teleop` node reads Linux joystick device `/dev/input/js0` and
publishes both motor topics:

* drive motor: `/motor_cmd`
* steering motor: `/steering_motor_cmd`

Default axis mapping:

* `axis_drive=1` (left stick vertical)
* `axis_steering=0` (left stick horizontal)

Default scaling:

* `max_drive=0.6`
* `max_steering=0.8`
* `deadzone=0.08`

If the controller is detected by kernel logs but teleop does not move motors,
verify the joystick node exists and is readable:

```bash
ls -l /dev/input/js0
```

Optional: require a deadman button (example button index `4`):

```bash
ros2 launch robot_bringup robot_launch.py \
	enable_lidar:=false \
	enable_motor:=true \
	enable_steering_motor:=true \
	enable_gamepad:=true \
	gamepad_deadman_button:=4
```
