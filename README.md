# robot-ros2
ROS2 workspace for a robot with Ackermann steering

## Parts of robot
### Implemented
* Slamtec RPLIDAR C1
* DFRobot DRI0042 motor driver support
* H-Bridge for steering - DFrobot DRI0042
* Camera - Eternico ET201 webcam
* simple GNSS

## Running The Robot

Use the top-level bringup launch when you want the full robot stack. It always starts
the Foxglove bridge, and by default also starts the traction motor, steering motor, camera,
lidar and GNSS:

```bash
cd /home/dano/robot-ros2
ros
robot_ros
ros2 launch robot_bringup robot_launch.py
```

You can disable one subsystem while debugging:

```bash
ros2 launch robot_bringup robot_launch.py enable_traction_motor:=false
ros2 launch robot_bringup robot_launch.py enable_lidar:=false
ros2 launch robot_bringup robot_launch.py enable_gamepad:=false
ros2 launch robot_bringup robot_launch.py enable_camera:=true
```

## USB Gamepad Teleop

The `gamepad_teleop` node reads Linux joystick device `/dev/input/js0` and
publishes both motor topics:

* traction motor: `/traction_motor_cmd`
* steering motor: `/steering_motor_cmd`

Default axis mapping:

* `axis_traction=1` (left stick vertical)
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
	enable_traction_motor:=true \
	enable_steering_motor:=true \
	enable_gamepad:=true \
	gamepad_deadman_button:=4
```

## Obstacle avoidance

The `obstacle_avoidance` package provides a simple open-loop obstacle-avoidance
controller for the Ackermann robot. It uses the C1 lidar scan and directly
commands the traction and steering motors.

### Inputs and outputs

* Subscribes to `/scan_c1` (`sensor_msgs/msg/LaserScan`)
* Publishes `/traction_motor_cmd` (`std_msgs/msg/Float32`)
* Publishes `/steering_motor_cmd` (`std_msgs/msg/Float32`)

Motor command values are normalized to `-1.0` through `1.0`. Positive steering
values command left, negative values command right, and `0.0` is neutral.

### Controller behavior

The controller has three states:

* `DRIVING`: moves forward at `forward_power` with neutral steering
* `AVOIDING`: moves forward more slowly and steers toward the clearer side
* `STRAIGHTENING`: applies opposite steering to return the wheels toward center

The lidar angles are corrected for the current sensor transform. If an obstacle
is closer than `0.30 m` in the forward cone, the controller publishes an
emergency stop (`0.0` traction and `0.0` steering).

### Launch

Build and source the workspace, then start the lidar and obstacle avoider in
separate terminals:

```bash
cd /home/dano/robot-ros2
source /opt/ros/lyrical/setup.bash
source install/setup.bash

# Terminal 1
ros2 launch lidar lidar_launch.py

# Terminal 2
ros2 launch obstacle_avoidance avoidance.launch.py
```

The obstacle avoider is currently launched separately and is not enabled by
`robot_bringup` automatically. Do not run it at the same time as gamepad teleop,
because both nodes publish commands to the same motor topics.

### Parameters

Parameters are configured in
`src/obstacle_avoidance/config/params.yaml`:

* `forward_power`: normal traction power while the path is clear (`0.20`)
* `avoid_power`: traction power while avoiding (`0.15`)
* `steer_power`: steering command during avoidance (`1.00`)
* `front_half_angle`: half-width of the forward detection cone (`45` degrees)
* `stop_distance`: distance that triggers avoidance (`0.80 m`)
* `resume_distance`: distance required before returning to driving (`1.20 m`)
* `avoid_duration`: duration of the avoidance turn (`2.00 s`)
* `straighten_factor`: straightening duration as a fraction of avoid duration (`0.2`)

The YAML values can be adjusted and the node restarted to apply them. The
traction and steering motor nodes still apply their own watchdog and optional
soft-start/stop behavior to the commands published by this controller.

## Display
### XFCE
```bash
sudo apt update
sudo apt install xfce4 xorg xinit lightdm
sudo systemctl enable lightdm
sudo reboot
```
If it doesn't start try this:
`sudo nano /etc/X11/xorg.conf.d/99-fbdev.conf`
```
Section "Device"
    Identifier "RPi GPU"
    Driver     "modesetting"
    Option     "kmsdev" "/dev/dri/card1"
EndSection
```

### Foxglove
```bash
sudo apt install ros-lyrical-foxglove-bridge
curl -O https://get.foxglove.dev/desktop/latest/foxglove-studio-latest-linux-arm64.deb
sudo apt install ./foxglove-studio-*.deb
```

The Foxglove bridge is started automatically by:

```bash
ros2 launch robot_bringup robot_launch.py
```

## Camera
Eternico Webcam ET201

Set JPEG quality
```bash
ros2 param get /v4l2_camera_node image_raw.compressed.jpeg_quality
# Integer value is: 95
ros2 param set /v4l2_camera_node image_raw.compressed.jpeg_quality 50
```

### 1. Install build dependencies
```bash
sudo apt install ros-lyrical-v4l2-camera
sudo apt install ros-lyrical-image-transport-plugins
sudo usermod -aG video $USER

source /opt/ros/lyrical/setup.bash

# Run the camera through robot bringup
ros2 launch robot_bringup robot_launch.py enable_camera:=true

# Verify it's publishing
ros2 topic list
ros2 topic hz /image_raw
```

Camera parameters used by bringup:

```text
video_device=/dev/video0
image_size=[640,480]
pixel_format=YUYV
publish_format=compressed
```

More info:
```bash
# For verifying V4L2 compliance and controls before launching ROS
sudo apt install v4l-utils

# Run compliance check on your camera
v4l2-compliance -d /dev/video0

# See all supported formats/resolutions
v4l2-ctl --list-formats-ext -d /dev/video0
```

Record a bag:
```bash
ros2 bag record /image_raw/compressed /camera_info -o ~/bags/camera_test
```
