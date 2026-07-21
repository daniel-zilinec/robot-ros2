# Robot setup

ROS2 workspace for running Slamtec lidar C1 on Ubuntu 26.04 LTS
using ROS2 Lyrical. Includes launch files, udev rules for stable device naming, and bag
recording setup.

Additional setup guides:

- [ros2_install.md](ros2_install.md) for ROS2 installation on the RPi

---

## Hardware

| Device | Model | Connection |
|---|---|---|
| Lidar 1 | Slamtec RPLIDAR C1 | USB (`/dev/ttyUSB_lidar_c1`) |
| Computer | Lenovo ThinkPad E15 | — |

### C1 Specs (as reported by driver)
- Scan mode: `Standard`
- Max distance: 16.0 m
- Point number: 16.1K
- Baudrate: 460800

---

## Software Requirements

- Ubuntu 26.04 LTS
- ROS2 Lyrical Luth
- `sllidar_ros2` driver (included in this repo, modified for Lyrical compatibility)
- `rviz2` (included with ROS2 desktop install)

---

## Initial Machine Setup

### 1. Install ROS2 Lyrical

Follow the official ROS2 installation guide for Ubuntu 26.04. After installation, put this line to your .bashrc:
```
alias ros='source /opt/ros/lyrical/setup.bash'
```
And source the environment everytime you start a console:

```bash
ros
```

### 2. Add User to `dialout` Group

Required for serial port access without `sudo`:

```bash
sudo usermod -aG dialout $USER
```

Log out and back in for this to take effect.

---

## udev Rules — Stable Device Names

Without udev rules, the lidars may swap between `/dev/ttyUSB0` and `/dev/ttyUSB1` on
reboot or replug. These rules assign permanent names based on each device's unique USB
serial number.

### Get the USB Serial Numbers
Run this for each lidar (both plugged in):
```Bash
udevadm info -a -n /dev/ttyUSB0 | grep '{serial}' | head -1
udevadm info -a -n /dev/ttyUSB1 | grep '{serial}' | head -1
```

### Create the rules file

```bash
sudo nano /etc/udev/rules.d/99-slamtec-lidars.rules
```

Paste the following (serial numbers are specific to this hardware):

```
SUBSYSTEM=="tty", ATTRS{serial}=="00465b8ef16ef011be5b4d9b1045c30f", SYMLINK+="ttyUSB_lidar_c1", MODE="0666"
SUBSYSTEM=="tty", ATTRS{serial}=="923975239873ed11824a68eefdf7b791", SYMLINK+="ttyUSB_lidar_s2", MODE="0666"
```

### Apply the rules

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Unplug and replug both lidars, then verify:

```bash
ls -la /dev/ttyUSB_lidar_*
# Expected:
# lrwxrwxrwx ... /dev/ttyUSB_lidar_c1 -&gt; ttyUSB0
# lrwxrwxrwx ... /dev/ttyUSB_lidar_s2 -&gt; ttyUSB1
```

> **Note:** The `-> ttyUSBx` target may vary — that's fine. The symlink always points
> to the correct device regardless of boot order.
---

## Building the Workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Add the workspace overlay alias to your shell startup:

```
alias rosws='source ~/ros2_ws/install/setup.bash'
```

Start the overlay in every new console
```bash
rosws
```

---

## Running the Lidar

### Launch lidar

```bash
cd ~/ros2_ws
ros2 launch src/my_lidar_bringup/launch/lidar_launch.py
```

This starts:
- `sllidar_node` for C1 → publishes to `/scan_c1`
- Static TF publisher for lidar frame
- RViz2

### RViz2 Configuration

When RViz2 opens:

1. Set **Fixed Frame** → `base_link`
2. Click **Add** → **By topic** → `/scan_c1` → **LaserScan** → OK

### Verify topics are publishing

```bash
ros2 topic list
ros2 topic echo /scan_c1 --once
```

### Verify TF tree

```bash
ros2 run tf2_tools view_frames
# Generates frames.pdf showing: base_link → laser_c1
```

---

## Recording Bag Files

Record both lidar scans and TF data:

```bash
ros2 bag record -a -o ./lidar_recordings/my_recording
```

Stop recording with `Ctrl+C`.

### Inspect a recording

```bash
ros2 bag info ~/lidar_recordings/my_recording
```

### Replay a recording (no lidars needed)

```bash
# Terminal 1
rviz2

# Terminal 2
ros2 bag play ~/lidar_recordings/my_recording --loop
```

Useful replay options:

```bash
# Half speed
ros2 bag play ~/lidar_recordings/my_recording --rate 0.5

# Start 10 seconds in
ros2 bag play ~/lidar_recordings/my_recording --start-offset 10.0

# Single topic only
ros2 bag play ~/lidar_recordings/my_recording --topics /scan_c1
```

> **Note:** Bag files (`.mcap`) are excluded from this git repository via `.gitignore`.
> Store recordings separately.

---

## Repository Structure

```
ros2_ws/
├── .gitignore
├── readme.md                          ← this file
└── src/
    ├── sllidar_ros2/                  ← Slamtec driver (modified for Lyrical)
    └── my_lidar_bringup/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/
        ├── my_lidar_bringup/
        │   ├── __init__.py
        │   └── py.typed
        ├── launch/
        │   └── lidar_launch.py   ← main launch file
        └── test/
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `Permission denied` on `/dev/ttyUSB*` | User not in `dialout` group | `sudo usermod -aG dialout $USER` then re-login |
| Device not found at `/dev/ttyUSB_lidar_*` | udev rules not applied | Replug lidars after `sudo udevadm trigger` |
| RViz2: `Frame [base_link] does not exist` | TF not publishing | Check static_transform_publisher nodes started correctly |
| RViz2: `queue is full` warning | Missing TF data | Ensure `/tf_static` is being published; check launch file |
| No scan data after replaying bag | Fixed Frame wrong in RViz2 | Set Fixed Frame to `base_link` or `laser_c1` |

---

## Notes on `sllidar_ros2` Modifications

The `sllidar_ros2` driver included in this repository has been modified from the upstream
[Slamtec/sllidar_ros2](https://github.com/Slamtec/sllidar_ros2) to work correctly with
ROS2 Lyrical on Ubuntu 26.04. The upstream version is not used directly.

If you need to update the driver in the future, apply changes carefully and re-test both
lidars before committing.

---

## Quick Reference

```bash
# Launch everything
ros2 launch src/my_lidar_bringup/launch/dual_lidar_launch.py

# Record
ros2 bag record -a -o ~/lidar_recordings/my_recording

# Replay
ros2 bag play ~/lidar_recordings/my_recording --loop

# Check active topics
ros2 topic list

# Check TF tree
ros2 run tf2_tools view_frames
```
