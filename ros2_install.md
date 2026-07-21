# ROS2 install guide
## RPi 5 with Ubuntu Server 26.04 LTS
### Step 1 — Locale
```Bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```
### Step 2 — Add ROS 2 Repository
```Bash
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

### Step 3 — Install
```Bash
sudo apt update && sudo apt upgrade
sudo apt install ros-lyrical-desktop
sudo apt install ros-dev-tools
```

### Step 4 — Alias
```Bash
echo "alias ros='source /opt/ros/lyrical/setup.bash'" >> ~/.bashrc
source ~/.bashrc
```

