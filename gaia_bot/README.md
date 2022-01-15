# Gaia Platform GaiaBot Example
This repository contains ROS2 nodes intended to be run on this inexpensive [Raspberry Pi based robot](https://www.amazon.com/Freenove-Raspberry-Tracking-Avoidance-Ultrasonic/dp/B07YD2LT9D). It can also serve as a starting point for incorporating Gaia into other types of robots.

## Requirements
- Ubuntu 20.04 for ARM64
- ROS2 Galactic
- Gaia Platform SDK 0.3.3 or higher

## Setup
### [Install Ubuntu Server 20.04 for ARM64](https://www.raspberrypi.com/documentation/computers/getting-started.html)
After installation of Ubuntu run the following in a terminal to install graphical desktop, lightdm display manager, and other prerequisites:
```bash
sudo apt update
sudo apt upgrade
sudo apt install lubuntu-desktop lightdm ssh python3-pip RPi.GPIO
```

### [Install ROS2](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
After installation of ROS2 run the following to install associated components:
```bash
sudo apt install python3-colcon-common-extensions python3-rosdep2 python3-smbus
```

### Modify Raspberry Pi firmware configuration
This is required in order for the camera to work.
```bash
sudo nano /boot/firmware/config.txt
```

Then add following lines at the end of the file:
```
start_x=1
gpu_mem=128
dtoverlay=vc4-fkms-v3d
```
When done press 'Ctl-x' followed by 'y' to save and exit.

Reboot
```bash
sudo reboot
```

### Install python packages
```bash
sudo pip3 install rpi_ws281x
```

### Update gpio permissions
In order for the range sensor node to work, permissions on the Raspberry Pi's memory mapped gpio pins need to be updated. First add yourself to the dialout group. This only needs to be done once.
```bash
sudo usermod -aG dialout $USER
```

Then change the group ownership on /dev/gpiomem to dialout and update permissions on the i2c bus. This must be done each time the Raspberry Pi is started.
```bash
sudo chown root.dialout /dev/gpiomem
sudo chmod g+rw /dev/gpiomem
sudo chmod a+rw /dev/i2c-*
```

### Clone project
Create a ROS2 workspace if you have not already, `cd` into its `src` directory and clone this repo.
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/gaia-platform/examples.git
```

### Source ROS2
In order to use ROS2 execute the following. Consider adding to .bashrc or similar in order to execute in each new terminal window.
```bash
source /opt/ros/galactic/setup.bash
```

### Install dependencies listed in package.xml files
```bash
cd ~/ros2_ws
rosdep install --from-paths src -i -y
```

### Build
Navigate into the main ros2_ws folder then build and install.
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Launch

TODO: Create launch file to launch all nodes

Launch the camera node:
```bash
ros2 run v4l2_camera v4l2_camera_node
```

In a new terminal launch the faces node:
```bash
ros2 run faces faces
```

In another new terminal launch the range node:
```bash
ros2 run range range
```

In another new terminal launch the neck_pose node:
```bash
ros2 run pca9685 neck_pose
```

In another new terminal launch the gaia_bot node:
```bash
ros2 launch gaia_bot gaia_bot.launch.py
```
