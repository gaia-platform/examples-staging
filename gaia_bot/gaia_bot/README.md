# Gaia Platform ROS2 Example Template
This repository contains a ROS2 node which uses several basic Gaia Platform features.
It is structured as both an example and a template which can be used to write ROS2 nodes with Gaia rulesets.

## Requirements
- Ubuntu 20.04
- ROS2 Foxy or Galactic
- Gaia Platform SDK 0.3.2 or higher
- [Install and initialize](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Binary.html?highlight=rosdep#installing-and-initializing-rosdep) `rosdep`

## Try it out
In a terminal, run the following:
```bash
# If you already have a ROS2 workspace, `cd` into its `src` directory.
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/gaia-platform/GaiaPlatform.git
cd ..

source /opt/ros/$ROS_DISTRO/setup.bash

# Install the dependencies listed in package.xml.
rosdep install --from-paths src -i -y

colcon build

source install/setup.bash
ros2 launch gaia_ros2_example gaia_ros2_example.launch.py
```

Open **second** terminal and run:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 topic echo /output
```
That `ros2 topic echo` command will seem to "hang" because it is waiting for messages to arrive on the `/output` topic.

Open a **third** terminal and run:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 topic pub -1 /shapes shape_msgs/msg/SolidPrimitive "{type: 1, dimensions: [1.0, 1.0, 1.0]}"
```

In the second terminal, you should see:
```
data: 'We got a BOX! The dimensions are: X: 1, Y: 1, Z: 1, '
---
```
To reuse the first and second terminals, press `Ctrl+C` to terminate the `ros2 launch` and `ros2 topic echo` commands.

Try different [shape types](https://github.com/ros2/common_interfaces/blob/galactic/shape_msgs/msg/SolidPrimitive.msg) and dimensions in the `ros2 topic echo` command and check out the results!