# Tutorial: Webots installation

This tutorial describes how to install the Webots simulator on a Linux environment so that it can be used with ROS1 Noetic.

## Prerequisites
- a PC environment compatible with ROS1 Noetic (e.g. Ubuntu 20.xx)
- administrator rights in this environment (sudo under Linux)
- have a working installation of ROS1 (preferably ROS Noetic)

An Internet connection is required to install the software and download certain simulator data (3D models, textures, etc.). 

## Webots installation procedure :
Under Linux, in a terminal, enter the commands :
```bash
wget -qO- https://cyberbotics.com/Cyberbotics.asc | sudo apt-key add -
sudo apt-add-repository 'deb https://cyberbotics.com/debian/ binary-amd64/'
```
```bash
sudo apt-get update
sudo apt-get install webots
```

Once installation is complete, you can launch WeBots directly from the terminal using the command :
```bash
webots
```
 
To use the robot simulation environment, you also need the **webots_ros** ROS package:

```bash
sudo apt-get install ros-noetic-webots-ros
```

## Tutorials

- [tuto_installation.md](tutorials/tuto_installation.md) : This tutorial describes how to install the simulator.
- [tuto_usage.md](tutorials/tuto_usage.md) : This tutorial describes how to use the simulator.
- [tuto_modifying_robot.md](tutorials/tuto_modifying_robot.md) : This tutorial describes how to modify the 3D models used in the simulator.
- [tuto_modifying_world.md](tutorials/tuto_modifying_world.md) : This tutorial describes how to modify the worlds used in the simulator.
- [tuto_modifying_controller.md](tutorials/tuto_modifying_controller.md) : This tutorial describes how to modify the controllers used in the simulator.

[main_readme](../README.md)