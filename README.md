# Autonomous RC Car Race 2024


## Project Overview
This is a project made by Sorbonne University students. The goal is to make an autonomous RC car that can drive on a track and avoid obstacles for the ENS 2024 race.

The project is divided into three main packages:
1. **Perception:** [perception_bolide](https://github.com/SorbonneUniversityBolideContributors/course_2024_pkgs/tree/main/perception_bolide)
2. **Planning:** [planning_bolide](https://github.com/SorbonneUniversityBolideContributors/course_2024_pkgs/tree/main/planning_bolide)
3. **Control:** [control_bolide](https://github.com/SorbonneUniversityBolideContributors/course_2024_pkgs/tree/main/control_bolide)


## ENS Race Overview
https://ajuton-ens.github.io/CourseVoituresAutonomesSaclay/

## Getting Started

Check the [Robot_setup.md](documentation/Robot_setup.md) file to get started with the robot.

Bolide's IP when connected to the rooter (SSID=R15-AF66) at St-Cyr
- IP bolide2: 192.168.137.78
- IP bolide1: 192.168.137.165

Check the [ros_bashrc_lines.md](documentation/ros_bashrc_lines.md) file and copy ir to you're own `~/.bashrc`.
It'll save you a lot of trouble when connecting to the robots or interacting with the simulation or simply with ROS.
Make sure that there are no double declarations and that the ROS environement variables `ROS_NAMESPACE` and `ROS_MASTER_URI` are unset.
