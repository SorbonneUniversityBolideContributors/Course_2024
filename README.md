# Autonomous RC Car Race 2024


## Project Overview
This is a project made by Sorbonne University students. The goal is to make an autonomous RC car that can drive on a track and avoid obstacles for the ENS 2024 race.

The project is divided into three main packages:
1. **Perception:** [perception_bolide](https://github.com/Pfecourse/perception_bolide)
2. **Planning:** [planning_bolide](https://github.com/Pfecourse/planning_bolide)
3. **Control:** [control_bolide](https://github.com/Pfecourse/control_bolide)


## ENS Race Overview
https://ajuton-ens.github.io/CourseVoituresAutonomesSaclay/

## Getting Started

Check the `Robot_setup.md` file to get started with the robot.

Here are the credentials for the rooter at St-Cyr
SSID: R15-AF66
mdp: eprtu89734

Bolide's IP when connected to the rooter (SSID=R15-AF66) at St-Cyr
IP bolide2: 192.168.137.78
IP bolide1: 192.168.137.165

## Aliases

```bash

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# ROS aliases
alias eb='nano ~/.bashrc'
alias sb='source ~/.bashrc'
alias cm='cd ~/catkin_ws && catkin_make'
alias cw='cd ~/catkin_ws'
alias cs='cd ~/catkin_ws/src'

export BOLIDE1_IP=192.168.137.165
export BOLIDE2_IP=192.168.137.78

# get the ip of the current machine
export MY_IP=$(hostname -I | cut -d' ' -f1)

# envs
alias bolide1_env="export ROS_MASTER_URI=http://$BOLIDE1_IP:11311 && export ROS_NAMESPACE=bolide1"
alias bolide2_env="export ROS_MASTER_URI=http://$BOLIDE2_IP:11311 && export ROS_NAMESPACE=bolide2"
alias local_env="export ROS_MASTER_URI=http://localhost:11311 && unset ROS_NAMESPACE"

# namespaces
alias bolide1_namespace="export ROS_NAMESPACE=bolide1" # when working with .bag file
alias bolide2_namespace="export ROS_NAMESPACE=bolide2" # when working with .bag file
alias simu_bot_namespace="export ROS_NAMESPACE=simu_bot"

# if namespace is bolide1 or bolide2, set ROS_IP to the IP of this machine else set to localhost
export ROS_IP=$(if [ "$ROS_NAMESPACE" = "bolide1" ] || [ "$ROS_NAMESPACE" = "bolide2" ]; then echo $MY_IP; else echo localhost; fi)
export ROS_HOSTNAME=$ROS_IP

alias bolide1_ssh="ssh bolide1@$BOLIDE1_IP"
alias bolide2_ssh="ssh bolide2@$BOLIDE2_IP"

# pull all the ros packages
alias giga_git_pull="cd ~/catkin_ws/src/perception_bolide && git pull && cd ~/catkin_ws/src/planning_bolide && git pull && cd ~/catkin_ws/src/control_bolide && git pull && cd ~/Course_2024 && git pull && cd"
```
