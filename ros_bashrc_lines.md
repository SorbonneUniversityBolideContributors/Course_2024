```bash

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash


# ROS env variables ==========================================

# robot IPs
export BOLIDE1_IP=192.168.137.165
export BOLIDE2_IP=192.168.137.78

# if namespace is bolide1 or bolide2, set ROS_IP to the IP of this machine else set to localhost
export ROS_IP=$(if [ "$ROS_NAMESPACE" = "bolide1" ] || [ "$ROS_NAMESPACE" = "bolide2" ]; then echo $MY_IP; else echo localhost; fi)
export ROS_HOSTNAME=$ROS_IP

# ROS aliases =================================================

# usefull daily shortcuts
alias eb='nano ~/.bashrc'
alias sb='source ~/.bashrc'
alias cm='cd ~/catkin_ws && catkin_make'
alias cw='cd ~/catkin_ws'
alias cs='cd ~/catkin_ws/src'

# get the ip of the current machine
export MY_IP=$(hostname -I | cut -d' ' -f1)

# envs (changing the ROS_MASTER_URI and the ROS_NAMESPACE env variable de pending on the environement)
alias bolide1_env="export ROS_MASTER_URI=http://$BOLIDE1_IP:11311 && export ROS_NAMESPACE=bolide1"
alias bolide2_env="export ROS_MASTER_URI=http://$BOLIDE2_IP:11311 && export ROS_NAMESPACE=bolide2"
alias local_env="export ROS_MASTER_URI=http://localhost:11311 && unset ROS_NAMESPACE"

# namespaces (changing only the ROS_NAMESPACE env variable)
alias bolide1_namespace="export ROS_NAMESPACE=bolide1" # when working with .bag file
alias bolide2_namespace="export ROS_NAMESPACE=bolide2" # when working with .bag file
alias simu_bot_namespace="export ROS_NAMESPACE=simu_bot" # when working with the Webots simulation

# ssh (connexion to one of the robots)
alias bolide1_ssh="ssh bolide1@$BOLIDE1_IP"
alias bolide2_ssh="ssh bolide2@$BOLIDE2_IP"
```
