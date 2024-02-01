# Tutorial: Using the Simulator

This tutorial describes the procedure for using the Webots simulator for self-driving car racing.

## Startup

### ROS Master
It is **crucial** to launch a ROS Master first. Without it, the simulated vehicle will not be able to subscribe or publish in ROS topics.
```bash
roscore
```

### Simulator launch
To launch **webots**, use the command:
```bash
webots
```

### Launching a simulation world
From the main WeBots window, go to ```File``` then ```Open World``` and select one of the ```.wbt``` files in [worlds](worlds/)

The [Piste_SU_2023b.wbt](../worlds/Piste_SU_2023b.wbt) file is the simulation world used for self-driving car racing. It can be modified to test the navigation algorithms in a different environment. (see [tuto_modifying_world.md](tuto_modifying_world.md) for more information)

To start the simulation, click on the **play** button at the top of the graphical viewport (To check if the simulator is running, a time indicator is located to the left of the play button. If this indicator displays a stopwatch, then the simulation is running). If necessary, the simulation can be stopped and restarted with the pause and restart buttons, next to the play button.

## Common Errors
### Simulator launch error
If the simulator does not launch, the ROS package **webots_ros** may not be installed. In this case, use the command:

```bash
sudo apt-get install ros-noetic-webots-ros
```
As shown in the installation tutorial.

### Module error not found
If the simulator launches but displays a module error not found, this is more of a ROS environment configuration issue. It is then necessary to check that the other ROS packages used by the simulator are installed and that the ROS environment is configured (forgetting an ROS package for example in ```catkin_ws/src/```). Don't forget to do the command
```bash
catkin_make
```
in the ```catkin_ws``` folder after installing a new ROS package.


## More information on using Webots
For more information on using Webots, refer to the [official Webots documentation](https://cyberbotics.com/doc/guide/tutorials?tab-language=python).

We also made a number of tutorials to help you modify the simulation worlds and vehicles used according to the progress of the project. These tutorials are available in the [tutorials](../tutorials/) folder.


## Tutorials

- [tuto_installation.md](tuto_installation.md) : This tutorial describes how to install the simulator.
- [tuto_usage.md](tuto_usage.md) : This tutorial describes how to use the simulator.
- [tuto_modifying_robot.md](tuto_modifying_robot.md) : This tutorial describes how to modify the 3D models used in the simulator.
- [tuto_modifying_world.md](tuto_modifying_world.md) : This tutorial describes how to modify the worlds used in the simulator.
- [tuto_modifying_controller.md](tuto_modifying_controller.md) : This tutorial describes how to modify the controllers used in the simulator.

[main_readme](../README.md)
