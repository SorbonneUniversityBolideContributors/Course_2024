# Tutorial: Modifying the robot controller

This tutorial describes how to modify the simulated robot controller in Webots.

## Description of the controller used

The controller used by the simulated robot is a ROS controller. It is therefore possible to modify it using the ROS tools. The controller is implemented in the [controllers/simu_ROS_standard/](../controllers/simu_ROS_standard/) folder. This folder contains the files required to use the ROS controller in Webots.

When starting the simulation, Webots launches the python script [simu_ROS_standard.py](../controllers/simu_ROS_standard/simu_ROS_standard.py). This python script is the entry point of the ROS controller.

To standardize the operation of the simulated vehicle, this input point instantiates python classes located in the same folder and that correspond to the different parts of the simulated robot. One can find **a class for the control** of the robot, ie the application of the speed and direction controls and **a class per sensor** used by the simulated robot.

Each sensor class defines a ROS topic on which it publishes the sensor data. Similarly, the control class defines a ROS topic on which it subscribes to receive the speed and direction commands to be applied to the simulated robot.

The names of the ROS topics used by the controller are defined so as to be consistent with the names of the ROS topics used by the real robot. Thus, it is possible to test a navigation algorithm on the simulated robot before implementing it on the real robot without having to modify the code of the algorithm.

## Adding a sensor

To add a sensor to the simulated robot, create a new class in the [simu_ROS_standard/] folder (../controllers/simu_ROS_standard/). It is advisable to copy the code of an already existing class and modify it to fit the new sensor.


### Prerequisites

To publish the data of a new sensor, it is necessary to have first added the sensor to the 3D model of the simulated robot (see [tuto_modifying_robot.md](tuto_modifying_robot.md) for more information).

### Import python modules
Import the python modules necessary for its operation.
- The ```rospy``` module is required for communication with ROS.
- The ```Driver``` module is required for communication with Webots.
- Finally, there is a module by type of sensor used by the simulated robot in Webots (see https://cyberbotics.com/doc/guide/sensors). 
- Import the ROS message type corresponding to the data published by the sensor. (It is recommended to use the ```sensor_msgs``` for standard sensors, see http://wiki.ros.org/sensor_msgs).

Example for the camera:
```python
from vehicle import Driver
from controller import Camera

from sensor_msgs.msg import Image as SensorImage
import rospy
```

## Tutorials

- [tuto_installation.md](tuto_installation.md) : This tutorial describes how to install the simulator.
- [tuto_usage.md](tuto_usage.md) : This tutorial describes how to use the simulator.
- [tuto_modifying_robot.md](tuto_modifying_robot.md) : This tutorial describes how to modify the 3D models used in the simulator.
- [tuto_modifying_world.md](tuto_modifying_world.md) : This tutorial describes how to modify the worlds used in the simulator.
- [tuto_modifying_controller.md](tuto_modifying_controller.md) : This tutorial describes how to modify the controllers used in the simulator.

[main_readme](../README.md)
