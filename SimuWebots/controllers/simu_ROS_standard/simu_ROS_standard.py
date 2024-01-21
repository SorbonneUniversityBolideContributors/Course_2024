__author__ = "Eliot CHRISTON"
__date__ = "2023-01"

#%% IMPORTS ==============================================================================
from vehicle import Driver

from simu_controller import SimuController
from simu_lidar_publisher import SimuLidarPublisher
from simu_camera_publisher import SimuCameraPublisher
from simu_imu_publisher import SimuImuPublisher
from simu_rear_ranges_publisher import SimuRearRangesPublisher

import rospy

#%% INITIALIZATION =======================================================================
my_bot_name = "simu_bot"

rospy.init_node(my_bot_name + '_node', anonymous=True)

my_driver = Driver()
my_controller = SimuController(my_driver, my_bot_name)
my_lidar_publisher = SimuLidarPublisher(my_driver, my_bot_name)
my_camera_publisher = SimuCameraPublisher(my_driver, my_bot_name)
my_imu_publisher = SimuImuPublisher(my_driver, my_bot_name)
my_rear_ranges_publisher = SimuRearRangesPublisher(my_driver, my_bot_name)



#%% TIMERS ===============================================================================
F_LIDAR = 12
rospy.Timer(rospy.Duration(1 / F_LIDAR), my_lidar_publisher.publish_lidar_data)

F_CAMERA = 30
rospy.Timer(rospy.Duration(1 / F_CAMERA), my_camera_publisher.publish_camera_data)

F_IMU = 12
rospy.Timer(rospy.Duration(1 / F_IMU), my_imu_publisher.publish_imu_data)

F_REAR_RANGES = 12
rospy.Timer(rospy.Duration(1 / F_REAR_RANGES), my_rear_ranges_publisher.publish_rear_range_data)


rospy.loginfo("Controller initialized, now publishing data...")


#%% MAIN LOOP ===========================================================================
rate = rospy.Rate(30)

while not rospy.is_shutdown():
    # Your control logic here
    if my_driver.step() == -1:
        break
    rate.sleep()