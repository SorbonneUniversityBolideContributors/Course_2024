
__author__ = "Eliot CHRISTON"
__date__ = "2023-01"


from vehicle import Driver

from simu_controller import SimuController
from simu_lidar_publisher import SimuLidarPublisher
from simu_camera_publisher import SimuCameraPublisher

import rospy


my_bot_name = "simu_bot"

# init ros node
rospy.init_node(my_bot_name + '_node', anonymous=True)

my_driver = Driver()
my_controller = SimuController(my_driver, my_bot_name)
my_lidar_publisher = SimuLidarPublisher(my_driver, my_bot_name)
my_camera_publisher = SimuCameraPublisher(my_driver, my_bot_name)


F_LIDAR = 12
rospy.Timer(rospy.Duration(1 / F_LIDAR), my_lidar_publisher.publish_lidar_data)

F_CAMERA = 30
rospy.Timer(rospy.Duration(1 / F_CAMERA), my_camera_publisher.publish_camera_data)

rospy.loginfo("Controller initialized, publishing raw lidar data and raw camera data.")

rate = rospy.Rate(30)
while not rospy.is_shutdown():
    # Your control logic here
    if my_driver.step() == -1:
        break
    rate.sleep()