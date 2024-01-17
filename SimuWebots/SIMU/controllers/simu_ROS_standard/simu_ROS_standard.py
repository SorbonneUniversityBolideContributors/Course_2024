
__author__ = "Eliot CHRISTON"
__date__ = "2023-01"


from vehicle import Driver

from node_controller import SimuController
from node_lidar_publisher import SimuLidarPublisher

import rospy


my_bot_name = "simu_bot"

# init ros node
rospy.init_node(my_bot_name + '_node', anonymous=True)

my_driver = Driver()
my_controller = SimuController(my_driver, my_bot_name)
my_lidar_publisher = SimuLidarPublisher(my_driver, my_bot_name)


TIME_SIMU = 1.0/12.0
rospy.Timer(rospy.Duration(TIME_SIMU), my_lidar_publisher.publish_lidar_data)
rospy.loginfo("Controller initialized, publishing raw lidar data")

rate = rospy.Rate(30)
while not rospy.is_shutdown():
    # Your control logic here
    if my_driver.step() == -1:
        break
    rate.sleep()