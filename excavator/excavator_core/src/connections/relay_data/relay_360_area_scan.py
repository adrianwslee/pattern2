#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading

from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from pcl_msgs.msg import PolygonMesh
from sensor_msgs.msg import PointCloud2
from doosan_msgs.msg import RelayPCDInfo

class Relay360AreaScan:

    def __init__(self):
        
        self.__original_data_subscriber = rospy.Subscriber('/svc/360_area_scan', PolygonMesh, self.__message_callback, queue_size = 1)
        self.__relay_data_publisher = rospy.Publisher('/excavator/relay/360_area_scan', RelayPCDInfo, queue_size=1)
        
        self.__current_header = Header()
        self.__current_pcd = PointCloud2()
        self.__current_transform = Pose()

        self.__relay_rate = rospy.get_param("RELAY_RATE")

        thread = threading.Thread(target=self.__data_publish_loop)
        thread.start()
    

    def __message_callback(self, msg):
        self.__current_header = msg.header
        self.__current_pcd = msg.cloud

        # self.__current_transform = msg.transform
        # TODO



    def __data_publish_loop(self):

        # TODO Controlerble LOOP TIME
        rate = rospy.Rate(self.__relay_rate)
        while not rospy.is_shutdown():

            relay_data = RelayPCDInfo()            
            relay_data.header = self.__current_header
            relay_data.cloud = self.__current_pcd
            relay_data.transform = self.__current_transform

            self.__relay_data_publisher.publish(relay_data)
            rate.sleep()

            self.__relay_rate = rospy.get_param("RELAY_RATE")
            rate = rospy.Rate(self.__relay_rate)


def main():
    rospy.init_node('excavator_core_relay_360_area_scan', anonymous=True)
    try:
        relay_360_area_scan = Relay360AreaScan()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()