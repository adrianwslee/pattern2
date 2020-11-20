#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import PointCloud2

class SVCCurrentPcdMap25Subscriber:

    def __init__(self):
        
        self.__current_pcd_map_25_subscriber = rospy.Subscriber('/svc/current_pcd_map_25', PointCloud2, self.__update_current_pcd_map_25, queue_size=1 )

    
    def __update_current_pcd_map_25(self, msg):
        header = msg.header
        
        # 
        #   TODO IMPLEMENTS UPDATE CODE HERE
        #

