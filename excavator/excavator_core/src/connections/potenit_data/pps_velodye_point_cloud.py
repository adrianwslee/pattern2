#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import PointCloud2

class PPSVelodynePointCloud:

    def __init__(self):
        
        self.__velodyne_pcl_subscriber = rospy.Subscriber('/pps/velodyne_points2', PointCloud2, self.__update_velodyne_point2, queue_size=1 )

    
    def __update_velodyne_point2(self, msg):
        header = msg.header
        
        # 
        #   TODO IMPLEMENTS UPDATE CODE HERE
        #

