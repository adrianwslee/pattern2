#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from pcl_msgs.msg import PolygonMesh

class SVC360AreaScanSubscriber:

    def __init__(self):
        
        self.__360_area_scan_subscriber = rospy.Subscriber('/svc/360_area_scan', PolygonMesh, self.__update_360_area_scan_info, queue_size=1 )


 
    def __update_360_area_scan_info(self, msg):
        header = msg.header
        vertices = msg.polygons
        pcd_data = msg.cloud

        # 
        #   TODO IMPLEMENTS UPDATE CODE HERE
        #

