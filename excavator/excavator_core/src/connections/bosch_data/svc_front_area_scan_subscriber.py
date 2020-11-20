#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from pcl_msgs.msg import PolygonMesh

class SVCFrontAreaScanSubscriber:

    def __init__(self):
        
        self.__front_area_scan_subscriber = rospy.Subscriber('/svc/front_area_scan', PolygonMesh, self.__update_front_area, queue_size=1 )

    
    def __update_front_area(self, msg):
        header = msg.header
        
        # 
        #   TODO IMPLEMENTS UPDATE CODE HERE
        #

