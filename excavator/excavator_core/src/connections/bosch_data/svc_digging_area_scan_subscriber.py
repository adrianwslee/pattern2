#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from pcl_msgs.msg import PolygonMesh

class SVCDiggingAreaScanSubscriber:

    def __init__(self):
        
        self.__digging_area_scan_subscriber = rospy.Subscriber('/svc/digging_area_scan', PolygonMesh, self.__update_digging_area, queue_size=1 )

    
    def __update_digging_area(self, msg):
        header = msg.header
        
        # 
        #   TODO IMPLEMENTS UPDATE CODE HERE
        #

