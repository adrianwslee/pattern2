#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from pcl_msgs.msg import PolygonMesh

class SVCBecketLoadScanSubscriber:

    def __init__(self):
        
        self.__bucket_load_scan_subscriber = rospy.Subscriber('/svc/bucket_load_scan', PolygonMesh, self.__update_bucket_load_scan, queue_size=1 )

    
    def __update_bucket_load_scan(self, msg):
        header = msg.header
        
        # 
        #   TODO IMPLEMENTS UPDATE CODE HERE
        #

