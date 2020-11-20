#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from pcl_msgs.msg import PolygonMesh

class SVCAdtDumpbedScanSubscriber:

    def __init__(self):
        
        self.__dumpbed_scan_subscriber = rospy.Subscriber('/svc/adt_dumpbed_scan', PolygonMesh, self.__update_dumpbed_scan_info, queue_size=1 )



    def __update_dumpbed_scan_info(self, msg):
        header = msg.header
        vertices = msg.polygons
        pcd_data = msg.cloud

        # 
        #   TODO IMPLEMENTS UPDATE CODE HERE
        #

