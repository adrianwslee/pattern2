#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from bosch_msgs.msg import ADTPoseStamped

class PPSAdtPoseDetection:

    def __init__(self):
        self.__pps_adt_pose_detection_subscriber = rospy.Subscriber('/pps/adt_pose_detection', ADTPoseStamped, self.__update_adt_pose_detection, queue_size=1 )

    def __update_adt_pose_detection(self, msg):
        header = msg.header
        
        # 
        #   TODO IMPLEMENTS UPDATE CODE HERE
        #

