#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from bosch_msgs.msg import SensorObjectList

class DasyObjectListSubscriber:

    def __init__(self):
        
        self.__object_list_subscriber = rospy.Subscriber('/dasy/object_list', SensorObjectList, self.__update_object_list, queue_size=1 )


    def __update_object_list(self, msg):
        pass

        # 
        #   TODO IMPLEMENTS UPDATE CODE HERE
        #

