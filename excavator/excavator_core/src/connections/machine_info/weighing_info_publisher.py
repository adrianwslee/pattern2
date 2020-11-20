#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading

from doosan_msgs.msg import WeighingInfo

class WeighingInfoPublisher:

    def __init__(self):
        
        self.__weighing_info_publisher = rospy.Publisher('/excavator/info/weighing_info', WeighingInfo, queue_size=1)

        thread = threading.Thread(target=self.__weighing_info_update_loop)
        thread.start()


    
    def __weighing_info_update_loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():

            weighing_info = WeighingInfo()
            
            # 
            #   TODO IMPLEMENTS WEIGHING INFO CODE HERE
            #
            weighing_info.total_load_weight = 3.7
            weighing_info.total_load_count = 2

             
            self.__weighing_info_publisher.publish(weighing_info)
            rate.sleep()


def main():
    rospy.init_node('excavator_core_weighing_info_publisher', anonymous=True)
    try:
        weighing_info_publisher = WeighingInfoPublisher()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()