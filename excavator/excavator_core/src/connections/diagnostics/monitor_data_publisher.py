#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

class MonitorDataPublisher:

    def __init__(self):
        
        self.__monitor_data_publisher = rospy.Publisher('/excavator/diagnostic', DiagnosticArray, queue_size=1)

        thread = threading.Thread(target=self.__monitor_data_update_loop)
        thread.start()


    
    def __monitor_data_update_loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():

            diagnostic_array = DiagnosticArray()

            control_status = DiagnosticStatus()
            
            # 
            #   TODO IMPLEMENTS CONTROL STATUS CODE HERE
            #

            

            diagnostic_array.header.stamp = rospy.Time.now()
            diagnostic_array.status.append(control_status)

            self.__monitor_data_publisher.publish(diagnostic_array)
            rate.sleep()


def main():
    rospy.init_node('excavator_core_monitor_data_publisher', anonymous=True)
    try:
        monitor_data_publisher = MonitorDataPublisher()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()