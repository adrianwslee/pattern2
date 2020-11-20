#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

class TaskStatusPublisher:

    def __init__(self):
        
        self.__task_status_publisher = rospy.Publisher('/excavator/diagnostic', DiagnosticArray, queue_size=1)

        thread = threading.Thread(target=self.__task_status_update_loop)
        thread.start()


    
    def __task_status_update_loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():

            diagnostic_array = DiagnosticArray()

            control_status = DiagnosticStatus()
            
            # 
            #   TODO IMPLEMENTS CONTROL STATUS CODE HERE
            #

            

            diagnostic_array.header.stamp = rospy.Time.now()
            diagnostic_array.status.append(control_status)

            self.__task_status_publisher.publish(diagnostic_array)
            rate.sleep()


def main():
    rospy.init_node('excavator_core_task_status_publisher', anonymous=True)
    try:
        task_status_publisher = TaskStatusPublisher()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()