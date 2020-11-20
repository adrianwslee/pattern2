#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading

from doosan_msgs.msg import GPSFix, GPSStatus

class GNSSPublisher:

    def __init__(self):
        
        self.__gnss_position_publisher = rospy.Publisher('/gnss_pose', GPSFix, queue_size=1)
        self.__gnss_status_publisher = rospy.Publisher('/gnss_status', GPSStatus, queue_size=1)

        thread_pose = threading.Thread(target=self.__gnss_pose_update_loop)
        thread_pose.start()

        thread_status = threading.Thread(target=self.__gnss_status_update_loop)
        thread_status.start()


    
    def __gnss_pose_update_loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            gps_pose = GPSFix()
            gps_pose.header.stamp = current_time
            # gps_pose.header.frame_id = ''

            # 
            #   TODO IMPLEMENTS GNSS POSITION INFO CODE HERE
            # 

            self.__gnss_position_publisher.publish(gps_pose)
            rate.sleep()


    def __gnss_status_update_loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            gps_status = GPSStatus()
            gps_status.header.stamp = current_time
            # gps_status.header.frame_id = ''

            # 
            #   TODO IMPLEMENTS GNSS STATUS INFO CODE HERE
            # 

            self.__gnss_status_publisher.publish(gps_status)
            rate.sleep()


def main():
    rospy.init_node('excavator_core_gnss_publisher', anonymous=True)
    try:
        gnss_publisher = GNSSPublisher()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()