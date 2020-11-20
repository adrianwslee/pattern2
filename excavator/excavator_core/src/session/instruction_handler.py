#!/usr/bin/env python
# -*- coding: utf-8 -*-

import threading

import rospy
from doosan_msgs.msg import InstructionCodeStamped

class InstructionHandler:

    def __init__(self):

        self.__instruction_publisher = rospy.Publisher('/excavator/doosan_instruction_code', InstructionCodeStamped, queue_size=10)
        self.__instruction_subscriber = rospy.Subscriber('/excavator/doosan_instruction_code', InstructionCodeStamped, self.instuction_msg_callback, queue_size = 1)



    def instuction_msg_callback(self, msg):
        rospy.logdebug("[excavator_core] Receive instruction message")
        rospy.logdebug(msg.subscriber)
        rospy.logdebug(msg.type)
        rospy.logdebug(msg.name)
        rospy.logdebug(msg.description)
        rospy.logdebug("-----")




    def __update_odom_by_gnss(self):
        #TODO implements...
        pass


    def __update_odom_by_velocity(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.__current_time = rospy.Time.now()

                       
            rate.sleep()


    def __test_update_velocity(self, twist):
        self.__vx = twist.linear.x
        self.__vy = twist.linear.y
        self.__vth = twist.angular.z


def main():
    rospy.init_node('excavator_odom')
    try:
        instruction_setup = InstructionHandler()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()