#!/usr/bin/env python
# -*- coding: utf-8 -*-

import threading

import rospy

from std_msgs.msg import Header
from doosan_msgs.msg import InstructionCodeStamped

class InstructionHandler:

    def __init__(self):

        self.__instruction_subscriber = rospy.Subscriber('/excavator/doosan_instruction_code', InstructionCodeStamped, self.__instuction_msg_callback, queue_size = 1)

        # self.__instruction_publisher = rospy.Publisher('/excavator/doosan_instruction_code', InstructionCodeStamped, queue_size=10)
        self.__instruction_publisher = rospy.Publisher('/tx2/instruction_code_svc', InstructionCodeStamped, queue_size=10)
        self.__instruction_pps_publisher = rospy.Publisher('/tx2/instruction_code_pps', InstructionCodeStamped, queue_size=10)


    def __instuction_msg_callback(self, msg):
        rospy.logdebug("[excavator_core] Receive instruction message")
        rospy.logdebug(msg.subscriber)
        rospy.logdebug(msg.type)
        rospy.logdebug(msg.name)
        rospy.logdebug(msg.description)
        rospy.logdebug("-----")

        # Parse Instruction & send svc or pps


    def send_instruction(self, input_subscribers, input_type, input_name, input_description):
        instruction = InstructionCodeStamped()
        instruction.header.stamp = rospy.Time.now()

        instruction.subscriber = input_subscribers
        instruction.type = input_type
        instruction.name = input_name
        instruction.description = input_description

        self.__instruction_publisher.publish(instruction)


    def send_pps_instruction(self, input_subscribers, input_type, input_name, input_description):
        instruction = InstructionCodeStamped()
        instruction.header.stamp = rospy.Time.now()

        instruction.subscriber = input_subscribers
        instruction.type = input_type
        instruction.name = input_name
        instruction.description = input_description

        self.__instruction_pps_publisher.publish(instruction)


def main():
    rospy.init_node('excavator_odom')
    try:
        instruction_setup = InstructionHandler()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()