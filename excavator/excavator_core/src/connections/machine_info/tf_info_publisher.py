#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading

import numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import JointState

class TFInfoPublisher:

    def __init__(self):
        self.__state_publisher = rospy.Publisher('/joint_states', JointState, queue_size=1)
        self.__command_subscriber = rospy.Subscriber('joint_commands', JointState, self.__joint_commands_callback, queue_size = 1)

        zeros = rospy.get_param('zeros')

        self.__upper_body_joint = zeros.get('upper_body_joint')
        self.__boom_joint = zeros.get('boom_joint')
        self.__arm_joint = zeros.get('arm_joint')
        self.__bucket_joint = zeros.get('bucket_joint')
        
        if rospy.has_param('IS_FAKE_MODE'):
            self.__is_fake_mode = rospy.get_param('IS_FAKE_MODE')
            
            thread = threading.Thread(target=self.__update_joint_state)
            thread.start()
        else :
            rospy.logdebug('[JointStatePublisher] can not find IS_FAKE_MODE parameter... skipped! ')


    def __update_joint_state(self):

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            joint_msg = JointState()    
            joint_msg.header = Header()
            joint_msg.header.stamp = rospy.Time.now()
            joint_msg.name = ['upper_body_joint', 'boom_joint', 'arm_joint', 'bucket_joint']
            joint_msg.position = [self.__upper_body_joint, self.__boom_joint, self.__arm_joint, self.__bucket_joint]
            joint_msg.velocity = []
            joint_msg.effort = []
            self.__state_publisher.publish(joint_msg)

            rate.sleep()
        
       
    def __joint_commands_callback(self, joint_commands):
        if np.array(joint_commands.name).size == 0:
            return

        # For test mode.
        if self.__is_fake_mode:
            for idx, value in enumerate(joint_commands.name) :
                if value == 'upper_body_joint' :
                    self.__upper_body_joint = joint_commands.position[idx]
                if value == 'boom_joint' :
                    self.__boom_joint = joint_commands.position[idx]
                if value == 'arm_joint' :
                    self.__arm_joint = joint_commands.position[idx]
                if value == 'bucket_joint' :
                    self.__bucket_joint = joint_commands.position[idx]

        # update with cylinder value?
        else :
            pass


def main():
    rospy.init_node('excavator_core_tf_info_publisher')
    try:
        tf_info_publisher = TFInfoPublisher
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()