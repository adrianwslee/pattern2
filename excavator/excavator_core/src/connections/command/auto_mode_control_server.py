#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from doosan_msgs.srv import AutoModeControl, AutoModeControlRequest, AutoModeControlResponse
from doosan_msgs.msg import UserCommand

class AutoModeControlServer:

    def __init__(self):
        
        rospy.Service('/excavator/command/auto_mode_control', AutoModeControl, self.__handle_auto_mode_control)


    # 
    #   commands    : doosan_msgs/UserCommand.msg
    # 
    def __handle_auto_mode_control(self, request):
        user_commands = request.commands

        # 
        #   TODO IMPLEMENTS WORK MODE HANDLER
        # 


        return request.SUCCESS


def main():
    rospy.init_node('excavator_core_auto_mode_control_server', anonymous=True)
    try:
        auto_mode_control_server = AutoModeControlServer()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()