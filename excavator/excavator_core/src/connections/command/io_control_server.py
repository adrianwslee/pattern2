#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from doosan_msgs.srv import IOControl, IOControlRequest, IOControlResponse
from doosan_msgs.msg import UserCommand

class IOControlServer:

    def __init__(self):
        
        rospy.Service('/excavator/command/io_control', IOControl, self.__handle_io_control)


    # 
    #   commands    : doosan_msgs/UserCommand.msg
    # 
    def __handle_io_control(self, request):
        user_commands = request.commands

        # 
        #   TODO IMPLEMENTS IO Control HANDLER
        # 


        return request.SUCCESS


def main():
    rospy.init_node('excavator_core_io_control_server', anonymous=True)
    try:
        io_control_server = IOControlServer()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()