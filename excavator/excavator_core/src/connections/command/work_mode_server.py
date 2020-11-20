#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from doosan_msgs.srv import WorkMode, WorkModeRequest, WorkModeResponse
from doosan_msgs.msg import UserCommand

class WorkModeServer:

    def __init__(self):
        
        rospy.Service('/excavator/command/work_mode', WorkMode, self.__handle_work_mode)


    # 
    #   commands    : doosan_msgs/UserCommand.msg
    # 
    def __handle_work_mode(self, request):
        user_commands = request.commands

        # 
        #   TODO IMPLEMENTS WORK MODE HANDLER
        # 


        return request.SUCCESS


def main():
    rospy.init_node('excavator_core_work_mode_server', anonymous=True)
    try:
        work_mode_server = WorkModeServer()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()