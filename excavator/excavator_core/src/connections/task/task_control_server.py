#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from doosan_msgs.srv import TaskControl, TaskControlRequest, TaskControlResponse

class TaskControlServer:

    def __init__(self):
        
        rospy.Service('/excavator/task/task_control', TaskControl, self.__handle_task_control)


    # 
    #   task_control_type   : uint8     : Defined at doosan_msgs/TaskControl.srv
    #   task_id             : uint32
    # 
    def __handle_task_control(self, request):
        task_control_type = request.task_control_type
        task_id = request.task_id

        # 
        #   TODO IMPLEMENTS MOVE TASK HANDLER
        # 


        return request.SUCCESS


def main():
    rospy.init_node('excavator_core_task_control_server', anonymous=True)
    try:
        task_control_server = TaskControlServer()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()