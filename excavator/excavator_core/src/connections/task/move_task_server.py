#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from doosan_msgs.srv import MoveTask, MoveTaskRequest, MoveTaskResponse

class MoveTaskServer:

    def __init__(self):
        
        rospy.Service('/excavator/task/move_task', MoveTask, self.__handle_move_task)


    # 
    #   task_id         : uint32
    #   destination     : geometry_msgs/Pose.msg
    #   way_point       : geometry_msgs/Point.msg   : ARRAY
    #   boundary        : geometry_msgs/Polygon.msg
    # 
    def __handle_move_task(self, request):
        task_id = request.task_id
        destination = request.destination
        way_point = request.way_point
        boundary = request.boundary

        # 
        #   TODO IMPLEMENTS MOVE TASK HANDLER
        # 


        return request.SUCCESS


def main():
    rospy.init_node('excavator_core_move_task_server', anonymous=True)
    try:
        move_task_server = MoveTaskServer()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()