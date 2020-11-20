#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from doosan_msgs.srv import DigTask, DigTaskRequest, DigTaskResponse

class DigTaskServer:

    def __init__(self):
        
        rospy.Service('/excavator/task/dig_task', DigTask, self.__handle_dig_task)


    # 
    #   task_id         : uint32
    #   work_boundary   : pcl_msgs/PolygonMesh.msg
    #   dump_option     : doosan_msgs/DumpOption.msg
    #   work_quility    : uint16
    # 
    def __handle_dig_task(self, request):
        task_id = request.task_id
        work_boundary = request.work_boundary
        dump_option = request.dump_option
        work_quility = request.work_quility

        # 
        #   TODO IMPLEMENTS DIG TASK HANDLER
        # 


        return request.SUCCESS


def main():
    rospy.init_node('excavator_core_dig_task_server', anonymous=True)
    try:
        dig_task_server = DigTaskServer()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()