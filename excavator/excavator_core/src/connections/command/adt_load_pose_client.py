#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from doosan_msgs.srv import ADTLoadingPose, ADTLoadingPoseRequest, ADTLoadingPoseResponse

class ADTLoadPoseClient:

    def __init__(self):
        
        # rospy.wait_for_service('/xcenter/command/aoz_load_pose')

        self.__request_loading_pose = rospy.ServiceProxy('/xcenter/command/aoz_load_pose', ADTLoadingPose)


    # 
    #   input_pose  : geometry_msgs/Pose.msg
    # 
    def request_aoz_load_pose(self, input_pose):

        rospy.wait_for_service('/xcenter/command/aoz_load_pose')

        request = ADTLoadingPoseRequest()
        request.loading_pose = input_pose
        
        response = self.__request_loading_pose.call(request)
        # response = ADTLoadingPoseResponse()
        response.response



def main():
    rospy.init_node('excavator_core_adt_loading_pose_client', anonymous=True)
    try:
        adt_loading_pose_client = ADTLoadPoseClient()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()