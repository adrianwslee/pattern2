#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from doosan_msgs.srv import GeoFenceInfo, GeoFenceInfoRequest, GeoFenceInfoResponse
from doosan_msgs.msg import GeoFence

class GeoFenceServer:

    def __init__(self):
        
        rospy.Service('/excavator/command/geo_fence', GeoFenceInfo, self.__handle_geo_fence_info)


    # 
    #   count       : uint16
    #   geo_fences  : doosan_msgs/GeoFence.msg
    # 
    def __handle_geo_fence_info(self, request):
        count = request.count
        geo_fences = request.geo_fences

        # 
        #   TODO IMPLEMENTS GEO FENCE INFO HANDLER
        # 


        return request.SUCCESS


def main():
    rospy.init_node('excavator_core_geo_fence_server', anonymous=True)
    try:
        geo_fence_server = GeoFenceServer()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()