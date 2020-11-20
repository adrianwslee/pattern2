#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import PointCloud2

class TerrainMapSubscriber:

    def __init__(self):
        
        self.__terrain_map_subscriber = rospy.Subscriber('/xcenter/info/terrain_map', PointCloud2, self.__update_terrain_map, queue_size=1 )


    # 
    #   sensor_msgs/PointCloud2.msg
    # 
    #   std_msgs/Header header
    #   uint32 height
    #   uint32 width
    #   sensor_msgs/PointField[] fields
    #   bool is_bigendian
    #   uint32 point_step
    #   uint32 row_step
    #   uint8[] data
    #   bool is_dense
    # 
    def __update_terrain_map(self, msg):
        
        # 
        #   TODO IMPLEMENTS TERRAIN MAP UPDATE CODE HERE
        #
        pass


def main():
    rospy.init_node('excavator_core_terrain_map_subscriber', anonymous=True)
    try:
        terrain_map_subscriber = TerrainMapSubscriber()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()