#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from doosan_msgs.msg import EquipmentInfo, EquipmentInfoArray

class EquipmentInfoSubscriber:

    def __init__(self):
        
        self.__equipment_info_subscriber = rospy.Subscriber('/xcenter/info/equipments_info', EquipmentInfoArray, self.__update_equipment_info, queue_size=1 )


    # 
    #   equipments  : doosan_msgs/EquipmentInfo.msg     : ARRAY
    # 
    def __update_equipment_info(self, msg):
        equipments = msg.equipments

        # 
        #   TODO IMPLEMENTS EQUIPMENT INFO UPDATE CODE HERE
        #


def main():
    rospy.init_node('excavator_core_equipment_info_subscriber', anonymous=True)
    try:
        equipment_info_subscriber = EquipmentInfoSubscriber()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()