#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from std_msgs.msg import String

#from connections.command.adt_enter_confirm_server import ADTEnterConfirmServer
#from connections.command.adt_load_pose_client import ADTLoadPoseClient
#from connections.command.auto_mode_control_server import AutoModeControlServer
#from connections.command.geo_fence_server import GeoFenceServer
#from connections.command.instruction_handler import InstructionHandler
#from connections.command.io_control_server import IOControlServer
#from connections.command.work_mode_server import WorkModeServer

#from connections.diagnostics.control_state_publisher import ControlStatusPublisher
#from connections.diagnostics.machine_status_publisher import MachineStatusPublisher
#from connections.diagnostics.monitor_data_publisher import MonitorDataPublisher
#from connections.diagnostics.system_alert_publisher import SystemAlertPublisher
#from connections.diagnostics.system_fault_publiser import SystemFaultPublisher
#from connections.diagnostics.task_status_publisher import TaskStatusPublisher

#from connections.machine_info.gnss_publisher import GNSSPublisher
#from connections.machine_info.odom_info_publisher import OdomInfoPublisher
#from connections.machine_info.tf_info_publisher import TFInfoPublisher
#from connections.machine_info.weighing_info_publisher import WeighingInfoPublisher

#from connections.relay_data.relay_360_area_scan import Relay360AreaScan
#from connections.relay_data.relay_current_pcd_map_25 import RelayCurrentPcdMap25

#from connections.server_info.equipment_info_subscriber import EquipmentInfoSubscriber
#from connections.server_info.terrain_map_subscriber import TerrainMapSubscriber

#from connections.task.dig_task_server import DigTaskServer
##from connections.task.move_task_server import MoveTaskServer
#from connections.task.task_control_server import TaskControlServer

#from connections.bosch_data.bosch_connections import BoschConnectionManager

class ConnectionManager:

    def __init__(self):
        self.__init_connection()

        rospy.Subscriber('/excavator/event/test_trigger', String, self.__test_event_trigger, queue_size=1 )
        rospy.logdebug('[excavator_core] conntection created.')


    def __init_connection(self):

        self.__bosch_connections = BoschConnectionManager()

        self.__adt_enter_confirm_server = ADTEnterConfirmServer()
        self.__adt_load_pose_client = ADTLoadPoseClient()
        self.__auto_mode_control_server = AutoModeControlServer()
        self.__geo_fence_server = GeoFenceServer()
        self.__instruction_handler = InstructionHandler()
        self.__io_control_server = IOControlServer()
        self.__work_mode_server = WorkModeServer()

        self.__control_state_publisher = ControlStatusPublisher()
        self.__machine_status_publisher = MachineStatusPublisher()
        self.__monitor_data_publisher = MonitorDataPublisher()
        self.__system_alert_publisher = SystemAlertPublisher()
        self.__system_fault_publiser = SystemFaultPublisher()
        self.__task_status_publisher = TaskStatusPublisher()

        self.__gnss_publisher = GNSSPublisher()
        self.__odom_info_publisher = OdomInfoPublisher()
        self.__tf_info_publisher = TFInfoPublisher()
        self.__weighing_info_publisher = WeighingInfoPublisher()


        self.__relay_360_area_scan = Relay360AreaScan()
        self.__relay_current_pcd_map_25 = RelayCurrentPcdMap25()


        self.__equipment_info_subscriber = EquipmentInfoSubscriber()
        self.__terrain_map_subscriber = TerrainMapSubscriber()

        self.__dig_task_server = DigTaskServer()
        self.__move_task_server = MoveTaskServer()
        self.__task_control_server = TaskControlServer()


    def __test_event_trigger(self):
        self.__test_commands()
        self.__test_diagnostics()

    def __test_commands(self):
        from geometry_msgs.msg import Pose

        load_pose = Pose()
        self.__adt_load_pose_client.request_aoz_load_pose(load_pose)
        self.__instruction_handler.send_instruction('SVC', 'CONTROL', 'SYSTEM_ON','System control instruction')

    
    def __test_diagnostics(self):
        # self.__control_state_publisher.
        self.__machine_status_publisher = MachineStatusPublisher()
        self.__monitor_data_publisher = MonitorDataPublisher()
        self.__system_alert_publisher = SystemAlertPublisher()
        self.__system_fault_publiser = SystemFaultPublisher()
        self.__task_status_publisher = TaskStatusPublisher()
