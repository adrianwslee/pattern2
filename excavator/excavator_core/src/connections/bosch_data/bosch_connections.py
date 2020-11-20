#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from std_msgs.msg import String

from dasy_object_list_subscriber import DasyObjectListSubscriber
from svc_360_area_scan_subscriber import SVC360AreaScanSubscriber
from svc_adt_dumpbed_scan_subscriber import SVCAdtDumpbedScanSubscriber
from svc_adt_pose_detection_subscriber import SVCAdtPoseDetectionSubscriber
from svc_bucket_load_scan_subscriber import SVCBecketLoadScanSubscriber
from svc_current_pcd_map_25_subscriber import SVCCurrentPcdMap25Subscriber
from svc_digging_area_scan_subscriber import SVCDiggingAreaScanSubscriber
from svc_front_area_scan_subscriber import SVCFrontAreaScanSubscriber


class BoschConnectionManager:

    def __init__(self):
        self.__dasy_object_list_subscriber = DasyObjectListSubscriber()
        self.__svc_360_area_scan_subscriber = SVC360AreaScanSubscriber()
        self.__svc_adt_dumpbed_scan_subscriber = SVCAdtDumpbedScanSubscriber()
        self.__svc_adt_pose_detection_subscriber = SVCAdtPoseDetectionSubscriber()
        self.__svc_bucket_load_scan_subscriber = SVCBecketLoadScanSubscriber()
        self.__svc_current_pcd_map_25_subscriber = SVCCurrentPcdMap25Subscriber()
        self.__svc_digging_area_scan_subscriber = SVCDiggingAreaScanSubscriber()
        self.__svc_front_area_scan_subscriber = SVCFrontAreaScanSubscriber()
