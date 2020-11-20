#!/usr/bin/env python
# -*- coding: utf-8 -*-

import threading
import math
from math import sin, cos, pi

import rospy
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class OdomInfoPublisher:

    def __init__(self):
        self.__odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.__odom_broadcaster = tf.TransformBroadcaster()

        self.__cmd_vel_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.__test_update_velocity, queue_size = 1)

        self.__current_time = rospy.Time.now()
        self.__last_time = rospy.Time.now()

        self.__x = 0.0
        self.__y = 0.0
        self.__th = 0.0

        self.__vx = 0.0
        self.__vy = 0.0
        self.__vth = 0.0

        if rospy.has_param('IS_FAKE_MODE'):
            is_fake_mode = rospy.get_param('IS_FAKE_MODE')
            rospy.logdebug('[OdometryPublisher] fake mode : ' + str(is_fake_mode))

            if is_fake_mode:
                # For test mode.
                thread = threading.Thread(target=self.__update_odom_by_velocity)
                thread.start()
            else:
                thread = threading.Thread(target=self.__update_odom_by_gnss)
                thread.start()

        else :
            rospy.logdebug('[OdometryPublisher] can not find IS_FAKE_MODE parameter... skipped! ')

            

    def __update_odom_by_gnss(self):
        #TODO implements...
        pass


    def __update_odom_by_velocity(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.__current_time = rospy.Time.now()

            dt = (self.__current_time - self.__last_time).to_sec()
            delta_x = (self.__vx * cos(self.__th) - self.__vy * sin(self.__th)) * dt
            delta_y = (self.__vx * sin(self.__th) + self.__vy * cos(self.__th)) * dt
            delta_th = self.__vth * dt

            self.__x += delta_x
            self.__y += delta_y
            self.__th += delta_th

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.__th)

            # first, we'll publish the transform over tf
            self.__odom_broadcaster.sendTransform(
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0, 1.0),
                rospy.Time.now(),
                "base_footprint",
                # "base_link"
                "cs_ground"
            )
            tf.TransformBroadcaster().sendTransform(
                (self.__x, self.__y, 0.),
                odom_quat,
                self.__current_time,
                # "base_link",
                "cs_ground",
                "odom"
            )


            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = self.__current_time
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(self.__x, self.__y, 0.), Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "cs_ground"
            odom.twist.twist = Twist(Vector3(self.__vx, self.__vy, 0), Vector3(0, 0, self.__vth))

            # publish the message
            self.__odom_publisher.publish(odom)
            self.__last_time = self.__current_time
            
            rate.sleep()


    def __test_update_velocity(self, twist):
        self.__vx = twist.linear.x
        self.__vy = twist.linear.y
        self.__vth = twist.angular.z


def main():
    rospy.init_node('excavator_core_odom_info_publisher')
    try:
        odom_info_publisher = OdomInfoPublisher()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()