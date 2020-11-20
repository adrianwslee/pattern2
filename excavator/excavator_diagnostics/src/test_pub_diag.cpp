#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_pub");
  ros::NodeHandle n;
  ros::Publisher P3D_system_pub = n.advertise<std_msgs::String>("/P3D/P3D_SYSTEM", 1000);
  ros::Publisher P3D_app_pub = n.advertise<std_msgs::String>("/P3D/P3D_APP", 1000);
  ros::Publisher P3D_current_map_25_pub = n.advertise<std_msgs::String>("/P3D/CURRENT_PCD_MAP_25", 1000);
  ros::Publisher P3D_front_area_scan_pub = n.advertise<std_msgs::String>("/P3D/FRONT_AREA_SCAN", 1000);
  ros::Publisher P3D_digging_area_scan_pub = n.advertise<std_msgs::String>("/P3D/DIGGING_AREA_SCAN", 1000);


  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "potenit " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    P3D_system_pub.publish(msg);
    P3D_app_pub.publish(msg);
    P3D_current_map_25_pub.publish(msg);
    P3D_front_area_scan_pub.publish(msg);
    P3D_digging_area_scan_pub.publish(msg);


    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

