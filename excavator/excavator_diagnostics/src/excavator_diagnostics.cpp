#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>


bool lms_on, dynamixel_on, can_display_on, can_jointstate_on;
bool rs_485_on;
unsigned short lms_pollution_level;
sensor_msgs::JointState temp_jt_msg;

/////Diagnostics_msgs::DiagnosticStatus === diag
///
/// Pollution ERROR / WARN
/// LMS Ethernet ON / OFF || IP ERROR?
/// Dynamixel ON / OFF || FORCED STOP ERROR || NO SENSOR DATA
/// CAN Display CAN ON / OFF
/// CAN JointState CAN ON / OFF
///
///

void setDiagnosisMsg(diagnostic_msgs::DiagnosticStatus* diag, uint8_t level,
                     std::string name, std::string message,
                     std::string hardware_id)
{
  diag->level = level;
  diag->name = name;
  diag->message = message; // NOT AVAILABLE BLOCKED BLOCKED BLOCKED RUNNING
  diag->hardware_id = hardware_id;
}

void set_level_Pollution(int pollution_level, diagnostic_msgs::DiagnosticStatus *diag)
{
  if(pollution_level == 0)
  {
    setDiagnosisMsg(diag, diagnostic_msgs::DiagnosticStatus::OK, "LMS::POLLUTION", "CLEAN", "LMS");
  }
  else if (pollution_level > 0 && pollution_level  <= 3)
  {
    setDiagnosisMsg(diag, diagnostic_msgs::DiagnosticStatus::WARN, "LMS::POLLUTION", "WARNING", "LMS");
  }
  else if (pollution_level > 3)
  {
    setDiagnosisMsg(diag, diagnostic_msgs::DiagnosticStatus::ERROR, "LMS::POLLUTION", "ERROR", "LMS");
  }
  else
  {
    ROS_WARN("No Pollution Level Set");
  }
}

void set_level_LMSConnection(bool lms_connection, diagnostic_msgs::DiagnosticStatus *diag)
{
  if(lms_connection)
  {
    setDiagnosisMsg(diag, diagnostic_msgs::DiagnosticStatus::OK, "LMS::CONNECTION", "CONNECTED", "LMS");
  }
  else
  {
    setDiagnosisMsg(diag, diagnostic_msgs::DiagnosticStatus::ERROR, "LMS::CONNECTION", "DISCONNECTED", "LMS");
  }
}

void set_level_DynamixelConnection(bool dynamixel_connection, diagnostic_msgs::DiagnosticStatus *diag)
{
  if(dynamixel_connection)
  {
    setDiagnosisMsg(diag, diagnostic_msgs::DiagnosticStatus::OK, "DYNAMIXEL::CONNECTION", "CONNECTED", "DYNAMIXEL");
  }
  else
  {
    setDiagnosisMsg(diag, diagnostic_msgs::DiagnosticStatus::ERROR, "DYNAMIXEL::CONNECTION", "DISCONNECTED", "DYNAMIXEL");
  }
}

void set_level_Controller(bool lms_connection, bool dynamixel_connection, diagnostic_msgs::DiagnosticStatus *diag)
{
  if(lms_connection == false && dynamixel_connection == false)
  {
    setDiagnosisMsg(diag, diagnostic_msgs::DiagnosticStatus::OK, "CONTROLLER::POWER", "OFF", "CONTROLLER");
  }
  else
  {
    setDiagnosisMsg(diag, diagnostic_msgs::DiagnosticStatus::ERROR, "CONTROLLER::POWER", "ON", "CONTROLLER");
  }
}

void set_level_ControllerConnection(bool lms_connection, diagnostic_msgs::DiagnosticStatus *diag)
{
  if(lms_connection)
  {
    setDiagnosisMsg(diag, diagnostic_msgs::DiagnosticStatus::OK, "CONTROLLER::SENSOR", "CONNECTED", "CONTROLLER");
  }
  else
  {
    setDiagnosisMsg(diag, diagnostic_msgs::DiagnosticStatus::ERROR, "CONTROLLER::SENSOR", "DISCONNECTED", "CONTROLLER");
  }
}

void set_level_CAN_Display(bool can_display_connection, diagnostic_msgs::DiagnosticStatus *diag)
{
  if(can_display_connection)
  {
    setDiagnosisMsg(diag, diagnostic_msgs::DiagnosticStatus::OK, "CAN::DISPLAY", "CONNECTED", "DISPLAY");
  }
  else
  {
    setDiagnosisMsg(diag, diagnostic_msgs::DiagnosticStatus::ERROR, "CAN::DISPLAY", "DISCONNECTED", "DISPLAY");
  }
}

void set_level_CAN_JointState(bool can_jointstate_connection, diagnostic_msgs::DiagnosticStatus *diag)
{
  if(can_jointstate_connection)
  {
    setDiagnosisMsg(diag, diagnostic_msgs::DiagnosticStatus::OK, "CAN::JOINTSTATE", "CONNECTED", "CAN_JOINT");
  }
  else
  {
    setDiagnosisMsg(diag, diagnostic_msgs::DiagnosticStatus::ERROR, "CAN::JOINTSTATE", "DISCONNECTED", "CAN_JOINT");
  }
}


void pollution_stat_cb (const std_msgs::Int16::ConstPtr& msg)
{
  lms_pollution_level = msg->data;
}

void lms_stat_cb (const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  if (msg->data.size() > 0)
  {
    lms_on = true;
  }
  else
  {
    lms_on = false;
  }
}

void dynamixel_stat_cb (const sensor_msgs::JointState::ConstPtr& msg)
{
  temp_jt_msg = *msg;

  if (msg != NULL)
  {
    dynamixel_on = true;
  }
  else
  {
    dynamixel_on = false;
  }
}

void can_display_stat_cb (const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data == true)
  {
    can_display_on = true;
  }
  else
  {
    can_display_on = false;
  }
}

void can_jointstate_stat_cb (const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data == true)
  {
    can_jointstate_on = true;
  }
  else
  {
    can_jointstate_on = false;
  }
}

int main(int argc, char** argv)
{
  lms_pollution_level = -1;
  lms_on = false;
  dynamixel_on = false;
  can_display_on = false;
  can_jointstate_on = false;

  ros::Publisher diagnostics_pub;

  ros::init(argc, argv, "diagnostic");
  ros::NodeHandle nh;

  diagnostics_pub =
      nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);

  ros::Subscriber pollution_stat_sub =
      nh.subscribe("/pollution_status", 10, pollution_stat_cb);

  ros::Subscriber lms_stat_sub =
      nh.subscribe("/raw_cloud1", 10, lms_stat_cb);

  ros::Subscriber dynamixel_stat_sub =
      nh.subscribe("/dynamixel_workbench/joint_states", 10, dynamixel_stat_cb);

  ros::Subscriber can_display_stat_sub =
      nh.subscribe("/can/display", 10, can_display_stat_cb);

  ros::Subscriber can_jointstate_stat_sub = nh.subscribe("/can/joint", 10, can_jointstate_stat_cb);

  ros::Rate loop_rate(10);

  int save_count = 0;
  sensor_msgs::JointState old_dynamixel_joint_msg;
  old_dynamixel_joint_msg.position.resize(1);
  old_dynamixel_joint_msg.velocity.resize(1);
  old_dynamixel_joint_msg.position[0] = 0.0;
  old_dynamixel_joint_msg.velocity[0] = 0.0;


  while (ros::ok())
  {

    diagnostic_msgs::DiagnosticArray pattern_diagnostics;
    pattern_diagnostics.header.stamp = ros::Time::now();

    diagnostic_msgs::DiagnosticStatus pollution_state;
    diagnostic_msgs::DiagnosticStatus lms_connection_state;
    diagnostic_msgs::DiagnosticStatus dynamixel_connection_state;
    diagnostic_msgs::DiagnosticStatus controller_state;
    diagnostic_msgs::DiagnosticStatus controller_connection_state;
    diagnostic_msgs::DiagnosticStatus can_display_state;
    diagnostic_msgs::DiagnosticStatus can_joint_state;

    if(dynamixel_on && temp_jt_msg.position.size() != 0 && temp_jt_msg.velocity.size() != 0)
    {
      if(save_count == 1)
      {
        old_dynamixel_joint_msg = temp_jt_msg;
      }
      if(save_count > 30)
      {
        if(old_dynamixel_joint_msg.position[0] != 0)
        {
          if(std::fabs(old_dynamixel_joint_msg.position[0] - temp_jt_msg.position[0]) > 0 && std::fabs(old_dynamixel_joint_msg.velocity[0] - temp_jt_msg.velocity[0]) > 0)
          {
            rs_485_on = true;
          }
          else
          {
            rs_485_on = false;
          }
        }
        save_count = 0;
      }
    }

    set_level_Pollution(lms_pollution_level, &pollution_state);
    set_level_LMSConnection(lms_on, &lms_connection_state);
    set_level_DynamixelConnection(rs_485_on, &dynamixel_connection_state);
    set_level_Controller(lms_on, rs_485_on, &controller_state);
    set_level_ControllerConnection(lms_on, &controller_connection_state);
    set_level_CAN_Display(can_display_on, &can_display_state);
    set_level_CAN_JointState(can_jointstate_on, &can_joint_state);


    pattern_diagnostics.status.clear();
    pattern_diagnostics.status.push_back(pollution_state);
    pattern_diagnostics.status.push_back(lms_connection_state);
    pattern_diagnostics.status.push_back(dynamixel_connection_state);
    pattern_diagnostics.status.push_back(controller_state);
    pattern_diagnostics.status.push_back(controller_connection_state);
    pattern_diagnostics.status.push_back(can_display_state);
    pattern_diagnostics.status.push_back(can_joint_state);

    diagnostics_pub.publish(pattern_diagnostics);

    lms_pollution_level = -1;
    lms_on = false;
    dynamixel_on = false;
    can_display_on = false;
    can_jointstate_on = false;


    save_count++;
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}
