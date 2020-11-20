#include "../include/icp_landscape/icp_landscape.h"

class ICPLandscape
{
public:
  ICPLandscape(ros::NodeHandle nh) : nhPrivate(nh)
  {
    firstPCDSubscriber = nhPrivate.subscribe<sensor_msgs::PointCloud2>("/lms_demo", 10, &ICPLandscape::firstPCDCb, this);
    secondPCDSubscriber = nhPrivate.subscribe<sensor_msgs::PointCloud2>("/lms_demo_2", 10, &ICPLandscape::secondPCDCb, this);
    finalCloudPublisher = nhPrivate.advertise<sensor_msgs::PointCloud2>("/icp_cloud", 10);
  }

private:
  ros::NodeHandle nhPrivate;
  ros::Subscriber firstPCDSubscriber;
  ros::Subscriber secondPCDSubscriber;
  ros::Publisher finalCloudPublisher;
  std::mutex lockMutex;

  bool checkFirst = false;
  bool checkSecond = false;

  pcl::PointCloud<pcl::PointXYZ>::Ptr firstPCD;
  pcl::PointCloud<pcl::PointXYZ>::Ptr secondPCD;

  void firstPCDCb(const sensor_msgs::PointCloud2::ConstPtr & msg)
  {
    firstPCD.reset(new pcl::PointCloud<pcl::PointXYZ>());
    checkPCD(msg, firstPCD);
    ROS_INFO("firstPCD received");
    checkFirst = true;
    landscapeICP();
  }

  void secondPCDCb(const sensor_msgs::PointCloud2::ConstPtr &msg)
  {
    secondPCD.reset(new pcl::PointCloud<pcl::PointXYZ>());
    checkPCD(msg, secondPCD);
    ROS_INFO("secondPCD received");
    checkSecond = true;
  }

  void landscapeICP()
  {
    if (checkFirst == true && checkSecond == true)
    {
      std::unique_lock<std::mutex> lock(lockMutex);
      pcl::PointCloud<pcl::PointXYZ> finalCloud;
      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      icp.setInputSource(firstPCD);
      icp.setInputTarget(secondPCD);

      icp.align(finalCloud);
      std::cout << icp.hasConverged() << " score: " << icp.getFitnessScore() << "\n\n" << icp.getFinalTransformation() << std::endl;

      sensor_msgs::PointCloud2 finalCloudMsg;
      pcl::toROSMsg(finalCloud, finalCloudMsg);
      finalCloudPublisher.publish(finalCloudMsg);
      checkFirst = false;
      checkSecond = false;
      ROS_INFO("Publish Success");

    }
  }

  /*
  void publishFinalPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr &finalCloud)
  {
    sensor_msgs::PointCloud2 finalCloudMsg;
    pcl::toROSMsg(*finalCloud, finalCloudMsg);
    finalCloudPublisher(finalCloudMsg);
  }*/

  void checkPCD(const sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud<pcl::PointXYZ>::Ptr &outPCL)
  {
    static int flagCheck = 0;
    pcl::fromROSMsg(*msg, *outPCL);
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(*outPCL, *outPCL, idx);
    if(outPCL->is_dense == false)
    {
      ROS_ERROR("Contains NaN");
      ros::shutdown();
    }
    if(flagCheck == 0)
    {
      flagCheck = -1;
      for(size_t i = 0; i < msg->fields.size();++i)
      {
        flagCheck = 1;
        break;
      }
      if(flagCheck == -1)
      {
        ROS_ERROR("No field availble");
        ros::shutdown();
      }
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "icp");
  ros::NodeHandle n("~");
  ros::NodeHandle nhPriv("~");
  ICPLandscape icpClouds(nhPriv);
  ros::spin();
}
