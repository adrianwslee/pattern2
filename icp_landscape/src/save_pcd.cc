#include "../include/icp_landscape/save_pcd.h"

class SavePCD
{
public:
  SavePCD(ros::NodeHandle& nh) : nhPrivate(nh)
  {
    splitSourceSubscriber = nhPrivate.subscribe<sensor_msgs::PointCloud2>(
        "/velodyne_points2", 10, &SavePCD::sourceCloudCb, this);
    splitTargetSubscirber = nhPrivate.subscribe<sensor_msgs::PointCloud2>(
        "/velodyne_points", 10, &SavePCD::targetCloudCb, this);
    overlapPublisher = nhPrivate.advertise<sensor_msgs::PointCloud2>(
        "/filtered_overlap", 10);
    sourceFinalPublisher = nhPrivate.advertise<sensor_msgs::PointCloud2>("/tranformed_source", 10);
    targetFinalPublisher = nhPrivate.advertise<sensor_msgs::PointCloud2>("/transformed_target", 10);

    sourceRawPublisher = nhPrivate.advertise<sensor_msgs::PointCloud2>("/raw_source", 10);
    targetRawPublisher = nhPrivate.advertise<sensor_msgs::PointCloud2>("/raw_target", 10);

  }

  void publish() { landscapeICP(); }

private:
  ros::NodeHandle nhPrivate;
  ros::Subscriber splitSourceSubscriber;
  ros::Subscriber splitTargetSubscirber;
  ros::Publisher overlapPublisher;
  ros::Publisher sourceFinalPublisher;
  ros::Publisher targetFinalPublisher;

  ros::Publisher sourceRawPublisher;
  ros::Publisher targetRawPublisher;

  std::mutex lockMutex;

  pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePCL;
  pcl::PointCloud<pcl::PointXYZ>::Ptr targetPCL;
  pcl::PointCloud<pcl::PointXYZ>::Ptr outputPCL;
  pcl::PointCloud<pcl::PointXYZ>::Ptr sourceOverlap;
  pcl::PointCloud<pcl::PointXYZ>::Ptr targetOverlap;

  pcl::PointCloud<pcl::PointXYZ>::Ptr sourceVoxel;
  pcl::PointCloud<pcl::PointXYZ>::Ptr targetVoxel;

  sensor_msgs::PointCloud2 rawSourceMsg;
  sensor_msgs::PointCloud2 rawTargetMsg;

  bool checkSource = false;
  bool checkTarget = false;

  void sourceCloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    sourcePCL.reset(new pcl::PointCloud<pcl::PointXYZ>());
    sourceOverlap.reset(new pcl::PointCloud<pcl::PointXYZ>());
    sourceVoxel.reset(new pcl::PointCloud<pcl::PointXYZ>());
    checkPCD(msg, sourcePCL);

    voxelGrid(sourcePCL,sourceVoxel);
    overlapSection(sourcePCL, sourceOverlap);
    splitPCDRight(sourceVoxel);
    splitPCDRight(sourcePCL);
    pcl::toROSMsg(*sourcePCL, rawSourceMsg);
    sourceRawPublisher.publish(rawSourceMsg);
    checkSource = true;
  }

  void targetCloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    targetPCL.reset(new pcl::PointCloud<pcl::PointXYZ>());
    targetOverlap.reset(new pcl::PointCloud<pcl::PointXYZ>());
    targetVoxel.reset(new pcl::PointCloud<pcl::PointXYZ>());
    checkPCD(msg, targetPCL);
    voxelGrid(targetPCL, targetVoxel);
    overlapSection(targetPCL, targetOverlap);
    splitPCDLeft(targetVoxel);
    splitPCDLeft(targetPCL);
    pcl::toROSMsg(*targetPCL, rawTargetMsg);
    targetRawPublisher.publish(rawTargetMsg);
    checkTarget = true;
  }

  void checkPCD(const sensor_msgs::PointCloud2::ConstPtr& msg,
                pcl::PointCloud<pcl::PointXYZ>::Ptr& outPCL)
  {
    static int flagCheck = 0;
    pcl::fromROSMsg(*msg, *outPCL);
    std::vector<int> idx;
    // pcl::removeNaNFromPointCloud(*outPCL, *outPCL, idx);
    if (outPCL->is_dense == false)
    {
      ROS_ERROR("Contains NaN");
      ros::shutdown();
    }
    if (flagCheck == 0)
    {
      flagCheck = -1;
      for (size_t i = 0; i < msg->fields.size(); ++i)
      {
        flagCheck = 1;
        break;
      }
      if (flagCheck == -1)
      {
        ROS_ERROR("No field availble");
        ros::shutdown();
      }
    }
  }

  void overlapSection(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& overlapCloud)
  {
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(-2, -10, -10, 1.0));
    boxFilter.setMax(Eigen::Vector4f(2, 30, 10, 1.0));
    // boxFilter.setRotation(Eigen::Vector3f(0, 0, -M_PI * 30 /180));
    // boxFilter.setTranslation(Eigen::Vector3f(x,y,z);
    boxFilter.setInputCloud(inputCloud);
    boxFilter.filter(*overlapCloud);
  }

  void voxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &outputCloud)
  {
    pcl::VoxelGrid<pcl::PointXYZ> voxGridFilter;
    voxGridFilter.setInputCloud(inputCloud);
    voxGridFilter.setLeafSize(0.1f, 0.1f, 0.1f);
    voxGridFilter.filter (*outputCloud);
  }

  void splitPCDLeft(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud)
  {
    pcl::CropBox<pcl::PointXYZ> splitBox;
    splitBox.setMin(Eigen::Vector4f(-20, -10, -10, 1.0));
    splitBox.setMax(Eigen::Vector4f(2, 30, 10, 1.0));
    splitBox.setInputCloud(inputCloud);
    splitBox.filter(*inputCloud);
  }

  void splitPCDRight(pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud)
  {
    pcl::CropBox<pcl::PointXYZ> splitBox;
    splitBox.setMin(Eigen::Vector4f(-2, -10, -10, 1.0));
    splitBox.setMax(Eigen::Vector4f(20, 30, 10, 1.0));
    splitBox.setInputCloud(inputCloud);
    splitBox.filter(*inputCloud);
  }

  void landscapeICP()
  {
    if (checkSource == true && checkTarget == true)
    {
      std::unique_lock<std::mutex> lock(lockMutex);
      pcl::PointCloud<pcl::PointXYZ> finalCloud;
      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

      std::cout << sourcePCL->size() << " " << sourceVoxel->size() << std::endl;
      icp.setInputSource(sourceOverlap);
      icp.setInputTarget(targetOverlap);

      icp.align(finalCloud);
      std::cout << icp.hasConverged() << " score: " << icp.getFitnessScore()
                << "\n\n"
                << icp.getFinalTransformation() << std::endl;

      pcl::transformPointCloud(*sourceVoxel, *sourceVoxel, icp.getFinalTransformation());

      sensor_msgs::PointCloud2 overlapCloudMsg;
      sensor_msgs::PointCloud2 sourceFinalMsg;
      sensor_msgs::PointCloud2 targetFinalMsg;

      pcl::toROSMsg(finalCloud, overlapCloudMsg);
      pcl::toROSMsg(*targetVoxel, targetFinalMsg);
      pcl::toROSMsg(*sourceVoxel, sourceFinalMsg);

      overlapPublisher.publish(overlapCloudMsg);
      targetFinalPublisher.publish(targetFinalMsg);
      sourceFinalPublisher.publish(sourceFinalMsg);

      checkSource = false;
      checkTarget = false;
    }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "splitPCD");
  ros::NodeHandle nh("~");
  ros::NodeHandle nhPriv("~");
  SavePCD splitPCD(nhPriv);
  while(ros::ok())
  {
    splitPCD.publish();
    ros::spinOnce();
  }
}
