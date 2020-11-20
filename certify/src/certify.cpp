#include "../include/certify/certify.h"

extern const int groundIdx = 10;
extern const int upperIdx = 10;

class PatternCertify
{
public:
  PatternCertify(ros::NodeHandle nh) : nhPrivate(nh)
  {
    pcdSubscriber = nhPrivate.subscribe<sensor_msgs::PointCloud2>(
        "/velodyne_points", 10, &PatternCertify::pcdCb, this);
    filteredPcdPublisher =
        nhPrivate.advertise<sensor_msgs::PointCloud2>("/filted_pcd", 10);
  }

private:
  ros::NodeHandle nhPrivate;
  ros::Subscriber pcdSubscriber;
  ros::Publisher filteredPcdPublisher;

  // PCD Related
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcdInRing;
  sensor_msgs::PointCloud2 cloudFinal;

  // TF Related
  tf2_ros::Buffer tfBuffer;

  void pcdCb(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    sensor_msgs::PointCloud2 baseFramePCD;
    if (transformPCDToBase(msg, baseFramePCD) == false)
    {
      return;
    }
    pcdInRing.reset(new pcl::PointCloud<pcl::PointXYZ>());
    if (checkPCD(baseFramePCD) == false)
    {
      return;
    }
    ROS_INFO("cloud Check Passed");

    cutPCD(4.0, 5.2, -0.7, 0.7, 0.0, 2.0);
    ROS_INFO("cutPCD success");
    convetToROSMsg();
    filteredPcdPublisher.publish(cloudFinal);
    pcdInRing->clear();
  }

  bool checkPCD(sensor_msgs::PointCloud2 msg)
  {
    static int flagCheck = 0;

    pcl::fromROSMsg(msg, *pcdInRing);
    std::vector<int> indx;
    pcl::removeNaNFromPointCloud(*pcdInRing, *pcdInRing, indx);
    if (pcdInRing->is_dense == false)
    {
      ROS_ERROR(
          "This PCD contains NaN Points. Please check sensor data to proceed");
      ros::shutdown();
    }
    if (flagCheck == 0)
    {
      flagCheck = -1;
      for (size_t i = 0; i < msg.fields.size(); ++i)
      {
        flagCheck = 1;
        break;
      }
      if (flagCheck == -1)
      {
        ROS_ERROR("Poing Cloud shows no valid fields. Please "
                  "Check your sensor data");
        ros::shutdown();
      }
    }
    return true;
  }

  void cutPCD(float xMin, float xMax, float yMin, float yMax, float zMin,
              float zMax)
  {
    pcl::CropBox<pcl::PointXYZ> cutBoxFilter;
    cutBoxFilter.setMin(Eigen::Vector4f(xMin, yMin, zMin, 1.0));
    cutBoxFilter.setMax(Eigen::Vector4f(xMax, yMax, zMax, 1.0));
    // cutBoxFilter.setNegative(true); true == leaves out side of the box;
    cutBoxFilter.setInputCloud(pcdInRing);
    cutBoxFilter.filter(*pcdInRing);
  }

  void convetToROSMsg() { pcl::toROSMsg(*pcdInRing, cloudFinal); }

  bool transformPCDToBase(const sensor_msgs::PointCloud2::ConstPtr& cloudIn,
                          sensor_msgs::PointCloud2& cloudOut)
  {
    sensor_msgs::PointCloud2 test;
    geometry_msgs::TransformStamped tfStamped;
    tf2_ros::TransformListener tfListener(tfBuffer);
    try
    {
      tfStamped = tfBuffer.lookupTransform("cs_ground", "velodyne",
                                           ros::Time(0), ros::Duration(3.0));
      tf2::doTransform(*cloudIn, cloudOut, tfStamped);
      return true;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("No tf available %s", ex.what());
      return false;
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "certifyNode");
  ros::NodeHandle nh("");
  ros::NodeHandle nhPrivate("~");
  PatternCertify patternBox(nhPrivate);
  ros::spin();
}
