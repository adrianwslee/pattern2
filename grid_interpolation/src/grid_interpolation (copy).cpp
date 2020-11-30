#include "../include/grid_interpolation/grid_interpolation.h"

class GridInterpolation
{
public:
  GridInterpolation(ros::NodeHandle &nh) : nhPrivate(nh)
  {
    pcdSubscriber = nhPrivate.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10, &GridInterpolation::pcdCb, this);
    gridPublisher = nhPrivate.advertise<sensor_msgs::PointCloud2>("/mesh_grid", 10);
  }
private:
  ros::NodeHandle nhPrivate;
  ros::Subscriber pcdSubscriber;
  ros::Publisher gridPublisher;

  pcl::PointCloud<pcl::PointXYZ>::Ptr inputPCL;
  double xMin, xMax, yMin, yMax, zMin, ZMax;

  void getParams()
  {
    nhPrivate.param<double>("xMin",xMin, 3.0);
    nhPrivate.param<double>("xMax",xMax, 10.0);
    nhPrivate.param<double>("yMin",yMin, -2.0);
    nhPrivate.param<double>("yMax",yMax, 2.0);
    nhPrivate.param<double>("zMin",zMin, -5.0);
    nhPrivate.param<double>("zMax",zMax, 10.0);
  }

  void pcdCb(const sensor_msgs::PointCloud2::ConstPtr & msg)
  {
    getParams();
    ROS_INFO("%f, %f, %f, %f, %f, %f", xMin, xMax, yMin, yMax, zMin, zMax);
  }

  bool checkPCD(const sensor_msgs::PointCloud2::ConstPtr& msg, pcl::PointCloud<pcl::PointXYZ>::Ptr& outPCL)
  {
    pcl::fromROSMsg(*msg, *outPCL);
    if(outPCL->is_dense == false)
    {
      ROS_ERROR("The data contains Nan. Please check sensor data");
    }
    if(outPCL->fields.empty())
    {
      ROS_ERROR("The data show no fields. Please chekc sensor data");
    }
  }
  bool filterROI(pcl::PointCloud<pcl::PointXYZ>::Ptr& sourceCloud)
  {
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(xMin,yMin,zMin,1.0));
    boxFilter.setMax(Eigen::Vector4f(xMax,yMax,zMax,1.0));
    boxFilter.setInputCloud(sourceCloud);
    boxFilter.filter(*sourceCloud);
  }
  bool meshPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr &sourceCloud)
  {
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    int xMinIndex = 0;
    int xMaxIndex = (xMax + 0.4) * 100;
    int yMinIndex = 0;
    int yMaxIndex = ((std::abs(yMin) + std::abs(yMax)) + 0.2) * 100;

    float height[xMaxIndex][yMaxIndex];
    bool heightCheck[xMaxIndex][yMaxIndex];
    bool triCheck[xMaxIndex][yMaxIndex];


    ////Initialize Arrays to 10s and falses
    for (int i = 0; i < xMaxIndex*yMaxIndex; i++)
    {
      *((float*)height + i) = 10;
      *((bool*)heightCheck + i) = false;
      *((bool*)triCheck + i) = false;
    } //End of initialization

    unsigned int xIndex, yIndex;
    float zValue;

    for(size_t i = 0; i < sourceCloud->points.size(); i++)
    {
      xIndex = static_cast<int>(std::round((sourceCloud->points[i].x * 100)));
      yIndex = static_cast<int>(std::round(((sourceCloud->points[i].y + yMax) * 100)));

      if (xIndex < xMinIndex || xIndex >= xMaxIndex || yIndex < yMinIndex || yIndex >= yMaxIndex)
      {
        continue;
      }
      else
      {
        if (heightCheck[xIndex][yIndex] == false)
        {
          height[xIndex][yIndex] = sourceCloud->points[i].z;
          heightCheck[xIndex][yIndex] = true;
        }
        else //If z values overlaps so I had to choose which value to use
        {

        }
      }
    }
  }
  bool interpolatePCD();
};
