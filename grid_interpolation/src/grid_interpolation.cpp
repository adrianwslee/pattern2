#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/filters/passthrough.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <cmath>
#include <iostream>
#include <stdio.h>

#include <vector>
#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkDelaunay2D.h>
#include <vtkMath.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolygon.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>

class Publisher
{
private:
  tf::TransformListener base_velodyne_listener;
  ros::Subscriber velodyne_subscriber;
  ros::Publisher velodyne_points2;
  tf2_ros::Buffer tf_buffer_;

  double x_roi_min;
  double x_roi_max;
  double y_roi_min;
  double y_roi_max;
  double z_roi_min;
  double z_roi_max;

  void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    ros::WallTime start_, end_;
    //////Not used
    start_ = ros::WallTime::now();
    ros::Time target_time = ros::Time(0);
    std::string target_frame = "/cs_ground";
    sensor_msgs::PointCloud2 cloud_out;

    pcl::PointCloud<pcl::PointXYZ> cloud_in_pcl;
    pcl::PointCloud<pcl::PointXYZ> cloud_out_pcl;

    pcl::fromROSMsg(*msg, cloud_in_pcl);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_out_z(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_out_y(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_out_x(new pcl::PointCloud<pcl::PointXYZ>);

    *temp_cloud_in = cloud_in_pcl;

    ros::param::get("/x_roi_max", x_roi_max);
    ros::param::get("/x_roi_min", x_roi_min);
    ros::param::get("/y_roi_max", y_roi_max);
    ros::param::get("/y_roi_min", y_roi_min);
    ros::param::get("/z_roi_max", z_roi_max);
    ros::param::get("/z_roi_min", z_roi_min);

    ros::param::set("/x_roi_max", 11.0);
    ros::param::set("/x_roi_min", 3.0);
    ros::param::set("/y_roi_max", 1.5);
    ros::param::set("/y_roi_min", -1.5);
    ros::param::set("/z_roi_max", 0.5);
    ros::param::set("/z_roi_min", -5.0);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(temp_cloud_in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_roi_min, z_roi_max);
    pass.filter(*temp_cloud_out_z);

    pcl::PassThrough<pcl::PointXYZ> pass1;
    pass1.setInputCloud(temp_cloud_out_z);
    pass1.setFilterFieldName("y");
    pass1.setFilterLimits(y_roi_min, y_roi_max);
    pass1.filter(*temp_cloud_out_y);

    pcl::PassThrough<pcl::PointXYZ> pass2;
    pass2.setInputCloud(temp_cloud_out_y);
    pass2.setFilterFieldName("x");
    pass2.setFilterLimits(x_roi_min, x_roi_max);
    pass2.filter(*temp_cloud_out_x);

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    int x_min_index = 0;
    int x_max_index = x_roi_max * 100;
    int y_min_index = 0;
    int y_max_index = (std::abs(y_roi_min) + std::abs(y_roi_max)) * 100;
    //    ROS_WARN("y_max_index : %i", y_max_index);

    float height[x_max_index][y_max_index];
    bool height_check[x_max_index][y_max_index];
    bool tri_check[x_max_index][y_max_index];

    for (int i = 0; i < x_max_index; i++) // for(int i = 0; i < 110; i++)
    {
      for (int j = 0; j < y_max_index; j++) // for(int j = 0; j <30; j++)
      {
        height[i][j] = 1.0;
        height_check[i][j] = false;
        tri_check[i][j] = false;
      }
    }

    // ROS_ERROR("start1");

    int step_factor = 10;

    int x_index, y_index;
    float z_value;
    for (size_t i = 0; i < temp_cloud_out_x->points.size(); i++)
    {
      x_index = int(temp_cloud_out_x->points[i].x * 100);
      y_index = int((temp_cloud_out_x->points[i].y + y_roi_max) * 100);
      //        std::cout << x_index << std::endl;

      if (x_index < x_min_index || x_index >= x_max_index || y_index < y_min_index || y_index >= y_max_index)
      {
        continue;
      }
      else
      {
        if (height_check[x_index][y_index] == false)
        {
          z_value = temp_cloud_out_x->points[i].z;
          height[x_index][y_index] = z_value;
          height_check[x_index][y_index] = true;
        }
        else if (height_check[x_index][y_index] != false)
        {
          z_value = std::min(z_value, temp_cloud_out_x->points[i].z);
          height[x_index][y_index] = z_value;
          //          std::cout << z_value << " " << x_index << " " << y_index << std::endl;
        }
      }
    } // setting points for VTK

    for (int i = x_min_index; i < x_max_index; i += 2)
    {
      for (int j = y_min_index; j < y_max_index; j += 4) ///////j should be 2
      {
        if (height[i][j] != 1.0)
        {
          points->InsertNextPoint(i, j, height[i][j]);
          //            std::cout << i << " " << j << " " << height[i][j] << std::endl;
        }
      }
    }

    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(points);

    vtkSmartPointer<vtkDelaunay2D> delaunay = vtkSmartPointer<vtkDelaunay2D>::New();
    delaunay->SetInputData(polydata);
    delaunay->Update();

    vtkPolyData* apolydata = delaunay->GetOutput();
    vtkIdType npts, *pts;
    apolydata->GetPolys()->InitTraversal();

    struct vert_points
    {
      long long int vert1, vert2, vert3;
    };
    std::vector<vert_points> vertices;

    while (apolydata->GetPolys()->GetNextCell(npts, pts))
    {
      vertices.push_back(vert_points{pts[0], pts[1], pts[2]});
    }

    struct pointXYZ
    {
      double x, y, z;
    };
    std::vector<pointXYZ> allPoints;
    for (vtkIdType i = 0; i < apolydata->GetNumberOfPoints(); i++)
    {
      double p[3];
      apolydata->GetPoint(i, p);
      allPoints.push_back(pointXYZ{p[0], p[1], p[2]});
      //   std::cout << p[0] << " " << p[1] <<" " << p[2] << std::endl;
    }
    std::cout << allPoints.size() << std::endl;
    bool check_data = false;
    if (allPoints.size() == 0)
    {
      check_data = false;
      ROS_ERROR("No data received.");
    }
    else if (allPoints.size() > 50)
    {
      check_data = true;
    }
    else
    {
      check_data = false;
      ROS_ERROR("Too few Point Cloud Data. Wait");
    }

    // ROS_INFO("start2");

    /// Plane equation and put empty grids inside a triangles
    if (check_data)
    {
      for (int i = x_min_index; i < x_max_index; i += step_factor)
      {
        for (int j = y_min_index; j < y_max_index; j += step_factor)
        {
          if (height[i][j] == 1.0)
          {
            //           ROS_ERROR("%d ,%d", i , j);
            bool find_point = false;
            while (!find_point) /////////////IN FUTURE THIS MIGHT CAUSE A PROBLEM CHANGE THIS ACCORDINGLY TO YOUR ROI
            {
              // set triangles in a grid
              for (size_t index = 0; index < vertices.size(); index++)
              {
                //               std::cout << allPoints[vertices[index].vert1].x << allPoints[vertices[index].vert2].x << allPoints[vertices[index].vert3].x  << " " << index << " " << triangle_coord.size() << std::endl;
                double tri_area = 0.5 * (-allPoints[vertices[index].vert2].y * allPoints[vertices[index].vert3].x + allPoints[vertices[index].vert1].y * (-allPoints[vertices[index].vert2].x + allPoints[vertices[index].vert3].x) +
                                         allPoints[vertices[index].vert1].x * (allPoints[vertices[index].vert2].y - allPoints[vertices[index].vert3].y) + allPoints[vertices[index].vert2].x * allPoints[vertices[index].vert3].y);
                //               std::cout <<"Triangle area is: " << tri_area << std::endl;
                int sign_tri;
                if (tri_area < 0)
                {
                  sign_tri = -1;
                }
                if (tri_area > 0)
                {
                  sign_tri = 1;
                }
                if (tri_area == 0)
                {
                  ROS_ERROR("Somthing wrong");
                }

                auto s = (allPoints[vertices[index].vert1].y * allPoints[vertices[index].vert3].x - allPoints[vertices[index].vert1].x * allPoints[vertices[index].vert3].y + (allPoints[vertices[index].vert3].y - allPoints[vertices[index].vert1].y) * i +
                          (allPoints[vertices[index].vert1].x - allPoints[vertices[index].vert3].x) * j) *
                         sign_tri;
                auto t = (allPoints[vertices[index].vert1].x * allPoints[vertices[index].vert2].y - allPoints[vertices[index].vert1].y * allPoints[vertices[index].vert2].x + (allPoints[vertices[index].vert1].y - allPoints[vertices[index].vert2].y) * i +
                          (allPoints[vertices[index].vert2].x - allPoints[vertices[index].vert1].x) * j) *
                         sign_tri;

                if (s >= 0 && t >= 0 && (s + t) <= 2 * tri_area * sign_tri)
                {
                  float a1 = allPoints[vertices[index].vert2].x - allPoints[vertices[index].vert1].x;
                  float b1 = allPoints[vertices[index].vert2].y - allPoints[vertices[index].vert1].y;
                  float c1 = allPoints[vertices[index].vert2].z - allPoints[vertices[index].vert1].z;

                  float a2 = allPoints[vertices[index].vert3].x - allPoints[vertices[index].vert1].x;
                  float b2 = allPoints[vertices[index].vert3].y - allPoints[vertices[index].vert1].y;
                  float c2 = allPoints[vertices[index].vert3].z - allPoints[vertices[index].vert1].z;

                  float a = b1 * c2 - b2 * c1;
                  float b = a2 * c1 - a1 * c2;
                  float c = a1 * b2 - b1 * a2;
                  float d = (-a * allPoints[vertices[index].vert1].x - b * allPoints[vertices[index].vert1].y - c * allPoints[vertices[index].vert1].z);

                  height[i][j] = (-d - a * i - b * j) / c;
                  if (std::isnan(height[i][j] != 0))
                  {
                    tri_check[i][j] = true;
                  }
                  find_point = true;
                  break;
                }
                else if (index == vertices.size() - 1)
                {
                  find_point = true;
                  tri_check[i][j] = true;
                }
              }
            }
          }
        }
      }
    }
    // ROS_WARN("start3");
    for (int i = x_min_index; i < x_max_index; i += step_factor)
    {
      for (int j = y_min_index; j < y_max_index; j += step_factor)
      {
        if (tri_check[i][j] == true)
        {
          bool find_right = false;
          bool find_left = false;
          bool find_up = false;
          bool find_down = false;

          int delta_right = 0;
          int delta_left = 0;
          int delta_up = 0;
          int delta_down = 0;

          int delta_right_index = 0;
          int delta_left_index = 0;
          int delta_up_index = 0;
          int delta_down_index = 0;

          float right_value = 0.0;
          float left_value = 0.0;
          float up_value = 0.0;
          float down_value = 0.0;

          while (!find_left && ((j - delta_left_index) >= y_min_index))
          {
            if (height[i][j - delta_left_index] != 1.0)
            {
              find_left = true;
              left_value = height[i][j - delta_left_index];
              delta_left = delta_left_index;
            }
            delta_left_index++;
          }
          while (!find_right && ((j + delta_right_index) < y_max_index))
          {
            if (height[i][j + delta_right_index] != 1.0)
            {
              find_right = true;
              right_value = height[i][j + delta_right_index];
              delta_right = delta_right_index;
            }
            delta_right_index++;
          }
          if (find_left || find_right)
          {
            height[i][j] = (delta_left * left_value + delta_right * right_value) / (delta_left + delta_right);
            if (std::isnan(height[i][j]) != 0)
            {
              ROS_ERROR("Empty slot y: %d, %d", i, j);
            }
          }
          else
          {
            while (!find_up && ((i - delta_up_index) >= x_min_index))
            {
              if (height[i - delta_up_index][j] != 1.0)
              {
                find_up = true;
                up_value = height[i - delta_up_index][j];
                delta_up = delta_up_index;
              }
              delta_up_index++;
            }
            while (!find_down && ((i + delta_down_index) < x_max_index))
            {
              if (height[i + delta_down_index][j] != 1.0)
              {
                find_down = true;
                down_value = height[i + delta_down_index][j];
                delta_down = delta_down_index;
              }
              delta_down_index++;
            }
            if (find_up || find_down)
            {
              height[i][j] = (delta_up * up_value + delta_down * down_value) / (delta_up + delta_down);
              if (std::isnan(height[i][j]) != 0)
              {
                ROS_ERROR("Empty slot x: %d, %d", i, j);
              }
            }
          }
        }
      }
    }
    pcl::PointCloud<pcl::PointXYZ> cloud_grid;
    cloud_grid.width = (std::abs(x_roi_max - x_roi_min) * 10) * (std::abs(y_roi_max - y_roi_min) * 10); // Tmrw 2100 (ROI 4~11 -1.5 1.5) //2400
    cloud_grid.height = 1;
    cloud_grid.points.resize(cloud_grid.width * cloud_grid.height);
    int count_cloud = 0;
    for (int i = (x_roi_min * 100); i < x_max_index; i += step_factor) // change x_min_index to 40 (from ROI)
    {
      for (int j = y_min_index; j < y_max_index; j += step_factor)
      {
        float xi = i * 0.01;
        float yi = (j - 150) * 0.01;
        cloud_grid.points[count_cloud].x = xi;
        cloud_grid.points[count_cloud].y = yi;
        cloud_grid.points[count_cloud].z = height[i][j];
        //        std::cout << cloud_grid.points[count_cloud].x << " " << cloud_grid.points[count_cloud].y << " " << cloud_grid.points[count_cloud].z << std::endl;
        //        std::cout << cloud_grid.width << std::endl;
        count_cloud++;
      }
    }
    sensor_msgs::PointCloud2 cloud_out_final;
    pcl::toROSMsg(cloud_grid, cloud_out_final);
    cloud_out_final.header.frame_id = "/cs_ground";
    cloud_out_final.header.stamp = ros::Time(0);
    velodyne_points2.publish(cloud_out_final);

    //    end_ = ros::WallTime::now();
    //    double execution_time = (end_ - start_).toNSec() * 1e-6;
    //    ROS_INFO_STREAM("Execution time (ms): " << execution_time);
  }

public:
  Publisher(ros::NodeHandle& nh_priv)
  {
    velodyne_subscriber = nh_priv.subscribe<sensor_msgs::PointCloud2>("/pps/velodyne_points_removal", 10, &Publisher::velodyne_callback, this);
    velodyne_points2 = nh_priv.advertise<sensor_msgs::PointCloud2>("/pps/velodyne_points2", 10);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pps_velodyne_points2");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_priv("~");
  Publisher publisher(nh_priv);
  ros::spin();
  return 0;
}
