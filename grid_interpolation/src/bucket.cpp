#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/filters/passthrough.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <Eigen/Geometry>

#include <pcl/kdtree/kdtree_flann.h>
#include <stdio.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/io.h>
#include <pcl/console/parse.h>
#include <pcl/io/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>

//VTK
#include <vtkSmartPointer.h>
#include <vtkDelaunay2D.h>
#include <vtkCellArray.h>
#include <vtkProperty.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolygon.h>
#include <vtkMath.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkAppendPolyData.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkCleanPolyData.h>
#include <vtkGenericDataObjectReader.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkExtractSelection.h>
#include <vtkIdTypeArray.h>
#include <vtkUnstructuredGrid.h>

class Publisher
{
private:
  tf::TransformListener base_velodyne_listener;
  ros::Subscriber velodyne_subscriber;
  ros::Publisher velodyne_points2;
  tf2_ros::Buffer tf_buffer_;
  bool vtp_save;
  bool calculate_volume;
  float bucket_volume_m3;

  double x_roi_min;
  double x_roi_max;
  double y_roi_min;
  double y_roi_max;
  double z_roi_min;
  double z_roi_max;

  void save_to_vtp(const vtkSmartPointer<vtkPolyData> polydata)
  {
     vtkSmartPointer<vtkXMLPolyDataWriter> writer =
      vtkSmartPointer<vtkXMLPolyDataWriter>::New();
    writer->SetFileName("empty_bucket1.vtp");
    writer->SetInputData(polydata);
    writer->SetDataModeToAscii();
    writer->Write();
  }

  void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    ros::Time target_time = ros::Time(0);
    std::string target_frame = "/cs_ground";
    sensor_msgs::PointCloud2 cloud_out;
    tf::StampedTransform transform;
    try
    {
      base_velodyne_listener.waitForTransform("/cs_ground", "/velodyne_link", target_time, ros::Duration(3.0));
      base_velodyne_listener.lookupTransform("/cs_ground", "/velodyne_link", target_time, transform);
    }
    catch (tf::LookupException &e)
    {
      ROS_ERROR("%s", e.what());
      ros::Duration(1.0).sleep();
    }
    catch (tf::ExtrapolationException &e)
    {
      ROS_ERROR("%s", e.what());
      ros::Duration(1.0).sleep();
    }
    pcl_ros::transformPointCloud(target_frame, transform, *msg, cloud_out);
    pcl::PointCloud<pcl::PointXYZ> cloud_in_pcl;
    pcl::PointCloud<pcl::PointXYZ> cloud_out_pcl;
    pcl::fromROSMsg(cloud_out, cloud_in_pcl);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_out_z (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_out_y (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_out_x (new pcl::PointCloud<pcl::PointXYZ>);

    *temp_cloud_in = cloud_in_pcl;

    ros::param::get("/x_roi_max", x_roi_max);
    ros::param::get("/x_roi_min", x_roi_min);
    ros::param::get("/y_roi_max", y_roi_max);
    ros::param::get("/y_roi_min", y_roi_min);
    ros::param::get("/z_roi_max", z_roi_max);
    ros::param::get("/z_roi_min", z_roi_min);

    ros::param::set("/x_roi_max", 4.75);
    ros::param::set("/x_roi_min", 3.3);
    ros::param::set("/y_roi_max", 0.65);
    ros::param::set("/y_roi_min", -0.75);
    ros::param::set("/z_roi_max", 2.0);
    ros::param::set("/z_roi_min", 0.5);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(temp_cloud_in);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (z_roi_min, z_roi_max);
    pass.filter (*temp_cloud_out_z);

    pcl::PassThrough<pcl::PointXYZ> pass1;
    pass1.setInputCloud(temp_cloud_out_z);
    pass1.setFilterFieldName ("y");
    pass1.setFilterLimits (y_roi_min, y_roi_max);
    pass1.filter (*temp_cloud_out_y);

    pcl::PassThrough<pcl::PointXYZ> pass2;
    pass2.setInputCloud(temp_cloud_out_y);
    pass2.setFilterFieldName ("x");
    pass2.setFilterLimits (x_roi_min, x_roi_max);
    pass2.filter (*temp_cloud_out_x);

    //Transforming cloud so those are orthogonal
/*    float theta = 0.509833;
    Eigen::Affine3f transform_bucket = Eigen::Affine3f::Identity();
    transform_bucket.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));
//    std::cout << transform_bucket.matrix() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_bucket (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*temp_cloud_out_x, *transformed_bucket, transform_bucket);
*/

    int x_min_index = x_roi_min * 100;
    int x_max_index = x_roi_max * 100 +1;
    int y_min_index = 0;
    int y_max_index = (std::abs(y_roi_min) + std::abs(y_roi_max)) * 100 + 1;
    int x_value, y_value;
    float z_value;

    int step_factor = 2;

    float height[x_max_index][y_max_index];
    bool height_check[x_max_index][y_max_index];
    bool trIangle_check[x_max_index][y_max_index];

    for(int i = 0; i < x_max_index; i++)
    {
      for(int j = 0; j < y_max_index; j++)
      {
        height[i][j] = -123;
        height_check[i][j] = false;
        trIangle_check[i][j] = false;
      }
    }

    std::cout << "y_max_index: " << y_max_index << std::endl;

    for(size_t i = 0; i < temp_cloud_out_x->points.size(); i++)
    {
      x_value = int (temp_cloud_out_x->points[i].x * 100);
      y_value = int ((temp_cloud_out_x->points[i].y + std::abs(y_roi_min)) * 100);

      if(x_value < x_min_index || x_value >= x_max_index || y_value < y_min_index || y_value >= y_max_index)
      {
        continue;
      }
      else
      {
        if(height_check[x_value][y_value] == false)
        {
          z_value = temp_cloud_out_x->points[i].z;
          height[x_value][y_value] = z_value;
          height_check[x_value][y_value] = 111;
        }
        else if(height_check[x_value][y_value] != false)
        {
          z_value = std::min(z_value, temp_cloud_out_x->points[i].z);
          height[x_value][y_value] = z_value;
        }
      }
    }

    vtkSmartPointer<vtkPoints> points =
        vtkSmartPointer<vtkPoints>::New();

    ROS_WARN("Making triangles");

    for(int i = x_min_index; i < x_max_index; i+=2)
    {
      for(int j = y_min_index; j < y_max_index; j+=2)
      {
        if(height[i][j] != -123)
        {
          points->InsertNextPoint(i, j, height[i][j]);
        }
      }
    }

    vtkSmartPointer<vtkPolyData> polydata =
        vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(points);

    vtkSmartPointer<vtkDelaunay2D> delaunay = vtkSmartPointer<vtkDelaunay2D>::New();
    delaunay->SetInputData(polydata);
    delaunay->Update();

    vtkPolyData* apolydata = delaunay->GetOutput();
    vtkIdType npts, *pts;
    apolydata->GetPolys()->InitTraversal();

    struct vert_points {long long int vert1, vert2, vert3;};
    std::vector<vert_points> vertices;

    while(apolydata->GetPolys()->GetNextCell(npts, pts))
    {
      vertices.push_back(vert_points{pts[0], pts[1], pts[2]});
    }

    struct pointXYZ {double x,y,z;};
    std::vector<pointXYZ> allPoints;
    for(vtkIdType i =0; i < apolydata->GetNumberOfPoints(); i++)
    {
      double p[3];
      apolydata -> GetPoint(i,p);
      allPoints.push_back(pointXYZ{p[0], p[1], p[2]});
   //   std::cout << p[0] << " " << p[1] <<" " << p[2] << std::endl;
    }
    std::cout << allPoints.size() << std::endl;
    bool check_data = false;
    if(allPoints.size() == 0)
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
      ROS_ERROR("Too less Point Cloud Data. Wait");
    }

    ROS_WARN("Making points");

    if(check_data)
    {
      for(int i = x_min_index; i < x_max_index; i++)
      {
        for(int j = y_min_index; j < y_max_index; j++)
        {
          if(height[i][j] == -123)
          {
            bool find_point = false;
            while(!find_point)
            {
              for(size_t index = 0; index < vertices.size(); index++)
              {
                double trIangle_area = 0.5 * (-allPoints[vertices[index].vert2].y * allPoints[vertices[index].vert3].x + allPoints[vertices[index].vert1].y * (-allPoints[vertices[index].vert2].x + allPoints[vertices[index].vert3].x) + allPoints[vertices[index].vert1].x * (allPoints[vertices[index].vert2].y -allPoints[vertices[index].vert3].y) + allPoints[vertices[index].vert2].x * allPoints[vertices[index].vert3].y);
                int sign_triangle;
                if(trIangle_area < 0)
                {
                  sign_triangle = -1;
                }
                if(trIangle_area > 0)
                {
                  sign_triangle = 1;
                }
                if(trIangle_area == 0)
                {
                  ROS_ERROR("Something went wrong");
                }

                auto s = (allPoints[vertices[index].vert1].y * allPoints[vertices[index].vert3].x - allPoints[vertices[index].vert1].x * allPoints[vertices[index].vert3].y + (allPoints[vertices[index].vert3].y - allPoints[vertices[index].vert1].y) * i + (allPoints[vertices[index].vert1].x - allPoints[vertices[index].vert3].x) * j) * sign_triangle;
                auto t = (allPoints[vertices[index].vert1].x * allPoints[vertices[index].vert2].y - allPoints[vertices[index].vert1].y * allPoints[vertices[index].vert2].x + (allPoints[vertices[index].vert1].y - allPoints[vertices[index].vert2].y) * i + (allPoints[vertices[index].vert2].x - allPoints[vertices[index].vert1].x) * j) * sign_triangle;

                if(s >= 0 && t >= 0 && (s+t) <= 2 * trIangle_area * sign_triangle)
                {
 //                 ROS_ERROR("Inside a triangle!");
                  float a1 = allPoints[vertices[index].vert2].x - allPoints[vertices[index].vert1].x;
                  float b1 = allPoints[vertices[index].vert2].y - allPoints[vertices[index].vert1].y;
                  float c1 = allPoints[vertices[index].vert2].z - allPoints[vertices[index].vert1].z;

                  float a2 = allPoints[vertices[index].vert3].x - allPoints[vertices[index].vert1].x;
                  float b2 = allPoints[vertices[index].vert3].y - allPoints[vertices[index].vert1].y;
                  float c2 = allPoints[vertices[index].vert3].z - allPoints[vertices[index].vert1].z;

                  float a = b1 * c2 - b2 * c1;
                  float b = a2 * c1 - a1 * c2;
                  float c = a1 * b2 - b1 * a2;
                  float d = (-a * allPoints[vertices[index].vert1].x - b*allPoints[vertices[index].vert1].y - c * allPoints[vertices[index].vert1].z);

                  height[i][j] = (-d -a*i - b*j) / c;
                  if(std::isnan(height[i][j]) != 0)
                  {
                    trIangle_check[i][j] = true;
                  }
                  find_point = true;
                  break;
                }

                else if(index == vertices.size()-1)
                {
                  find_point = true;
                  trIangle_check[i][j] = true;
                }
              }
            }
          }
        }
      }
    }
    ROS_WARN("start3");
   for(int i = x_min_index; i< x_max_index; i++)
    {
      for(int j = y_min_index;  j < y_max_index; j++)
      {
        if(trIangle_check[i][j] == true)
        {
          bool find_right= false;
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

          while(!find_left && ((j - delta_left_index) >= y_min_index))
          {
            if(height[i][j-delta_left_index] != -123)
            {
              find_left= true;
              left_value = height[i][j-delta_left_index];
              delta_left = delta_left_index;
            }
            delta_left_index++;
          }
          while(!find_right && ((j+delta_right_index) < y_max_index))
          {
            if(height[i][j + delta_right_index] != -123)
            {
              find_right = true;
              right_value = height[i][j+delta_right_index];
              delta_right = delta_right_index;
            }
            delta_right_index++;
          }

          if(find_left || find_right)
          {
            height[i][j] = (delta_left * left_value + delta_right * right_value) / (delta_left + delta_right);
            if(std::isnan(height[i][j]) != 0)
            {
//              std::cout << "Empty slot " << i << " " << j << std::endl;
              height[i][j] = 123;
            }
          }
          else
          {
            while(!find_up && ((i-delta_up_index) >= x_min_index))
            {
              if(height[i-delta_up_index][j] != -123)
              {
                find_up = true;
                up_value = height[i-delta_up_index][j];
                delta_up = delta_up_index;
              }
              delta_up_index++;
            }

            while(!find_down && ((j+delta_down_index) < x_max_index))
            {
              if(height[i+delta_down_index][j] != -123)
              {
                find_down = true;
                down_value = height[i+delta_down_index][j];
                delta_down = delta_down_index;
              }
              delta_down_index++;
            }
            if(find_up || find_down)
            {
              height[i][j] = (delta_up * up_value + delta_down * down_value) / (delta_up + delta_down);
              if(std::isnan(height[i][j] != 0))
              {
//                std::cout << "Empty slot " << i << " " << j << std::endl;
                height[i][j] = 123;
              }
            }
          }
        }
      }
    }
    ROS_WARN("Done making");
    pcl::PointCloud<pcl::PointXYZ> cloud_grid;
    vtkSmartPointer<vtkPoints> vtp_points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkPolyData> vtp_poly = vtkSmartPointer<vtkPolyData>::New();

    cloud_grid.width = (std::abs(x_roi_max-x_roi_min)*100 +1)*(std::abs(y_roi_max-y_roi_min)*100 +1); //2511 31*81
    cloud_grid.height = 1;
    cloud_grid.points.resize (cloud_grid.width * cloud_grid.height);
    int count_cloud = 0;
    for(int i = (x_roi_min*100); i < x_max_index; i++) //change x_min_index to 40 (from ROI)
    {
      for(int j = y_min_index; j < y_max_index; j++)
      {
        float xi = i * 0.01;
        float yi = (j - (std::abs(y_roi_min)*100)) * 0.01;
        cloud_grid.points[count_cloud].x = xi;
        cloud_grid.points[count_cloud].y = yi;
        cloud_grid.points[count_cloud].z = height[i][j];
        vtp_points->InsertNextPoint(xi, yi, height[i][j]);
  //        std::cout << cloud_grid.points[count_cloud].x << " " << cloud_grid.points[count_cloud].y << std::endl;
  //        std::cout << cloud_grid.width << std::endl;
        count_cloud++;
      }
    }

    vtp_poly -> SetPoints(vtp_points);

    if(vtp_save == true)
    {
      std::cout << "extracting empty bucket" << std::endl;
      save_to_vtp(vtp_poly);
    }

    sensor_msgs::PointCloud2 cloud_out_final;
    pcl::toROSMsg(cloud_grid, cloud_out_final);
    cloud_out_final.header.frame_id = "/cs_ground";
    cloud_out_final.header.stamp = ros::Time::now();
    velodyne_points2.publish(cloud_out_final);

    if(calculate_volume == true)
    {
      std::string empty_bucket = "/home/nvidia/potenit_ws/empty_bucket.vtp";
      vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
      reader->SetFileName(empty_bucket.c_str());
      reader->Update();

      vtkPolyData* empty_poly = reader->GetOutput();
      vtkSmartPointer<vtkPoints> empty_points = vtkSmartPointer<vtkPoints>::New();
      vtkIdType NumPoints = empty_poly->GetNumberOfPoints();
      if(!(NumPoints >0))
      {
        ROS_ERROR("Empty file");
      }
      struct bucket_XYZ {double e_x, e_y, e_z;};
      std::vector<bucket_XYZ> bucket_empty_coords;
      std::vector<bucket_XYZ> bucket_flat_coords;
      double bucket_volume = 0;
      double bucket_volume_final = 0;
      for(vtkIdType k = 0; k < empty_poly->GetNumberOfPoints(); k++)
      {
        double bpts[3];
        empty_poly->GetPoint(k,bpts);
        bucket_empty_coords.push_back(bucket_XYZ{bpts[0], bpts[1], bpts[2]});
      }
      for(vtkIdType r = 0; r < vtp_poly->GetNumberOfPoints(); r++)
      {
        double bpts[3];
        vtp_poly->GetPoint(r, bpts);
        bucket_flat_coords.push_back(bucket_XYZ{bpts[0], bpts[1], bpts[2]});
      }

      int min_vector_size = std::min(bucket_empty_coords.size(), bucket_flat_coords.size());
      for(size_t i = 0; i < min_vector_size; i++)
      {
        if(bucket_flat_coords[i].e_x == bucket_empty_coords[i].e_x && bucket_flat_coords[i].e_y == bucket_empty_coords[i].e_y && bucket_flat_coords[i].e_z != 123 && bucket_empty_coords[i].e_z != 123)
        {
          bucket_volume = std::abs(bucket_flat_coords[i].e_z - bucket_empty_coords[i].e_z);
          bucket_volume_final = bucket_volume_final + bucket_volume;
          bucket_volume_m3 = bucket_volume_final * 0.0001; //Unit conversion
        }
      }
      std::cout << "Excavated volume is: " << bucket_volume_m3 << "m^3" << std::endl;
    }
  }
public:

  Publisher(ros::NodeHandle& nh_priv)
  {
    velodyne_subscriber = nh_priv.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &Publisher::velodyne_callback, this);
    velodyne_points2 = nh_priv.advertise<sensor_msgs::PointCloud2>("/pps/velodyne_points_bucket", 1);
    nh_priv.getParam("empty_bucket", vtp_save);
    nh_priv.setParam("empty_bucket", false);
    nh_priv.getParam("bucket_volume", calculate_volume);
    nh_priv.setParam("bucket_volume", false);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bucket_volume");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_priv("~");
  Publisher publisher(nh_priv);
  ros::spin();
  return 0;
}
