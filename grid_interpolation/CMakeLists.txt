cmake_minimum_required(VERSION 2.8.3)
project(grid_interpolation)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  sensor_msgs
  pcl_ros
)
set(VTK_DIR "/home/src/vtk-master/vtk_build")
find_package(VTK COMPONENTS 
  vtkCommonCore
  vtkCommonDataModel
  vtkFiltersCore
  vtkFiltersExtraction
  vtkFiltersGeometry
  vtkFiltersCore
  vtkFiltersModeling
  vtkIOXML
  vtkIOLegacy
  vtkIOGeometry
  vtkInteractionStyle
  vtkRenderingCore
  vtkRenderingFreeType
  vtkRenderingOpenGL2 QUIET)
if (NOT VTK_FOUND)
  message("Skipping ConstrainedDelaunay2D: ${VTK_NOT_FOUND_MESSAGE}")
  return ()
endif()
message (STATUS "VTK_VERSION: ${VTK_VERSION}")

catkin_package(CATKIN_DEPENDS roscpp std_msgs tf pcl_ros)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/grid_interpolation.cpp)
add_executable(bucket_node src/bucket.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${VTK_LIBRARIES})
target_link_libraries(bucket_node ${catkin_LIBRARIES} ${VTK_LIBRARIES})

