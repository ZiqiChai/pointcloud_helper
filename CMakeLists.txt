cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_helper)
add_compile_options(-std=c++11)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(
  include
  /usr/include/eigen3/
)
############################## BUILD ##############################

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})


add_subdirectory(src/demo)
add_subdirectory(src/libs)
add_subdirectory(src/tools)

############################## executable ##############################
add_executable (merge_pointcloud_icp src/merge_pointcloud_icp.cpp)
target_link_libraries (merge_pointcloud_icp ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} config pointcloud_helper)
