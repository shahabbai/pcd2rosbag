cmake_minimum_required(VERSION 3.0.2)
project(read_pcd)


add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rosbag
  rosbag_storage
  sensor_msgs
  std_msgs
)

# Find PCL
find_package(PCL 1.10 REQUIRED)

# Package configuration
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs rosbag_storage
)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Add definitions
add_definitions(${PCL_DEFINITIONS})

# Create executable
add_executable(read_pcd src/read_pcd.cpp)

# Add compile flags to the specific target
target_compile_features(read_pcd PRIVATE cxx_std_14)
target_compile_options(read_pcd PRIVATE -O3 -march=native -pthread)

# Link libraries
target_link_libraries(read_pcd
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
