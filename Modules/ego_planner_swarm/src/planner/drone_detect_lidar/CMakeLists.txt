cmake_minimum_required(VERSION 3.0.2)
project(drone_detect_lidar)

## Compile as C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Set build type
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Find catkin and dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  message_filters
  tf2
  tf2_ros
  eigen_conversions
)

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree search registration)
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}_feature_extractor
    ${PROJECT_NAME}_icp_wrapper
    ${PROJECT_NAME}_pose_fusion
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    pcl_ros
    message_filters
    tf2
  DEPENDS
    EIGEN3
    PCL
    OpenCV
)

###########
## Build ##
###########

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
)

## Definitions
add_definitions(
  ${PCL_DEFINITIONS}
  -DEIGEN_NO_DEBUG
)

## Libraries
add_library(${PROJECT_NAME}_feature_extractor
  src/feature_extractor.cpp
)
target_link_libraries(${PROJECT_NAME}_feature_extractor
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

add_library(${PROJECT_NAME}_icp_wrapper
  src/icp_wrapper.cpp
)
target_link_libraries(${PROJECT_NAME}_icp_wrapper
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

add_library(${PROJECT_NAME}_pose_fusion
  src/pose_fusion.cpp
)
target_link_libraries(${PROJECT_NAME}_pose_fusion
  ${PROJECT_NAME}_feature_extractor
  ${PROJECT_NAME}_icp_wrapper
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

## Executables
add_executable(feature_extractor_node src/feature_extractor_node.cpp)
target_link_libraries(feature_extractor_node
  ${PROJECT_NAME}_feature_extractor
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(relative_loc_node src/relative_loc_node.cpp)
target_link_libraries(relative_loc_node
  ${PROJECT_NAME}_pose_fusion
  ${PROJECT_NAME}_feature_extractor
  ${PROJECT_NAME}_icp_wrapper
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

## Install targets
install(TARGETS
  ${PROJECT_NAME}_feature_extractor
  ${PROJECT_NAME}_icp_wrapper
  ${PROJECT_NAME}_pose_fusion
  feature_extractor_node
  relative_loc_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
