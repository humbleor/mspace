cmake_minimum_required(VERSION 3.0.2)
project(drone_detect_lidar)

set(CMAKE_CXX_STANDARD 14)
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
  message_generation
)

## Find system libraries
find_package(Eigen3 REQUIRED)
## Drop explicit find_package(PCL) — pcl_ros (catkin component above)
## already pulls in PCL 1.10 headers and libs via ${catkin_INCLUDE_DIRS}
## and ${catkin_LIBRARIES}.  Calling find_package(PCL) here would resolve
## to the custom 1.12 install at /home/mspace/pcl-1.12, creating a
## header/library version mismatch.
find_package(GTSAM QUIET)

if(GTSAM_FOUND)
  message(STATUS "GTSAM found")
  add_definitions(-DHAVE_GTSAM)
else()
  message(WARNING "GTSAM not found. tree_graph_node will not be built.")
endif()

###################################
## catkin specific configuration ##
###################################

## Generate custom messages
add_message_files(
  FILES
  Tree.msg
  TreeDetection.msg
  TreeRelativePose.msg
  TreeRelativePoseOptimized.msg
  DriftCorrection.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}_tree_detector
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    pcl_ros
    message_filters
    tf2
    message_runtime
  DEPENDS
    EIGEN3
    PCL
    GTSAM
)

###########
## Build ##
###########

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Definitions
## PCL_NO_PRECOMPILE: force implicit template instantiation for PointXYZ.
## Both PCL 1.10 and 1.12 only pre-compile PCLBase/Filter/Search/VoxelGrid
## for specific point types (PointXYZRGB, PointXYZI, etc.), not plain PointXYZ.
## Without this flag, template method bodies are invisible and the linker
## fails with undefined vtable references.
add_definitions(
  -DEIGEN_NO_DEBUG
  -DPCL_NO_PRECOMPILE
)

## Tree detector library
add_library(${PROJECT_NAME}_tree_detector
  src/tree_detector.cpp
)
target_link_libraries(${PROJECT_NAME}_tree_detector
  ${catkin_LIBRARIES}
)

## Executables
add_executable(tree_detector_node src/tree_detector_node.cpp src/patchworkpp.cpp)
target_link_libraries(tree_detector_node
  ${PROJECT_NAME}_tree_detector
  ${catkin_LIBRARIES}
)
add_dependencies(tree_detector_node
  ${PROJECT_NAME}_tree_detector
  ${catkin_EXPORTED_TARGETS}
)

add_executable(tree_loc_node src/tree_loc_node.cpp)
target_link_libraries(tree_loc_node
  ${catkin_LIBRARIES}
)
add_dependencies(tree_loc_node
  ${catkin_EXPORTED_TARGETS}
)

if(GTSAM_FOUND)
  add_executable(tree_graph_node src/tree_graph_node.cpp)
  target_link_libraries(tree_graph_node
    ${catkin_LIBRARIES}
    gtsam
  )
  add_dependencies(tree_graph_node
    ${catkin_EXPORTED_TARGETS}
  )
endif()

#############
## Install ##
#############

## Install targets
install(TARGETS
  ${PROJECT_NAME}_tree_detector
  tree_detector_node
  tree_loc_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(GTSAM_FOUND)
  install(TARGETS
    tree_graph_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

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
