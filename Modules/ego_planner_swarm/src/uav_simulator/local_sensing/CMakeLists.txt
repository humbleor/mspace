PROJECT(local_sensing_node)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
# SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
# SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
# SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

  find_package(Eigen3 REQUIRED)
  find_package(OpenCV REQUIRED)
  # find_package(Pangolin 0.4 REQUIRED)
  find_package(GLEW REQUIRED)
  find_package(OpenGL REQUIRED)
  find_package(glfw3 REQUIRED)
  find_package(catkin REQUIRED COMPONENTS
      roscpp roslib cmake_modules pcl_ros sensor_msgs geometry_msgs nav_msgs quadrotor_msgs cv_bridge)

      FIND_PACKAGE(Boost REQUIRED COMPONENTS filesystem iostreams program_options system serialization)
      if(Boost_FOUND)
        INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
        LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
      endif()

      set(CMAKE_CXX_STANDARD_REQUIRED ON)
      set(CMAKE_CXX_EXTENSIONS ON)
      set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")
      link_directories(${CMAKE_SOURCE_DIR}/lib)

  add_library(GLAD include/glad.c)
  set(LIBS ${LIBS} GLAD)

  catkin_package(
      DEPENDS Eigen
      CATKIN_DEPENDS roscpp roslib pcl_ros image_transport
  )

# add_compile_options(-pg)
# SET(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)

  include_directories(
    SYSTEM
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${GLEW_INCLUDE_DIRS}
    ${glfw3_INCLUDE_DIRS}
  )


  add_executable(
    opengl_render_node
    src/opengl_render_node.cpp
    include/ikd-Tree/ikd_Tree.cpp
    include/FOV_Checker/FOV_Checker.cpp
  )
 set(glfw3_LIBS /usr/lib/x86_64-linux-gnu/libglfw.so)
  target_link_libraries( opengl_render_node
    # include
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    # pango_display
    ${Pangolin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${glfw3_LIBS}
    ${LIBS}
  )

# ============================================================
# 激光雷达点云模拟器
# 
# 本节点提供激光雷达点云模拟功能：
# - 旧版本 pcl_render_node ：基础局部感知，基于KD-tree的简单半径搜索和过滤
# - 新版本 pointcloud_render_node ：增强功能，支持动态物体模拟、多机协同、多种雷达模式、
#           点云插值、深度图生成、碰撞检测和性能优化
# ============================================================

  add_executable(
    pointcloud_render_node
    src/pointcloud_render_node.cpp
    include/ikd-Tree/ikd_Tree.cpp
    include/FOV_Checker/FOV_Checker.cpp
    #src/raycast.cpp
  )

  target_link_libraries( pointcloud_render_node
    #include
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
  )

  add_executable(
    pcl_render_node
    src/pointcloud_render_node_old.cpp
  )

  target_link_libraries( pcl_render_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
  )


# if(ENABLE_CUDA)
#   find_package(CUDA REQUIRED)
#   SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3 -use_fast_math)
#   set(CUDA_NVCC_FLAGS 
# #       -gencode arch=compute_20,code=sm_20;
# #       -gencode arch=compute_20,code=sm_21;
# #       -gencode arch=compute_30,code=sm_30;
# #       -gencode arch=compute_35,code=sm_35;
# #       -gencode arch=compute_50,code=sm_50;
# #       -gencode arch=compute_52,code=sm_52; 
# #       -gencode arch=compute_60,code=sm_60;
# #       -gencode arch=compute_61,code=sm_61;
#       -gencode arch=compute_75,code=sm_75;
#   ) 

#   SET(CUDA_PROPAGATE_HOST_FLAGS OFF)

#   find_package(OpenCV REQUIRED)
#   find_package(Eigen3 REQUIRED)
#   find_package(Boost REQUIRED COMPONENTS system filesystem)
#   find_package(GLEW REQUIRED)
#   find_package(OpenGL REQUIRED)
#   find_package(glfw3 REQUIRED)
#   find_package(catkin REQUIRED COMPONENTS
#       roscpp roslib cmake_modules cv_bridge image_transport pcl_ros sensor_msgs geometry_msgs nav_msgs quadrotor_msgs cv_bridge dynamic_reconfigure)
#   generate_dynamic_reconfigure_options(
#     cfg/local_sensing_node.cfg
#   )
#   catkin_package(
#       DEPENDS OpenCV Eigen Boost
#       CATKIN_DEPENDS roscpp roslib image_transport pcl_ros
#   #INCLUDE_DIRS include
#       LIBRARIES depth_render_cuda
#   )

#   include_directories(
#     SYSTEM 
#     include
#     ${catkin_INCLUDE_DIRS}
#     ${Eigen_INCLUDE_DIRS}
#     ${OpenCV_INCLUDE_DIRS}
#     ${Pangolin_INCLUDE_DIRS}
#     ${OPENGL_INCLUDE_DIR}
#     ${GLEW_INCLUDE_DIRS}
#     ${glfw3_INCLUDE_DIRS}
#   )

#   CUDA_ADD_LIBRARY( depth_render_cuda
#       src/depth_render.cu
#   )

#   add_executable(
#     pcl_render_node
#     src/pcl_render_node.cpp
#   )
#   target_link_libraries( pcl_render_node
#     depth_render_cuda
#     ${OpenCV_LIBS}
#     ${Boost_LIBRARIES}
#     ${catkin_LIBRARIES}
#   )

