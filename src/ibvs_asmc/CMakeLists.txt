cmake_minimum_required(VERSION 3.0.2)
project(ibvs_asmc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gazebo_msgs
  geometry_msgs
  image_transport
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
)


find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
CATKIN_DEPENDS cv_bridge gazebo_msgs geometry_msgs image_transport nav_msgs roscpp sensor_msgs std_msgs tf2
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_3_INCLUDE_DIRS}
)

add_executable(cam_coord_node src/camera_coordinates.cpp)
target_link_libraries(cam_coord_node ${catkin_LIBRARIES})
target_link_libraries(cam_coord_node ${OpenCV_LIBRARIES})

add_executable(asmc_feat_node src/asmc_feat.cpp)
target_link_libraries(asmc_feat_node ${catkin_LIBRARIES})

add_executable(IBVS_attitude_ctrl_node src/attitude_controller.cpp)
target_link_libraries(IBVS_attitude_ctrl_node ${catkin_LIBRARIES})

add_executable(IBVS_uav_dynamics_node src/uav_dynamics_ibvs.cpp)
target_link_libraries(IBVS_uav_dynamics_node ${catkin_LIBRARIES})

add_executable(gazebo_broadcasting src/quad_gazebo_broadcaster.cpp)
target_link_libraries(gazebo_broadcasting ${catkin_LIBRARIES})

add_executable(flag_node src/Flag.cpp)
target_link_libraries(flag_node ${catkin_LIBRARIES})

add_executable(cam_sim_node src/camera_sim.cpp)
target_link_libraries(cam_sim_node ${catkin_LIBRARIES})
