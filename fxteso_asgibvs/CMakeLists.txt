cmake_minimum_required(VERSION 3.0.2)
project(fxteso_asgibvs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gazebo_msgs
  geometry_msgs
  image_transport
  mavros_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
CATKIN_DEPENDS cv_bridge gazebo_msgs geometry_msgs image_transport nav_msgs roscpp sensor_msgs std_msgs tf2 mavros_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_3_INCLUDE_DIRS}
)

#IBVS-Adaptive controller
add_executable(aibvs_controller_node src/aibvs_pos_ctrl.cpp)
target_link_libraries(aibvs_controller_node	${catkin_LIBRARIES})

#Attitude controller
add_executable(aibvs_attitude_node src/att_ctrl.cpp)
target_link_libraries(aibvs_attitude_node	${catkin_LIBRARIES})

#Image features processing
add_executable(aibvs_imfeat_node src/image_features.cpp)
target_link_libraries(aibvs_imfeat_node	${catkin_LIBRARIES})
target_link_libraries(aibvs_imfeat_node  ${OpenCV_LIBRARIES})

#Target position
add_executable(aibvs_tgtPos_node src/target_position.cpp)
target_link_libraries(aibvs_tgtPos_node	${catkin_LIBRARIES})

#FxTESO
add_executable(aibvs_fxteso_node src/fixed_eso.cpp)
target_link_libraries(aibvs_fxteso_node	${catkin_LIBRARIES})

#UAV dynamics
add_executable(aibvs_uav_node src/uav_dynamics.cpp)
target_link_libraries(aibvs_uav_node	${catkin_LIBRARIES})

#TD Attitude 
add_executable(aibvs_td_att_node src/td_attitude.cpp)
target_link_libraries(aibvs_td_att_node ${catkin_LIBRARIES})

#TD Desired attitude 
add_executable(aibvs_td_att_des_node src/td_attitude_desired.cpp)
target_link_libraries(aibvs_td_att_des_node ${catkin_LIBRARIES})

#Disturbances
add_executable(aibvs_dist_node src/disturbances.cpp)
target_link_libraries(aibvs_dist_node	${catkin_LIBRARIES})
