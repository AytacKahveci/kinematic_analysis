cmake_minimum_required(VERSION 3.0.2)
project(kinematic_analysis)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  kdl_parser
  roscpp
  tf
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES kinematic_analysis
 CATKIN_DEPENDS geometry_msgs kdl_parser roscpp
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(forward_kinematics src/forward_kinematics.cpp)
target_link_libraries(forward_kinematics ${catkin_LIBRARIES})

add_executable(inverse_kinematics src/inverse_kinematics.cpp)
target_link_libraries(inverse_kinematics ${catkin_LIBRARIES})

add_executable(moveit_planner src/moveit_planner.cpp)
target_link_libraries(moveit_planner ${catkin_LIBRARIES})