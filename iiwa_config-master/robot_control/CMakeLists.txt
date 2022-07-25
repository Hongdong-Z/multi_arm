cmake_minimum_required(VERSION 3.5)
project(robot_control)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  rviz_visual_tools
  geometry_msgs
  moveit_msgs
  sensor_msgs
  cmake_modules
  eigen_conversions
  tf
  tf_conversions
  #mav_state_machine
)
add_message_files(DIRECTORY msg FILES 
  dispatch_result.msg
  dispatch_non_traj.msg
  )
add_service_files(
  FILES
  control_service.srv
  moveit_service.srv
  symbolic_maneuver.srv
)
#add_action_files(
#  DIRECTORY action
#  FILES
#  tracking.action
#)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
  moveit_msgs
  sensor_msgs
  trajectory_msgs
)

catkin_package(

CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp rospy std_msgs

)


include_directories(
  include/robot_control
#  ../mav_state_machine/include/state_machine
  ${catkin_INCLUDE_DIRS}
)


add_library(traj_coordinator SHARED src/traj_coordinator.cpp)
add_dependencies(traj_coordinator ${${PROJECT_NAME}_EXPORTED_TARGETS} )#${catkin_EXPORTED_TARGETS} panda_control_generate_messages_cpp)

add_library(traj_worker SHARED src/traj_worker.cpp)
add_dependencies(traj_worker ${${PROJECT_NAME}_EXPORTED_TARGETS} )#${catkin_EXPORTED_TARGETS} panda_control_generate_messages_cpp)


add_executable(nonblocking_dual_driver src/nonblocking_dual_driver.cpp)
target_link_libraries(nonblocking_dual_driver
  traj_coordinator
  traj_worker 
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
