cmake_minimum_required(VERSION 2.8.3)
project(import_manipulate_modul)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  rviz_visual_tools
  geometry_msgs
  cmake_modules
  eigen_conversions
  tf
  tf_conversions
  #mav_state_machine
)

# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
# catkin_python_setup()

# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
)

catkin_package(

#  LIBRARIES obstacle_avoidance
  CATKIN_DEPENDS message_generation roscpp rospy std_msgs

)

include_directories(
  include/import_manipulate_modul
#  ../mav_state_machine/include/state_machine
  ${catkin_INCLUDE_DIRS}
)


#add_library(import_manipulate_k_modul SHARED src/import_manipulate_k_modul.cpp)
#add_dependencies(import_manipulate_k_modul ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(imkm src/import_manipulate_modul.cpp)
target_link_libraries(imkm
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)


