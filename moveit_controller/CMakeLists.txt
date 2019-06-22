cmake_minimum_required(VERSION 2.8.3)
project(moveit_controller)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

set(THIS_PACKAGE_INCLUDE_DIRS
  interactivity/include
)

catkin_package(
  LIBRARIES
    interactivity_utils
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(${THIS_PACKAGE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_executable(moveit_controller src/moveit_controller.cpp)
target_link_libraries(moveit_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS moveit_controller DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})