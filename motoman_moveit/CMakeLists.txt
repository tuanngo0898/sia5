cmake_minimum_required(VERSION 2.8.3)
project(motoman_moveit)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  moveit_msgs
  #moveit_commander
  moveit_core
  #moveit_ros_planning
  moveit_ros_planning_interface
  motoman_description
  pluginlib
  roscpp
  roslaunch
  )

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  LIBRARIES
  interactivity_utils
  INCLUDE_DIRS
  CATKIN_DEPENDS
  moveit_visual_tools
  interactive_markers
  tf2_geometry_msgs
  DEPENDS
  )

# Build
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

# Install
foreach(dir config launch scene)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

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

include_directories(${THIS_PACKAGE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_executable(rviz_belt_conveyor_spawner src/rviz_belt_conveyor_spawner.cpp)
target_link_libraries(rviz_belt_conveyor_spawner ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS rviz_belt_conveyor_spawner DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
