cmake_minimum_required(VERSION 2.8.3)
project(motoman_gazebo)
find_package(catkin REQUIRED COMPONENTS
    #actionlib_msg
    motoman_description
    motoman_point_cloud
    roslaunch
    )

catkin_package(
    CATKIN_DEPENDS
    #actionlib_msgs
    motoman_description
    motoman_point_cloud
    )

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare the executable, along with its source files.
add_executable(blocks_spawner src/blocks_spawner.cpp)
add_executable(cylinders_spawner src/cylinders_spawner.cpp)

# Specify libraries against which to link.
target_link_libraries(blocks_spawner ${catkin_LIBRARIES})
target_link_libraries(cylinders_spawner ${catkin_LIBRARIES})

# Install
foreach(dir launch models worlds)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# Test
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
