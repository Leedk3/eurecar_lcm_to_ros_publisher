cmake_minimum_required(VERSION 2.8.3)
project(eurecar_lcm_to_ros_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  ackermann_msgs
  message_generation
  pcl_ros
)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})


########################
## message generation ##
########################
add_message_files(FILES
  eurecar_can_t.msg
)

#add_service_files(FILES
#  wheel_odom_init.srv
#)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES eurecar_lcm_to_ros_publisher
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)


set( sources
  src/eurecar_lcm_to_ros_publisher.cpp
)


find_package(lcm REQUIRED)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${LCM_INCLUDE_DIRS}
)

add_library(eurecar_lcm_to_ros_publisher
  src/lcmtoros_can_t.cpp
  src/lcmtoros_pos_t.cpp
)

add_executable(${PROJECT_NAME}_node ${sources} )

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   eurecar_lcm_to_ros_publisher
   ${catkin_LIBRARIES}
   lcm
   ${PCL_LIBRARIES}
)


