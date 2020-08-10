cmake_minimum_required(VERSION 3.0.2)
project(fbow_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
#  message_generation
  cv_bridge
  image_transport
)


find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV 3 REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

set(${fbow_INCLUDE_DIRS} "/usr/local/include")

##  Boost
SET(Boost_USE_STATIC_LIBS ON)
SET(Boost_USE_MULTITHREADED ON)
find_package(Boost REQUIRED COMPONENTS program_options)
include_directories(${Boost_INCLUDE_DIRS})


#generate_messages(
#  DEPENDENCIES
#    geometry_msgs
#    sensor_msgs
#    std_msgs
#)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    sensor_msgs
#    message_runtime
    std_msgs
    cv_bridge
    image_transport
)




include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}  
  ${fbow_INCLUDE_DIRS}
)


########################## System nodes ###########################

add_executable(fbow_ros_core_node
  src/fbow_ros_core.cpp
)
target_link_libraries(fbow_ros_core_node
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${fbow_INCLUDE_DIRS}
  /usr/local/lib/libfbow.so /usr/local/lib/libfbow.so.0.0  /usr/local/lib/libfbow.so.0.0.1 )
add_dependencies(fbow_ros_core_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

######################## Test nodes ###########################


############################################################


