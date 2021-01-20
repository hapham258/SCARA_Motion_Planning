cmake_minimum_required(VERSION 2.8.3)
project(model)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    tf
    urdf
    xacro
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(tf_publisher src/publish_tf.cpp)
target_link_libraries(tf_publisher ${catkin_LIBRARIES})
