cmake_minimum_required(VERSION 3.0.2)
project(trajectory_control_node)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs tf mavros_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(trajectory_control_node src/trajectory_control_node.cpp src/trajectory_control.cpp)

target_link_libraries(trajectory_control_node ${catkin_LIBRARIES})
