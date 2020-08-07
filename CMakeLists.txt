cmake_minimum_required(VERSION 3.0.2)

project(gazebo_simulation)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS nav_msgs roscpp rospy std_msgs tf)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

