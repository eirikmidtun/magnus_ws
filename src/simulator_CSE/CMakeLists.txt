cmake_minimum_required(VERSION 3.0.2)
project(simulator_CSE)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rospy
  std_msgs
)

catkin_package()