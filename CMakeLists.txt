cmake_minimum_required(VERSION 3.0.2)
project(moonranger_kinematics)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)


catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_add_nosetests(tests/test_two_wheel.py)
# catkin_add_nosetests(tests/test_four_wheel.py)

catkin_python_setup()