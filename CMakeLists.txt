cmake_minimum_required(VERSION 2.8.3)

project(drone_control)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
	nodes/control_node
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

