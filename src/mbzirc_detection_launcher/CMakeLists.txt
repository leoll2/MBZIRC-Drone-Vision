cmake_minimum_required(VERSION 2.8.3)
project(mbzirc_detection_launcher)

find_package(catkin REQUIRED COMPONENTS
	darknet_ros
)

find_package(zed_wrapper)

catkin_package()

include_directories(
	# include
 	${catkin_INCLUDE_DIRS}
)

