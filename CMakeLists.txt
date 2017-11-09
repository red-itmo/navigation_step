cmake_minimum_required(VERSION 2.8.3)
project(navigation_step)

find_package (catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	nav_msgs
	geometry_msgs
	move_base_msgs
	actionlib
	message_generation
	actionlib_msgs
	tf
)
#add_compile_options(-std=c++11)

find_package (Boost REQUIRED COMPONENTS filesystem)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
FILES
Twist.srv
Manual.srv
Dict.srv
PointData.srv
BasePoint.srv
SetOrientation.srv
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES Dest.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	actionlib_msgs geometry_msgs std_msgs
	# Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES agitr
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
		${catkin_INCLUDE_DIRS}
		src
		include)


## Declare a C++ library
# add_library(agitr
#   src/${PROJECT_NAME}/agitr.cpp
# )


## Declare a C++ executable
add_executable(navi_node src/main.cpp src/navigation.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(navi_node navigation_step_generate_messages_cpp)
#${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(navi_node
  ${catkin_LIBRARIES}
)
