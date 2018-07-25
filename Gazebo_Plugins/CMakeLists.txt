cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)


find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(cmake_modules REQUIRED)
SET (CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../CMakeModules)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

#add_library(actor_custom SHARED ActorPlugin.cc)
#add_library(minion_actuator SHARED ActuatorPlugin.cc)
add_library(piston_plugin SHARED piston_plugin.cc)

target_link_libraries(
piston_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES}
#actor_custom ${GAZEBO_LIBRARIES}
#minion_actuator ${GAZEBO_LIBRARIES}
${catkin_LIBRARIES}
)
