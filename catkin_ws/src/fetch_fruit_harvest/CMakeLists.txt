cmake_minimum_required(VERSION 2.8.3)
project(fetch_fruit_harvest)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(gazebo REQUIRED)

find_package(catkin
  REQUIRED
    angles
    control_toolbox
    gazebo_ros
    robot_controllers
    robot_controllers_interface
)
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    control_toolbox
    gazebo_ros
    robot_controllers
    robot_controllers_interface
  DEPENDS
    gazebo
  LIBRARIES
    fetch_gazebo_plugin
    fetch_grasping
  INCLUDE_DIRS
    include
)

add_library(fetch_gazebo_plugin src/plugin.cpp)
target_link_libraries(fetch_gazebo_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

### Build grasp_planner_node
add_executable(fetch_grasp_planner_node 
  src/grasp_planner_node.cpp
  src/grasp_planner.cpp)
target_link_libraries(fetch_grasp_planner_node
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(fetch_grasp_planner_node grasping_msgs_generate_messages_cpp)

install(
  PROGRAMS
    scripts/pick_place_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS fetch_gazebo_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  TARGETS fetch_grasp_planner_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS
    scripts/prepare_simulated_robot.py
    scripts/prepare_simulated_robot_pick_place.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config include launch robots worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

