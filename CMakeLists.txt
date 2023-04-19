cmake_minimum_required(VERSION 3.1.3)
project(motion_planner)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)


catkin_package(
  LIBRARIES
  INCLUDE_DIRS
   ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_ros_perception
    moveit_core
    moveit_ros_planning_interface
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS} 
    ${Boost_INCLUDE_DIR} 
    ${EIGEN3_INCLUDE_DIRS}
    ${moveit_ros_planning_INCLUDE_DIRS}
    ${moveit_ros_planning_interface_INCLUDE_DIRS}
    ${MOVEIT_INCLUDE_DIRS}
  )

include_directories(${moveit_core_INCLUDE_DIRS})
include_directories(/home/aditya/ws_moveit/src/moveit)

add_executable(motion_planner src/planning.cpp)

target_link_libraries(motion_planner ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(state src/states.cpp)

target_link_libraries(state ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_install_python(PROGRAMS
  src/states.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
