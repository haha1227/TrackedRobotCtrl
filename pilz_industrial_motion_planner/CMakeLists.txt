cmake_minimum_required(VERSION 3.1.3)
project(pilz_industrial_motion_planner)

find_package(catkin REQUIRED COMPONENTS
  joint_limits_interface
  moveit_core
  moveit_msgs
  moveit_ros_move_group
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  roscpp
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_kdl
  tf2_ros
)

find_package(orocos_kdl)
find_package(Boost REQUIRED COMPONENTS )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    moveit_msgs
    roscpp
    tf2_geometry_msgs
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
    src/cartesian_limit.cpp
    src/cartesian_limits_aggregator.cpp
    src/command_list_manager.cpp
    src/joint_limits_aggregator.cpp
    src/joint_limits_container.cpp
    src/joint_limits_validator.cpp
    src/limits_container.cpp
    src/move_group_sequence_action.cpp
    src/move_group_sequence_service.cpp
    src/path_circle_generator.cpp
    src/pilz_industrial_motion_planner.cpp
    src/plan_components_builder.cpp
    src/planning_context_loader.cpp
    src/planning_context_loader_circ.cpp
    src/planning_context_loader_lin.cpp
    src/planning_context_loader_ptp.cpp
    src/trajectory_blender_transition_window.cpp
    src/trajectory_functions.cpp
    src/trajectory_generator.cpp
    src/trajectory_generator_circ.cpp
    src/trajectory_generator_lin.cpp
    src/trajectory_generator_ptp.cpp
    src/velocity_profile_atrap.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
