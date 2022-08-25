cmake_minimum_required(VERSION 3.0.2)
project(tracked_robot_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    trac_ik_lib
    moveit_core
    moveit_ros_planning
    tracked_uart_comm
    pilz_industrial_motion_planner
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pkg_nlopt REQUIRED nlopt)

find_package(Boost REQUIRED COMPONENTS date_time)
find_package(orocos_kdl REQUIRED)
#find_package(Eigen3 REQUIRED)


catkin_package(
  CATKIN_DEPENDS
    trac_ik_lib
  DEPENDS
    Boost
    orocos_kdl
    pkg_nlopt
#    Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${pkg_nlopt_INCLUDE_DIRS}
#  ${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
    include/${PROJECT_NAME}/robot_control.h
    include/${PROJECT_NAME}/robot_planning.h
    include/${PROJECT_NAME}/MiniPID.h
    src/robot_control.cpp
    src/robot_planning.cpp
    src/MiniPID.cpp)

add_executable(robot_control src/main.cpp src/robot_control.cpp src/MiniPID.cpp)

target_link_libraries(robot_control
  ${catkin_LIBRARIES}
  nlopt
#  Eigen3::Eigen
)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY src/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

