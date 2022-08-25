cmake_minimum_required(VERSION 3.0.2)
project(tracked_uart_comm)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  serial
  std_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS src
  CATKIN_DEPENDS
    serial
    std_msgs
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
 src
 ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED src/uart_comm.cpp)

add_executable(uart_comm src/main.cpp src/uart_comm.cpp)
target_link_libraries(uart_comm
  ${catkin_LIBRARIES}
  nlopt
)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY src/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
