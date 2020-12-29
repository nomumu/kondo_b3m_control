cmake_minimum_required(VERSION 3.0.2)
project(kondo_b3m_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  roscpp
  rospy
  hardware_interface
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp hardware_interface controller_manager
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_executable(
  kondo_b3m_control
  src/main.cpp
  src/b3m_config.cpp
  src/b3m_hardware.cpp
  src/b3m_joint.cpp
  src/b3m_port.cpp
  src/b3m_servo.cpp
)

target_link_libraries(kondo_b3m_control
  ${catkin_LIBRARIES}
)
