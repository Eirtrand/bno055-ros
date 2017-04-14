cmake_minimum_required(VERSION 2.8.3)
project(sensor_interface)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
  std_msgs
  sensor_msgs
  eigen_conversions
)


find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)


rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}

)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)




find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  eigen_conversions
  roscpp
  std_msgs
)

find_package(Eigen3 REQUIRED)



catkin_package(
  CATKIN_DEPENDS
  eigen_conversions
  std_msgs

)


include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  
)



rosserial_add_client_target(firmware sensor_interface ALL)
rosserial_add_client_target(firmware sensor_interface-upload)


