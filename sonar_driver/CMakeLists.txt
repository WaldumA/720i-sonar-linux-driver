cmake_minimum_required(VERSION 2.8.3)
project(sonar_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
   eigen_conversions
   nav_msgs
   roscpp
   sensor_msgs
   std_msgs
   tf2
   tf2_geometry_msgs
   tf2_ros
   vortex_msgs
   sonar_msgs
)

## Generate messages
#add_message_files(
#   FILES 
#   sonar_raw_data.msg
#)
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

## Adding EIGEN
find_package(Eigen3)
set(EIGEN_PACKAGE EIGEN3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  set(EIGEN_PACKAGE Eigen)
endif()

catkin_package(
   INCLUDE_DIRS
      include
   CATKIN_DEPENDS
      eigen_conversions
      nav_msgs
      roscpp
      sensor_msgs
      std_msgs
      tf2
      tf2_geometry_msgs
      tf2_ros
      vortex_msgs
      sonar_msgs
      
   DEPENDS
      ${EIGEN_PACKAGE}
)


include_directories(
 include

  PRIVATE
  include/
  include/Gemini/
  include/ECDLogDataTypes/
  include/MathsLib/
  include/Svs5Seq
  libs/GeminiComms/x86_64-linux-gnu/x64/Release

  



  PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})

add_executable(sonar_driver_node src/sonar_driver.cpp)

target_link_libraries(sonar_driver_node PRIVATE ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} "${CMAKE_SOURCE_DIR}/libs/GeminiComms/x86_64-linux-gnu/x64/Release/libGeminiComms.so")


