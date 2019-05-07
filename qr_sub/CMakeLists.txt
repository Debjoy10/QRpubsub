cmake_minimum_required(VERSION 2.8.3)
project(qr_sub)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge 
  image_geometry
  roscpp
  rospy
  std_msgs
  geometry_msgs
  image_transport
  message_generation
)
 

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)
catkin_package(
  CATKIN_DEPENDS message_runtime nodelet cv_bridge roscpp std_msgs geometry_msgs
  LIBRARIES qr_sub barcode
)
find_package(barcode REQUIRED)
add_executable(textprep src/textprep.cpp)

target_link_libraries(textprep  ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(textprep ${catkin_EXPORTED_TARGETS})





