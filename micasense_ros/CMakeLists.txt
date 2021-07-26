cmake_minimum_required(VERSION 3.0.2)
project(micasense_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  message_generation
  cv_bridge
)
find_package( OpenCV REQUIRED )
add_message_files(
  FILES
  Multispectral.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES micasense_ros
   CATKIN_DEPENDS message_runtime std_msgs geometry_msgs rospy roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/micasense_control.py scripts/micasense_image_processor.py scripts/micasense_image_processor_viewer.py scripts/micasense_viewer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
