cmake_minimum_required(VERSION 3.0.2)
project(detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  tf
  tf2
  sensor_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# find_package(yaml-cpp REQUIRED)

add_message_files(
  FILES
  GimbalOrientation.msg
  Armor.msg
  Armors.msg
  Target.msg
  TargetCommand.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo02_talker_listener
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime sensor_msgs cv_bridge
#  DEPENDS system_lib
)

add_executable(detector_node
  src/detector_node.cpp
  src/detector_wrapper.cpp
  src/detector.cpp
  src/number_classifier.cpp
  src/coord_transformer.cpp
  src/SerialPort.cpp
  src/Checksum.cpp
)
add_executable(tf_node
  src/tf_node.cpp
  src/gimbal_control.cpp
  src/Checksum.cpp
  src/SerialPort.cpp
)
add_executable(tracker_node
    src/trajectory.cpp
    src/tracker_node.cpp
    src/tracker_wrapper.cpp
    src/tracker.cpp
    src/Extended_KF.cpp
    src/gimbal_control.cpp
    src/SerialPort.cpp
    src/Checksum.cpp
)


set(OpenCV_DIR /home/infantry_1/opencv-4.7.0/build)

include_directories(include ${OpenCV_INCLUDE_DIRS})
include_directories(include /opt/MVS/include/)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${cv_bridge_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})
include_directories(${tf2_INCLUDE_DIRS})

LINK_DIRECTORIES(
  /opt/MVS/lib/64/
)

target_link_libraries(detector_node
  MvCameraControl
  MVGigEVisionSDK
  MVRender
  MvUsb3vTL
  pthread
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${tf_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  yaml-cpp
)

target_link_libraries(tf_node
  pthread
  ${catkin_LIBRARIES}
  ${tf2_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  yaml-cpp
)

target_link_libraries(tracker_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${tf2_LIBRARIES}
)

add_dependencies(detector_node detector_generate_messages_cpp)
add_dependencies(tf_node detector_generate_messages_cpp)

install(TARGETS
  detector_node
  DESTINATION bin/
)

