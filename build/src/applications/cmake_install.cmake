# Install script for directory: /home/fri/asha_ritu/catkin_ws_kinect/src/kinect_body_tracking/kinect_hallway/src/applications

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/fri/asha_ritu/catkin_ws_kinect/src/kinect_body_tracking/kinect_hallway/build/src/applications/multikinect/cmake_install.cmake")
  include("/home/fri/asha_ritu/catkin_ws_kinect/src/kinect_body_tracking/kinect_hallway/build/src/applications/aprilTagSnapper/cmake_install.cmake")
  include("/home/fri/asha_ritu/catkin_ws_kinect/src/kinect_body_tracking/kinect_hallway/build/src/applications/quickVideoRecorder/cmake_install.cmake")
  include("/home/fri/asha_ritu/catkin_ws_kinect/src/kinect_body_tracking/kinect_hallway/build/src/applications/calibrator/cmake_install.cmake")
  include("/home/fri/asha_ritu/catkin_ws_kinect/src/kinect_body_tracking/kinect_hallway/build/src/applications/threadPoolTest/cmake_install.cmake")

endif()

