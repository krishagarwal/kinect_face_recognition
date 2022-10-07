cmake_minimum_required(VERSION 2.8)
project(quickVideoRecorder)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

message("quickVideoRecorder.libkinectwrapper_INCLUDE_DIRS:  " ${libkinectwrapper_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
find_package(Cairo REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Ceres REQUIRED)
find_package( Boost REQUIRED COMPONENTS program_options)

pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

include_directories(
    ${libkinectwrapper_INCLUDE_DIRS}
    ${GTK3_INCLUDE_DIRS}
    ${CAIRO_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${APRIL_TAG_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

SET(APRIL_TAG_DISPLAY_SRC
    quickVideoRecorder.cpp
)

add_executable(quickVideoRecorder ${APRIL_TAG_DISPLAY_SRC})

target_link_libraries(quickVideoRecorder kinectwrapper
    k4a
    k4abt
    k4arecord
    pthread
    apriltag
    GL
    GLU
    glfw
    ${OpenCV_LIBRARIES}
    ${GTK3_LIBRARIES}
    ${CAIRO_LIBRARIES}
    ${CERES_LIBRARIES}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
)