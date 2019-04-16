cmake_minimum_required(VERSION 2.8)
project(libseek-thermal)

# Request C++11 standard, using new CMake variables.
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_STANDARD_REQUIRED True)

find_package(catkin REQUIRED COMPONENTS image_transport camera_info_manager dynamic_reconfigure )

catkin_package(
    DEPENDS image_transport camera_info_manager dynamic_reconfigure
    #CATKIN_DEPENDS
    #INCLUDE_DIRS
    #LIBRARIES
)


message (STATUS "Build type: ${CMAKE_BUILD_TYPE}")

find_package (OpenCV REQUIRED)

set (HEADERS
    src/SeekCam.h
    src/SeekDevice.h
    src/seek.h
    src/SeekLogging.h
    src/SeekThermal.h
    src/SeekThermalPro.h
)

set (SOURCES
    src/SeekCam.cpp
    src/SeekDevice.cpp
    src/SeekThermal.cpp
    src/SeekThermalPro.cpp
)

set (SRC ${SOURCES} ${HEADERS})

include_directories (
    src
    /usr/include/libusb-1.0
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_library (seek ${SRC})

target_link_libraries (seek
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    -lusb-1.0
)
