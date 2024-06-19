cmake_minimum_required(VERSION 3.10)
project(libseek-thermal)

# Request C++11 standard, using new CMake variables.
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(diagnostic_updater REQUIRED)
# find_package(dynamic_reconfigure REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp_components REQUIRED)


message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

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

add_library(seek_thermal_ros SHARED src/seek_thermal_ros.cpp)
target_link_libraries(seek_thermal_ros seek ${catkin_LIBRARIES})
ament_target_dependencies(seek_thermal_ros
  camera_info_manager
  cv_bridge
  diagnostic_updater
  image_transport
  sensor_msgs
  std_msgs
  rclcpp
  rclcpp_components
)
rclcpp_components_register_nodes(seek_thermal_ros "SeekThermalRos")

install(TARGETS seek_thermal_ros seek
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(FILES nodelet_plugins.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()