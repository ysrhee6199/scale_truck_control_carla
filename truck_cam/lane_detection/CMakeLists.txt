cmake_minimum_required(VERSION 3.8)
project(lane_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ros2_msg REQUIRED)

set(PROJECT_SOURCES
    src/main.cpp
    src/sensor_cam.cpp
    
    include/sensor_cam.hpp
)

set(PROJECT_SOURCES_DEPENDENCIES
    rclcpp
    std_msgs
    sensor_msgs
    ament_index_cpp
    cv_bridge
    ros2_msg
)

include_directories(
    include
    #{OpenCV_INCLUDE_DIRS}
)

add_executable()
ament_target_dependencies()

install(TARGETS
)
install(DIRECTORY
    install/
    DESTINATION include
)

if(BUILD_TESTING)

endif()

ament_package
