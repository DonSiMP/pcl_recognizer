cmake_minimum_required(VERSION 3.2)
project(pcl_recognizer)

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")

find_package(catkin COMPONENTS
    std_msgs
    sensor_msgs
    roscpp
    pcl_ros
    pcl_conversions
    dynamic_reconfigure
    REQUIRED)

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/params.cfg)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    pcl_ros
    pcl_conversions
    dynamic_reconfigure
)

include_directories(include)

file(GLOB HEADER_FILES
    include/pcl_recognizer/*.h
    include/pcl_recognizer/*/*.h
    )
file(GLOB SOURCE_FILES src/*.cpp )

add_executable(pcl_recognizer ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(pcl_recognizer ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(pcl_recognizer ${PROJECT_NAME}_gencfg)

set_property(TARGET pcl_recognizer PROPERTY CXX_STANDARD 14)
set_property(TARGET pcl_recognizer PROPERTY CXX_STANDARD_RE4QUIRED ON)
