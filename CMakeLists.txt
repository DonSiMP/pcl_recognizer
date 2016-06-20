cmake_minimum_required(VERSION 3.3)
project(pcl_recognize)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include)

file(GLOB HEADER_FILES
    include/pcl_recognize/*.h
    include/pcl_recognize/*/*.h
    )
file(GLOB SOURCE_FILES src/*.cpp )

add_executable(pcl_recognize ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(pcl_recognize ${PCL_LIBRARIES})

set_property(TARGET pcl_recognize PROPERTY CXX_STANDARD 14)
set_property(TARGET pcl_recognize PROPERTY CXX_STANDARD_RE4QUIRED ON)
