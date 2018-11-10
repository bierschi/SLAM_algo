cmake_minimum_required(VERSION 3.0)

project(alf)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             nav_msgs
             std_msgs)

include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS})

file(GLOB SOURCES "alf_main.cpp" "src/motioncontrol/*.cpp" "src/pathfinder/*.cpp" "src/slam/*.cpp")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")

add_executable(alf ${SOURCES})
target_link_libraries(alf ${catkin_LIBRARIES})