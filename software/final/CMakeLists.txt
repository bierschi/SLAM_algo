cmake_minimum_required(VERSION 3.0)

project(alf)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             nav_msgs
             std_msgs)

include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")


# build executable with all sources
#file(GLOB ALL_SOURCES "alf_main.cpp" "src/motioncontrol/*.cpp" "src/pathfinder/*.cpp" "src/slam/*.cpp")
#add_executable(alf ${ALL_SOURCES})
#target_link_libraries(alf ${catkin_LIBRARIES})

# build each module independently
file(GLOB PATHFINDER "pathfinder_main.cpp" "src/pathfinder/*.cpp")
file(GLOB SLAM "alf_main.cpp" "src/slam/*.cpp")

add_executable(pathfinder ${PATHFINDER})
add_executable(slam ${SLAM})
target_link_libraries(slam ${catkin_LIBRARIES})