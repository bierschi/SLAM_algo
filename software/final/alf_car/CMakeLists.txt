cmake_minimum_required(VERSION 3.0)

project(alf)

set(CMAKE_CXX_STANDARD 11)

find_package(Threads REQUIRED)
find_package(catkin REQUIRED COMPONENTS
             roscpp
             nav_msgs
             std_msgs)

include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")

set(SPIINTERFACE_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/src/motioncontrol/SPIInterface.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/motioncontrol/SPILibrary.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/motioncontrol/Path.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/motioncontrol/PositionUpdater.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/motioncontrol/StateModel.cpp
)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include/motioncontrol)


# build executable with all sources # without motioncontrol
#file(GLOB ALL_SOURCES "alf_main.cpp" "src/pathfinder/*.cpp" "src/slam/*.cpp" "src/communication/server/*.cpp")
#add_executable(alf ${ALL_SOURCES})
#target_link_libraries(alf ${catkin_LIBRARIES})
#target_link_libraries(alf ${CMAKE_THREAD_LIBS_INIT})


# build each module independently
#file(GLOB PATHFINDER "pathfinder_main.cpp" "src/pathfinder/*.cpp")
#file(GLOB SLAM "alf_main.cpp" "src/slam/*.cpp")

#file(GLOB SERVER "server_main.cpp" "src/communication/server/*.cpp")
#file(GLOB CLIENT "client_main.cpp" "src/communication/client/*.cpp")

add_executable(SPIInterface ${SPIINTERFACE_SRCS})

#add_executable(pathfinder ${PATHFINDER})

#add_executable(slam ${SLAM})
#target_link_libraries(slam ${catkin_LIBRARIES})

#add_executable(server ${SERVER})
#target_link_libraries(server ${CMAKE_THREAD_LIBS_INIT})

#add_executable(client ${CLIENT})
