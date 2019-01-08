cmake_minimum_required(VERSION 3.0)

# set project name to alf
project(alf)

# set c++ standard to 11
set(CMAKE_CXX_STANDARD 11)

# find all required packages
find_package(Threads REQUIRED)
find_package(catkin REQUIRED COMPONENTS
             roscpp
             nav_msgs
             std_msgs)

# include catkin libs
include_directories(include ${catkin_INCLUDE_DIRS})

# set build directory for executables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")

# set *.cpp and *.h files for submodule communication
include_directories(${CMAKE_CURRENT_LIST_DIR}/include/communication/server)
file(GLOB SERVER_SRCS ${CMAKE_CURRENT_LIST_DIR}/src/communication/server/*.cpp)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include/communication/client)
file(GLOB CLIENT_SRCS ${CMAKE_CURRENT_LIST_DIR}/src/communication/client/*.cpp)

# set *.cpp and *.h files for submodule motioncontrol
include_directories(${CMAKE_CURRENT_LIST_DIR}/include/motioncontrol)
set(SPIINTERFACE_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/src/motioncontrol/SPIInterface.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/motioncontrol/SPILibrary.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/motioncontrol/Path.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/motioncontrol/PositionUpdater.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/motioncontrol/StateModel.cpp
)

# set *.cpp and *.h files for submodule pathfinder
include_directories(${CMAKE_CURRENT_LIST_DIR}/include/pathfinder)
file(GLOB PATHFINDER_SRCS ${CMAKE_CURRENT_LIST_DIR}/src/pathfinder/*.cpp)

# set *.cpp and *.h files for submodule slam
include_directories(${CMAKE_CURRENT_LIST_DIR}/include/slam)
file(GLOB SLAM_SRCS ${CMAKE_CURRENT_LIST_DIR}/src/slam/*.cpp)

# build the executable "alf" with all sources
add_executable(alf "alf_main.cpp" ${SLAM_SRCS} ${SERVER_SRCS} ${PATHFINDER_SRCS})
target_link_libraries(alf ${catkin_LIBRARIES})
target_link_libraries(alf ${CMAKE_THREAD_LIBS_INIT})




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
