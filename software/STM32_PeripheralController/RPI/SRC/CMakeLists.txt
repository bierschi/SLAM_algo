cmake_minimum_required(VERSION 3.5)

#generator: "Eclipse CDT4 - MinGW Makefiles"

if(CMAKE_HOST_WIN32)
include(MINGW_GCCToolchainFile.cmake)
endif(CMAKE_HOST_WIN32)

project(SPIInterface CXX)

set(CMAKE_VERBOSE_MAKEFILE TRUE)

# define the source files to compile:
set(SRC_FILES
    ${CMAKE_CURRENT_LIST_DIR}/SPIInterface.cpp
    ${CMAKE_CURRENT_LIST_DIR}/SPILibrary.cpp
    ${CMAKE_CURRENT_LIST_DIR}/Path.cpp
    ${CMAKE_CURRENT_LIST_DIR}/PositionUpdater.cpp
    ${CMAKE_CURRENT_LIST_DIR}/StateModel.cpp
)

#define the include directories:
set(INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}
)

# define compiler options:
set(C_FLAGS "-O0 -g -pthread -DRPI_BUILD")
set(CXX_FLAGS "-O0 -g -pthread -std=gnu++11 -DRPI_BUILD")

# set compiler options to CMake:
set(CMAKE_C_FLAGS ${C_FLAGS})
set(CMAKE_CXX_FLAGS ${CXX_FLAGS})

include_directories(${INCLUDE_DIRS})

add_executable(SPIInterface ${SRC_FILES})
