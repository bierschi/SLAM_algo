cmake_minimum_required(VERSION 3.0)

project(breezyslam)

set(CMAKE_CXX_STANDARD 11)
set (PROJECT_LINK_LIBS libbreezylidar.so)

include_directories(lib)
include_directories(include)

add_executable(slam src/main.cpp)
target_link_libraries(slam ${PROJECT_LINK_LIBS})