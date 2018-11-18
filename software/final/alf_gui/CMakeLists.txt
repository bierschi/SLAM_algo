cmake_minimum_required(VERSION 3.7)
project(alf_gui)

set(CMAKE_CXX_STANDARD 11)

# create executable in /build directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")

# instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)

# create code form a list of Qt designer ui files
set(CMAKE_AUTOUIC ON)

# find the Qt Widgets Core libraries
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
find_package(Threads REQUIRED)

include_directories(include)

# populate a CMake variable with the sources
set(alf_gui_src include/mainwindow.h include/communication/ClientSocket.h include/communication/Socket.h include/communication/SocketException.h
                src/mainwindow.ui src/mainwindow.cpp src/communication/ClientSocket.cpp src/communication/Socket.cpp
                alf_gui.cpp)


# tell CMake to create a main_gui executable
add_executable(alf_gui ${alf_gui_src})

# link libraries
target_link_libraries(alf_gui ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(alf_gui Qt5::Widgets)