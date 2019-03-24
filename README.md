# SLAM Algo

This Repository was created for a project @ [Ostbayerische Technische Hochschule Regensburg](https://www.oth-regensburg.de/) in WS 18/19


## Documentation

A detailed documentation about the implementation of this project can be found in folder `/latex` as
`hsp_ws18_19.pdf`


## Modules
This project were written in C++ and build with [CMake](https://cmake.org/)

In the current state, 4 modules were implemented:

- communication
- motioncontrol
- pathfinder
- slam

## Project Layout
<pre><code>
/hardware
/latex
    /bibtex
    /chapters
    /images
    /inputs
    hsp_ws18_19.pdf
    hsp_ws18_19.tex
/old
    /datasheets
    /docu
    /software
        /Documentation
        /Software_ARM
        /Software_HQ
        /Software_NIOS2
        /common
/software
    /final
        /STM_data
        /alf_car
            /include
                /communication
                /motioncontrol
                /pathfinder
                /slam
            /launch
                hokuyo_hector_slam.launch
            /scripts
                resetMap
            /src
                /communication
                /motioncontrol
                /pathfinder
                /slam
            CMakeLists.txt
            alf_main.cpp
        /alf_gui
            /include
            /src
            CMakeLists.txt
            alf_gui.cpp
    /testing
        /LidarConnectProj
        /PathFinderProj
        /STM32_PeripheralController
        /slam
        /testdata
        Architektur.odt
LICENSE
README.md

</pre></code>