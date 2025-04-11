cmake_minimum_required(VERSION 3.10)

project(Visualize_pointclouds LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17) #17
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

cmake_policy(SET CMP0146 OLD)

add_executable(Visualize_pointclouds 
    src/main.cpp 
    src/rDICOM.cpp
    src/visualizer.cpp
    include/includes.h
    include/rDICOM.h
    include/visualizer.h
    )

# Set StartUp Project (Option)
# (This setting is able to enable by using CMake 3.6.0 RC1 or later.)
#set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "project" )

# Find Packages
find_package( PCL REQUIRED )
find_package( OpenCV REQUIRED )
#find_package(Franka REQUIRED PATHS /home/filip/libfranka/build)
find_package(Eigen3 REQUIRED)
#find_package(realsense2 REQUIRED)
################################################################################
##Ultrasound
#find_package(OpenIGTLink REQUIRED)

include_directories(include)
# [C/C++]>[General]>[Additional Include Directories]
include_directories( ${PCL_INCLUDE_DIRS} )
include_directories( ${OpenCV_INCLUDE_DIRS}

#${OpenIGTLink_INCLUDE_DIRS}
)

# [C/C++]>[Preprocessor]>[Preprocessor Definitions]
add_definitions( ${PCL_DEFINITIONS} )

# For Use Not PreCompiled Features
add_definitions( -DPCL_NO_PRECOMPILE )

# [Linker]>[General]>[Additional Library Directories]
link_directories( ${PCL_LIBRARY_DIRS}
/usr/local/lib
#${OpenIGTLink_LIBRARY_DIRS}
)


# [Linker]>[Input]>[Additional Dependencies]
target_link_libraries(Visualize_pointclouds
${PCL_LIBRARIES}
${OpenCV_LIBS}
# Franka::Franka
Eigen3::Eigen
# ${realsense2_LIBRARY}
#ortools
#glog
#/usr/local/lib/igtl/libigtlutil.a
#/usr/local/lib/igtl/libOpenIGTLink.a
)




