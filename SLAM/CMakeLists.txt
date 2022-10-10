cmake_minimum_required(VERSION 3.20)
project(SemanticSLAM)

set(semanticSLAM_VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 14)

set(USE_CUDA OFF)
set(ROS_VISUAL OFF)

# we want to use some C++11 features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_LIBRARY_PATH /opt/intel/mkl/lib/)

# ROS package
if(ROS_VISUAL)
    find_package(catkin REQUIRED COMPONENTS
            roscpp
            std_msgs
            geometry_msgs
            nav_msgs
            tf
            cv_bridge
            image_transport)
    include_directories(${catkin_INCLUDE_DIRS})
endif()

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen3
include_directories("/usr/include/eigen3")
find_package (Eigen3 3.3 REQUIRED)

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

###################################################################################
# find GTSAM and GTSAM CMake tools
find_package(GTSAM REQUIRED)
find_package(GTSAMCMakeTools REQUIRED)

# include gtsam cmake tools
include(GtsamBuildTypes) 			# Load build type flags and default to Debug mode
include(GtsamTesting)    			# Easy functions for creating unit tests and scripts
include(GtsamMakeConfigFile) 	# Defines a function for generating config file so package can be found with find_package function

set(Torch_DIR /home/lacie/Github/libtorch/share/cmake/Torch)
find_package(Torch PATHS ${Torch_DIR} NO_DEFAULT REQUIRED)
if (Torch_FOUND)
    message(STATUS "Torch library found!")
    message(STATUS "    include path: ${TORCH_INCLUDE_DIRS}" \n)

else ()
    message(FATAL_ERROR "Could not locate Torch" \n)
endif()
include_directories(${TORCH_INCLUDE_DIRS})

include_directories(BEFORE "${PROJECT_SOURCE_DIR}")
include_directories(
        include
        camera_models/include
        odometry/include
        semanticBase/include
        vins_estimator/include)

add_library(camera_models SHARED IMPORTED)
set_target_properties(camera_models PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/../camera_models/build/libcamera_models.so)

if (ROS_VISUAL)
    catkin_package()
endif ()

add_subdirectory(odometry)
add_subdirectory(semanticBase)
add_subdirectory(vins_estimator)

add_subdirectory(test)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
