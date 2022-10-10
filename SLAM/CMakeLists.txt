cmake_minimum_required(VERSION 3.20)
project(SemanticSLAM)

set(semanticSLAM_VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 14)

# we want to use some C++11 features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_LIBRARY_PATH /opt/intel/mkl/lib/)
# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen3
include_directories("/usr/include/eigen3")
find_package (Eigen3 3.3 REQUIRED)

# set up options
option(BUILD_PYTHON_WRAP "Enable/Disable building of cython wrapper" ON)
set(PYTHON_VERSION "3" CACHE STRING "The version of python to build the cython wrapper for (or Default)")

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
include_directories( include )

set(SOURCE_FILES
        src/semanticBase/QuadricProjectionException.cpp
        src/semanticBase/Utilities.cpp
        src/semanticBase/yolo.cpp
        src/core/Camera.cpp
        src/core/ConstrainedDualQuadric.cpp
        src/core/AlignedBox2.cpp
        src/core/AlignedBox3.cpp
        src/core/DualConic.cpp
        src/core/BoundingBoxFactor.cpp
        src/core/QuadricAngleFactor.cpp
        src/odometry/bucket.cpp
        src/odometry/evaluate_odometry.cpp
        src/odometry/feature.cpp
        src/odometry/frame.cpp
        src/odometry/matrix.cpp
        src/odometry/utils.cpp
        src/odometry/visualOdometry.cpp
        src/imu/imudata.cpp
        src/imu/IMUPreintegrator.cpp include/vins_estimator/initial_pose_factor.h include/vins_estimator/initial_bias_factor.h include/vins_estimator/integration_base.h include/vins_estimator/imu_factor.h src/vins_estimator/marginalization_factor.cpp include/vins_estimator/marginalization_factor.h src/vins_estimator/pose_local_parameterization.cpp include/vins_estimator/pose_local_parameterization.h src/vins_estimator/projection_factor.cpp include/vins_estimator/projection_factor.h src/vins_estimator/projectionOneFrameTwoCamFactor.cpp include/vins_estimator/projectionOneFrameTwoCamFactor.h src/vins_estimator/projectionTwoFrameOneCamFactor.cpp include/vins_estimator/projectionTwoFrameOneCamFactor.h src/vins_estimator/projectionTwoFrameTwoCamFactor.cpp include/vins_estimator/projectionTwoFrameTwoCamFactor.h src/vins_estimator/initial_alignment.cpp include/vins_estimator/initial_alignment.h src/vins_estimator/initial_ex_rotation.cpp include/vins_estimator/initial_ex_rotation.h)

set(HEADER_FILES
        include/semanticSLAM.h
        include/semanticBase/QuadricProjectionException.h
        include/semanticBase/Utilities.h
        include/semanticBase/yolo.h
        include/core/Camera.h
        include/core/ConstrainedDualQuadric.h
        include/core/AlignedBox2.h
        include/core/AlignedBox3.h
        include/core/DualConic.h
        include/core/QuadricAngleFactor.h
        include/core/BoundingBoxFactor.h
        include/odometry/bucket.h
        include/odometry/camera.h
        include/odometry/evaluate_odometry.h
        include/odometry/feature.h
        include/odometry/frame.h
        include/odometry/matrix.h
        include/odometry/utils.h
        include/odometry/visualOdometry.h
        include/imu/imudata.h
        include/imu/IMUPreintegrator.h
        include/imu/so3.h
        include/imu/se3.hpp
        include/imu/so3.hpp
        include/imu/sophus.hpp)

set(CONVENIENCE_LIB_NAME "semanticSLAM")
add_library(${CONVENIENCE_LIB_NAME} ${SOURCE_FILES})
# set_target_properties(${CONVENIENCE_LIB_NAME} PROPERTIES PREFIX "")
target_link_libraries(${CONVENIENCE_LIB_NAME} gtsam
                                              ${OpenCV_LIBS}
                                              ${TORCH_LIBRARIES})

set_property(TARGET ${CONVENIENCE_LIB_NAME} PROPERTY CXX_STANDARD 14)

add_subdirectory(test)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
