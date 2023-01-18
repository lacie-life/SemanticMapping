cmake_minimum_required(VERSION 3.1)

set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)

SET(CMAKE_BUILD_TYPE "Release")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_PREFIX_PATH "~/Qt/5.15.2/gcc_64")
list(APPEND CMAKE_PREFIX_PATH "~/Github/libtorch/")

add_definitions(-w)

find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin 0.6 REQUIRED)
find_package(g2o REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(Torch REQUIRED)

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})
message("PCL Include Directory: ${PCL_INCLUDE_DIRS}")

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${TORCH_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/Slam/include
    ${CMAKE_SOURCE_DIR}/Slam/Thirdparty/g2o
    ${CMAKE_SOURCE_DIR}/Slam/Thirdparty/Sophus
    ${CMAKE_SOURCE_DIR}/Slam/Thirdparty/DBoW2
)

file(GLOB HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/CameraModels/*.h
        )

file(GLOB SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/CameraModels/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
        )

source_group("Header Files" FILES ${HEADERS})
source_group("Source Files" FILES ${SOURCES})

set(CONVENIENCE_LIB_NAME "slam_lib")

add_library(${CONVENIENCE_LIB_NAME} SHARED
        ${HEADERS}
        ${SOURCES}
)

target_link_libraries(${CONVENIENCE_LIB_NAME} PRIVATE
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${TORCH_LIBRARIES}
        ${CMAKE_SOURCE_DIR}/Slam/Thirdparty/DBoW2/lib/libDBoW2_New.so
        g2o_core
        g2o_types_slam3d
        g2o_solver_csparse
        g2o_stuff
        g2o_csparse_extension
        g2o_types_sim3
        g2o_types_sba
        -lboost_serialization
        -lcrypto
)

