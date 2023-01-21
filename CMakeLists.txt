cmake_minimum_required(VERSION 3.10)

project(AG_Mapping)

#set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

set(CMAKE_PREFIX_PATH "/home/lacie/Qt/5.15.2/gcc_64")

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV 4.2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)

find_package(ZED 3 REQUIRED)
include_directories(${ZED_INCLUDE_DIRS})
link_directories(${ZED_LIBRARY_DIR})

find_package(Boost COMPONENTS system filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(PCL 1.10 REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
message("PCL Include Directory: ${PCL_INCLUDE_DIRS}")

list(APPEND CMAKE_PREFIX_PATH "~/Github/libtorch/")
find_package(Torch REQUIRED)
include_directories(${TORCH_INCLUDE_DIRS})

include_directories(
        ${CMAKE_SOURCE_DIR}/Slam
        ${CMAKE_SOURCE_DIR}/Slam/include
        ${CMAKE_SOURCE_DIR}/Slam/include/CameraModels
        ${CMAKE_SOURCE_DIR}/Slam/Thirdparty/g2o
        ${CMAKE_SOURCE_DIR}/Slam/Thirdparty/Sophus
        ${CMAKE_SOURCE_DIR}/Slam/Thirdparty/DBoW2
        ${CMAKE_SOURCE_DIR}/Slam/Thirdparty
)

link_directories(${PCL_LIBRARY_DIRS})

set(THIRDPARTY_LIBS
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${TORCH_LIBRARIES}
        ${CMAKE_SOURCE_DIR}/Slam/Thirdparty/DBoW2/lib/libDBoW2_New.so
        ${CMAKE_SOURCE_DIR}/Slam/Thirdparty/detect_3d_cuboid/lib/libdetect_3d_cuboid.so
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

add_subdirectory(Slam)
#add_subdirectory(Model)
add_subdirectory(Engine)
add_subdirectory(Test)
add_subdirectory(Tools)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/Data
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

