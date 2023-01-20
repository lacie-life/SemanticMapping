cmake_minimum_required(VERSION 3.0.0)
SET(CMAKE_BUILD_TYPE Release)

project(detect_3d_cuboid)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4.2 REQUIRED)

set(CUBOID_SOURCE_FILES
    object_3d_util.cpp
    matrix_utils.cpp
    box_proposal_detail.cpp
    matrix_utils.h
    object_3d_util.h
)

add_library(${PROJECT_NAME} SHARED ${CUBOID_SOURCE_FILES})

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)
target_include_directories(${PROJECT_NAME} PRIVATE
                           ${PROJECT_SOURCE_DIR}
                           ${Eigen_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}  
)
target_link_libraries(${PROJECT_NAME}
                      ${OpenCV_LIBRARIES}
)
