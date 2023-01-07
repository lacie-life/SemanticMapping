cmake_minimum_required(VERSION 3.21)
project(AG_Mapping)

set(CMAKE_CXX_STANDARD 14)

find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)

add_subdirectory(Slam)
add_subdirectory(Model)
add_subdirectory(Engine)
add_subdirectory(Test)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/Data
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

