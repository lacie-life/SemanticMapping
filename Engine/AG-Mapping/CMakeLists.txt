cmake_minimum_required(VERSION 3.5)

project(AG-Mapping VERSION 0.1 LANGUAGES CXX)

cmake_policy(SET CMP0057 NEW)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-std=c++17")

set(CMAKE_PREFIX_PATH "~/Qt/5.15.2/gcc_64")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Widgets Concurrent)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Widgets Concurrent)

find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(PCL 1.12 REQUIRED)

find_package(Boost COMPONENTS system filesystem REQUIRED)

find_package(VTK 9.2 CONFIG REQUIRED)
#include(${VTK_USE_FILE})

add_definitions(${PCL_DEFINITIONS})

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/include/App/
    ${CMAKE_SOURCE_DIR}/include/Camera_models/
    ${CMAKE_SOURCE_DIR}/include/Estimator/
)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIR})

file(GLOB HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/App/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/App/*.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Camera_models/camodocal/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Estimator/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Estimator/*.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Estimator/sophus/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/Estimator/sophus/*.hpp
)

file(GLOB SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/App/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Camera_models/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Camera_models/camera_models/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Estimator/*.cpp
)

source_group("Header Files" FILES ${HEADERS})
source_group("Source Files" FILES ${SOURCES})

set(UI
    ${CMAKE_CURRENT_LIST_DIR}/ui/mainwindow.ui
    ${CMAKE_CURRENT_LIST_DIR}/ui/qconfigdialog.ui
)

qt5_wrap_ui(UI_HEADERS ${UI})

list(APPEND RESOURCES
    ${CMAKE_CURRENT_LIST_DIR}/resources.qrc
)

qt5_add_resources(RCC_SOURCES
    ${RESOURCES}
)

add_executable(AG-Mapping
    ${HEADERS}
    ${SOURCES}
    ${UI_HEADERS}
    ${RCC_SOURCES}
)

target_link_libraries(AG-Mapping PRIVATE
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Concurrent
    ${VTK_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Boost_LIBRARIES}
    Ceres::ceres
    -lpthread
)

# Ref: http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html
set_property(TARGET AG-Mapping PROPERTY CXX_STANDARD 17)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

