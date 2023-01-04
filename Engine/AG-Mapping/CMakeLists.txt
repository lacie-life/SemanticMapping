cmake_minimum_required(VERSION 3.5)

project(AG-Mapping VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-std=c++14")

set(CMAKE_PREFIX_PATH "~/Qt/5.15.2/gcc_64")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Widgets)

find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
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
    ${CERES_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/vins_estimator/include/
    ${CMAKE_SOURCE_DIR}/camera_models/include/
)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIR})

add_subdirectory(camera_models)
add_subdirectory(vins_estimator)

file(GLOB HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp
)

file(GLOB SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
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
    ${VTK_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Boost_LIBRARIES}
    vins_lib
    camera_models
    Ceres::ceres
    -lpthread
)

set_property(TARGET AG-Mapping PROPERTY CXX_STANDARD 14)

#file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data
#     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

