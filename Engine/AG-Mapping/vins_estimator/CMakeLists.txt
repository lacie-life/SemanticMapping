
find_package(Ceres REQUIRED)

include_directories(
    include
    ${CERES_INCLUDE_DIRS}
)

file(GLOB HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)

file(GLOB SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

source_group("Header Files" FILES ${HEADERS})
source_group("Source Files" FILES ${SOURCES})

set(CONVENIENCE_LIB_NAME "vins_lib")

add_library(${CONVENIENCE_LIB_NAME} SHARED
        ${HEADERS}
        ${SOURCES})

if (ROS_VISUAL)
    target_link_libraries(${CONVENIENCE_LIB_NAME}
            camera_models
            ${OpenCV_LIBS}
            ${catkin_LIBRARIES}
            ${CERES_LIBRARIES})
else()
    target_link_libraries(${CONVENIENCE_LIB_NAME}
            camera_models
            ${OpenCV_LIBS}
            ${CERES_LIBRARIES})
endif()


set_property(TARGET ${CONVENIENCE_LIB_NAME} PROPERTY CXX_STANDARD 14)
