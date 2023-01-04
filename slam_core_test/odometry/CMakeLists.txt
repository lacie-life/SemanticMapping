
include_directories(include)

file(GLOB HEADERS
#        ${CMAKE_CURRENT_SOURCE_DIR}/include/imu/*.h
#        ${CMAKE_CURRENT_SOURCE_DIR}/include/imu/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/odometry/*.h)

file(GLOB SOURCES
#        ${CMAKE_CURRENT_SOURCE_DIR}/src/imu/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/odometry/*.cpp)

source_group("Header Files" FILES ${HEADERS})
source_group("Source Files" FILES ${SOURCES})

set(CONVENIENCE_LIB_NAME "odometry_lib")
add_library(${CONVENIENCE_LIB_NAME} SHARED
        ${HEADERS}
        ${SOURCES})

target_link_libraries(${CONVENIENCE_LIB_NAME} ${OpenCV_LIBS})

set_property(TARGET ${CONVENIENCE_LIB_NAME} PROPERTY CXX_STANDARD 14)
