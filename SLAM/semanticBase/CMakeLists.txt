
include_directories(include)

file(GLOB HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/semanticSLAM.h)

file(GLOB SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

source_group("Header Files" FILES ${HEADERS})
source_group("Source Files" FILES ${SOURCES})

set(CONVENIENCE_LIB_NAME "semantic_lib")
add_library(${CONVENIENCE_LIB_NAME} SHARED
        ${HEADERS}
        ${SOURCES})

target_link_libraries(${CONVENIENCE_LIB_NAME}
        gtsam
        ${OpenCV_LIBS}
        ${TORCH_LIBRARIES})

set_property(TARGET ${CONVENIENCE_LIB_NAME} PROPERTY CXX_STANDARD 14)
