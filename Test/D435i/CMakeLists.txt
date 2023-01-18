
find_package(realsense2)
include_directories(${realsense_INCLUDE_DIR})

add_executable(d435i-test main.cpp)

target_link_libraries(d435i-test
        slam_lib
        ${OpenCV_LIBS}
        ${realsense2_LIBRARY}
        ${Pangolin_LIBRARIES}
        ${PCL_LIBRARIES}
)
