add_executable(zed-test main.cpp)

target_link_libraries(zed-test slam_lib ${OpenCV_LIBS})