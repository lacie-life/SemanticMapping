add_subdirectory(TUM-RGBD)
add_subdirectory(ZED-2)

add_executable(test_app main.cpp)
target_link_libraries(test_app slam_lib )
