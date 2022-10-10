find_package(GTSAM REQUIRED)

add_executable(test_vo main.cpp)
target_link_libraries(test_vo odometry_lib)
