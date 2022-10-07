find_package(GTSAM REQUIRED)

add_executable(test_semanticSLAM main.cpp)
target_link_libraries(test_semanticSLAM semanticSLAM gtsam)
