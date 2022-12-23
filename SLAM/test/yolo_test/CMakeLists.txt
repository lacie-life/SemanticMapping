find_package(GTSAM REQUIRED)

add_executable(yolo_test main.cpp)
target_link_libraries(yolo_test semantic_lib gtsam)