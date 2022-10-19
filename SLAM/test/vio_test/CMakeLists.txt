add_executable(test_vio main.cpp)
target_link_libraries(test_vio vins_lib ${PCL_LIBRARIES})
