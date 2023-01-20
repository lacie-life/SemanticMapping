add_executable(3D_BB_test main.cpp)

target_link_libraries(3D_BB_test
        ${CMAKE_SOURCE_DIR}/Slam/Thirdparty/detect_3d_cuboid/lib/libdetect_3d_cuboid.so
)
