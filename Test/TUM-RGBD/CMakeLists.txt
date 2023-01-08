add_executable(tum-rgbd-test rgbd_tum.cc)

target_link_libraries(tum-rgbd-test
        slam_lib
        ${Pangolin_LIBRARIES}
        ${OpenCV_LIBS}
)

