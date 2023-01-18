add_executable(tum-rgbd-test rgbd_tum.cc)

target_link_libraries(tum-rgbd-test
        slam_lib
        ${THIRDPARTY_LIBS}
)


