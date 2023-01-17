add_executable(recorder_realsense_D435i
            recorder_realsense_D435i.cc)
target_link_libraries(recorder_realsense_D435i ${realsense2_LIBRARY} ${OpenCV_LIBS})

add_executable(recorder_realsense_T265
            recorder_realsense_T265.cc)
target_link_libraries(recorder_realsense_T265 ${realsense2_LIBRARY} ${OpenCV_LIBS})
