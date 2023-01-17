add_executable(bin_vocabulary text2binary.cpp)

target_link_libraries(bin_vocabulary
        ${CMAKE_SOURCE_DIR}/Slam/Thirdparty/DBoW2/lib/libDBoW2_New.so
)