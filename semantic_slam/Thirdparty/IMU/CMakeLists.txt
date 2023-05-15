set (imu_ROOR_DIR "${PROJECT_SOURCE_DIR}/semantic_slam/Thirdparty/IMU")

# add_definitions(-std=c99)

add_library(icm20948 SHARED ICM20948.h ICM20948.cpp)

