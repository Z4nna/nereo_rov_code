cmake_minimum_required(VERSION 3.8)
project(nereo_sensors_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

add_executable(imu_unit_test src/imuPub.cpp)

# imu libs
add_library(imu_i2c 
  src/imu_sources/i2c.c 
)
add_library(wit_c_sdk
  src/imu_sources/wit_c_sdk.c
)
add_library(WT61P
  src/imu_sources/WT61P.c
)

# imu link libraries
target_link_libraries(WT61P
  imu_i2c
  wit_c_sdk
)
target_link_libraries(imu_unit_test
  WT61P
)

install(TARGETS
  imu_unit_test
  DESTINATION lib/${PROJECT_NAME}
)
