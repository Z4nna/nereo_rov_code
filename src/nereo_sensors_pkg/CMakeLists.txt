cmake_minimum_required(VERSION 3.8)
project(nereo_sensors_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(imu_pub src/imuPub.cpp)
add_executable(barometer_pub src/barPub.cpp)

# barometer libs
add_library(bar_iic src/barometer_sources/iic.c 
  nereo_sensors_pkg/barometer_libs/iic.h
)
add_library(barometer_driver 
  src/barometer_sources/driver_ms5837.c 
  nereo_sensors_pkg/barometer_libs/driver_ms5837.h
)
add_library(barometer_driver_basic 
  src/barometer_sources/driver_ms5837_basic.c 
  nereo_sensors_pkg/barometer_libs/driver_ms5837_basic.h
)
add_library(barometer_interface 
  src/barometer_sources/raspberrypi4b_driver_ms5837_interface.c 
  nereo_sensors_pkg/barometer_libs/raspberrypi4b_driver_ms5837_interface.h
)

# imu libs
add_library(imu_i2c 
  src/imu_sources/i2c.c 
  nereo_sensors_pkg/imu_libs/i2c.h
)
add_library(wit_c_sdk
  src/imu_sources/wit_c_sdk.c
  nereo_sensors_pkg/imu_libs/wit_c_sdk.h
)
add_library(WIT61P
  src/imu_sources/WIT61P.c
  nereo_sensors_pkg/imu_libs/WIT61P.h
)

# barometer link libraries
target_link_libraries(barometer_interface 
  bar_iic
  barometer_driver
)
target_link_libraries(barometer_driver_basic
  barometer_interface
)
target_link_libraries(barometer_pub 
  barometer_driver
  barometer_driver_basic
  barometer_interface
)

# imu link libraries
target_link_libraries(WIT61P
  imu_i2c
  wit_c_sdk
)

ament_target_dependencies(imu_pub 
	rclcpp 
	std_msgs 
	sensor_msgs
	diagnostic_msgs
)
ament_target_dependencies(barometer_pub 
	rclcpp 
	std_msgs 
	sensor_msgs 
	diagnostic_msgs
)

install(TARGETS
  imu_pub
  barometer_pub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
