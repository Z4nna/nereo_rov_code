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

#add_executable(imu_pub src/imuPub.cpp)
add_executable(barometer_pub src/barPub.cpp)

add_library(iic src/iic.c)
add_library(barometer_driver src/driver_ms5837.c)
add_library(barometer_driver_basic src/driver_ms5837_basic.c)
add_library(barometer_interface src/raspberrypi4b_driver_ms5837_interface.c)

target_link_libraries(barometer_interface 
  iic
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

#ament_target_dependencies(imu_pub 
#	rclcpp 
#	std_msgs 
#	sensor_msgs
#	diagnostic_msgs
#)
ament_target_dependencies(barometer_pub 
	rclcpp 
	std_msgs 
	sensor_msgs 
	diagnostic_msgs
)


install(
  TARGETS
  #imu_pub
  barometer_pub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
