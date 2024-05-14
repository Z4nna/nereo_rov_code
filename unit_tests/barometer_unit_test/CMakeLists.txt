cmake_minimum_required(VERSION 3.8)
project(nereo_sensors_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_executable(barometer barPub.cpp)

add_library(iic iic.c)
add_library(barometer_driver driver_ms5837.c)
add_library(barometer_driver_basic driver_ms5837_basic.c)
add_library(barometer_interface raspberrypi4b_driver_ms5837_interface.c)

target_link_libraries(barometer_interface 
  iic
  barometer_driver
)
target_link_libraries(barometer_driver_basic
  barometer_interface
)

target_link_libraries(barometer
  barometer_driver
  barometer_driver_basic
  barometer_interface
)

install(
  TARGETS
  barometer
  DESTINATION lib/${PROJECT_NAME}
)