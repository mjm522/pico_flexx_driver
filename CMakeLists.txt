# Required CMake Version
cmake_minimum_required(VERSION 3.9)

# Package name
project(pico_flexx_driver_ros2)

set(CMAKE_BUILD_TYPE DEBUG)

set(CMAKE_CXX_FLAGS "-fPIE -fPIC -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")

# find dependencies
find_package(ament_cmake REQUIRED)

# Find packages needed for build
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(royale_DIR ${CMAKE_CURRENT_SOURCE_DIR}/royale/bin)

find_library(royale_LIBRARIES royale ${royale_DIR})
find_package(royale REQUIRED PATHS ${royale_DIR})

add_executable(pico_flexx_driver_ros2 
               "${CMAKE_CURRENT_SOURCE_DIR}/src/pico_flexx_driver_ros2.cpp")
#add_library(pico_flexx_driver_ros2 SHARED
 # "${CMAKE_CURRENT_SOURCE_DIR}/src/pico_flexx_driver_ros2.cpp"
#)
set_target_properties(pico_flexx_driver_ros2 PROPERTIES CXX_STANDARD 14)
target_compile_features(pico_flexx_driver_ros2 PUBLIC cxx_std_14)
ament_target_dependencies(pico_flexx_driver_ros2
  rclcpp
  sensor_msgs
  std_msgs
  royale
)

# Install binaries
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()