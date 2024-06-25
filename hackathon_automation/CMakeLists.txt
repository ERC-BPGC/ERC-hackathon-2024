cmake_minimum_required(VERSION 3.5)
project(hackathon_automation)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(tf2 REQUIRED)

################################################################################
# Build
################################################################################
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

set(dependencies
  "geometry_msgs"
  # "nav_msgs"
  # "rclcpp"
  "sensor_msgs"
  # "tf2"
)

install(DIRECTORY launch models worlds
  DESTINATION share/${PROJECT_NAME}/
)

# install(PROGRAMS  
#   scripts/controller.pt
#   DESTINATION lib/${PROJECT_NAME} )

# Add additional files for installation, if needed
# install(FILES my_file.py
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_python_install_package(src)
ament_package()