cmake_minimum_required(VERSION 3.5)
project(ERC-Hackathon-24)

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)

# Install directories
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Add additional files for installation, if needed
# install(FILES my_file.py
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()
