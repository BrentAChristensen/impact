cmake_minimum_required(VERSION 3.5)
project(impact_motion)

find_package(ament_cmake REQUIRED)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()



