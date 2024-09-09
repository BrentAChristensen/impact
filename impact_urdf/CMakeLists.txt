cmake_minimum_required(VERSION 3.5)
project(impact_urdf)


find_package(ament_cmake REQUIRED)

install(DIRECTORY config scripts urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()



