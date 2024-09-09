cmake_minimum_required(VERSION 3.5)
project(impact_camera_calibration_description)


find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()