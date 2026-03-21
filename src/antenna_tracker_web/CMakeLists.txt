cmake_minimum_required(VERSION 3.8)
project(antenna_tracker_web)

find_package(ament_cmake REQUIRED)

install(DIRECTORY web/
  DESTINATION share/${PROJECT_NAME}/web
)

ament_package()
