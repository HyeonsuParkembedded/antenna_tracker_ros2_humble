cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(antenna_tracker_uros_firmware)

FILE(GLOB app_sources
  src/main.c
  src/bmi270_driver.c
  src/mlx90393_driver.c
  src/gps_parser.c
  src/hall_encoder.c
  src/motor_driver.c
)

target_sources(app PRIVATE ${app_sources})
target_include_directories(app PRIVATE src)
