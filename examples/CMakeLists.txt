cmake_minimum_required(VERSION 3.16)

# Use the local copy of the VL53L8CX library (component name = folder name)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../VL53L8CX-Library")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(check_tof)
