# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(solar_controller)

target_add_binary_data(solar_controller.elf "main/cacert.pem" TEXT)
