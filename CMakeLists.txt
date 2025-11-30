# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(BLUEPAD32_ROOT /)
set(BTSTACK_ROOT btstack)

set(EXTRA_COMPONENT_DIRS src/components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(robots)

set(IDF_TARGET "esp32")
