# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Set the CMake build type (Release, Debug, etc.)
set(CMAKE_BUILD_TYPE Debug)

# Set the IDF_PATH variable to the path of the ESP-IDF framework
set(IDF_PATH $ENV{IDF_PATH})

# Include the ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Add your source files here
# For example, if you have a main.c file in the src directory:
# list(APPEND EXTRA_COMPONENT_DIRS "src")
# list(APPEND EXTRA_COMPONENT_DIRS "src/directory")

# Add any additional components or libraries here
list(APPEND EXTRA_COMPONENT_DIRS "components")

# Set the project name
project(TMC2209_ESP-IDF)
