cmake_minimum_required(VERSION 3.16)

# Project name
project(my_project)

# Set the ESP-IDF environment
#set(ENV{IDF_PATH} "/path/to/your/esp-idf")

# Include the ESP-IDF CMake build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Project configuration
idf_configure_project()

# Specify the source files for your main application
set(app_sources
    main/main.c     # Main application file
)

# Create the executable target
idf_build_executable(my_project ${app_sources})

# Add any additional project-level configurations or dependencies here if needed
# For example, if you have any custom components outside the "components" directory,
# you would add them here using `idf_component_register`.