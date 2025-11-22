cmake_minimum_required(VERSION 3.10)

project(robot_linux VERSION 1.0 LANGUAGES CXX)

# 1) Global project settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(BUILD_TESTS "Build the test executables" ON)
option(BUILD_MERAI "Build merai module" ON)
# Options for other modules if/when they are added
option(BUILD_FIELDBUS "Build fieldbus module" OFF)
option(BUILD_LOGIC "Build logic module" OFF)
option(BUILD_CONTROL "Build control module" OFF)

# 2) Subdirectories for available modules
if(BUILD_MERAI)
    add_subdirectory(src/merai)
endif()

if(BUILD_FIELDBUS)
    add_subdirectory(src/fieldbus)
endif()

if(BUILD_LOGIC)
    add_subdirectory(src/logic)
endif()

if(BUILD_CONTROL)
    add_subdirectory(src/control)
endif()
