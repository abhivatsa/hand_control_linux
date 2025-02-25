cmake_minimum_required(VERSION 3.10)

project(RealTimeProject VERSION 1.0 LANGUAGES CXX)

# 1) Global project settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(BUILD_TESTS "Build the test executables" ON)

# 2) Subdirectories for your various modules
#    Each module has its own CPack config and .service install
add_subdirectory(src/common)
add_subdirectory(src/launcher)
add_subdirectory(src/fieldbus)
# add_subdirectory(src/logic)
# add_subdirectory(src/control)
