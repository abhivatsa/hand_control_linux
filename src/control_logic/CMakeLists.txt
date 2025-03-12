cmake_minimum_required(VERSION 3.10)
project(ControlLogic)

# Set the C++ standard (modify to whichever version you prefer)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the control module
add_subdirectory(control)

# Add the logic module
add_subdirectory(logic)
