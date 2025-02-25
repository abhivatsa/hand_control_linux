# Located at core/robotics_lib/CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(robotics_lib_6axis)

# Create a library from the .cpp files
add_library(robotics_lib_6axis
    src/six_axis/SixAxisModel.cpp
    src/six_axis/SixAxisKinematics.cpp
    src/six_axis/SixAxisDynamics.cpp
)

# Public include directory for other code to #include <robotics_lib/six_axis/...>
target_include_directories(robotics_lib_6axis PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# If you use your math library in e.g. core/math_lib:
# find_package(math_lib REQUIRED) or add_subdirectory(...) if it's part of the same project
# Link to it if needed:
# target_link_libraries(robotics_lib_6axis PUBLIC math_lib)

# (Optional) If you want to install the library/headers:
# install(TARGETS robotics_lib_6axis
#         DESTINATION lib)
# install(DIRECTORY include/robotics_lib
#         DESTINATION include)

# You can also add tests here if you want, or in a separate tests/CMakeLists.txt
