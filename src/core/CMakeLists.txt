cmake_minimum_required(VERSION 3.10)
project(hand_control_core VERSION 1.0.0)

# ----------------------------------------------------------------------------
# 0) Find the hand_control_merai package so we can link to merai_foundation
#    This is critical if HapticDeviceModel code includes "merai/ParameterServer.h"
# ----------------------------------------------------------------------------
find_package(hand_control_merai REQUIRED)

# ----------------------------------------------------------------------------
# 1) math_lib
#    Build a STATIC library for math operations (MatrixVectorOps.cpp, etc.)
# ----------------------------------------------------------------------------
add_library(math_lib STATIC
    math_lib/src/MatrixVectorOps.cpp
    # Add more .cpp files under math_lib/src if needed
)

target_compile_features(math_lib PUBLIC cxx_std_17)

# Set include paths so that "math_lib/MatrixVectorOps.h" etc. can be found
target_include_directories(math_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/math_lib/include>
        $<INSTALL_INTERFACE:include/hand_control_core/math_lib>
)

# Export name => hand_control::math_lib
set_target_properties(math_lib PROPERTIES
    EXPORT_NAME math_lib
)

# ----------------------------------------------------------------------------
# 2) robotics_lib_haptic_device
#    Build a STATIC library for the HapticDevice code
# ----------------------------------------------------------------------------
add_library(robotics_lib_haptic_device STATIC
    robotics_lib/src/haptic_device/HapticDeviceModel.cpp
    robotics_lib/src/haptic_device/HapticDeviceKinematics.cpp
    robotics_lib/src/haptic_device/HapticDeviceDynamics.cpp
)

target_compile_features(robotics_lib_haptic_device PUBLIC cxx_std_17)

# Include paths for "robotics_lib/haptic_device/HapticDeviceModel.h", etc.
target_include_directories(robotics_lib_haptic_device
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/robotics_lib/include>
        $<INSTALL_INTERFACE:include/hand_control_core/robotics_lib>
)

# Link robotics_lib_haptic_device to math_lib AND merai_foundation
# so that "merai/FixedString.h" is found
target_link_libraries(robotics_lib_haptic_device
    PUBLIC
        math_lib
        hand_control_merai::merai_foundation
)

# Export name => hand_control::robotics_lib_haptic_device
set_target_properties(robotics_lib_haptic_device PROPERTIES
    EXPORT_NAME robotics_lib_haptic_device
)

# ----------------------------------------------------------------------------
# 3) (Optional) If you have a main.cpp for the core module, create an executable
# ----------------------------------------------------------------------------
# add_executable(hand_control_core_exe src/main.cpp)
# target_link_libraries(hand_control_core_exe
#     PRIVATE
#         robotics_lib_haptic_device
# )
#
# install(TARGETS hand_control_core_exe
#     RUNTIME DESTINATION bin
# )

# ----------------------------------------------------------------------------
# 4) Install the libraries & headers
# ----------------------------------------------------------------------------
install(TARGETS math_lib robotics_lib_haptic_device
    EXPORT CoreTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Place math_lib headers under /usr/include/hand_control_core/math_lib
install(
    DIRECTORY math_lib/include/math_lib/
    DESTINATION include/hand_control_core/math_lib
)

# Place robotics_lib headers under /usr/include/hand_control_core/robotics_lib
install(
    DIRECTORY robotics_lib/include/robotics_lib/
    DESTINATION include/hand_control_core/robotics_lib
)

# ----------------------------------------------------------------------------
# 5) Export the package config for find_package
# ----------------------------------------------------------------------------
install(EXPORT CoreTargets
    FILE hand_control_coreConfig.cmake
    NAMESPACE hand_control::
    DESTINATION lib/cmake/hand_control_core
)

# ----------------------------------------------------------------------------
# 6) CPack config for generating a .deb
# ----------------------------------------------------------------------------
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "hand-control-core")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_PACKAGE_CONTACT "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Core libraries for real-time hand control (HapticDevice, math ops, etc.)")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION
    "This package provides the HapticDevice library (Model, Kinematics, Dynamics)\n"
    "and math library (Matrix/Vector ops) for real-time hand control applications."
)

# If your libs depend on other packages:
# set(CPACK_DEBIAN_PACKAGE_DEPENDS "some_other_package (>= 1.0.0)")

# If you have a post-install script:
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_CURRENT_SOURCE_DIR}/scripts/postinst")

# Where files will be placed when installed:
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")

include(CPack)
