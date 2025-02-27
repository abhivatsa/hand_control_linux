cmake_minimum_required(VERSION 3.10)
project(hand_control_core VERSION 1.0.0)

#
# 0) Optionally find the hand_control_merai package (if your code uses merai_foundation)
#
find_package(hand_control_merai REQUIRED)

#
# 1) Combine all sources (math_lib + robotics_lib) into one library
#
add_library(hand_control_core STATIC
    # math_lib .cpp files
    math_lib/src/MatrixVectorOps.cpp

    # robotics_lib (haptic_device) .cpp files
    robotics_lib/src/haptic_device/HapticDeviceModel.cpp
    robotics_lib/src/haptic_device/HapticDeviceKinematics.cpp
    robotics_lib/src/haptic_device/HapticDeviceDynamics.cpp
    # Add more .cpp if needed
)

target_compile_features(hand_control_core PUBLIC cxx_std_17)

#
# 2) Combine include directories for both math_lib and robotics_lib
#
target_include_directories(hand_control_core
    PUBLIC
        # For math_lib headers
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/math_lib/include>
        # For robotics_lib headers
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/robotics_lib/include>
        # When installed, we place all headers under include/hand_control_core
        $<INSTALL_INTERFACE:include/hand_control_core>
)

#
# 3) Link to merai_foundation, if your code includes "merai/ParameterServer.h", etc.
#
target_link_libraries(hand_control_core
    PUBLIC
        hand_control_merai::merai_foundation
)

#
# 4) Set an export name => "core"
#    So downstream references it as hand_control_core::core
#
set_target_properties(hand_control_core PROPERTIES
    EXPORT_NAME core
)

#
# 5) Install the single library + headers
#
install(TARGETS hand_control_core
    EXPORT CoreTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# If your code references "math_lib/Matrix.h" or "robotics_lib/haptic_device/HapticDeviceModel.h",
# install those headers in consistent subfolders under /usr/include/hand_control_core
install(
    DIRECTORY math_lib/include/math_lib/
    DESTINATION include/hand_control_core/math_lib
)
install(
    DIRECTORY robotics_lib/include/robotics_lib/
    DESTINATION include/hand_control_core/robotics_lib
)

#
# 6) Export the package config
#    This generates hand_control_coreConfig.cmake in lib/cmake/hand_control_core
#    and uses the namespace hand_control_core::
#
install(EXPORT CoreTargets
    FILE hand_control_coreConfig.cmake
    NAMESPACE hand_control_core::
    DESTINATION lib/cmake/hand_control_core
)

#
# 7) CPack config for .deb
#
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "hand-control-core")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_PACKAGE_CONTACT "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
    "Core libraries for real-time hand control (HapticDevice, math ops, etc.)"
)
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION
    "This package provides the HapticDevice library (Model, Kinematics, Dynamics)\n"
    "and math library (Matrix/Vector ops) for real-time hand control applications."
)
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_CURRENT_SOURCE_DIR}/scripts/postinst")
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")

include(CPack)
