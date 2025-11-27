cmake_minimum_required(VERSION 3.10)
project(seven_axis_robot_core VERSION 1.0.0)

#
# 0) Optionally find the seven_axis_robot_merai package (if your code uses merai_foundation)
#
find_package(seven_axis_robot_merai REQUIRED)

#
# 1) Combine all sources (math_lib + robotics_lib) into one library
#
add_library(seven_axis_robot_core STATIC
    # math_lib .cpp files
    math_lib/src/MatrixVectorOps.cpp

    # robotics_lib (haptic_device) .cpp files
    robotics_lib/src/haptic_device/HapticDeviceModel.cpp
    robotics_lib/src/haptic_device/HapticDeviceKinematics.cpp
    robotics_lib/src/haptic_device/HapticDeviceDynamics.cpp
    # Add more .cpp if needed
)

target_compile_features(seven_axis_robot_core PUBLIC cxx_std_17)

#
# 2) Combine include directories for both math_lib and robotics_lib
#
target_include_directories(seven_axis_robot_core
    PUBLIC
        # For math_lib headers
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/math_lib/include>
        # For robotics_lib headers
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/robotics_lib/include>
        # When installed, we place all headers under include/seven_axis_robot_core
        $<INSTALL_INTERFACE:include/seven_axis_robot_core>
)

#
# 3) Link to merai_foundation, if your code includes "merai/ParameterServer.h", etc.
#
target_link_libraries(seven_axis_robot_core
    PUBLIC
        seven_axis_robot_merai::merai_foundation
)

#
# 4) Set an export name => "core"
#    So downstream references it as seven_axis_robot_core::core
#
set_target_properties(seven_axis_robot_core PROPERTIES
    EXPORT_NAME core
)

#
# 5) Install the single library + headers
#
install(TARGETS seven_axis_robot_core
    EXPORT CoreTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# If your code references "math_lib/Matrix.h" or "robotics_lib/haptic_device/HapticDeviceModel.h",
# install those headers in consistent subfolders under /usr/include/seven_axis_robot_core
install(
    DIRECTORY math_lib/include/math_lib/
    DESTINATION include/seven_axis_robot_core/math_lib
)
install(
    DIRECTORY robotics_lib/include/robotics_lib/
    DESTINATION include/seven_axis_robot_core/robotics_lib
)

#
# 6) Export the package config
#    This generates seven_axis_robot_coreConfig.cmake in lib/cmake/seven_axis_robot_core
#    and uses the namespace seven_axis_robot_core::
#
install(EXPORT CoreTargets
    FILE seven_axis_robot_coreConfig.cmake
    NAMESPACE seven_axis_robot_core::
    DESTINATION lib/cmake/seven_axis_robot_core
)

#
# 7) CPack config for .deb
#
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "seven-axis-robot-core")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_PACKAGE_CONTACT "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
    "Core libraries for the seven-axis robot (HapticDevice, math ops, etc.)"
)
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION
    "This package provides the HapticDevice library (Model, Kinematics, Dynamics)\n"
    "and math library (Matrix/Vector ops) for real-time seven-axis robot applications."
)
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_CURRENT_SOURCE_DIR}/scripts/postinst")
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")

include(CPack)
