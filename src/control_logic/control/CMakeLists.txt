cmake_minimum_required(VERSION 3.10)
project(hand_control_control VERSION 1.0.0)

# 1) Find the installed core package.
#    This requires that hand_control_coreConfig.cmake is on CMake’s search path,
#    typically installed to /usr/lib/cmake/hand_control_core or similar.
find_package(hand_control_core REQUIRED)

# 2) Build the control library
add_library(hand_control_control_lib STATIC
    src/Control.cpp
    src/ControllerManager.cpp
    src/DriveStateManager.cpp

    # Controllers
    src/controllers/GravityCompController.cpp
    src/controllers/JointTrajectoryController.cpp
    src/controllers/TeleOpController.cpp

    # Bridge Controllers
    src/bridge_controllers/TorqueToPositionBridgingController.cpp
    src/bridge_controllers/TrajectoryAlignBridgingController.cpp

    # Hardware Abstraction
    src/hardware_abstraction/MockHardwareAbstractionLayer.cpp
    src/hardware_abstraction/RealHardwareAbstractionLayer.cpp
)

# 3) Require C++17
target_compile_features(hand_control_control_lib PUBLIC cxx_std_17)

# 4) If you have local headers in this "control" project, expose them:
target_include_directories(hand_control_control_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 5) Link to the core libraries that were found via find_package.
#    Since your core library exports "hand_control::math_lib" and
#    "hand_control::robotics_lib_6axis", we link to whichever is needed.
target_link_libraries(hand_control_control_lib
    PUBLIC
        hand_control::robotics_lib_6axis
        hand_control::math_lib   # Uncomment if your code directly uses math_lib calls
        merai_foundation
)

# 6) Create an executable (optional) from main.cpp
add_executable(hand_control_control src/main.cpp)
target_link_libraries(hand_control_control PRIVATE hand_control_control_lib)

# 7) Install the library
install(TARGETS hand_control_control_lib
    EXPORT ControlTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# 8) Install the executable
install(TARGETS hand_control_control
    RUNTIME DESTINATION bin
)

# 9) Export the library config so other projects can find this "control" library
install(EXPORT ControlTargets
    FILE hand_control_controlConfig.cmake
    NAMESPACE hand_control::
    DESTINATION lib/cmake/control
)

# 10) (Optional) Install your systemd service file
install(
    FILES ${CMAKE_CURRENT_SOURCE_DIR}/services/control.service
    DESTINATION lib/systemd/system
)

# 11) CPack config for .deb
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "control")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_PACKAGE_CONTACT "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Control library for real-time motion controllers")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION
    "This package provides the Control library for implementing various motion control "
    "strategies (e.g. teleop, trajectory, etc.)."
)

# If you have dependencies on the 'core' package, you can list them:
# set(CPACK_DEBIAN_PACKAGE_DEPENDS "core (>= 1.0.0)")

set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/postinst"
)

# (Optionally) use /usr as the default install prefix in .deb packages
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")

include(CPack)
