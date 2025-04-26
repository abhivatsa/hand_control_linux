cmake_minimum_required(VERSION 3.10)
project(hand_control_control VERSION 1.0.0)

#
# 1) Find dependencies
#    Make sure "hand_control_core" and "hand_control_merai" are installed
#    so their config files can be found.
#
find_package(hand_control_core REQUIRED)
find_package(hand_control_merai REQUIRED)

#
# 2) Build the control library
#    The sources below come from "control/src/..."
#
add_library(hand_control_control_lib STATIC
    src/Control.cpp
    src/ControllerManager.cpp
    src/DriveStateManager.cpp

    # Controllers
    src/controllers/GravityCompController.cpp
    src/controllers/HomingController.cpp
    
    # Hardware Abstraction
    src/hardware_abstraction/SimHAL.cpp
    src/hardware_abstraction/RealHAL.cpp
)

#
# 3) Require C++17
#
target_compile_features(hand_control_control_lib PUBLIC cxx_std_17)

#
# 4) Include paths for local headers
#    (Installs to a unique directory to avoid collisions)
#
target_include_directories(hand_control_control_lib
    PUBLIC
        # If this CMakeLists is inside the 'control/' folder:
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/hand_control_control>
)

#
# 5) Link with core (single library) and merai foundation
#
target_link_libraries(hand_control_control_lib
    PUBLIC
        hand_control_core::core
        hand_control_merai::merai_foundation
)

#
# 6) Optional main executable from "control/src/main.cpp"
#
add_executable(hand_control_control_exe
    src/main.cpp
)
target_link_libraries(hand_control_control_exe
    PRIVATE
        hand_control_control_lib
)

#
# 7) Install the library & executable
#
install(TARGETS hand_control_control_lib hand_control_control_exe
    EXPORT ControlTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

#
# 8) Install the headers under /usr/include/hand_control_control
#
install(
    DIRECTORY include/
    DESTINATION include/hand_control_control
)

#
# 9) Export the library config so other projects can do:
#    find_package(hand_control_control REQUIRED)
#    target_link_libraries(... hand_control_control::hand_control_control_lib ...)
#
install(EXPORT ControlTargets
    FILE hand_control_controlConfig.cmake
    NAMESPACE hand_control_control::
    DESTINATION lib/cmake/hand_control_control
)

#
# 10) (Optional) Install systemd service files
#
install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/../services/control.service
    DESTINATION lib/systemd/system
)

#
# 11) CPack configuration for .deb
#
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "hand-control-control")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_PACKAGE_CONTACT "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Control library for real-time motion controllers")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION
    "This package provides the control library for implementing various motion control\n"
    "strategies (e.g. teleop, trajectory, etc.) in a hand-control robotics system."
)

# If your control libs specifically depend on the 'core' or 'merai' .deb packages:
# set(CPACK_DEBIAN_PACKAGE_DEPENDS "hand-control-core (>= 1.0.0), hand-control-merai (>= 1.0.0)")

# Post-install script
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
    "${CMAKE_CURRENT_SOURCE_DIR}/../scripts/postinst"
)

# Install prefix => /usr in the .deb
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")

include(CPack)
