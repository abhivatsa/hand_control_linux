cmake_minimum_required(VERSION 3.10)
project(seven_axis_robot_control VERSION 1.0.0)

#
# 1) Find dependencies
#    Make sure "seven_axis_robot_core" and "seven_axis_robot_merai" are installed
#    so their config files can be found.
#
find_package(seven_axis_robot_core REQUIRED)
find_package(seven_axis_robot_merai REQUIRED)

#
# 2) Build the control library
#    The sources below come from "control/src/..."
#
add_library(seven_axis_robot_control_lib STATIC
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
target_compile_features(seven_axis_robot_control_lib PUBLIC cxx_std_17)

#
# 4) Include paths for local headers
#    (Installs to a unique directory to avoid collisions)
#
target_include_directories(seven_axis_robot_control_lib
    PUBLIC
        # If this CMakeLists is inside the 'control/' folder:
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/seven_axis_robot_control>
)

#
# 5) Link with core (single library) and merai foundation
#
target_link_libraries(seven_axis_robot_control_lib
    PUBLIC
        seven_axis_robot_core::core
        seven_axis_robot_merai::merai_foundation
)

#
# 6) Optional main executable from "control/src/main.cpp"
#
add_executable(seven_axis_robot_control_exe
    src/main.cpp
)
target_link_libraries(seven_axis_robot_control_exe
    PRIVATE
        seven_axis_robot_control_lib
)

include(CTest)
if(BUILD_TESTING)
    add_executable(controller_manager_tests
        tests/ControllerManagerTests.cpp
    )
    target_link_libraries(controller_manager_tests PRIVATE seven_axis_robot_control_lib)
    add_test(NAME controller_manager_tests COMMAND controller_manager_tests)
endif()

#
# 7) Install the library & executable
#
install(TARGETS seven_axis_robot_control_lib seven_axis_robot_control_exe
    EXPORT ControlTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

#
# 8) Install the headers under /usr/include/seven_axis_robot_control
#
install(
    DIRECTORY include/
    DESTINATION include/seven_axis_robot_control
)

#
# 9) Export the library config so other projects can do:
#    find_package(seven_axis_robot_control REQUIRED)
#    target_link_libraries(... seven_axis_robot_control::seven_axis_robot_control_lib ...)
#
install(EXPORT ControlTargets
    FILE seven_axis_robot_controlConfig.cmake
    NAMESPACE seven_axis_robot_control::
    DESTINATION lib/cmake/seven_axis_robot_control
)

#
# 10) (Optional) Install systemd service files
#
install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/services/control.service
    DESTINATION lib/systemd/system
)

#
# 11) CPack configuration for .deb
#
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "seven-axis-robot-control")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_PACKAGE_CONTACT "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Control library for real-time motion controllers on the seven-axis robot")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION
    "This package provides the control library for implementing various motion control\n"
    "strategies (e.g. teleop, trajectory, etc.) in a seven-axis robot system."
)

# If your control libs specifically depend on the 'core' or 'merai' .deb packages:
# set(CPACK_DEBIAN_PACKAGE_DEPENDS "seven-axis-robot-core (>= 1.0.0), seven-axis-robot-merai (>= 1.0.0)")

# Post-install script
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/postinst"
)

# Install prefix => /usr in the .deb
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")

include(CPack)
