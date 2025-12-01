cmake_minimum_required(VERSION 3.10)
project(control VERSION 1.0.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

#
# 1) Find dependencies
#    Prefer in-tree targets first; fall back to find_package when building out of tree.
#    Requires "robot_core" (robot_core::core) and "merai" (merai::merai).
#
if(NOT TARGET robot_core::core)
    find_package(robot_core REQUIRED)
endif()
if(NOT TARGET merai::merai)
    find_package(merai REQUIRED)
endif()

#
# 2) Build the control library
#    The sources below come from "control/src/..."
#
add_library(control_lib STATIC
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
# 3) Require C++20 (for std::span, etc.)
#
target_compile_features(control_lib PUBLIC cxx_std_20)

#
#
# 4) Include paths for local headers
#    (Installs to a unique directory to avoid collisions)
#    Build interface: control/include
#    Install interface: ${CMAKE_INSTALL_INCLUDEDIR}/control
#
target_include_directories(control_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/control>
)

#
# 5) Link with core (single library) and merai
#
target_link_libraries(control_lib
    PUBLIC
        robot_core::core
        merai::merai
)

#
# 6) Optional main executable from "control/src/main.cpp"
#
add_executable(control
    src/main.cpp
)
target_link_libraries(control
    PRIVATE
        control_lib
)

include(CTest)
if(BUILD_TESTING)
    add_executable(controller_manager_tests
        tests/ControllerManagerTests.cpp
    )
    target_link_libraries(controller_manager_tests PRIVATE control_lib)
    add_test(NAME controller_manager_tests COMMAND controller_manager_tests)
endif()

#
# 7) Install the library & executable
#
install(TARGETS control_lib control
    EXPORT ControlTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

#
# 8) Install the headers under /usr/include/control
#
install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/control
)

#
# 9) CMake package export (supports find_package(control))
#    and a version file similar to other modules.
#
set(CONTROL_CMAKE_CONFIG_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/control")

install(
    EXPORT ControlTargets
    FILE controlTargets.cmake
    NAMESPACE control::
    DESTINATION ${CONTROL_CMAKE_CONFIG_DIR}
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/controlConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/controlConfig.cmake
    INSTALL_DESTINATION ${CONTROL_CMAKE_CONFIG_DIR}
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/controlConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/controlConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/controlConfigVersion.cmake
    DESTINATION ${CONTROL_CMAKE_CONFIG_DIR}
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
# set(CPACK_DEBIAN_PACKAGE_DEPENDS "seven-axis-robot-core (>= 1.0.0), merai (>= 1.0.0)")

# Post-install script
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/postinst"
)

# Install prefix => /usr in the .deb
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")

include(CPack)
