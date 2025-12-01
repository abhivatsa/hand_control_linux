cmake_minimum_required(VERSION 3.10)
project(logic VERSION 1.0.0 LANGUAGES CXX)

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
# 2) Build the logic library
#    The sources below come from "logic/src/..."
#
add_library(logic_lib STATIC
    src/Logic.cpp
    src/StateMachine.cpp
    src/SafetyManager.cpp
    # src/ErrorManager.cpp
    # Add other .cpp if needed, e.g. LogicTypes.cpp
)

#
# 3) Require C++20 (for std::span in controllers)
#
target_compile_features(logic_lib PUBLIC cxx_std_20)

#
# 4) Include paths for local headers
#    (Installs to a unique directory to avoid collisions)
#
target_include_directories(logic_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/logic>
)

#
# 5) Link with core and merai
#
target_link_libraries(logic_lib
    PUBLIC
        robot_core::core
        merai::merai
)

#
# 6) Optional main executable from "logic/src/main.cpp"
#
add_executable(logic
    src/main.cpp
)
target_link_libraries(logic
    PRIVATE
        logic_lib
)

#
# 7) Install the library & executable
#
install(TARGETS logic_lib logic
    EXPORT LogicTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

#
# 8) Install the headers under /usr/include/logic
#
install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/logic
)

#
# 9) Export the library config so other projects can do:
#    find_package(logic REQUIRED)
#    target_link_libraries(... logic::logic_lib ...)
#
set(LOGIC_CMAKE_CONFIG_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/logic")

install(EXPORT LogicTargets
    FILE logicTargets.cmake
    NAMESPACE logic::
    DESTINATION ${LOGIC_CMAKE_CONFIG_DIR}
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/logicConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/logicConfig.cmake
    INSTALL_DESTINATION ${LOGIC_CMAKE_CONFIG_DIR}
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/logicConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/logicConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/logicConfigVersion.cmake
    DESTINATION ${LOGIC_CMAKE_CONFIG_DIR}
)

#
# 10) (Optional) Install systemd service files
#
install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/services/logic.service
    DESTINATION lib/systemd/system
)

#
# 11) CPack configuration for .deb (similar to control app)
#
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "seven-axis-robot-logic")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_PACKAGE_CONTACT "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Logic library for high-level state machine and safety checks on the seven-axis robot")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION
    "This package provides the logic library for orchestrating high-level state transitions,\n"
    "safety management, and error handling in a seven-axis robot system."
)

# If your logic depends on specific versions of the 'core' or 'merai' .deb packages:
# set(CPACK_DEBIAN_PACKAGE_DEPENDS "seven-axis-robot-core (>= 1.0.0), merai (>= 1.0.0)")

# Post-install script (if any)
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/postinst"
)

# Install prefix => /usr in the .deb
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")

include(CPack)
