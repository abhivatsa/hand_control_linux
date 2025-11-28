cmake_minimum_required(VERSION 3.16)
project(robot_core VERSION 1.0.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# ----------------------------------------------------------------------------
# Dependencies
# ----------------------------------------------------------------------------
if(NOT TARGET merai::merai)
    find_package(merai REQUIRED)
endif()

# ----------------------------------------------------------------------------
# Library (math_lib + robotics_lib in one)
# ----------------------------------------------------------------------------
add_library(robot_core STATIC
    # math_lib sources
    math_lib/src/MatrixVectorOps.cpp

    # robotics_lib sources
    robotics_lib/src/RobotModel.cpp
    robotics_lib/src/RobotKinematics.cpp
    robotics_lib/src/RobotDynamics.cpp
)

# Build-tree friendly alias (so you can link robot_core::core before install)
add_library(robot_core::core ALIAS robot_core)

# Language / flags
target_compile_features(robot_core PUBLIC cxx_std_20)
set_target_properties(robot_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Includes (build vs install)
target_include_directories(robot_core
    BEFORE
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/math_lib/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/robotics_lib/include>
        $<INSTALL_INTERFACE:include/robot_core>
)

# Link deps
target_link_libraries(robot_core
    PUBLIC merai::merai
)

# Export name so the installed target is robot_core::core
set_target_properties(robot_core PROPERTIES EXPORT_NAME core)

# ----------------------------------------------------------------------------
# Install
# ----------------------------------------------------------------------------
install(TARGETS robot_core
    EXPORT CoreTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install headers under /usr/include/robot_core/{math_lib,robotics_lib}
install(
    DIRECTORY math_lib/include/math_lib/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot_core/math_lib
)
install(
    DIRECTORY robotics_lib/include/robotics_lib/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot_core/robotics_lib
)

# ----------------------------------------------------------------------------
# Package config (find_package(robot_core REQUIRED) -> robot_core::core)
# ----------------------------------------------------------------------------
set(ROBOT_CORE_CMAKE_CONFIG_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/robot_core")

install(
    EXPORT CoreTargets
    FILE robot_coreTargets.cmake
    NAMESPACE robot_core::
    DESTINATION ${ROBOT_CORE_CMAKE_CONFIG_DIR}
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/robot_coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/robot_coreConfig.cmake
    INSTALL_DESTINATION ${ROBOT_CORE_CMAKE_CONFIG_DIR}
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/robot_coreConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/robot_coreConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/robot_coreConfigVersion.cmake
    DESTINATION ${ROBOT_CORE_CMAKE_CONFIG_DIR}
)

# ----------------------------------------------------------------------------
# CPack (.deb)
# ----------------------------------------------------------------------------
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "seven-axis-robot-core")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_PACKAGE_CONTACT           "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Core math and seven-axis robot libraries")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION
"Core libraries for seven-axis robot applications:\n\
- math_lib (Matrix/Vector, quaternion, utilities)\n\
- robotics_lib (RobotModel/RobotKinematics/RobotDynamics).")

# If you want hard deps:
# set(CPACK_DEBIAN_PACKAGE_DEPENDS "merai (>= 1.0.0)")

set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_CURRENT_SOURCE_DIR}/scripts/postinst")
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")

include(CPack)
