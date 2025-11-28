cmake_minimum_required(VERSION 3.10)
project(fieldbus VERSION 1.0.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# ----------------------------------------------------------------------------
# Dependencies
# ----------------------------------------------------------------------------
find_package(EtherCAT REQUIRED)
if(NOT TARGET merai::merai)
    find_package(merai REQUIRED)
endif()

# ----------------------------------------------------------------------------
# Library
# ----------------------------------------------------------------------------
add_library(fieldbus STATIC
    src/EthercatMaster.cpp
    src/drives/ServoDrive.cpp
    src/drives/IODrive.cpp
)

add_library(fieldbus::fieldbus ALIAS fieldbus)
target_compile_features(fieldbus PUBLIC cxx_std_20)

target_include_directories(fieldbus
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external_libraries
)

target_link_libraries(fieldbus
    PUBLIC
        EtherLab::ethercat
        merai::merai
)

set_target_properties(fieldbus PROPERTIES EXPORT_NAME fieldbus)

# ----------------------------------------------------------------------------
# Executable (optional main)
# ----------------------------------------------------------------------------
add_executable(fieldbus_exe src/main.cpp)
target_link_libraries(fieldbus_exe PRIVATE fieldbus EtherLab::ethercat)

# ----------------------------------------------------------------------------
# Install rules
# ----------------------------------------------------------------------------
install(TARGETS fieldbus fieldbus_exe
    EXPORT fieldbusTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
    FILES services/fieldbus.service
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/systemd/system
)

install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# ----------------------------------------------------------------------------
# Package config
# ----------------------------------------------------------------------------
set(FIELDBUS_CMAKE_CONFIG_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/fieldbus")

install(
    EXPORT fieldbusTargets
    FILE fieldbusTargets.cmake
    NAMESPACE fieldbus::
    DESTINATION ${FIELDBUS_CMAKE_CONFIG_DIR}
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/fieldbusConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/fieldbusConfig.cmake
    INSTALL_DESTINATION ${FIELDBUS_CMAKE_CONFIG_DIR}
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/fieldbusConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/fieldbusConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/fieldbusConfigVersion.cmake
    DESTINATION ${FIELDBUS_CMAKE_CONFIG_DIR}
)

# ----------------------------------------------------------------------------
# CPack
# ----------------------------------------------------------------------------
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "fieldbus")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_PACKAGE_CONTACT "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Fieldbus library and service for real-time robot control")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION
    "The fieldbus package provides EtherCAT master logic, drive handling, and\n"
    "a systemd service for real-time robot control."
)
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_CURRENT_SOURCE_DIR}/scripts/postinst")

include(CPack)
