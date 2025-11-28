cmake_minimum_required(VERSION 3.10)
project(merai VERSION 1.0.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CTest)

# ---------------------------------------------------------------------------
# Core library (canonical name: merai)
# ---------------------------------------------------------------------------
add_library(merai
    src/RAII_SharedMemory.cpp
    src/ParameterServer.cpp
)

add_library(merai::merai ALIAS merai)

target_compile_features(merai PUBLIC cxx_std_17)

target_include_directories(merai
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external_libraries
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# ---------------------------------------------------------------------------
# Logger executable (systemd journal integration)
# ---------------------------------------------------------------------------
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSYSTEMD REQUIRED libsystemd)

add_executable(merai_logger src/LoggerProcess.cpp)
target_compile_definitions(merai_logger PRIVATE USE_SYSTEMD_JOURNAL=1)
target_include_directories(merai_logger PRIVATE ${LIBSYSTEMD_INCLUDE_DIRS})
target_link_libraries(merai_logger PRIVATE merai ${LIBSYSTEMD_LIBRARIES})

# ---------------------------------------------------------------------------
# Launcher executable
# ---------------------------------------------------------------------------
set(MERAI_CONFIG_INSTALL_DIR "${CMAKE_INSTALL_DATAROOTDIR}/merai/config" CACHE PATH "Install path for merai config files")

add_executable(merai_launcher src/Launcher.cpp)
target_link_libraries(merai_launcher PRIVATE merai)
target_compile_definitions(merai_launcher
    PRIVATE
        MERAI_CONFIG_DIR_INSTALL="${MERAI_CONFIG_INSTALL_DIR}"
        MERAI_CONFIG_DIR_SOURCE="${CMAKE_SOURCE_DIR}/config"
)

# ---------------------------------------------------------------------------
# Install rules: library, executables, scripts, services, headers, configs
# ---------------------------------------------------------------------------
install(TARGETS merai
    EXPORT meraiTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(TARGETS merai_logger merai_launcher
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
    FILES
        services/merai_stack.target
        services/launcher.service
        services/logger.service
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/systemd/system
)

install(
    PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/merai
    DESTINATION ${CMAKE_INSTALL_BINDIR}
    RENAME merai_seven_axis_robot
)

install(
    DIRECTORY include/merai
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/../../config/comm_data.json
        ${CMAKE_CURRENT_SOURCE_DIR}/../../config/ethercat_config.json
        ${CMAKE_CURRENT_SOURCE_DIR}/../../config/robot_parameters.json
    DESTINATION ${MERAI_CONFIG_INSTALL_DIR}
)

# ---------------------------------------------------------------------------
# CMake package export (supports find_package(merai))
# ---------------------------------------------------------------------------
set_target_properties(merai PROPERTIES EXPORT_NAME merai)

set(MERAI_CMAKE_CONFIG_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/merai")

install(
    EXPORT meraiTargets
    FILE meraiTargets.cmake
    NAMESPACE merai::
    DESTINATION ${MERAI_CMAKE_CONFIG_DIR}
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/meraiConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/meraiConfig.cmake
    INSTALL_DESTINATION ${MERAI_CMAKE_CONFIG_DIR}
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/meraiConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/meraiConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/meraiConfigVersion.cmake
    DESTINATION ${MERAI_CMAKE_CONFIG_DIR}
)

# ---------------------------------------------------------------------------
# CPack configuration for .deb packaging
# ---------------------------------------------------------------------------
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "merai")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_PACKAGE_CONTACT "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
    "Systemd target, services, configs, and scripts for managing the merai stack"
)
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION
    "The merai package provides systemd targets, services, configs, and scripts\n"
    "that help manage the broader robot stack."
)

set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")

include(CPack)

# ---------------------------------------------------------------------------
# Tests (optional, not installed)
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
    set(DEFAULT_CONFIG_DIR "${CMAKE_SOURCE_DIR}/config")
    if(NOT EXISTS "${DEFAULT_CONFIG_DIR}/ethercat_config.json")
        set(DEFAULT_CONFIG_DIR "${CMAKE_SOURCE_DIR}/../config")
    endif()
    if(NOT EXISTS "${DEFAULT_CONFIG_DIR}/ethercat_config.json")
        set(DEFAULT_CONFIG_DIR "${CMAKE_SOURCE_DIR}/../../config")
    endif()

    add_executable(merai_param_server_tests
        tests/ParameterServerTests.cpp
    )
    target_compile_definitions(merai_param_server_tests
        PRIVATE
            SOURCE_DIR="${CMAKE_SOURCE_DIR}"
            CONFIG_DIR="${DEFAULT_CONFIG_DIR}"
    )
    target_link_libraries(merai_param_server_tests PRIVATE merai)
    add_test(NAME merai_param_server_tests COMMAND merai_param_server_tests)

    add_executable(merai_logger_tests
        tests/LoggerTests.cpp
    )
    target_link_libraries(merai_logger_tests PRIVATE merai)
    add_test(NAME merai_logger_tests COMMAND merai_logger_tests)

    add_executable(merai_rtipc_tests
        tests/RTIpcTests.cpp
    )
    target_link_libraries(merai_rtipc_tests PRIVATE merai)
    add_test(NAME merai_rtipc_tests COMMAND merai_rtipc_tests)
endif()
