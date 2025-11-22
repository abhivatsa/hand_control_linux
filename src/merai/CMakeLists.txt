cmake_minimum_required(VERSION 3.10)
project(hand_control_merai VERSION 1.0.0)

include(GNUInstallDirs)
include(CTest)

include(GNUInstallDirs)

#
# 1) Foundation Library (merai_foundation)
#    Provides functionalities like RAII_SharedMemory, ParameterServer, etc.
#    We exclude LoggerProcess.cpp to build it as a separate executable.
#
add_library(merai_foundation
    src/RAII_SharedMemory.cpp
    src/ParameterServer.cpp
    # src/Watchdog.cpp  # Uncomment if Watchdog is implemented in this module
    # ... add more .cpp files as needed
)

target_compile_features(merai_foundation PUBLIC cxx_std_17)

target_include_directories(merai_foundation
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external_libraries  # For internal build only
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>   # for building from source
        $<INSTALL_INTERFACE:include/hand_control_merai>          # for installed usage
)


# target_include_directories(merai_foundation
#     PUBLIC
#         # If you have other external includes:
#         ${CMAKE_CURRENT_SOURCE_DIR}/../../external_libraries
#         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#         # Install => unique directory to avoid conflicts
#         $<INSTALL_INTERFACE:include/hand_control_merai>
# )

#
# 2) Logger Executable (merai_logger)
#
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSYSTEMD REQUIRED libsystemd)

add_executable(merai_logger
    src/LoggerProcess.cpp
)

target_compile_definitions(merai_logger PRIVATE USE_SYSTEMD_JOURNAL=1)
target_include_directories(merai_logger PRIVATE ${LIBSYSTEMD_INCLUDE_DIRS})
target_link_libraries(merai_logger
    PRIVATE
        merai_foundation
        ${LIBSYSTEMD_LIBRARIES}
)

#
# 3) Launcher Executable (merai_launcher)
#
add_executable(merai_launcher
    src/Launcher.cpp
)

target_link_libraries(merai_launcher
    PRIVATE
        merai_foundation
)
target_compile_definitions(merai_launcher
    PRIVATE
        MERAI_CONFIG_DIR_INSTALL="${MERAI_CONFIG_INSTALL_DIR}"
        MERAI_CONFIG_DIR_SOURCE="${CMAKE_SOURCE_DIR}/config"
)

#
# 4) Installation of Library, Binaries, Scripts, and Services
#

# 4a) Foundation library (exported so we can create a package config file)
install(TARGETS merai_foundation
    EXPORT MeraiTargets            # <--- IMPORTANT FOR CMAKE EXPORT
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

# 4b) Logger & Launcher executables
install(TARGETS merai_logger merai_launcher
    RUNTIME DESTINATION bin
)

# 4c) Systemd services & target files
install(
    FILES
        services/merai_stack.target
        services/launcher.service
        services/logger.service
    DESTINATION lib/systemd/system
)

# 4d) Install the script under a unique name to avoid conflicts
install(
    PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/merai
    DESTINATION bin
    RENAME merai_hand_control
)

# 4e) Install headers under include/hand_control_merai/...
install(
    DIRECTORY include/merai
    DESTINATION include/hand_control_merai
)

# Config install directory (configurable)
set(MERAI_CONFIG_INSTALL_DIR "${CMAKE_INSTALL_DATAROOTDIR}/hand_control_merai/config" CACHE PATH "Install path for merai config files")

# 4f) Install JSON config files to a unique directory
install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/../../config/comm_data.json
        ${CMAKE_CURRENT_SOURCE_DIR}/../../config/ethercat_config.json
        ${CMAKE_CURRENT_SOURCE_DIR}/../../config/robot_parameters.json
        ${CMAKE_CURRENT_SOURCE_DIR}/../../config/startup_config.json
    DESTINATION ${MERAI_CONFIG_INSTALL_DIR}
)

#
# 5) Export the package config
#
# This step is CRITICAL. It creates "hand_control_meraiConfig.cmake" so that
# other projects can do "find_package(hand_control_merai REQUIRED)" and link
# to "hand_control_merai::merai_foundation".
#
set_target_properties(merai_foundation PROPERTIES
    EXPORT_NAME merai_foundation
)

install(
    EXPORT MeraiTargets
    FILE hand_control_meraiConfig.cmake
    NAMESPACE hand_control_merai::
    DESTINATION lib/cmake/hand_control_merai
)

#
# 6) CPack Configuration for .deb Packaging
#
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "hand-control-merai")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_PACKAGE_CONTACT "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
    "Systemd target, services, configs, and scripts for managing the hand-control robot stack"
)
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION
    "The hand-control-merai package provides systemd targets, services, configs, and scripts\n"
    "that help manage the broader hand control robot stack."
)

# If your libs depend on other packages:
# set(CPACK_DEBIAN_PACKAGE_DEPENDS "fieldbus (>= 1.0.0)")

# If you have a post-install script:
# set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_CURRENT_SOURCE_DIR}/scripts/postinst")

set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")

include(CPack)

# ----------------------------------------------------------
# 7) Tests (optional, not installed)
# ----------------------------------------------------------
if(BUILD_TESTING)
    add_executable(merai_param_server_tests
        tests/ParameterServerTests.cpp
    )
    target_compile_definitions(merai_param_server_tests PRIVATE SOURCE_DIR="${CMAKE_SOURCE_DIR}")
    target_link_libraries(merai_param_server_tests PRIVATE merai_foundation)
    add_test(NAME merai_param_server_tests COMMAND merai_param_server_tests)

    add_executable(merai_logger_tests
        tests/LoggerTests.cpp
    )
    target_link_libraries(merai_logger_tests PRIVATE merai_foundation)
    add_test(NAME merai_logger_tests COMMAND merai_logger_tests)
endif()
