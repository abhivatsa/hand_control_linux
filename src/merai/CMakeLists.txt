cmake_minimum_required(VERSION 3.10)
project(motion_control_merai VERSION 1.0.0)

# 1) Foundation Library (merai_foundation)
#    Provides functionalities like RAII_SharedMemory, ParameterServer, etc.
#    We exclude LoggerProcess.cpp to build it as a separate executable.
add_library(merai_foundation
    src/RAII_SharedMemory.cpp
    src/ParameterServer.cpp
    # src/Watchdog.cpp  # Uncomment if Watchdog is implemented in this module
    # ... add more .cpp files as needed
)

target_include_directories(merai_foundation
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external_libraries
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 2) Logger Executable (merai_logger)
#    Builds the logger process from LoggerProcess.cpp, linking merai_foundation.
add_executable(merai_logger
    src/LoggerProcess.cpp
)

target_link_libraries(merai_logger
    PRIVATE
        merai_foundation
)

# 3) Launcher Executable (merai_launcher)
#    For "launcher" logic, also links merai_foundation.
add_executable(merai_launcher
    src/Launcher.cpp
)

target_link_libraries(merai_launcher
    PRIVATE
        merai_foundation
)

# 4) Installation of Libraries, Binaries, Scripts, and Services
# 4a) Foundation library
install(TARGETS merai_foundation
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

# 4d) Install the merai script to /usr/bin (or /bin)
install(
    PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/merai
    DESTINATION bin
)

# 4e) (Optional) Public headers, if other modules require them
install(
    DIRECTORY include/merai
    DESTINATION include
)

# 5) Install JSON config files (defaults to /etc/merai_configs)
install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/../../config/comm_data.json
        ${CMAKE_CURRENT_SOURCE_DIR}/../../config/ethercat_config.json
        ${CMAKE_CURRENT_SOURCE_DIR}/../../config/robot_parameters.json
        ${CMAKE_CURRENT_SOURCE_DIR}/../../config/startup_config.json
    DESTINATION etc/merai_configs
)

# 6) CPack Configuration for .deb Packaging
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "merai")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_PACKAGE_CONTACT "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
    "Systemd target, services, configs, and scripts for managing the robot stack"
)
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION
    "The Merai package provides systemd targets, services, configs, and scripts\n"
    "that help manage the broader robot stack."
)

# Uncomment or add package dependencies, etc.
# set(CPACK_DEBIAN_PACKAGE_DEPENDS "fieldbus (>= 1.0.0)")
# set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_CURRENT_SOURCE_DIR}/scripts/postinst")

# Install to /usr vs. /usr/local by default
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")

include(CPack)
