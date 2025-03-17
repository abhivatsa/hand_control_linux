cmake_minimum_required(VERSION 3.10)
project(hand_control_fieldbus VERSION 1.0.0)

# ----------------------------------------------------------------------------
# 1) Find EtherLab (or EtherCAT), plus hand_control_merai
#    Adjust names to match your actual find scripts / config files
# ----------------------------------------------------------------------------
find_package(EtherCAT REQUIRED)
find_package(hand_control_merai REQUIRED)

# ----------------------------------------------------------------------------
# 2) Create a STATIC library for the fieldbus code
# ----------------------------------------------------------------------------
add_library(hand_control_fieldbus_lib STATIC
    src/EthercatMaster.cpp
    src/drives/ServoDrive.cpp
    # Add more .cpp files as needed
)

target_compile_features(hand_control_fieldbus_lib PUBLIC cxx_std_17)

# If you absolutely need ../../external_libraries just for building,
# mark it PRIVATE so it won't cause conflicts in the exported config:
target_include_directories(hand_control_fieldbus_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/hand_control_fieldbus>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external_libraries
)

# Link your external libraries (EtherLab + merai_foundation)
target_link_libraries(hand_control_fieldbus_lib
    PUBLIC
        EtherLab::ethercat           # or EtherCAT::EtherCAT if your find script calls it that
        hand_control_merai::merai_foundation
)

# Export name => hand_control_fieldbus::hand_control_fieldbus_lib
set_target_properties(hand_control_fieldbus_lib PROPERTIES
    EXPORT_NAME hand_control_fieldbus_lib
)

# ----------------------------------------------------------------------------
# 3) Create the 'hand_control_fieldbus_exe' (optional main.cpp)
# ----------------------------------------------------------------------------
add_executable(hand_control_fieldbus_exe src/main.cpp)
target_link_libraries(hand_control_fieldbus_exe PRIVATE
    hand_control_fieldbus_lib
    EtherLab::ethercat
)

# ----------------------------------------------------------------------------
# 4) Install the library & executable
# ----------------------------------------------------------------------------
install(TARGETS hand_control_fieldbus_lib hand_control_fieldbus_exe
    EXPORT FieldbusTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# ----------------------------------------------------------------------------
# 5) Install the systemd service (optional)
# ----------------------------------------------------------------------------
install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/services/fieldbus.service
    DESTINATION lib/systemd/system
)

# ----------------------------------------------------------------------------
# 6) Install headers to a distinct path to avoid collisions
# ----------------------------------------------------------------------------
install(
    DIRECTORY include/
    DESTINATION include/hand_control_fieldbus
)

# ----------------------------------------------------------------------------
# 7) Export the package config (hand_control_fieldbusConfig.cmake)
#    So that other projects can do:
#       find_package(hand_control_fieldbus REQUIRED)
#       target_link_libraries(... hand_control_fieldbus::hand_control_fieldbus_lib ...)
# ----------------------------------------------------------------------------
install(EXPORT FieldbusTargets
    FILE hand_control_fieldbusConfig.cmake
    NAMESPACE hand_control_fieldbus::
    DESTINATION lib/cmake/hand_control_fieldbus
)

# ----------------------------------------------------------------------------
# 8) CPack configuration for creating a .deb package
# ----------------------------------------------------------------------------
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "hand-control-fieldbus")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_PACKAGE_CONTACT "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Fieldbus library for real-time hand control")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION
    "This package provides the Fieldbus library (EthercatMaster, IoDrive, ServoDrive, etc.)\n"
    "for real-time hand control applications."
)

# If your libs depend on other packages:
# set(CPACK_DEBIAN_PACKAGE_DEPENDS "some_other_package (>= 1.0.0)")

set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")

# If you have a post-install script
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_CURRENT_SOURCE_DIR}/scripts/postinst")

include(CPack)
