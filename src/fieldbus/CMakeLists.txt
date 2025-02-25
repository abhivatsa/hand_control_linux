cmake_minimum_required(VERSION 3.10)
project(motion_control_fieldbus VERSION 1.0.0)

# 1) Find EtherCAT (assuming you have FindEtherCAT.cmake or similar).
#    If you have a different find script or are linking manually, adapt accordingly.
find_package(EtherCAT REQUIRED)

# Example: If merai_foundation is also a package/library:
# find_package(merai_foundation REQUIRED)

# 2) Build a static library for the fieldbus code
add_library(motion_control_fieldbus_lib STATIC
    src/EthercatMaster.cpp
    src/drives/IoDrive.cpp
    src/drives/ServoDrive.cpp
    # Add more .cpp as needed
)

# Ensure at least C++17 features
target_compile_features(motion_control_fieldbus_lib PUBLIC cxx_std_17)

target_include_directories(motion_control_fieldbus_lib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external_libraries
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Link your external libraries (merai_foundation, EtherLab::ethercat, pthread, etc.)
target_link_libraries(motion_control_fieldbus_lib
    PRIVATE
        merai_foundation
        EtherLab::ethercat
)

# 3) Create the 'motion_control_fieldbus' executable (the main entry point)
add_executable(motion_control_fieldbus src/main.cpp)
target_link_libraries(motion_control_fieldbus PRIVATE
    motion_control_fieldbus_lib
    EtherLab::ethercat
)

# 4) Install the 'motion_control_fieldbus' binary
install(TARGETS motion_control_fieldbus
    RUNTIME DESTINATION bin
)

# 5) Install the static library
install(TARGETS motion_control_fieldbus_lib
    EXPORT FieldbusTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# 6) Install the fieldbus.service file to /lib/systemd/system
install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/services/fieldbus.service
    DESTINATION lib/systemd/system
)

# 7) CPack configuration for creating a .deb package
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "fieldbus")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_PACKAGE_CONTACT "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Fieldbus library for real-time motion control")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION
    "This package provides the Fieldbus library (EthercatMaster, IoDrive, ServoDrive, etc.)\n"
    "for real-time motion control applications."
)
# Example dependency on 'merai' package
set(CPACK_DEBIAN_PACKAGE_DEPENDS "merai (>= 1.0.0)")
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_CURRENT_SOURCE_DIR}/scripts/postinst")

# Install prefix for packaging (using /usr instead of /usr/local by default)
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")

include(CPack)
