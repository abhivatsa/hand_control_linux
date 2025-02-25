cmake_minimum_required(VERSION 3.10)
project(control_module)

#
# 1) Create a STATIC library from the control sources
#    We'll call it "control_lib"
#
add_library(control_lib STATIC
    src/Control.cpp
    src/ControllerManager.cpp
    src/DriveStateManager.cpp

    # Controllers
    src/controllers/GravityCompController.cpp
    src/controllers/JointTrajectoryController.cpp
    src/controllers/TeleOpController.cpp

    # Bridge Controllers
    src/bridge_controllers/TorqueToPositionBridgingController.cpp
    src/bridge_controllers/TrajectoryAlignBridgingController.cpp

    # Hardware Abstraction
    src/hardware_abstraction/MockHardwareAbstractionLayer.cpp
    src/hardware_abstraction/RealHardwareAbstractionLayer.cpp
)

#
# 2) Enforce a specific C++ standard if needed
#
target_compile_features(control_lib PUBLIC cxx_std_17)

#
# 3) Include directories
#    - external_libraries for json.hpp or other 3rd-party single-header libs
#    - 'include' directory inside this module
#
target_include_directories(control_lib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external_libraries
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

#
# 4) Link with other libraries if needed
#    e.g. merai_foundation, fieldbus_lib, pthread, etc.
#
target_link_libraries(control_lib
    PRIVATE
        # merai_foundation
        # fieldbus_lib
        # pthread
        # ...
)

#
# 5) Create an executable named "control" (matching your systemd service ExecStart?)
#
add_executable(control src/main.cpp)
target_link_libraries(control PRIVATE control_lib)

#
# 6) Install the control LIBRARY
#
install(TARGETS control_lib
    EXPORT ControlTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

#
# 7) Install the "control" EXECUTABLE
#
install(TARGETS control
    RUNTIME DESTINATION bin
)

#
# 8) Export the library configuration (so external projects can do find_package on it)
#
install(EXPORT ControlTargets
    FILE ControlConfig.cmake
    NAMESPACE MyNamespace::
    DESTINATION lib/cmake/control
)

#
# 9) (Optional) Install the control.service file if you have one
#    This places control.service into /lib/systemd/system/
#
install(
    FILES ${CMAKE_CURRENT_SOURCE_DIR}/services/control.service
    DESTINATION lib/systemd/system
)

#
# 10) Configure CPack for generating a .deb package
#
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "control")
set(CPACK_PACKAGE_VERSION "1.0.0")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_PACKAGE_CONTACT "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Control library for real-time motion controllers")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION
    "This package provides the Control library for implementing various motion control "
    "strategies (e.g. teleop, trajectory, etc.)."
)

# If you need dependencies (e.g., merai >=1.0.0, etc.):
# set(CPACK_DEBIAN_PACKAGE_DEPENDS "merai (>= 1.0.0)")

#
# 11) (Optional) Provide a postinst script to trigger systemd daemon-reload or other tasks
#
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
   "${CMAKE_CURRENT_SOURCE_DIR}/scripts/postinst"
)

#
# Finally, include CPack to enable packaging
#
include(CPack)
