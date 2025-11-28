cmake_minimum_required(VERSION 3.10)
project(seven_axis_robot_logic VERSION 1.0.0 LANGUAGES CXX)

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
add_library(seven_axis_robot_logic_lib STATIC
    src/Logic.cpp
    src/StateMachine.cpp
    src/SafetyManager.cpp
    # src/ErrorManager.cpp
    # Add other .cpp if needed, e.g. LogicTypes.cpp
)

#
# 3) Require C++20 (for std::span in controllers)
#
target_compile_features(seven_axis_robot_logic_lib PUBLIC cxx_std_20)

#
# 4) Include paths for local headers
#    (Installs to a unique directory to avoid collisions)
#
target_include_directories(seven_axis_robot_logic_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/seven_axis_robot_logic>
)

#
# 5) Link with core and merai
#
target_link_libraries(seven_axis_robot_logic_lib
    PUBLIC
        robot_core::core
        merai::merai
)

#
# 6) Optional main executable from "logic/src/main.cpp"
#
add_executable(seven_axis_robot_logic_exe
    src/main.cpp
)
target_link_libraries(seven_axis_robot_logic_exe
    PRIVATE
        seven_axis_robot_logic_lib
)

#
# 7) Install the library & executable
#
install(TARGETS seven_axis_robot_logic_lib seven_axis_robot_logic_exe
    EXPORT LogicTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

#
# 8) Install the headers under /usr/include/seven_axis_robot_logic
#
install(
    DIRECTORY include/
    DESTINATION include/seven_axis_robot_logic
)

#
# 9) Export the library config so other projects can do:
#    find_package(seven_axis_robot_logic REQUIRED)
#    target_link_libraries(... seven_axis_robot_logic::seven_axis_robot_logic_lib ...)
#
install(EXPORT LogicTargets
    FILE seven_axis_robot_logicConfig.cmake
    NAMESPACE seven_axis_robot_logic::
    DESTINATION lib/cmake/seven_axis_robot_logic
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
