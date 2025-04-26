cmake_minimum_required(VERSION 3.10)
project(hand_control_logic VERSION 1.0.0)

#
# 1) Find dependencies
#    Make sure "hand_control_core" and "hand_control_merai" are installed
#
find_package(hand_control_core REQUIRED)
find_package(hand_control_merai REQUIRED)

#
# 2) Build the logic library
#    The sources below come from "logic/src/..."
#
add_library(hand_control_logic_lib STATIC
    src/Logic.cpp
    src/StateMachine.cpp
    src/SafetyManager.cpp
    # src/ErrorManager.cpp
    # Add other .cpp if needed, e.g. LogicTypes.cpp
)

#
# 3) Require C++17
#
target_compile_features(hand_control_logic_lib PUBLIC cxx_std_17)

#
# 4) Include paths for local headers
#    (Installs to a unique directory to avoid collisions)
#
target_include_directories(hand_control_logic_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/hand_control_logic>
)

#
# 5) Link with core and merai
#
target_link_libraries(hand_control_logic_lib
    PUBLIC
        hand_control_core::core
        hand_control_merai::merai_foundation
)

#
# 6) Optional main executable from "logic/src/main.cpp"
#
add_executable(hand_control_logic_exe
    src/main.cpp
)
target_link_libraries(hand_control_logic_exe
    PRIVATE
        hand_control_logic_lib
)

#
# 7) Install the library & executable
#
install(TARGETS hand_control_logic_lib hand_control_logic_exe
    EXPORT LogicTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

#
# 8) Install the headers under /usr/include/hand_control_logic
#
install(
    DIRECTORY include/
    DESTINATION include/hand_control_logic
)

#
# 9) Export the library config so other projects can do:
#    find_package(hand_control_logic REQUIRED)
#    target_link_libraries(... hand_control_logic::hand_control_logic_lib ...)
#
install(EXPORT LogicTargets
    FILE hand_control_logicConfig.cmake
    NAMESPACE hand_control_logic::
    DESTINATION lib/cmake/hand_control_logic
)

#
# 10) (Optional) Install systemd service files
#
install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/../services/logic.service
    DESTINATION lib/systemd/system
)

#
# 11) CPack configuration for .deb (similar to control app)
#
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "hand-control-logic")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_PACKAGE_CONTACT "Abhishek Vatsa <abhi.vatsa16@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Logic library for high-level state machine and safety checks")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION
    "This package provides the logic library for orchestrating high-level state transitions,\n"
    "safety management, and error handling in a hand-control robotics system."
)

# If your logic depends on specific versions of the 'core' or 'merai' .deb packages:
# set(CPACK_DEBIAN_PACKAGE_DEPENDS "hand-control-core (>= 1.0.0), hand-control-merai (>= 1.0.0)")

# Post-install script (if any)
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
    "${CMAKE_CURRENT_SOURCE_DIR}/../scripts/postinst"
)

# Install prefix => /usr in the .deb
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")

include(CPack)
