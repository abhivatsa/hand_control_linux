cmake_minimum_required(VERSION 3.16)

project(planner LANGUAGES CXX)

# Library with core planner logic
add_library(planner_core STATIC
    src/PlannerApp.cpp
    src/JointTrajectoryPlanner.cpp
)

add_library(planner::core ALIAS planner_core)

target_include_directories(planner_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(planner_core
    PUBLIC
        merai::merai        # ParameterServer, RTMemoryLayout, SharedLogger, RAII_SharedMemory
        robot_core::core    # RobotModel, kinematics/dynamics
)

# Executable
add_executable(planner_exe
    src/main.cpp
)

target_link_libraries(planner_exe
    PRIVATE
        planner_core
        pthread
        rt
)

set_target_properties(planner_exe PROPERTIES
    OUTPUT_NAME "merai_planner"
)

# Install rules (adjust paths to your layout)
install(TARGETS planner_core planner_exe
    EXPORT plannerTargets
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)

# Config file for find_package(planner)
include(CMakePackageConfigHelpers)

configure_package_config_file(
    cmake/plannerConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/plannerConfig.cmake
    INSTALL_DESTINATION lib/cmake/planner
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/plannerConfig.cmake
    DESTINATION lib/cmake/planner
)

install(EXPORT plannerTargets
    FILE plannerTargets.cmake
    NAMESPACE planner::
    DESTINATION lib/cmake/planner
)

# Optional: install systemd service + postinst script
install(FILES services/planner.service
        DESTINATION lib/systemd/system)

install(PROGRAMS scripts/postinst
        DESTINATION share/planner/scripts)
