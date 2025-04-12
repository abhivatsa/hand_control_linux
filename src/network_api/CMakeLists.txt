cmake_minimum_required(VERSION 3.10)
project(network_api)

set(CMAKE_CXX_STANDARD 17)

# If ENet is installed via find_package, ensure it's found.
# Otherwise, adjust or remove if you manually specify ENet location.
find_package(ENet REQUIRED)

# Add include paths (your local include plus any merai or system paths).
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    # Add your merai/ or other headers if needed, e.g.:
    # /path/to/merai/headers
)

# Create a library from the main .cpp files except main.cpp
add_library(network_api_lib
    src/HapticDeviceAPI.cpp
    src/UDPSession.cpp
    src/NetworkAPI.cpp
)

# Link your library to ENet (and anything else needed).
target_link_libraries(network_api_lib
    PRIVATE
        ENet::ENet
        # e.g., merai_shared_memory_lib, merai_logger_lib, etc.
)

# Build the standalone executable from main.cpp.
add_executable(network_api_node
    src/main.cpp
)

# Link the executable to the library.
target_link_libraries(network_api_node
    PRIVATE
        network_api_lib
)

# Install directives for the library and the executable.
install(TARGETS network_api_node network_api_lib
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

# (Optional) Install your systemd service file to /etc/systemd/system.
install(FILES services/network_api.service
        DESTINATION /etc/systemd/system)

# (Optional) If you have a scripts/postinst you'd like to install or run:
# install(PROGRAMS scripts/postinst
#         DESTINATION /usr/local/bin)

# (Optional) For testing_api
# add_executable(test_network_api tests/test_network_api.cpp)
# target_link_libraries(test_network_api PRIVATE network_api_lib)
# enable_testing()
# add_test(NAME NetApiTests COMMAND test_network_api)
