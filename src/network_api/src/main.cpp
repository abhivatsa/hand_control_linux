#include <iostream>
#include <stdexcept>
#include <cstdlib> // EXIT_SUCCESS, EXIT_FAILURE

#include "network_api/include/network_api/NetworkAPI.h"

int main(int argc, char *argv[])
{
    try
    {
        // Example shared memory config
        std::string paramServerShmName = "/ParameterServerShm";
        size_t paramServerShmSize = sizeof(seven_axis_robot::merai::ParameterServer);

        std::string rtDataShmName = "/RTDataShm";
        size_t rtDataShmSize = sizeof(seven_axis_robot::merai::RTMemoryLayout);

        std::string loggerShmName = "/LoggerShm";
        /// Pointer to your shared memory logger for optional logging.
        seven_axis_robot::merai::multi_ring_logger_memory *loggerMem_;
        size_t loggerShmSize = sizeof(seven_axis_robot::merai::multi_ring_logger_memory);

        // Create the NetworkAPI object
        seven_axis_robot::network_api::NetworkAPI networkApi(
            paramServerShmName, paramServerShmSize,
            rtDataShmName, rtDataShmSize,
            loggerShmName, loggerShmSize);

        // Initialize the network (ENet, etc.)
        if (!networkApi.init())
        {
            // std::cerr << "[NetworkAPI Main] init() failed.\n";
            log_error(loggerMem_, "NetworkAPI Main", 501, " init() failed. ");
            return EXIT_FAILURE;
        }

        // Start the real-time loop
        // std::cout << "[NetworkAPI Main] Starting network API cyclic task...\n";
        log_info(loggerMem_, "NetworkAPI Main", 502, " Starting network API cyclic task... ");
        networkApi.run(); // Blocks until requestStop()

        // Normal exit
        // std::cout << "[NetworkAPI Main] Exiting normally.\n";
        log_info(loggerMem_, "NetworkAPI Main", 503, " Exiting normally. ");
        return EXIT_SUCCESS;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[NetworkAPI Main] Exception: " << e.what() << "\n";
        return EXIT_FAILURE;
    }
}
