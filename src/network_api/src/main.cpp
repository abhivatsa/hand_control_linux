#include <iostream>
#include <stdexcept>
#include <cstdlib>  // for EXIT_SUCCESS, EXIT_FAILURE

#include "network_api/NetworkAPI.h"  // The class we’ll create to manage cyc. loop

int main(int argc, char* argv[])
{
    try
    {
        // 1) Shared memory names and sizes
        //    These should match your existing global config or "merai" structures
        std::string paramServerShmName = "/ParameterServerShm";
        size_t paramServerShmSize      = sizeof(hand_control::merai::ParameterServer);

        std::string rtDataShmName = "/RTDataShm";
        size_t rtDataShmSize      = sizeof(hand_control::merai::RTMemoryLayout);

        std::string loggerShmName = "/LoggerShm";
        size_t loggerShmSize      = sizeof(hand_control::merai::multi_ring_logger_memory);

        // 2) Create the NetworkAPI object using the three shared memories
        hand_control::network_api::NetworkAPI networkApi(
            paramServerShmName, paramServerShmSize,
            rtDataShmName,      rtDataShmSize,
            loggerShmName,      loggerShmSize
        );

        // 3) Initialize the network API (sets up UDP / enet, etc.)
        if (!networkApi.init())
        {
            std::cerr << "[NetworkAPI Main] init() failed.\n";
            return EXIT_FAILURE;
        }

        // 4) Start the real-time loop
        std::cout << "[NetworkAPI Main] Starting network API cyclic task...\n";
        networkApi.run();  // Blocks until requestStop()

        // 5) Normal exit
        std::cout << "[NetworkAPI Main] Exiting normally.\n";
        return EXIT_SUCCESS;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[NetworkAPI Main] Exception: " << e.what() << "\n";
        return EXIT_FAILURE;
    }
}
