#include <iostream>
#include <stdexcept>
#include <cstdlib>

#include "logic/Logic.h"

int main(int argc, char* argv[])
{
    try
    {
        std::string paramServerShmName = "/ParameterServerShm";
        size_t paramServerShmSize      = sizeof(hand_control::merai::ParameterServer);

        std::string rtDataShmName = "/RTDataShm";
        size_t rtDataShmSize      = sizeof(hand_control::merai::RTMemoryLayout);

        std::string loggerShmName = "/LoggerShm";
        size_t loggerShmSize      = sizeof(hand_control::merai::multi_ring_logger_memory);

        hand_control::logic::Logic logicApp(
            paramServerShmName, paramServerShmSize,
            rtDataShmName,      rtDataShmSize,
            loggerShmName,      loggerShmSize
        );

        if (!logicApp.init())
        {
            std::cerr << "[Logic Main] init failed.\n";
            return EXIT_FAILURE;
        }

        std::cout << "[Logic Main] Starting logic loop...\n";
        logicApp.run();  // blocks until requestStop()
        std::cout << "[Logic Main] Exiting.\n";

        return EXIT_SUCCESS;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[Logic Main] Exception: " << e.what() << "\n";
        return EXIT_FAILURE;
    }
}
