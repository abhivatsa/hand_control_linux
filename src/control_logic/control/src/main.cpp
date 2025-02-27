#include <iostream>
#include <stdexcept>
#include <cstdlib>  // for EXIT_SUCCESS, EXIT_FAILURE

#include "control/Control.h"  // hand_control::control::Control

int main(int argc, char* argv[])
{
    try
    {
        // 1) Shared memory names and sizes (matching your global config).
        std::string paramServerShmName = "/ParameterServerShm";
        size_t paramServerShmSize      = sizeof(hand_control::merai::ParameterServer);

        std::string rtDataShmName = "/RTDataShm";
        size_t rtDataShmSize      = sizeof(hand_control::merai::RTMemoryLayout);

        std::string loggerShmName = "/LoggerShm";
        size_t loggerShmSize      = sizeof(hand_control::merai::multi_ring_logger_memory);

        // 2) Create the Control object using the three shared memories
        hand_control::control::Control controlApp(
            paramServerShmName, paramServerShmSize,
            rtDataShmName,      rtDataShmSize,
            loggerShmName,      loggerShmSize
        );

        // 3) Initialize the control application
        if (!controlApp.init())
        {
            std::cerr << "[Control Main] Control init failed.\n";
            return EXIT_FAILURE;
        }

        // 4) Start the real-time loop
        std::cout << "[Control Main] Starting Control cyclic task...\n";
        controlApp.run();  // Blocks until requestStop()

        // 5) Normal exit
        std::cout << "[Control Main] Control exiting normally.\n";
        return EXIT_SUCCESS;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[Control Main] Exception: " << e.what() << "\n";
        return EXIT_FAILURE;
    }
}
