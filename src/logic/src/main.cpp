#include <iostream>
#include <stdexcept>
#include <cstdlib>

#include "logic/Logic.h"

int main(int argc, char* argv[])
{
    try
    {
        std::string paramServerShmName = "/ParameterServerShm";
        size_t paramServerShmSize      = sizeof(merai::ParameterServer);

        std::string rtDataShmName = "/RTDataShm";
        size_t rtDataShmSize      = sizeof(merai::RTMemoryLayout);

        std::string loggerShmName = "/LoggerShm";
        size_t loggerShmSize      = sizeof(merai::multi_ring_logger_memory);

        logic::Logic logicApp(
            paramServerShmName, paramServerShmSize,
            rtDataShmName,      rtDataShmSize,
            loggerShmName,      loggerShmSize
        );

        if (!logicApp.init())
        {
            std::cerr << "[Logic Main] init failed.\n";
            return EXIT_FAILURE;
        }

        logicApp.run();  // blocks until requestStop()

        return EXIT_SUCCESS;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[Logic Main] Exception: " << e.what() << "\n";
        return EXIT_FAILURE;
    }
}
