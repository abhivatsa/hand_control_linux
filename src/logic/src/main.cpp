#include <iostream>
#include <stdexcept>
#include <cstdlib>

#include "logic/Logic.h"

int main(int argc, char* argv[])
{
    try
    {
        std::string paramServerShmName = "/ParameterServerShm";
        size_t paramServerShmSize      = sizeof(seven_axis_robot::merai::ParameterServer);

        std::string rtDataShmName = "/RTDataShm";
        size_t rtDataShmSize      = sizeof(seven_axis_robot::merai::RTMemoryLayout);

        std::string loggerShmName = "/LoggerShm";
        size_t loggerShmSize      = sizeof(seven_axis_robot::merai::multi_ring_logger_memory);

        seven_axis_robot::logic::Logic logicApp(
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
