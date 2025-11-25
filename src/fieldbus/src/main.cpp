#include <iostream>
#include <stdexcept>
#include <cstdlib>  // for EXIT_SUCCESS, EXIT_FAILURE

#include "fieldbus/EthercatMaster.h"

int main(int argc, char* argv[])
{
    try
    {
        // Shared memory names and sizes must match the launcher-created segments.
        std::string paramServerShmName = "/ParameterServerShm";
        size_t paramServerShmSize      = sizeof(seven_axis_robot::merai::ParameterServer);

        std::string rtDataShmName = "/RTDataShm";
        size_t rtDataShmSize      = sizeof(seven_axis_robot::merai::RTMemoryLayout);

        std::string loggerShmName = "/LoggerShm";
        size_t loggerShmSize      = sizeof(seven_axis_robot::merai::multi_ring_logger_memory);

        // Map logger SHM so main can emit log messages (EthercatMaster maps its own instance).
        seven_axis_robot::merai::RAII_SharedMemory loggerShm(loggerShmName, loggerShmSize, false);
        auto* loggerMem = reinterpret_cast<seven_axis_robot::merai::multi_ring_logger_memory*>(loggerShm.getPtr());
        if (!loggerMem)
        {
            std::cerr << "[Error] Failed to map logger shared memory.\n";
            return EXIT_FAILURE;
        }

        // Create EthercatMaster (attaches to the three SHMs)
        seven_axis_robot::fieldbus::EthercatMaster master(
            paramServerShmName, paramServerShmSize,
            rtDataShmName,      rtDataShmSize,
            loggerShmName,      loggerShmSize
        );

        // Initialize the master (configure drives, domain, etc.)
        if (!master.initializeMaster())
        {
            seven_axis_robot::merai::log_error(loggerMem, "Main", 3400, "EthercatMaster initialization failed!");
            return EXIT_FAILURE;
        }

        seven_axis_robot::merai::log_info(loggerMem, "Main", 3200, "Starting EtherCAT cyclic task...");
        master.run();  // Blocks until stop() is called or loop ends

        seven_axis_robot::merai::log_info(loggerMem, "Main", 3201, "EtherCAT Master exiting normally");
        return EXIT_SUCCESS;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[Exception] " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
