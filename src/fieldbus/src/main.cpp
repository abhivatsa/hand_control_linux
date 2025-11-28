#include <iostream>
#include <stdexcept>
#include <cstdlib> // for EXIT_SUCCESS, EXIT_FAILURE
#include <sys/mman.h>
#include <unistd.h>

#include "fieldbus/EthercatMaster.h"

void prefault_region(void *ptr, size_t size)
{
    if (!ptr || size == 0)
    {
        return;
    }
    const long page = sysconf(_SC_PAGESIZE);
    volatile char *p = static_cast<volatile char *>(ptr);
    for (size_t i = 0; i < size; i += static_cast<size_t>(page))
    {
        (void)p[i];
    }
    (void)p[size - 1];
}

int main(int argc, char *argv[])
{
    try
    {
        // Shared memory names and sizes must match the launcher-created segments.
        std::string paramServerShmName = "/ParameterServerShm";
        size_t paramServerShmSize = sizeof(merai::ParameterServer);

        std::string rtDataShmName = "/RTDataShm";
        size_t rtDataShmSize = sizeof(merai::RTMemoryLayout);

        std::string loggerShmName = "/LoggerShm";
        size_t loggerShmSize = sizeof(merai::multi_ring_logger_memory);

        // Lock current/future pages to avoid minor faults in the RT loop.
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        {
            std::perror("[Error] mlockall failed");
        }

        // Map logger SHM so main can emit log messages (EthercatMaster maps its own instance).
        merai::RAII_SharedMemory loggerShm(loggerShmName, loggerShmSize, false);
        auto *loggerMem = reinterpret_cast<merai::multi_ring_logger_memory *>(loggerShm.getPtr());
        if (!loggerMem)
        {
            std::cerr << "[Error] Failed to map logger shared memory.\n";
            return EXIT_FAILURE;
        }
        prefault_region(loggerMem, loggerShmSize);

        // Create EthercatMaster (attaches to the three SHMs)
        fieldbus::EthercatMaster master(
            paramServerShmName, paramServerShmSize,
            rtDataShmName, rtDataShmSize,
            loggerShmName, loggerShmSize);

        // Initialize the master (configure drives, domain, etc.)
        if (!master.initializeMaster())
        {
            merai::log_error(loggerMem, "Main", 3400, "EthercatMaster initialization failed!");
            return EXIT_FAILURE;
        }

        merai::log_info(loggerMem, "Main", 3200, "Starting EtherCAT cyclic task...");
        master.run(); // Blocks until stop() is called or loop ends

        merai::log_info(loggerMem, "Main", 3201, "EtherCAT Master exiting normally");
        return EXIT_SUCCESS;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[Exception] " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
