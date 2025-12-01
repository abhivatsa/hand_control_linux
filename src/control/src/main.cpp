#include <stdexcept>
#include <cstdlib>
#include <iostream>
#include <sys/mman.h>
#include <unistd.h>

#include "merai/SharedLogger.h"
#include "merai/RAII_SharedMemory.h"
#include "merai/RTMemoryLayout.h"
#include "merai/ParameterServer.h"

#include "control/Control.h"

void prefault_region(void *ptr, std::size_t size)
{
    if (!ptr || size == 0)
    {
        return;
    }
    const long page = sysconf(_SC_PAGESIZE);
    volatile char *p = static_cast<volatile char *>(ptr);
    for (std::size_t i = 0; i < size; i += static_cast<std::size_t>(page))
    {
        (void)p[i];
    }
    (void)p[size - 1];
}

int main(int argc, char *argv[])
{
    try
    {
        std::string paramServerShmName = "/ParameterServerShm";
        std::size_t paramServerShmSize = sizeof(merai::ParameterServer);

        std::string rtDataShmName = "/RTDataShm";
        std::size_t rtDataShmSize = sizeof(merai::RTMemoryLayout);

        std::string loggerShmName = "/LoggerShm";
        std::size_t loggerShmSize = sizeof(merai::multi_ring_logger_memory);

        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        {
            std::perror("[Control Main] mlockall failed");
        }

        // Map logger SHM so we can log from main
        merai::RAII_SharedMemory loggerShm(loggerShmName, loggerShmSize, false);
        auto *loggerMem = reinterpret_cast<merai::multi_ring_logger_memory *>(
            loggerShm.getPtr());
        if (!loggerMem)
        {
            std::cerr << "[Control Main] Failed to map logger shared memory.\n";
            return EXIT_FAILURE;
        }
        prefault_region(loggerMem, loggerShmSize);

        // Optional: sanity-check RTMemoryLayout magic/version and prefault
        merai::RAII_SharedMemory rtDataShmTmp(rtDataShmName, rtDataShmSize, false);
        auto *rtLayoutTmp = reinterpret_cast<merai::RTMemoryLayout *>(
            rtDataShmTmp.getPtr());
        if (!rtLayoutTmp ||
            rtLayoutTmp->magic != merai::RT_MEMORY_MAGIC ||
            rtLayoutTmp->version != merai::RT_MEMORY_VERSION)
        {
            std::cerr << "[Control Main] RTMemoryLayout integrity check failed "
                         "(magic/version mismatch).\n";
            return EXIT_FAILURE;
        }
        prefault_region(rtLayoutTmp, rtDataShmSize);

        // Create Control application
        control::Control controlApp(
            paramServerShmName, paramServerShmSize,
            rtDataShmName,      rtDataShmSize,
            loggerShmName,      loggerShmSize);

        if (!controlApp.init())
        {
            merai::log_error(loggerMem, "Control Main", 4000,
                             "Control init failed");
            return EXIT_FAILURE;
        }

        merai::log_info(loggerMem, "Control Main", 4001,
                        "Starting Control cyclic task");
        controlApp.run();

        merai::log_info(loggerMem, "Control Main", 4002,
                        "Control exiting normally");
        return EXIT_SUCCESS;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[Control Main] Exception: " << e.what() << "\n";
        return EXIT_FAILURE;
    }
}
