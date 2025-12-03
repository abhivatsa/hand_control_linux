#include <iostream>
#include <cstdlib>
#include <sys/mman.h>
#include <unistd.h>

#include "merai/RAII_SharedMemory.h"
#include "merai/SharedLogger.h"
#include "merai/RTMemoryLayout.h"
#include "planner/PlannerApp.h"

static void prefault_region(void* ptr, std::size_t size)
{
    if (!ptr || size == 0)
    {
        return;
    }
    const long page = sysconf(_SC_PAGESIZE);
    volatile char* p = static_cast<volatile char*>(ptr);
    for (std::size_t i = 0; i < size; i += static_cast<std::size_t>(page))
    {
        (void)p[i];
    }
    (void)p[size - 1];
}

int main(int /*argc*/, char* /*argv*/[])
{
    try
    {
        // Shared memory names & sizes (must match launcher)
        std::string paramServerShmName = "/ParameterServerShm";
        std::size_t paramServerShmSize = sizeof(merai::ParameterServer);

        std::string rtDataShmName = "/RTDataShm";
        std::size_t rtDataShmSize = sizeof(merai::RTMemoryLayout);

        std::string loggerShmName = "/LoggerShm";
        std::size_t loggerShmSize = sizeof(merai::multi_ring_logger_memory);

        // Non-RT planner, but we can still lock memory to reduce paging
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        {
            std::perror("[Planner Main] mlockall failed");
        }

        // Map logger SHM so we can log early
        merai::RAII_SharedMemory loggerShm(loggerShmName, loggerShmSize, false);
        auto* loggerMem = reinterpret_cast<merai::multi_ring_logger_memory*>(loggerShm.getPtr());
        if (!loggerMem)
        {
            std::cerr << "[Planner Main] Failed to map logger shared memory.\n";
            return EXIT_FAILURE;
        }
        prefault_region(loggerMem, loggerShmSize);

        planner::PlannerApp app(
            paramServerShmName, paramServerShmSize,
            rtDataShmName,      rtDataShmSize,
            loggerShmName,      loggerShmSize);

        if (!app.init())
        {
            merai::log_error(loggerMem, "Planner Main", 5000, "PlannerApp init failed");
            return EXIT_FAILURE;
        }

        merai::log_info(loggerMem, "Planner Main", 5001, "PlannerApp starting");
        app.run();
        merai::log_info(loggerMem, "Planner Main", 5002, "PlannerApp exiting normally");

        return EXIT_SUCCESS;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[Planner Main] Exception: " << e.what() << "\n";
        return EXIT_FAILURE;
    }
}
