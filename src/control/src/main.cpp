#include <stdexcept>
#include <cstdlib>  // for EXIT_SUCCESS, EXIT_FAILURE
#include <sys/mman.h>
#include <unistd.h>

#include "merai/SharedLogger.h"
#include "merai/RAII_SharedMemory.h"
#include "control/Control.h"  // seven_axis_robot::control::Control

namespace
{
    void prefault_region(void* ptr, size_t size)
    {
        if (!ptr || size == 0)
        {
            return;
        }
        const long page = sysconf(_SC_PAGESIZE);
        volatile char* p = static_cast<volatile char*>(ptr);
        for (size_t i = 0; i < size; i += static_cast<size_t>(page))
        {
            (void)p[i];
        }
        (void)p[size - 1];
    }
}

int main(int argc, char* argv[])
{
    try
    {
        // 1) Shared memory names and sizes (matching your global config).
        std::string paramServerShmName = "/ParameterServerShm";
        size_t paramServerShmSize      = sizeof(seven_axis_robot::merai::ParameterServer);

        std::string rtDataShmName = "/RTDataShm";
        size_t rtDataShmSize      = sizeof(seven_axis_robot::merai::RTMemoryLayout);

        std::string loggerShmName = "/LoggerShm";
        size_t loggerShmSize      = sizeof(seven_axis_robot::merai::multi_ring_logger_memory);

        // Lock current/future pages to avoid minor faults during RT operation.
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        {
            std::perror("[Control Main] mlockall failed");
        }

        // Map logger SHM so we can log from main (Control maps its own handle internally).
        seven_axis_robot::merai::RAII_SharedMemory loggerShm(loggerShmName, loggerShmSize, false);
        auto* loggerMem = reinterpret_cast<seven_axis_robot::merai::multi_ring_logger_memory*>(loggerShm.getPtr());
        if (!loggerMem)
        {
            std::cerr << "[Control Main] Failed to map logger shared memory.\n";
            return EXIT_FAILURE;
        }
        prefault_region(loggerMem, loggerShmSize);

        // Map RTData just to validate magic/version before Control uses it
        seven_axis_robot::merai::RAII_SharedMemory rtDataShmTmp(rtDataShmName, rtDataShmSize, false);
        auto* rtLayoutTmp = reinterpret_cast<seven_axis_robot::merai::RTMemoryLayout*>(rtDataShmTmp.getPtr());
        if (!rtLayoutTmp || rtLayoutTmp->magic != seven_axis_robot::merai::RT_MEMORY_MAGIC || rtLayoutTmp->version != seven_axis_robot::merai::RT_MEMORY_VERSION)
        {
            std::cerr << "[Control Main] RTMemoryLayout integrity check failed (magic/version mismatch).\n";
            return EXIT_FAILURE;
        }
        prefault_region(rtLayoutTmp, rtDataShmSize);

        // 2) Create the Control object using the three shared memories
        seven_axis_robot::control::Control controlApp(
            paramServerShmName, paramServerShmSize,
            rtDataShmName,      rtDataShmSize,
            loggerShmName,      loggerShmSize
        );

        // 3) Initialize the control application
        if (!controlApp.init())
        {
            seven_axis_robot::merai::log_error(loggerMem, "Control Main", 4000, "Control init failed");
            return EXIT_FAILURE;
        }

        // 4) Start the real-time loop
        seven_axis_robot::merai::log_info(loggerMem, "Control Main", 4001, "Starting Control cyclic task");
        controlApp.run();  // Blocks until requestStop()

        // 5) Normal exit
        seven_axis_robot::merai::log_info(loggerMem, "Control Main", 4002, "Control exiting normally");
        return EXIT_SUCCESS;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[Control Main] Exception: " << e.what() << "\n";
        return EXIT_FAILURE;
    }
}
