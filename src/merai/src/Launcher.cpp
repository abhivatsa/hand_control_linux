#include <iostream>
#include <cstring> // for std::memset
#include <fcntl.h> // for shm_unlink

#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/SharedLogger.h"
#include "merai/RAII_SharedMemory.h"
#include <sys/mman.h>
#include <cstdlib>
#include <filesystem>

int main(int argc, char *argv[])
{
    try
    {
        // (Optional) Remove existing shared memory objects to start fresh.
        ::shm_unlink("/ParameterServerShm");
        ::shm_unlink("/RTDataShm");
        ::shm_unlink("/LoggerShm");

        // Resolve config directory: env override -> installed path -> source path
        std::filesystem::path configDir;
        if (const char* envDir = std::getenv("MERAI_CONFIG_DIR"))
        {
            configDir = envDir;
        }
        else
        {
            std::filesystem::path installDir = MERAI_CONFIG_DIR_INSTALL;
            std::filesystem::path sourceDir = MERAI_CONFIG_DIR_SOURCE;
            if (std::filesystem::exists(installDir))
            {
                configDir = installDir;
            }
            else if (std::filesystem::exists(sourceDir))
            {
                configDir = sourceDir;
            }
            else
            {
                throw std::runtime_error("Could not locate config directory. Set MERAI_CONFIG_DIR.");
            }
        }

        const std::string ecatFile = (configDir / "ethercat_config.json").string();
        const std::string robotFile = (configDir / "robot_parameters.json").string();
        // Parse system parameters
        seven_axis_robot::merai::ParameterServer paramServer =
            seven_axis_robot::merai::parseParameterServer(ecatFile, robotFile);

        std::cout << "Launcher: parsed "
                  << paramServer.driveCount << " drives, "
                  << paramServer.jointCount << " joints.\n";

        // -----------------------------------------------------------
        // 1) Create SHM for ParameterServer (static config data)
        // -----------------------------------------------------------
        const size_t configShmSize = sizeof(seven_axis_robot::merai::ParameterServer);
        seven_axis_robot::merai::RAII_SharedMemory configShm("/ParameterServerShm", configShmSize);

        auto *paramPtr =
            reinterpret_cast<seven_axis_robot::merai::ParameterServer *>(configShm.getPtr());

        // Copy the parsed data into shared memory
        std::memcpy(paramPtr, &paramServer, sizeof(seven_axis_robot::merai::ParameterServer));

        std::cout << "Launcher: ParameterServer shared memory created.\n"
                  << "          (name=\"/ParameterServerShm\", size=" << configShmSize << ")\n";

        // -----------------------------------------------------------
        // 2) Create SHM for RTMemoryLayout (real-time data)
        // -----------------------------------------------------------
        const size_t rtShmSize = sizeof(seven_axis_robot::merai::RTMemoryLayout);
        seven_axis_robot::merai::RAII_SharedMemory rtShm("/RTDataShm", rtShmSize);

        auto *rtLayout =
            reinterpret_cast<seven_axis_robot::merai::RTMemoryLayout *>(rtShm.getPtr());

        // Zero-initialize the real-time buffer region
        std::memset(rtLayout, 0, rtShmSize);
        rtLayout->magic = seven_axis_robot::merai::RT_MEMORY_MAGIC;
        rtLayout->version = seven_axis_robot::merai::RT_MEMORY_VERSION;

        std::cout << "Launcher: RTMemoryLayout shared memory created.\n"
                  << "          (name=\"/RTDataShm\", size=" << rtShmSize << ")\n\n";

        // -----------------------------------------------------------
        // 3) Create SHM for MultiRingLoggerMemory (log messages)
        // -----------------------------------------------------------
        const size_t loggerShmSize = sizeof(seven_axis_robot::merai::multi_ring_logger_memory);
        seven_axis_robot::merai::RAII_SharedMemory loggerShm("/LoggerShm", loggerShmSize);

        auto *multiLoggerPtr =
            reinterpret_cast<seven_axis_robot::merai::multi_ring_logger_memory *>(loggerShm.getPtr());

        // Clear all ring buffers
        std::memset(multiLoggerPtr, 0, loggerShmSize);
        multiLoggerPtr->magic = seven_axis_robot::merai::multi_ring_logger_memory::MAGIC;
        multiLoggerPtr->version = seven_axis_robot::merai::multi_ring_logger_memory::VERSION;

        std::cout << "Launcher: Logger shared memory created.\n"
                  << "          (name=\"/LoggerShm\", size=" << loggerShmSize << ")\n\n";

        // -----------------------------------------------------------
        // 4) (Optional) Let systemd manage other processes
        // -----------------------------------------------------------
        std::cout << "Launcher: Setup complete. Exiting.\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << "Launcher Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
