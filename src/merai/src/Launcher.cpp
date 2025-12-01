#include <iostream>
#include <cstring>      // std::memcpy, std::memset
#include <cstdlib>      // std::getenv
#include <filesystem>
#include <type_traits>

#include <fcntl.h>      // shm_unlink
#include <sys/mman.h>   // shm_unlink

#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/SharedLogger.h"
#include "merai/RAII_SharedMemory.h"

int main(int argc, char* argv[])
{
    try
    {
        // Ensure types we memcpy/memset into SHM are POD-style.
        static_assert(std::is_trivially_copyable<merai::ParameterServer>::value,
                      "ParameterServer must be trivially copyable for SHM.");
        static_assert(std::is_trivially_copyable<merai::RTMemoryLayout>::value,
                      "RTMemoryLayout must be trivially copyable for SHM.");
        static_assert(std::is_trivially_copyable<merai::multi_ring_logger_memory>::value,
                      "multi_ring_logger_memory must be trivially copyable for SHM.");

        // Start from a clean slate.
        ::shm_unlink("/ParameterServerShm");
        ::shm_unlink("/RTDataShm");
        ::shm_unlink("/LoggerShm");

        // -----------------------------------------------------------
        // Resolve config directory
        // -----------------------------------------------------------
        std::filesystem::path configDir;
        if (const char* envDir = std::getenv("MERAI_CONFIG_DIR"))
        {
            configDir = envDir;
        }
        else
        {
            const std::filesystem::path installDir = MERAI_CONFIG_DIR_INSTALL;
            const std::filesystem::path sourceDir  = MERAI_CONFIG_DIR_SOURCE;

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
                throw std::runtime_error(
                    "Launcher: Could not locate config directory. Set MERAI_CONFIG_DIR.");
            }
        }

        const std::string ecatFile  = (configDir / "ethercat_config.json").string();
        const std::string robotFile = (configDir / "robot_parameters.json").string();

        // Parse system parameters into a POD config
        merai::ParameterServer paramServer =
            merai::parseParameterServer(ecatFile, robotFile);

        std::cout << "Launcher: parsed "
                  << paramServer.driveCount  << " drives, "
                  << paramServer.jointCount  << " joints.\n";

        // -----------------------------------------------------------
        // 1) ParameterServer SHM (static config data)
        // -----------------------------------------------------------
        const std::size_t configShmSize = sizeof(merai::ParameterServer);
        merai::RAII_SharedMemory configShm("/ParameterServerShm", configShmSize);

        auto* paramPtr =
            static_cast<merai::ParameterServer*>(configShm.getPtr());

        std::memcpy(paramPtr, &paramServer, sizeof(merai::ParameterServer));

        std::cout << "Launcher: ParameterServer shared memory created.\n"
                  << "          (name=\"/ParameterServerShm\", size="
                  << configShmSize << ")\n";

        // -----------------------------------------------------------
        // 2) RTMemoryLayout SHM (real-time data)
        // -----------------------------------------------------------
        const std::size_t rtShmSize = sizeof(merai::RTMemoryLayout);
        merai::RAII_SharedMemory rtShm("/RTDataShm", rtShmSize);

        auto* rtLayout =
            static_cast<merai::RTMemoryLayout*>(rtShm.getPtr());

        std::memset(rtLayout, 0, rtShmSize);
        rtLayout->magic   = merai::RT_MEMORY_MAGIC;
        rtLayout->version = merai::RT_MEMORY_VERSION;

        std::cout << "Launcher: RTMemoryLayout shared memory created.\n"
                  << "          (name=\"/RTDataShm\", size="
                  << rtShmSize << ")\n\n";

        // -----------------------------------------------------------
        // 3) Logger SHM (multi-ring shared logger)
        // -----------------------------------------------------------
        const std::size_t loggerShmSize =
            sizeof(merai::multi_ring_logger_memory);
        merai::RAII_SharedMemory loggerShm("/LoggerShm", loggerShmSize);

        auto* multiLoggerPtr =
            static_cast<merai::multi_ring_logger_memory*>(loggerShm.getPtr());

        std::memset(multiLoggerPtr, 0, loggerShmSize);
        multiLoggerPtr->magic   = merai::multi_ring_logger_memory::MAGIC;
        multiLoggerPtr->version = merai::multi_ring_logger_memory::VERSION;

        std::cout << "Launcher: Logger shared memory created.\n"
                  << "          (name=\"/LoggerShm\", size="
                  << loggerShmSize << ")\n\n";

        std::cout << "Launcher: Setup complete. Exiting.\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "Launcher Error: " << e.what() << '\n';
        return 1;
    }

    return 0;
}
