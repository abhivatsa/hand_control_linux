#include <iostream>
#include <cstring> // for std::memset
#include <fcntl.h> // for shm_unlink

#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/SharedLogger.h"
#include "merai/RAII_SharedMemory.h"

// If parseParameterServer is implemented in ParameterServer.cpp within the same namespace,
// declare it here with the matching namespace:
namespace hand_control
{
    namespace merai
    {
        extern ParameterServer parseParameterServer(const std::string &ecatConfigFile,
                                                    const std::string &robotParamFile,
                                                    const std::string &startupFile);
    }
}

int main(int argc, char *argv[])
{
    try
    {
        // (Optional) Remove existing shared memory objects to start fresh.
        // ::shm_unlink("/ParameterServerShm");
        // ::shm_unlink("/RTDataShm");
        // ::shm_unlink("/LoggerShm");

        // Example file paths
        const std::string ecatFile = "../../../config/ethercat_config.json";
        const std::string robotFile = "../../../config/robot_parameters.json";
        const std::string startupFile = "../../../config/startup_config.json";

        // Parse system parameters
        hand_control::merai::ParameterServer paramServer =
            hand_control::merai::parseParameterServer(ecatFile, robotFile, startupFile);

        std::cout << "Launcher: parsed "
                  << paramServer.driveCount << " drives, "
                  << paramServer.jointCount << " joints.\n";

        // -----------------------------------------------------------
        // 1) Create SHM for ParameterServer (static config data)
        // -----------------------------------------------------------
        const size_t configShmSize = sizeof(hand_control::merai::ParameterServer);
        hand_control::merai::RAII_SharedMemory configShm("/ParameterServerShm", configShmSize);

        auto *paramPtr =
            reinterpret_cast<hand_control::merai::ParameterServer *>(configShm.getPtr());

        // Copy the parsed data into shared memory
        std::memcpy(paramPtr, &paramServer, sizeof(hand_control::merai::ParameterServer));

        std::cout << "Launcher: ParameterServer shared memory created.\n"
                  << "          (name=\"/ParameterServerShm\", size=" << configShmSize << ")\n";

        // -----------------------------------------------------------
        // 2) Create SHM for RTMemoryLayout (real-time data)
        // -----------------------------------------------------------
        const size_t rtShmSize = sizeof(hand_control::merai::RTMemoryLayout);
        hand_control::merai::RAII_SharedMemory rtShm("/RTDataShm", rtShmSize);

        auto *rtLayout =
            reinterpret_cast<hand_control::merai::RTMemoryLayout *>(rtShm.getPtr());

        // Zero-initialize the real-time buffer region
        std::memset(rtLayout, 0, rtShmSize);

        std::cout << "Launcher: RTMemoryLayout shared memory created.\n"
                  << "          (name=\"/RTDataShm\", size=" << rtShmSize << ")\n\n";

        // -----------------------------------------------------------
        // 3) Create SHM for MultiRingLoggerMemory (log messages)
        // -----------------------------------------------------------
        const size_t loggerShmSize = sizeof(hand_control::merai::multi_ring_logger_memory);
        hand_control::merai::RAII_SharedMemory loggerShm("/LoggerShm", loggerShmSize);

        auto *multiLoggerPtr =
            reinterpret_cast<hand_control::merai::multi_ring_logger_memory *>(loggerShm.getPtr());

        // Clear all ring buffers
        std::memset(multiLoggerPtr, 0, loggerShmSize);

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
