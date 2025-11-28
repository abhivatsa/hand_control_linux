#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <ecrt.h>

#include "fieldbus/drives/BaseDrive.h" // BaseDrive is in fieldbus
#include "merai/RTMemoryLayout.h"      // merai::RTMemoryLayout, MAX_SERVO_DRIVES
#include "merai/ParameterServer.h"     // merai::ParameterServer
#include "merai/RAII_SharedMemory.h"   // merai::RAII_SharedMemory
#include "merai/SharedLogger.h"        // merai::multi_ring_logger_memory

namespace fieldbus
{
    /**
     * @brief The EthercatMaster class manages:
     *  - Attaching to ParameterServer, RTMemory, Logger SHMs
     *  - Initializing EtherCAT master and drives
     *  - Running the real-time loop
     */
    class EthercatMaster
    {
    public:
        /**
         * @brief Constructor attaches to:
         *  - ParameterServer (paramServerShmName)
         *  - RTMemoryLayout (rtDataShmName)
         *  - multi_ring_logger_memory (loggerShmName)
         *
         * Throws std::runtime_error on any SHM attach failure.
         */
        EthercatMaster(const std::string &paramServerShmName, size_t paramServerShmSize,
                       const std::string &rtDataShmName, size_t rtDataShmSize,
                       const std::string &loggerShmName, size_t loggerShmSize);

        ~EthercatMaster();

        /**
         * @brief Sets up EtherCAT master and configures drives.
         * @return True if successful, false otherwise.
         */
        bool initializeMaster();

        /**
         * @brief Activates the master and enters the real-time loop (blocking).
         */
        void run();

        /**
         * @brief Requests the master to stop the real-time loop.
         */
        void stop();

    private:
        void cyclicTask();
        bool checkDomainState();
        bool checkMasterState();

        struct period_info
        {
            struct timespec next_period;
            long period_ns;
        };

        void inc_period(period_info *pinfo);
        void periodic_task_init(period_info *pinfo, long period_ns);
        void wait_rest_of_period(period_info *pinfo);

    private:
        struct FieldbusCycleInputs
        {
            const merai::ServoRxPdo *rxPdos = nullptr;
            std::size_t driveCount = 0;
        };

        struct FieldbusCycleOutputs
        {
            merai::ServoTxPdo *txPdos = nullptr;
            std::size_t driveCount = 0;
        };

        // EtherCAT references
        ec_master_t *master_ = nullptr;
        ec_domain_t *domain_ = nullptr;
        uint8_t *domainPd_ = nullptr;
        std::atomic_bool running_{false};

        ec_master_state_t masterState_{};
        ec_domain_state_t domainState_{};

        long loopPeriodNs = 1000000L; // default 1 ms

        // Array of drives (up to merai::MAX_SERVO_DRIVES)
        std::array<std::unique_ptr<BaseDrive>, merai::MAX_SERVO_DRIVES> drives_;

        // SharedMemory for ParameterServer
        merai::RAII_SharedMemory configShm_;
        const merai::ParameterServer *configPtr_ = nullptr;

        // SharedMemory for RTMemoryLayout
        merai::RAII_SharedMemory rtDataShm_;
        merai::RTMemoryLayout *rtLayout_ = nullptr;

        // SharedMemory for logging
        merai::RAII_SharedMemory loggerShm_;
        merai::multi_ring_logger_memory *loggerMem_ = nullptr;

        std::array<merai::ServoRxPdo,
                   merai::MAX_SERVO_DRIVES>
            servoRxShadow_{};
    };
} // namespace fieldbus
