#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <ecrt.h>

#include "fieldbus/drives/BaseDrive.h"        // BaseDrive is in hand_control::fieldbus
#include "hand_control_merai/merai/RTMemoryLayout.h"             // hand_control::merai::RTMemoryLayout, MAX_DRIVES
#include "hand_control_merai/merai/ParameterServer.h"            // hand_control::merai::ParameterServer
#include "hand_control_merai/merai/RAII_SharedMemory.h"          // hand_control::merai::RAII_SharedMemory
#include "hand_control_merai/merai/SharedLogger.h"               // hand_control::merai::multi_ring_logger_memory

namespace hand_control
{
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
            EthercatMaster(const std::string& paramServerShmName, size_t paramServerShmSize,
                           const std::string& rtDataShmName,       size_t rtDataShmSize,
                           const std::string& loggerShmName,       size_t loggerShmSize);

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

            void inc_period(period_info* pinfo);
            void periodic_task_init(period_info* pinfo, long period_ns);
            void wait_rest_of_period(period_info* pinfo);

        private:
            // EtherCAT references
            ec_master_t* master_ = nullptr;
            ec_domain_t* domain_ = nullptr;
            uint8_t*     domainPd_ = nullptr;
            std::atomic_bool running_{false};

            ec_master_state_t masterState_{};
            ec_domain_state_t domainState_{};

            long loopPeriodNs = 1000000L;  // default 1 ms

            // Array of drives (up to hand_control::merai::MAX_DRIVES)
            std::array<std::unique_ptr<BaseDrive>, hand_control::merai::MAX_DRIVES> drives_;

            // SharedMemory for ParameterServer
            hand_control::merai::RAII_SharedMemory      configShm_;
            const hand_control::merai::ParameterServer* configPtr_ = nullptr;

            // SharedMemory for RTMemoryLayout
            hand_control::merai::RAII_SharedMemory rtDataShm_;
            hand_control::merai::RTMemoryLayout*   rtLayout_ = nullptr;

            // SharedMemory for logging
            hand_control::merai::RAII_SharedMemory          loggerShm_;
            hand_control::merai::multi_ring_logger_memory*  loggerMem_ = nullptr;
        };
    } // namespace fieldbus
} // namespace hand_control
