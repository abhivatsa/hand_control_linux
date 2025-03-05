#pragma once

#include <atomic>
#include <string>
#include <cstddef>
#include <time.h>
#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/RAII_SharedMemory.h"
#include "logic/SystemOrchestrator.h"  // includes OrchestratorState, DriveCommand

namespace hand_control
{
    namespace logic
    {
        class Logic
        {
        public:
            Logic(const std::string& paramServerShmName, std::size_t paramServerShmSize,
                  const std::string& rtDataShmName,      std::size_t rtDataShmSize,
                  const std::string& loggerShmName,      std::size_t loggerShmSize);

            ~Logic();

            bool init();
            void run();
            void requestStop();

        private:
            void cyclicTask();

            /**
             * @brief readDriveSummaryAggregator
             *  - Reads aggregator from control that indicates "anyFaulted" and a severity
             */
            void readDriveSummaryAggregator(bool& outAnyFaulted, int& outFaultSeverity);

            /**
             * @brief readUserCommandsAggregator
             *  - Possibly read aggregator or user commands to see if
             *    user wants robot active or a controller switch
             */
            void readUserCommandsAggregator(bool& outUserRequestedActive,
                                            bool& outUserRequestedSwitch,
                                            char* outControllerName);

            /**
             * @brief writeDriveCommandAggregator
             *  - Writes the single enumerated drive command to shared memory,
             *    so control side can interpret it
             */
            void writeDriveCommandAggregator(hand_control::logic::DriveCommand cmd);

            /**
             * @brief writeControllerSwitchAggregator
             *  - If orchestrator wants a new controller, write aggregator to control side
             */
            void writeControllerSwitchAggregator(bool switchWanted, const std::string& ctrlName);

            struct period_info
            {
                struct timespec next_period;
                long period_ns;
            };

            void periodic_task_init(period_info* pinfo, long periodNs);
            void inc_period(period_info* pinfo);
            void wait_rest_of_period(period_info* pinfo);

        private:
            std::atomic_bool stopRequested_{false};

            // RAII handles for shared memory
            merai::RAII_SharedMemory paramServerShm_;
            const merai::ParameterServer* paramServerPtr_ = nullptr;

            merai::RAII_SharedMemory rtDataShm_;
            merai::RTMemoryLayout* rtLayout_ = nullptr;

            merai::RAII_SharedMemory loggerShm_;
            merai::multi_ring_logger_memory* loggerMem_ = nullptr;

            // Our orchestrator
            SystemOrchestrator systemOrchestrator_;
        };
    }
}
