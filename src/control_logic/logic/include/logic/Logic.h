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
             * @brief readDriveSummary
             *  - Reads aggregator from control that indicates "anyFaulted" and a severity
             */
            void readDriveSummary(bool& outAnyFaulted, int& outFaultSeverity);

            /**
             * @brief readControllerUserCommands
             *  - Possibly read user aggregator or commands to see if
             *    user wants the robot active or wants a controller switch
             */
            void readControllerUserCommands(bool& outUserRequestedActive,
                                            bool& outUserRequestedSwitch,
                                            char* outControllerName);

            /**
             * @brief writeDriveCommand
             *  - Writes the single enumerated drive command to shared memory,
             *    so control side can interpret it
             */
            void writeDriveCommand(hand_control::logic::DriveCommand cmd);

            /**
             * @brief writeControllerSwitch
             *  - If orchestrator wants a new controller, write aggregator to control side
             */
            void writeControllerSwitch(bool switchWanted, const std::string& ctrlName);

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

            merai::RAII_SharedMemory paramServerShm_;
            const merai::ParameterServer* paramServerPtr_ = nullptr;

            merai::RAII_SharedMemory rtDataShm_;
            merai::RTMemoryLayout* rtLayout_ = nullptr;

            merai::RAII_SharedMemory loggerShm_;
            merai::multi_ring_logger_memory* loggerMem_ = nullptr;

            SystemOrchestrator systemOrchestrator_;
        };
    }
}
