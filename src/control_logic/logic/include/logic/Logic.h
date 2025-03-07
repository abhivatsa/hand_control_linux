#pragma once

#include <atomic>
#include <string>
#include <cstddef>
#include <time.h>

// merai / Common headers
#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/RAII_SharedMemory.h"
#include "merai/Enums.h" // for DriveCommand, ControllerID

// Orchestrator (logic layer)
#include "logic/SystemOrchestrator.h"  // includes OrchestratorState, etc.

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

            /**
             * @brief Initializes the logic system, including orchestrator init.
             * @return true if successful
             */
            bool init();

            /**
             * @brief Runs the main (blocking) loop of the logic application,
             *        typically 10 ms cycle.
             */
            void run();

            /**
             * @brief Signals the logic loop to stop gracefully.
             */
            void requestStop();

        private:
            /**
             * @brief The logic loop, typically 10 ms. Reads aggregator from control
             *        (driveSummary), orchestrates system logic, writes drive commands
             *        and controller commands back.
             */
            void cyclicTask();

            /**
             * @brief readDriveSummaryAggregator
             *  - Reads aggregator from Control indicating any fault and severity.
             */
            void readDriveSummaryAggregator(bool& outAnyFaulted, int& outFaultSeverity);

            /**
             * @brief readUserCommandsAggregator
             *  - Possibly reads aggregator or user commands to see if
             *    user wants the robot active or a controller switch.
             *  - This is just an example; you can customize as needed.
             */
            void readUserCommandsAggregator(bool& outUserRequestedActive,
                                            bool& outUserRequestedSwitch,
                                            char* outControllerName);

            /**
             * @brief writeDriveCommandAggregator
             *  - Writes a single enumerated drive command (merai::DriveCommand) 
             *    to shared memory for the Control side to read.
             */
            void writeDriveCommandAggregator(hand_control::merai::DriveCommand cmd);

            /**
             * @brief writeControllerSwitchAggregator
             *  - If the orchestrator wants a new controller, write aggregator to the
             *    control side, passing an enum (merai::ControllerID).
             */
            void writeControllerSwitchAggregator(bool switchWanted, hand_control::merai::ControllerID ctrlId);

            // Periodic scheduling helper structs & methods
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

            // Our orchestrator (holds the higher-level state machine)
            SystemOrchestrator systemOrchestrator_;
        };
    } // namespace logic
} // namespace hand_control
