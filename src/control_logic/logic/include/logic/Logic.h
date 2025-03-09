#pragma once

#include <atomic>
#include <cstddef>
#include <time.h>

// merai / Common headers
#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/RAII_SharedMemory.h"
#include "merai/SharedLogger.h"
#include "merai/Enums.h" // for ControllerID, OrchestratorState, etc.

// Orchestrator (logic layer)
#include "logic/SystemOrchestrator.h"

namespace hand_control
{
    namespace logic
    {
        class Logic
        {
        public:
            Logic(const std::string& paramServerShmName, 
                  std::size_t paramServerShmSize,
                  const std::string& rtDataShmName, 
                  std::size_t rtDataShmSize,
                  const std::string& loggerShmName, 
                  std::size_t loggerShmSize);

            ~Logic();

            /**
             * @brief Initializes the logic system, including orchestrator init.
             * @return true if successful
             */
            bool init();

            /**
             * @brief Runs the main (blocking) loop of the logic application,
             *        typically at a 10 ms cycle.
             */
            void run();

            /**
             * @brief Signals the logic loop to stop gracefully.
             */
            void requestStop();

        private:
            /**
             * @brief The main logic loop (e.g., 10 ms). Reads aggregator data
             *        if needed, orchestrates system logic, writes per-drive signals
             *        and controller commands to shared memory.
             */
            void cyclicTask();

            /**
             * @brief writeDriveControlSignalsAggregator
             *  - Writes per-drive signals (enable, faultReset, etc.) determined by the
             *    orchestrator into shared memory so the Control side can read them.
             */
            void writeDriveControlSignalsAggregator();

            /**
             * @brief writeControllerSwitchAggregator
             *  - If the orchestrator wants a new controller, write aggregator to
             *    the control side, specifying which controller ID to switch to.
             */
            void writeControllerSwitchAggregator(bool switchWanted, 
                                                 hand_control::merai::ControllerID ctrlId);

            // --------------------------------------
            // Periodic scheduling helpers
            // --------------------------------------
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

            // Our orchestrator (manages high-level logic)
            SystemOrchestrator systemOrchestrator_;
        };
    } // namespace logic
} // namespace hand_control
