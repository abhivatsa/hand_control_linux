#pragma once

#include <memory>
#include <atomic>
#include <string>
#include <time.h>  // for timespec

#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/SharedLogger.h"
#include "merai/RAII_SharedMemory.h"

#include "logic/SystemOrchestrator.h"
#include "logic/SafetyManager.h"
#include "logic/ErrorManager.h"

namespace hand_control
{
    namespace logic
    {
        class Logic
        {
        public:
            Logic(const std::string &paramServerShmName, size_t paramServerShmSize,
                  const std::string &rtDataShmName,      size_t rtDataShmSize,
                  const std::string &loggerShmName,      size_t loggerShmSize);

            ~Logic();

            bool init();
            void run();
            void requestStop();

        private:
            void cyclicTask();

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

            // SHM attachments
            hand_control::merai::RAII_SharedMemory paramServerShm_;
            const hand_control::merai::ParameterServer* paramServerPtr_ = nullptr;

            hand_control::merai::RAII_SharedMemory rtDataShm_;
            hand_control::merai::RTMemoryLayout* rtLayout_ = nullptr;

            hand_control::merai::RAII_SharedMemory loggerShm_;
            hand_control::merai::multi_ring_logger_memory* loggerMem_ = nullptr;

            // Managers
            std::unique_ptr<SystemOrchestrator> orchestrator_;
            std::unique_ptr<SafetyManager>      safetyManager_;
            std::unique_ptr<ErrorManager>       errorManager_;
        };
    }
}
