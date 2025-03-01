#include "logic/Logic.h"
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <thread>

namespace hand_control
{
    namespace logic
    {
        Logic::Logic(const std::string &paramServerShmName, size_t paramServerShmSize,
                     const std::string &rtDataShmName,      size_t rtDataShmSize,
                     const std::string &loggerShmName,      size_t loggerShmSize)
            : paramServerShm_(paramServerShmName, paramServerShmSize, /*readOnly=*/true)
            , rtDataShm_(rtDataShmName, rtDataShmSize, /*readOnly=*/false)
            , loggerShm_(loggerShmName, loggerShmSize, /*readOnly=*/false)
        {
            // Attach & map ParameterServer
            paramServerPtr_ = reinterpret_cast<const merai::ParameterServer*>(
                paramServerShm_.getPtr());
            if (!paramServerPtr_)
            {
                throw std::runtime_error("[Logic] Failed to map ParameterServer memory.");
            }

            // Attach & map RTMemoryLayout
            rtLayout_ = reinterpret_cast<merai::RTMemoryLayout*>(rtDataShm_.getPtr());
            if (!rtLayout_)
            {
                throw std::runtime_error("[Logic] Failed to map RTMemoryLayout memory.");
            }

            // Attach & map Logger memory
            loggerMem_ = reinterpret_cast<merai::multi_ring_logger_memory*>(loggerShm_.getPtr());
            if (!loggerMem_)
            {
                throw std::runtime_error("[Logic] Failed to map logger memory.");
            }

            std::cout << "[Logic] Shared memory attached successfully.\n";
        }

        Logic::~Logic()
        {
            // Cleanup if needed
        }

        bool Logic::init()
        {
            // 1) Create managers
            orchestrator_ = std::make_unique<SystemOrchestrator>();
            safetyManager_ = std::make_unique<SafetyManager>();
            errorManager_  = std::make_unique<ErrorManager>();

            // 2) Initialize them (example)
            if (!orchestrator_->init(paramServerPtr_, rtLayout_, errorManager_.get()))
            {
                std::cerr << "[Logic] SystemOrchestrator init failed.\n";
                return false;
            }
            if (!safetyManager_->init(paramServerPtr_, rtLayout_, errorManager_.get()))
            {
                std::cerr << "[Logic] SafetyManager init failed.\n";
                return false;
            }
            if (!errorManager_->init())
            {
                std::cerr << "[Logic] ErrorManager init failed.\n";
                return false;
            }

            std::cout << "[Logic] init complete.\n";
            return true;
        }

        void Logic::run()
        {
            std::cout << "[Logic] Running main loop.\n";
            cyclicTask();
        }

        void Logic::requestStop()
        {
            stopRequested_.store(true, std::memory_order_relaxed);
        }

        void Logic::cyclicTask()
        {
            period_info pinfo;
            // e.g. 10 ms cycle
            periodic_task_init(&pinfo, 10'000'000L);

            while (!stopRequested_.load(std::memory_order_relaxed))
            {
                // 1) Update safety
                safetyManager_->update();

                // 2) If safety triggers a fault, orchestrator handles it
                //    but let's check if the safety manager reported something
                if (safetyManager_->isFaulted())
                {
                    orchestrator_->forceEmergencyStop(); // or something similar
                }

                // 3) Update orchestrator
                orchestrator_->update();

                // 4) Check if there's a critical error
                if (errorManager_->hasCriticalError())
                {
                    // Possibly ensure orchestrator is in FAULT
                    orchestrator_->forceEmergencyStop();
                }

                // 5) Sleep until next cycle
                wait_rest_of_period(&pinfo);
            }

            std::cout << "[Logic] Exiting main loop.\n";
        }

        void Logic::periodic_task_init(period_info* pinfo, long periodNs)
        {
            clock_gettime(CLOCK_MONOTONIC, &pinfo->next_period);
            pinfo->period_ns = periodNs;
        }

        void Logic::inc_period(period_info* pinfo)
        {
            pinfo->next_period.tv_nsec += pinfo->period_ns;
            if (pinfo->next_period.tv_nsec >= 1'000'000'000L)
            {
                pinfo->next_period.tv_nsec -= 1'000'000'000L;
                pinfo->next_period.tv_sec++;
            }
        }

        void Logic::wait_rest_of_period(period_info* pinfo)
        {
            inc_period(pinfo);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, nullptr);
        }

    } // namespace logic
} // namespace hand_control
