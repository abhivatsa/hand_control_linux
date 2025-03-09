#include "logic/Logic.h"
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <thread>
#include "merai/Enums.h"  // for OrchestratorState, ControllerID, etc.

namespace hand_control
{
    namespace logic
    {
        Logic::Logic(const std::string& paramServerShmName,
                     std::size_t paramServerShmSize,
                     const std::string& rtDataShmName,
                     std::size_t rtDataShmSize,
                     const std::string& loggerShmName,
                     std::size_t loggerShmSize)
            : paramServerShm_(paramServerShmName, paramServerShmSize, true),
              rtDataShm_(rtDataShmName, rtDataShmSize, false),
              loggerShm_(loggerShmName, loggerShmSize, false)
        {
            // 1) param server
            paramServerPtr_ = reinterpret_cast<const merai::ParameterServer*>(
                                  paramServerShm_.getPtr());
            if (!paramServerPtr_)
                throw std::runtime_error("[Logic] Failed to map ParameterServer memory.");

            // 2) RTMemoryLayout
            rtLayout_ = reinterpret_cast<merai::RTMemoryLayout*>(rtDataShm_.getPtr());
            if (!rtLayout_)
                throw std::runtime_error("[Logic] Failed to map RTMemoryLayout memory.");

            // 3) Logger
            loggerMem_ = reinterpret_cast<merai::multi_ring_logger_memory*>(
                             loggerShm_.getPtr());
            if (!loggerMem_)
                throw std::runtime_error("[Logic] Failed to map logger memory.");

            std::cout << "[Logic] Shared memory attached.\n";
        }

        Logic::~Logic()
        {
            // Cleanup if needed
        }

        bool Logic::init()
        {
            // e.g. orchestrator init
            if (!systemOrchestrator_.init())
            {
                std::cerr << "[Logic] SystemOrchestrator init failed.\n";
                return false;
            }
            std::cout << "[Logic] init complete.\n";
            return true;
        }

        void Logic::run()
        {
            std::cout << "[Logic] Entering main loop.\n";
            cyclicTask();
        }

        void Logic::requestStop()
        {
            stopRequested_.store(true, std::memory_order_relaxed);
        }

        // The main loop (10ms example)
        void Logic::cyclicTask()
        {
            period_info pinfo;
            // 10 ms = 10,000,000 ns
            periodic_task_init(&pinfo, 10'000'000L);

            while (!stopRequested_.load(std::memory_order_relaxed))
            {
                // (1) Possibly read aggregator from control if you want drive feedback, 
                //     but if you removed DriveSummary, there's nothing to read here.

                // (2) Possibly check user input or aggregator to see if user wants active, 
                //     or a controller switch, etc. 
                bool userActive   = false;
                bool userSwitch   = false;

                // Update your orchestrator logic
                systemOrchestrator_.update(
                    /* faultActive= */ false,
                    /* faultSeverity= */ 0,
                    userActive,
                    userSwitch
                    // etc. If you have more enum or ID for desiredController, pass it here
                );

                // (3) The orchestrator decides how each drive should be controlled.
                //     We'll write those signals into driveControlSignalsBuffer:
                writeDriveControlSignalsAggregator();

                // (4) Possibly handle a controller switch aggregator
                bool switchWanted = systemOrchestrator_.wantsControllerSwitch();
                auto ctrlID       = systemOrchestrator_.desiredControllerId();
                writeControllerSwitchAggregator(switchWanted, ctrlID);

                // (5) Wait until next cycle
                wait_rest_of_period(&pinfo);
            }

            std::cout << "[Logic] Exiting main loop.\n";
        }

        // ======================
        // Aggregator Helpers
        // ======================

        void Logic::writeDriveControlSignalsAggregator()
        {
            size_t driveCount = paramServerPtr_->driveCount;

            int frontIdx = rtLayout_->driveControlSignalsBuffer.frontIndex.load(std::memory_order_acquire);
            int backIdx  = 1 - frontIdx;

            auto &signalsData = rtLayout_->driveControlSignalsBuffer.buffer[backIdx];

            // Clear / set each drive's flags based on the orchestrator
            for (size_t i = 0; i < driveCount; ++i)
            {
                auto &sig = signalsData.signals[i];
                sig.faultReset     = false;
                sig.allowOperation = false;
                sig.quickStop      = false;
                sig.forceDisable   = false;

                // e.g. if orchestrator says drive i is healthy & user wants it on
                if (systemOrchestrator_.shouldEnableDrive(i))
                    sig.allowOperation = true;

                if (systemOrchestrator_.shouldForceDisableDrive(i))
                    sig.forceDisable = true;

                if (systemOrchestrator_.shouldQuickStopDrive(i))
                    sig.quickStop = true;

                if (systemOrchestrator_.shouldFaultResetDrive(i))
                    sig.faultReset = true;
            }

            // Publish 
            rtLayout_->driveControlSignalsBuffer.frontIndex.store(backIdx, std::memory_order_release);
        }

        void Logic::writeControllerSwitchAggregator(bool switchWanted, hand_control::merai::ControllerID ctrlId)
        {
            int frontIdx = rtLayout_->controllerCommandsAggBuffer.frontIndex.load(std::memory_order_acquire);
            int backIdx  = 1 - frontIdx;

            auto &ctrlAgg = rtLayout_->controllerCommandsAggBuffer.buffer[backIdx];
            ctrlAgg.requestSwitch    = switchWanted;
            ctrlAgg.targetController = ctrlId;

            rtLayout_->controllerCommandsAggBuffer.frontIndex.store(backIdx, std::memory_order_release);
        }

        // scheduling
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
