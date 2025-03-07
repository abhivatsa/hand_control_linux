#include "logic/Logic.h"
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <thread>
// Include your updated Enums.h if you need direct access to merai::DriveCommand, etc.
#include "merai/Enums.h"

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

        Logic::~Logic() {}

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
                // (1) read aggregator from Control: driveSummary
                bool faultActive = false;
                int  faultSeverity = 0;
                readDriveSummaryAggregator(faultActive, faultSeverity);

                // (2) read user aggregator if any (example: from controllerUserCmdBuffer)
                bool userActive = false;
                bool userSwitch = false;
                // If you still store a string in user commands, read it here; 
                // or if it's an enum, read the enum directly.
                readUserCommandsAggregator(userActive, userSwitch /*, maybe controllerID */);

                // (3) orchestrator update
                systemOrchestrator_.update(
                    faultActive, faultSeverity,
                    userActive, userSwitch,
                    /* if you have a textual or enum-based controller from user aggregator, pass it here */
                    /* e.g., "GRAVITY_COMP" or the actual enum if you've changed the orchestrator accordingly */
                );

                // (4) fetch orchestrator's single drive command & controller switch request
                // Now the orchestrator returns an enum
                auto driveCmd        = systemOrchestrator_.getDriveCommand();        // merai::DriveCommand
                bool wantCtrlSwitch  = systemOrchestrator_.wantsControllerSwitch(); 
                auto desiredCtrlID   = systemOrchestrator_.desiredControllerID();    // merai::ControllerID

                // (5) write aggregator => driveCommandBuffer
                writeDriveCommandAggregator(driveCmd);

                // (6) write aggregator => controllerCommandsAggBuffer
                writeControllerSwitchAggregator(wantCtrlSwitch, desiredCtrlID);

                // (7) wait for next cycle
                wait_rest_of_period(&pinfo);
            }

            std::cout << "[Logic] Exiting main loop.\n";
        }

        // ======================
        // Aggregator Helpers
        // ======================
        void Logic::readDriveSummaryAggregator(bool& outAnyFaulted, int& outSeverity)
        {
            int frontIdx = rtLayout_->driveSummaryBuffer.frontIndex.load(std::memory_order_acquire);
            const auto &summary = rtLayout_->driveSummaryBuffer.buffer[frontIdx];
            outAnyFaulted = summary.anyFaulted;
            outSeverity   = summary.faultSeverity;
        }

        void Logic::readUserCommandsAggregator(bool& outUserActive, bool& outUserSwitch /*, ... */)
        {
            // Example of reading from controllerUserCmdBuffer if it's still string-based or partially updated.
            // If you've changed it to store merai::ControllerID, do so here.

            int frontIdx = rtLayout_->controllerUserCmdBuffer.frontIndex.load(std::memory_order_acquire);
            const auto &ucBuf = rtLayout_->controllerUserCmdBuffer.buffer[frontIdx];

            outUserSwitch = ucBuf.commands[0].requestSwitch;
            // outUserActive might come from another aggregator, for now false
            outUserActive = false;

            // If you still have a string field (targetControllerName) in user commands, you can read it here
            // or bridging code if needed.
        }

        // **Updated**: Now we write the drive command aggregator with an enum
        void Logic::writeDriveCommandAggregator(hand_control::merai::DriveCommand cmd)
        {
            int frontIdx = rtLayout_->driveCommandBuffer.frontIndex.load(std::memory_order_acquire);
            int backIdx  = 1 - frontIdx;

            auto &agg = rtLayout_->driveCommandBuffer.buffer[backIdx];
            agg.driveCommand = cmd;  // Directly store the enum

            rtLayout_->driveCommandBuffer.frontIndex.store(backIdx, std::memory_order_release);
        }

        // **Updated**: Now we write the controller aggregator with an enum
        void Logic::writeControllerSwitchAggregator(bool switchWanted, hand_control::merai::ControllerID ctrlID)
        {
            int frontIdx = rtLayout_->controllerCommandsAggBuffer.frontIndex.load(std::memory_order_acquire);
            int backIdx  = 1 - frontIdx;

            auto &ctrlAgg = rtLayout_->controllerCommandsAggBuffer.buffer[backIdx];
            ctrlAgg.requestSwitch   = switchWanted;
            ctrlAgg.targetController = ctrlID;  // Directly store the enum

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
