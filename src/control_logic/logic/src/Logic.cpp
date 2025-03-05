#include "logic/Logic.h"
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <cstring>

namespace hand_control
{
    namespace logic
    {
        Logic::Logic(const std::string& paramServerShmName, std::size_t paramServerShmSize,
                     const std::string& rtDataShmName,      std::size_t rtDataShmSize,
                     const std::string& loggerShmName,      std::size_t loggerShmSize)
            : paramServerShm_(paramServerShmName, paramServerShmSize, true),
              rtDataShm_(rtDataShmName, rtDataShmSize, false),
              loggerShm_(loggerShmName, loggerShmSize, false)
        {
            // 1) param server
            paramServerPtr_ = reinterpret_cast<const merai::ParameterServer*>(paramServerShm_.getPtr());
            if (!paramServerPtr_)
                throw std::runtime_error("[Logic] Failed to map ParameterServer memory.");

            // 2) RTMemoryLayout
            rtLayout_ = reinterpret_cast<merai::RTMemoryLayout*>(rtDataShm_.getPtr());
            if (!rtLayout_)
                throw std::runtime_error("[Logic] Failed to map RTMemoryLayout memory.");

            // 3) Logger
            loggerMem_ = reinterpret_cast<merai::multi_ring_logger_memory*>(loggerShm_.getPtr());
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

        // The main loop (10ms for example)
        void Logic::cyclicTask()
        {
            period_info pinfo;
            periodic_task_init(&pinfo, 10'000'000L);

            while (!stopRequested_.load(std::memory_order_relaxed))
            {
                // (1) read aggregator from control: driveSummary
                bool faultActive=false;
                int faultSeverity=0;
                readDriveSummaryAggregator(faultActive, faultSeverity);

                // read user aggregator if any (for demonstration, we use controllerUserCmdBuffer)
                bool userActive=false, userSwitch=false;
                char ctrlName[64]={0};
                readUserCommandsAggregator(userActive, userSwitch, ctrlName);

                // (2) orchestrator update
                systemOrchestrator_.update(faultActive, faultSeverity,
                                           userActive, userSwitch,
                                           ctrlName);

                // (3) get orchestrator's single DriveCommand
                auto driveCmd = systemOrchestrator_.getDriveCommand();
                bool wantCtrlSwitch = systemOrchestrator_.wantsControllerSwitch();
                auto desiredCtrlName = systemOrchestrator_.desiredControllerName();

                // (4) write aggregator => driveCommandBuffer
                writeDriveCommandAggregator(driveCmd);

                // (5) write aggregator => controllerCommandsAggBuffer
                writeControllerSwitchAggregator(wantCtrlSwitch, desiredCtrlName);

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

        void Logic::readUserCommandsAggregator(bool& outUserActive,
                                               bool& outUserSwitch,
                                               char* outControllerName)
        {
            // For demonstration, read from controllerUserCmdBuffer
            int frontIdx = rtLayout_->controllerUserCmdBuffer.frontIndex.load(std::memory_order_acquire);
            const auto &ucBuf = rtLayout_->controllerUserCmdBuffer.buffer[frontIdx];

            outUserSwitch = ucBuf.commands[0].requestSwitch;
            if (outUserSwitch)
            {
                std::strncpy(outControllerName,
                             ucBuf.commands[0].targetControllerName,
                             sizeof(ucBuf.commands[0].targetControllerName)-1);
            }
            // outUserActive might come from another aggregator, for now false
            outUserActive = false;
        }

        void Logic::writeDriveCommandAggregator(hand_control::logic::DriveCommand cmd)
        {
            int driveCmdInt = static_cast<int>(cmd);
            int frontIdx  = rtLayout_->driveCommandBuffer.frontIndex.load(std::memory_order_acquire);
            int backIdx   = 1 - frontIdx;

            auto &agg = rtLayout_->driveCommandBuffer.buffer[backIdx];
            agg.driveCommand = driveCmdInt;

            rtLayout_->driveCommandBuffer.frontIndex.store(backIdx, std::memory_order_release);
        }

        void Logic::writeControllerSwitchAggregator(bool switchWanted, const std::string &ctrlName)
        {
            int frontIdx  = rtLayout_->controllerCommandsAggBuffer.frontIndex.load(std::memory_order_acquire);
            int backIdx   = 1 - frontIdx;

            auto &ctrlAgg = rtLayout_->controllerCommandsAggBuffer.buffer[backIdx];
            ctrlAgg.requestSwitch = false;
            std::memset(ctrlAgg.targetControllerName, 0, sizeof(ctrlAgg.targetControllerName));

            if (switchWanted)
            {
                ctrlAgg.requestSwitch = true;
                std::strncpy(ctrlAgg.targetControllerName,
                             ctrlName.c_str(),
                             sizeof(ctrlAgg.targetControllerName)-1);
            }

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
