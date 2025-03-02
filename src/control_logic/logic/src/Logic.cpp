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
            : paramServerShm_(paramServerShmName, paramServerShmSize, true)
            , rtDataShm_(rtDataShmName, rtDataShmSize, false)
            , loggerShm_(loggerShmName, loggerShmSize, false)
        {
            // Attach param server
            paramServerPtr_ = reinterpret_cast<const merai::ParameterServer*>(paramServerShm_.getPtr());
            if (!paramServerPtr_)
                throw std::runtime_error("[Logic] Failed to map ParameterServer memory.");

            // Attach RTMemoryLayout
            rtLayout_ = reinterpret_cast<merai::RTMemoryLayout*>(rtDataShm_.getPtr());
            if (!rtLayout_)
                throw std::runtime_error("[Logic] Failed to map RTMemoryLayout memory.");

            // Attach logger memory
            loggerMem_ = reinterpret_cast<merai::multi_ring_logger_memory*>(loggerShm_.getPtr());
            if (!loggerMem_)
                throw std::runtime_error("[Logic] Failed to map logger memory.");

            std::cout << "[Logic] Shared memory attached.\n";
        }

        Logic::~Logic()
        {
        }

        bool Logic::init()
        {
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

        void Logic::cyclicTask()
        {
            period_info pinfo;
            periodic_task_init(&pinfo, 10'000'000L); // 10ms

            while (!stopRequested_.load(std::memory_order_relaxed))
            {
                //============================
                // (1) Read aggregator from Control
                //============================
                bool faultActive = false;
                int faultSeverity = 0;
                readDriveSummary(faultActive, faultSeverity);

                bool userActive = false;
                bool userSwitch = false;
                char ctrlName[64] = {0};
                readControllerUserCommands(userActive, userSwitch, ctrlName);

                //============================
                // (2) Orchestrator update
                //============================
                // pass whether there's a fault, user wants active, user wants ctrl switch
                systemOrchestrator_.update(
                    faultActive, faultSeverity,
                    userActive,
                    userSwitch, 
                    ctrlName
                );

                // retrieve the orchestrator's single drive command
                auto driveCmd = systemOrchestrator_.getDriveCommand();

                // check if orchestrator wants a new controller
                bool wantCtrlSwitch = systemOrchestrator_.wantsControllerSwitch();
                auto desiredCtrlName = systemOrchestrator_.desiredControllerName();

                //============================
                // (3) Write drive command aggregator
                //============================
                writeDriveCommand(driveCmd);

                // (4) Write controller switch aggregator if needed
                writeControllerSwitch(wantCtrlSwitch, desiredCtrlName);

                //============================
                // (5) Sleep
                //============================
                wait_rest_of_period(&pinfo);
            }

            std::cout << "[Logic] Exiting main loop.\n";
        }

        //===============================
        // Helpers
        //===============================
        void Logic::readDriveSummary(bool& outAnyFaulted, int& outSeverity)
        {
            int frontIdx = rtLayout_->driveSummaryBuffer.frontIndex.load(std::memory_order_acquire);
            const auto& summary = rtLayout_->driveSummaryBuffer.buffer[frontIdx];
            outAnyFaulted = summary.anyFaulted;  
            outSeverity   = summary.faultSeverity; 
        }

        void Logic::readControllerUserCommands(bool& outUserRequestedActive,
                                               bool& outUserRequestedSwitch,
                                               char* outControllerName)
        {
            // For demonstration, we might read a user aggregator 
            // or something from paramServer. 
            // We'll keep it simple:
            // Suppose the control side (or a user interface) wrote a small aggregator
            // indicating "start operation" or "switch controller"

            // Example:
            int frontIdx = rtLayout_->controllerUserCmdBuffer.frontIndex.load(std::memory_order_acquire);
            const auto& cmdBuf = rtLayout_->controllerUserCmdBuffer.buffer[frontIdx];
            // e.g. we interpret userCmdBuf.commands[0] differently
            // For demonstration, let's say requestSwitch => user requests ctrl switch
            outUserRequestedSwitch = cmdBuf.commands[0].requestSwitch;
            if (outUserRequestedSwitch)
            {
                std::strncpy(outControllerName,
                             cmdBuf.commands[0].targetControllerName,
                             63);
            }

            // Maybe we have another aggregator for "start operation"
            // For demonstration, let's just default to false
            outUserRequestedActive = false;
        }

        void Logic::writeDriveCommand(hand_control::logic::DriveCommand cmd)
        {
            int frontIdx  = rtLayout_->driveCommandBuffer.frontIndex.load(std::memory_order_acquire);
            int backIndex = 1 - frontIdx;

            auto& driveCmdAgg = rtLayout_->driveCommandBuffer.buffer[backIndex];
            driveCmdAgg.driveCommand = cmd; // store the enum

            rtLayout_->driveCommandBuffer.frontIndex.store(backIndex, std::memory_order_release);
        }

        void Logic::writeControllerSwitch(bool switchWanted, const std::string& ctrlName)
        {
            int frontIdx  = rtLayout_->controllerCommandsAggBuffer.frontIndex.load(std::memory_order_acquire);
            int backIndex = 1 - frontIdx;

            auto& ctrlAgg = rtLayout_->controllerCommandsAggBuffer.buffer[backIndex];
            ctrlAgg.requestSwitch = false;
            std::memset(ctrlAgg.targetControllerName, 0, sizeof(ctrlAgg.targetControllerName));

            if (switchWanted)
            {
                ctrlAgg.requestSwitch = true;
                std::strncpy(ctrlAgg.targetControllerName,
                             ctrlName.c_str(),
                             sizeof(ctrlAgg.targetControllerName)-1);
            }

            rtLayout_->controllerCommandsAggBuffer.frontIndex.store(backIndex, std::memory_order_release);
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
