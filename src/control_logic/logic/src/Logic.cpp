#include "logic/Logic.h"
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <thread>
#include "merai/Enums.h" // for AppState, ControllerID, etc.

namespace hand_control
{
    namespace logic
    {
        Logic::Logic(const std::string &paramServerShmName,
                     std::size_t paramServerShmSize,
                     const std::string &rtDataShmName,
                     std::size_t rtDataShmSize,
                     const std::string &loggerShmName,
                     std::size_t loggerShmSize)
            : paramServerShm_(paramServerShmName, paramServerShmSize, true),
              rtDataShm_(rtDataShmName, rtDataShmSize, false),
              loggerShm_(loggerShmName, loggerShmSize, false)
        {
            // 1) Attach to ParameterServer shared memory
            paramServerPtr_ = reinterpret_cast<const merai::ParameterServer *>(paramServerShm_.getPtr());
            if (!paramServerPtr_)
                throw std::runtime_error("[Logic] Failed to map ParameterServer memory.");

            // 2) Attach to RTMemoryLayout shared memory
            rtLayout_ = reinterpret_cast<merai::RTMemoryLayout *>(rtDataShm_.getPtr());
            if (!rtLayout_)
                throw std::runtime_error("[Logic] Failed to map RTMemoryLayout memory.");

            // 3) Attach to Logger shared memory
            loggerMem_ = reinterpret_cast<merai::multi_ring_logger_memory *>(loggerShm_.getPtr());
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
            stateMachine_ = std::make_unique<StateMachine>(
                paramServerPtr_
            );

            // 1) Initialize the StateMachine
            if (!stateMachine_->init())
            {
                std::cerr << "[Logic] StateMachine init failed.\n";
                return false;
            }

            // 2) Load Haptic Device Model from paramServer
            if (!hapticDeviceModel_.loadFromParameterServer(*paramServerPtr_))
            {
                std::cerr << "[Logic] HapticDeviceModel load failed.\n";
                return false;
            }

            // 3) Create a new SafetyManager in a unique_ptr
            safetyManager_ = std::make_unique<SafetyManager>(
                paramServerPtr_,
                rtLayout_,
                hapticDeviceModel_
            );
            
            if (!safetyManager_->init())
            {
                std::cerr << "[Logic] SafetyManager init failed.\n";
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
            // Example: run at 10 ms cycle
            periodic_task_init(&pinfo, 1'000'000L);

            while (!stopRequested_.load(std::memory_order_relaxed))
            {
                // 1) Read aggregator inputs
                readUserCommands(userCmds);
                readDriveFeedback(driveFdbk);
                readControllerFeedback(ctrlFdbk);

                userCmds.resetFault = true;

                // std::cout<<""

                // 2) Safety checks
                isFaulted        = safetyManager_->update(driveFdbk, userCmds, ctrlFdbk);

                // std::cout<<"isFaulted : "<<isFaulted<<std::endl;

                isHomingCompleted = safetyManager_->isHomingCompleted();

                // 3) StateMachine update => output commands
                StateManagerOutput stateOutput = stateMachine_->update(isFaulted, isHomingCompleted, driveFdbk, userCmds, ctrlFdbk);

                // std::cout<<"Request Switch : "<<stateOutput.ctrlCmd.requestSwitch<<", Controller_id : "<<int(stateOutput.ctrlCmd.controllerId)<<std::endl;

                // 4) If controller not switching, write drive & controller commands
                if (ctrlFdbk.feedbackState != merai::ControllerFeedbackState::SWITCH_IN_PROGRESS)
                {
                    // std::cout<<"drive command : "<<int(stateOutput.driveCmd.commands[0])<<std::endl;
                    writeDriveCommands(stateOutput.driveCmd);
                    writeControllerCommand(stateOutput.ctrlCmd);
                }
                else{
                    // std::cout<<"switching Controller :  "<<int(ctrlFdbk.feedbackState)<<std::endl;
                    // std::cout<<"drive command : "<<int(stateOutput.driveCmd.commands[0])<<std::endl;
                    // std::cout<<"ctrl_cmd : "<<int(stateOutput.ctrlCmd.controllerId)<<std::endl;
                    // std::cout<<"ctrl_cmd requestSwitch: "<<int(stateOutput.ctrlCmd.requestSwitch)<<std::endl;
                }

                // 5) Update user feedback with the current application state
                writeUserFeedback(stateOutput.appState);

                // 6) Check if user requested shutdown
                if (userCmds.shutdownRequest)
                {
                    std::cout << "[Logic] Shutdown requested by user.\n";
                    break;
                }

                // 7) Sleep for remainder of period
                wait_rest_of_period(&pinfo);
            }

            std::cout << "[Logic] Exiting main loop.\n";
        }

        // -------------------------------------------------
        // Bridge-based I/O (formerly aggregator)
        // -------------------------------------------------
        void Logic::readUserCommands(hand_control::merai::UserCommands &out)
        {
            int frontIdx = rtLayout_->userCommandsBuffer.frontIndex.load(std::memory_order_acquire);
            out = rtLayout_->userCommandsBuffer.buffer[frontIdx];
        }

        void Logic::readDriveFeedback(hand_control::merai::DriveFeedbackData &out)
        {
            int frontIdx = rtLayout_->driveFeedbackBuffer.frontIndex.load(std::memory_order_acquire);
            out = rtLayout_->driveFeedbackBuffer.buffer[frontIdx];
        }

        void Logic::readControllerFeedback(hand_control::merai::ControllerFeedback &out)
        {
            int frontIdx = rtLayout_->controllerFeedbackBuffer.frontIndex.load(std::memory_order_acquire);
            out = rtLayout_->controllerFeedbackBuffer.buffer[frontIdx];
        }

        void Logic::writeDriveCommands(hand_control::merai::DriveCommandData &in)
        {
            int frontIdx = rtLayout_->driveCommandBuffer.frontIndex.load(std::memory_order_acquire);
            int backIdx = 1 - frontIdx;

            auto &dest = rtLayout_->driveCommandBuffer.buffer[backIdx];
            std::size_t driveCount = paramServerPtr_->driveCount; 
            if (driveCount > merai::MAX_SERVO_DRIVES) 
                driveCount = merai::MAX_SERVO_DRIVES;

            for (std::size_t i = 0; i < driveCount; ++i)
            {
                dest.commands[i] = in.commands[i];
            }

            rtLayout_->driveCommandBuffer.frontIndex.store(backIdx, std::memory_order_release);
        }

        void Logic::writeControllerCommand(hand_control::merai::ControllerCommand &in)
        {
            int frontIdx = rtLayout_->controllerCommandBuffer.frontIndex.load(std::memory_order_acquire);
            int backIdx = 1 - frontIdx;

            rtLayout_->controllerCommandBuffer.buffer[backIdx] = in;
            rtLayout_->controllerCommandBuffer.frontIndex.store(backIdx, std::memory_order_release);
        }

        void Logic::writeUserFeedback(hand_control::merai::AppState currentState)
        {
            int frontIdx = rtLayout_->userFeedbackBuffer.frontIndex.load(std::memory_order_acquire);
            int backIdx = 1 - frontIdx;

            auto &ufbk = rtLayout_->userFeedbackBuffer.buffer[backIdx];
            ufbk.currentState = currentState;

            rtLayout_->userFeedbackBuffer.frontIndex.store(backIdx, std::memory_order_release);
        }

        // -------------------------------------------------
        // Scheduling Helpers
        // -------------------------------------------------
        void Logic::periodic_task_init(period_info *pinfo, long periodNs)
        {
            clock_gettime(CLOCK_MONOTONIC, &pinfo->next_period);
            pinfo->period_ns = periodNs;
        }

        void Logic::inc_period(period_info *pinfo)
        {
            pinfo->next_period.tv_nsec += pinfo->period_ns;
            if (pinfo->next_period.tv_nsec >= 1'000'000'000L)
            {
                pinfo->next_period.tv_nsec -= 1'000'000'000L;
                pinfo->next_period.tv_sec++;
            }
        }

        void Logic::wait_rest_of_period(period_info *pinfo)
        {
            inc_period(pinfo);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, nullptr);
        }

    } // namespace logic
} // namespace hand_control
