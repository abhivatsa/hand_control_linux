#include "logic/Logic.h"
#include <stdexcept>
#include <chrono>
#include <thread>
#include "merai/Enums.h" // for AppState, ControllerID, etc.
#include "merai/RTIpc.h"

namespace seven_axis_robot
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
            if (!merai::validate_rt_layout(rtLayout_))
                throw std::runtime_error("[Logic] Failed to map RTMemoryLayout memory (magic/version).");

            // 3) Attach to Logger shared memory
            loggerMem_ = reinterpret_cast<merai::multi_ring_logger_memory *>(loggerShm_.getPtr());
            if (!loggerMem_)
                throw std::runtime_error("[Logic] Failed to map logger memory.");

            seven_axis_robot::merai::log_info(loggerMem_, "Logic", 3100, "[Logic] Shared memory attached");
        }

        Logic::~Logic()
        {
            // Cleanup if needed
        }

        bool Logic::init()
        {
            stateMachine_ = std::make_unique<StateMachine>(
                paramServerPtr_,
                loggerMem_
            );

            // 1) Initialize the StateMachine
            if (!stateMachine_->init())
            {
                seven_axis_robot::merai::log_error(loggerMem_, "Logic", 3101, "[Logic] StateMachine init failed");
                return false;
            }

            // 2) Load Haptic Device Model from paramServer
            if (!hapticDeviceModel_.loadFromParameterServer(*paramServerPtr_))
            {
                seven_axis_robot::merai::log_error(loggerMem_, "Logic", 3102, "[Logic] HapticDeviceModel load failed");
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
                seven_axis_robot::merai::log_error(loggerMem_, "Logic", 3103, "[Logic] SafetyManager init failed");
                return false;
            }

            seven_axis_robot::merai::log_info(loggerMem_, "Logic", 3104, "[Logic] init complete");
            return true;
        }

        void Logic::run()
        {
            seven_axis_robot::merai::log_info(loggerMem_, "Logic", 3105, "[Logic] Entering main loop");
            cyclicTask();
        }

        void Logic::requestStop()
        {
            stopRequested_.store(true, std::memory_order_relaxed);
        }

        void Logic::cyclicTask()
        {
            period_info pinfo;
            // Run logic slower than RT loops to reduce contention (10 ms)
            periodic_task_init(&pinfo, 10'000'000L);

            while (!stopRequested_.load(std::memory_order_relaxed))
            {
                // 1) Read aggregator inputs
                readUserCommands(userCmds);
                readDriveFeedback(driveFdbk);
                readControllerFeedback(ctrlFdbk);

                userCmds.resetFault = true;

                // 2) Safety checks
                isFaulted        = safetyManager_->update(driveFdbk, userCmds, ctrlFdbk);

                isHomingCompleted = safetyManager_->isHomingCompleted();

                // 3) StateMachine update => output commands
                StateManagerOutput stateOutput = stateMachine_->update(isFaulted, isHomingCompleted, driveFdbk, userCmds, ctrlFdbk);

                // 4) If controller not switching, write drive & controller commands
                if (ctrlFdbk.switchResult != merai::ControllerSwitchResult::IN_PROGRESS)
                {
                    writeDriveCommands(stateOutput.driveCmd);
                    writeControllerCommand(stateOutput.ctrlCmd);
                }
                else{
                }

                // 5) Update user feedback with the current application state
                writeUserFeedback(stateOutput.appState);

                // 6) Check if user requested shutdown
                if (userCmds.shutdownRequest)
                {
                    seven_axis_robot::merai::log_info(loggerMem_, "Logic", 3106, "[Logic] Shutdown requested by user");
                    break;
                }

                // 7) Sleep for remainder of period
                wait_rest_of_period(&pinfo);
            }

            seven_axis_robot::merai::log_info(loggerMem_, "Logic", 3107, "[Logic] Exiting main loop");
        }

        // -------------------------------------------------
        // Bridge-based I/O (formerly aggregator)
        // -------------------------------------------------
        void Logic::readUserCommands(seven_axis_robot::merai::UserCommands &out)
        {
            auto meta = merai::read_latest(rtLayout_->userCommandsBuffer, out, lastUserCmdSeq_);
            userCmdFresh_.store(meta.fresh, std::memory_order_relaxed);
            lastUserCmdSeq_ = meta.seq;
        }

        void Logic::readDriveFeedback(seven_axis_robot::merai::DriveFeedbackData &out)
        {
            auto meta = merai::read_latest(rtLayout_->driveFeedbackBuffer, out, lastDriveFdbkSeq_);
            driveFdbkFresh_.store(meta.fresh, std::memory_order_relaxed);
            lastDriveFdbkSeq_ = meta.seq;
        }

        void Logic::readControllerFeedback(seven_axis_robot::merai::ControllerFeedback &out)
        {
            auto meta = merai::read_latest(rtLayout_->controllerFeedbackBuffer, out, lastControllerFdbkSeq_);
            controllerFdbkFresh_.store(meta.fresh, std::memory_order_relaxed);
            lastControllerFdbkSeq_ = meta.seq;
        }

        void Logic::writeDriveCommands(seven_axis_robot::merai::DriveCommandData &in)
        {
            int backIdx = merai::back_index(rtLayout_->driveCommandBuffer);

            auto &dest = rtLayout_->driveCommandBuffer.buffer[backIdx];
            std::size_t driveCount = paramServerPtr_->driveCount; 
            if (driveCount > merai::MAX_SERVO_DRIVES) 
                driveCount = merai::MAX_SERVO_DRIVES;

            for (std::size_t i = 0; i < driveCount; ++i)
            {
                dest.commands[i] = in.commands[i];
            }

            merai::publish(rtLayout_->driveCommandBuffer, backIdx);
        }

        void Logic::writeControllerCommand(seven_axis_robot::merai::ControllerCommand &in)
        {
            int backIdx = merai::back_index(rtLayout_->controllerCommandBuffer);

            rtLayout_->controllerCommandBuffer.buffer[backIdx] = in;
            merai::publish(rtLayout_->controllerCommandBuffer, backIdx);
        }

        void Logic::writeUserFeedback(seven_axis_robot::merai::AppState currentState)
        {
            int backIdx = merai::back_index(rtLayout_->userFeedbackBuffer);

            auto &ufbk = rtLayout_->userFeedbackBuffer.buffer[backIdx];
            ufbk.currentState = currentState;

            merai::publish(rtLayout_->userFeedbackBuffer, backIdx);
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
} // namespace seven_axis_robot
