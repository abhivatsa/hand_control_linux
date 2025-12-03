#include "logic/Logic.h"

#include <stdexcept>
#include "merai/Enums.h"   // AppState, ControllerID, etc.
#include "merai/RTIpc.h"   // read_snapshot, back_index, publish

namespace logic
{
    // -------------------------------------------------------------------------
    // Per-cycle data (mirrors ControlCycleInputs/Outputs idea)
    // -------------------------------------------------------------------------
    struct LogicCycleInputs
    {
        merai::UserCommands          userCmds{};
        merai::DriveFeedbackData     driveFdbk{};
        merai::ControllerFeedback    ctrlFdbk{};
        merai::PlannerResult         plannerResult{};
        merai::JointTrajectoryStatus trajStatus{};
    };

    struct LogicCycleOutputs
    {
        merai::DriveCommandData   driveCmd{};
        merai::ControllerCommand  ctrlCmd{};
        merai::AppState           appState{merai::AppState::INIT};
        bool                      trajectoryActive = false;
    };

    // -------------------------------------------------------------------------
    // Construction / destruction
    // -------------------------------------------------------------------------
    Logic::Logic(const std::string &paramServerShmName,
                 std::size_t        paramServerShmSize,
                 const std::string &rtDataShmName,
                 std::size_t        rtDataShmSize,
                 const std::string &loggerShmName,
                 std::size_t        loggerShmSize)
        : paramServerShm_(paramServerShmName, paramServerShmSize, true),
          rtDataShm_(rtDataShmName, rtDataShmSize, false),
          loggerShm_(loggerShmName, loggerShmSize, false)
    {
        // 1) Attach to ParameterServer shared memory
        paramServerPtr_ =
            reinterpret_cast<const merai::ParameterServer *>(paramServerShm_.getPtr());
        if (!paramServerPtr_)
        {
            throw std::runtime_error("[Logic] Failed to map ParameterServer memory.");
        }

        // 2) Attach to RTMemoryLayout shared memory
        rtLayout_ =
            reinterpret_cast<merai::RTMemoryLayout *>(rtDataShm_.getPtr());
        if (!merai::validate_rt_layout(rtLayout_))
        {
            throw std::runtime_error("[Logic] Failed to map RTMemoryLayout memory (magic/version).");
        }

        // 3) Attach to Logger shared memory
        loggerMem_ =
            reinterpret_cast<merai::multi_ring_logger_memory *>(loggerShm_.getPtr());
        if (!loggerMem_)
        {
            throw std::runtime_error("[Logic] Failed to map logger memory.");
        }

        merai::log_info(loggerMem_, "Logic", 3100, "[Logic] Shared memory attached");
    }

    Logic::~Logic() = default;

    // -------------------------------------------------------------------------
    // init / run / stop
    // -------------------------------------------------------------------------
    bool Logic::init()
    {
        // 1) State machine
        stateMachine_ = std::make_unique<StateMachine>(
            paramServerPtr_,
            loggerMem_);
        if (!stateMachine_->init())
        {
            merai::log_error(loggerMem_, "Logic", 3101, "[Logic] StateMachine init failed");
            return false;
        }

        // 2) Robot model
        if (!robotModel_.loadFromParameterServer(*paramServerPtr_))
        {
            merai::log_error(loggerMem_, "Logic", 3102, "[Logic] RobotModel load failed");
            return false;
        }

        // 3) Safety manager
        safetyManager_ = std::make_unique<SafetyManager>(
            paramServerPtr_,
            rtLayout_,
            robotModel_);
        if (!safetyManager_->init())
        {
            merai::log_error(loggerMem_, "Logic", 3103, "[Logic] SafetyManager init failed");
            return false;
        }

        merai::log_info(loggerMem_, "Logic", 3104, "[Logic] init complete");
        return true;
    }

    void Logic::run()
    {
        merai::log_info(loggerMem_, "Logic", 3105, "[Logic] Entering main loop");
        cyclicTask();
        merai::log_info(loggerMem_, "Logic", 3107, "[Logic] Exiting main loop");
    }

    void Logic::requestStop()
    {
        stopRequested_.store(true, std::memory_order_relaxed);
    }

    // -------------------------------------------------------------------------
    // Main 10 ms logic loop
    // -------------------------------------------------------------------------
    void Logic::cyclicTask()
    {
        period_info pinfo;
        // Logic runs slower than RT loops to reduce contention (10 ms)
        periodic_task_init(&pinfo, 10'000'000L);

        while (!stopRequested_.load(std::memory_order_relaxed))
        {
            LogicCycleInputs  in{};
            LogicCycleOutputs out{};

            // PHASE 1: Read inputs from SHM
            readUserCommands(in.userCmds);
            readDriveFeedback(in.driveFdbk);
            readControllerFeedback(in.ctrlFdbk);
            readPlannerResult(in.plannerResult);
            readTrajectoryStatus(in.trajStatus);

            // PHASE 2: Derive trajectoryActive from JointTrajectoryStatus + controller
            bool trajectoryActive = false;
            if (in.trajStatus.active != 0 &&
                !in.trajStatus.finished &&
                !in.trajStatus.aborted)
            {
                trajectoryActive = true;
            }

            // Only treat as "active motion" if a motion controller is active.
            auto activeCtrl = in.ctrlFdbk.activeControllerId;
            if (trajectoryActive &&
                !(activeCtrl == merai::ControllerID::JOINT_TRAJECTORY ||
                  activeCtrl == merai::ControllerID::JOINT_JOG))
            {
                trajectoryActive = false;
            }

            // PHASE 3: Safety evaluation
            bool faultActive = safetyManager_->update(
                in.driveFdbk,
                in.userCmds,
                in.ctrlFdbk);

            // PHASE 4: High-level state machine -> app state + commands
            StateManagerOutput smOut =
                stateMachine_->update(
                    faultActive,
                    trajectoryActive,
                    in.driveFdbk,
                    in.userCmds,
                    in.ctrlFdbk);

            out.driveCmd         = smOut.driveCmd;
            out.ctrlCmd          = smOut.ctrlCmd;
            out.appState         = smOut.appState;
            out.trajectoryActive = trajectoryActive;

            // PHASE 5: Write drive/controller commands if controller is not mid-switch
            if (in.ctrlFdbk.switchResult != merai::ControllerSwitchResult::IN_PROGRESS)
            {
                writeDriveCommands(out.driveCmd);
                writeControllerCommand(out.ctrlCmd);
            }

            // PHASE 6: Publish user feedback (AppState)
            writeUserFeedback(out.appState);

            // PHASE 7: Check shutdown request
            if (in.userCmds.shutdownRequest)
            {
                merai::log_info(loggerMem_, "Logic", 3106,
                                "[Logic] Shutdown requested by user");
                break;
            }

            // PHASE 8: Sleep until next 10 ms tick
            wait_rest_of_period(&pinfo);
        }
    }

    // -------------------------------------------------------------------------
    // SHM I/O helpers
    // -------------------------------------------------------------------------
    void Logic::readUserCommands(merai::UserCommands &out)
    {
        merai::read_snapshot(rtLayout_->userCommandsBuffer, out);
    }

    void Logic::readDriveFeedback(merai::DriveFeedbackData &out)
    {
        merai::read_snapshot(rtLayout_->driveFeedbackBuffer, out);
    }

    void Logic::readControllerFeedback(merai::ControllerFeedback &out)
    {
        merai::read_snapshot(rtLayout_->controllerFeedbackBuffer, out);
    }

    void Logic::readPlannerResult(merai::PlannerResult &out)
    {
        merai::read_snapshot(rtLayout_->plannerResultBuffer, out);
    }

    void Logic::readTrajectoryStatus(merai::JointTrajectoryStatus &out)
    {
        merai::read_snapshot(rtLayout_->jointTrajectoryStatusBuffer, out);
    }

    void Logic::writeDriveCommands(const merai::DriveCommandData &in)
    {
        const int backIdx = merai::back_index(rtLayout_->driveCommandBuffer);
        auto &dest        = rtLayout_->driveCommandBuffer.buffer[backIdx];

        std::size_t driveCount = paramServerPtr_->driveCount;
        if (driveCount > merai::MAX_SERVO_DRIVES)
        {
            driveCount = merai::MAX_SERVO_DRIVES;
        }

        for (std::size_t i = 0; i < driveCount; ++i)
        {
            dest.commands[i] = in.commands[i];
        }

        merai::publish(rtLayout_->driveCommandBuffer, backIdx);
    }

    void Logic::writeControllerCommand(const merai::ControllerCommand &in)
    {
        const int backIdx = merai::back_index(rtLayout_->controllerCommandBuffer);
        rtLayout_->controllerCommandBuffer.buffer[backIdx] = in;
        merai::publish(rtLayout_->controllerCommandBuffer, backIdx);
    }

    void Logic::writeUserFeedback(merai::AppState currentState)
    {
        const int backIdx = merai::back_index(rtLayout_->userFeedbackBuffer);
        auto &ufbk        = rtLayout_->userFeedbackBuffer.buffer[backIdx];

        ufbk.currentState = currentState;
        merai::publish(rtLayout_->userFeedbackBuffer, backIdx);
    }

    void Logic::writePlannerRequest(const merai::PlannerRequest &in)
    {
        const int backIdx = merai::back_index(rtLayout_->plannerRequestBuffer);
        rtLayout_->plannerRequestBuffer.buffer[backIdx] = in;
        merai::publish(rtLayout_->plannerRequestBuffer, backIdx);
    }

    // -------------------------------------------------------------------------
    // Scheduling helpers (same pattern as Control)
    // -------------------------------------------------------------------------
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
