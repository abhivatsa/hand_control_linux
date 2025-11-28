#include <stdexcept>
#include <ctime>
#include <cstdint>
#include <chrono>

#include "control/Control.h"
#include "control/hardware_abstraction/SimHAL.h"
#include "control/hardware_abstraction/RealHAL.h"

// Example controllers
#include "control/controllers/GravityCompController.h"
#include "control/controllers/HomingController.h"

#include "merai/Enums.h" // for e.g. DriveCommand, ControllerID
#include "merai/RTIpc.h"

namespace
{
    inline uint64_t timespecToNs(const timespec &ts)
    {
        return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL +
               static_cast<uint64_t>(ts.tv_nsec);
    }
}

namespace control
{
    Control::Control(const std::string &paramServerShmName,
                     size_t paramServerShmSize,
                     const std::string &rtDataShmName,
                     size_t rtDataShmSize,
                     const std::string &loggerShmName,
                     size_t loggerShmSize)
        : paramServerShm_(paramServerShmName, paramServerShmSize, true),
          rtDataShm_(rtDataShmName, rtDataShmSize, false),
          loggerShm_(loggerShmName, loggerShmSize, false)
    {
        // 1) ParameterServer mapping
        paramServerPtr_ = reinterpret_cast<const merai::ParameterServer *>(
            paramServerShm_.getPtr());
        if (!paramServerPtr_)
        {
            throw std::runtime_error("Failed to map ParameterServer memory.");
        }

        // 2) RTMemoryLayout mapping
        rtLayout_ = reinterpret_cast<merai::RTMemoryLayout *>(
            rtDataShm_.getPtr());
        if (!rtLayout_)
        {
            throw std::runtime_error("Failed to map RTMemoryLayout memory.");
        }
        if (rtLayout_->magic != merai::RT_MEMORY_MAGIC || rtLayout_->version != merai::RT_MEMORY_VERSION)
        {
            throw std::runtime_error("RTMemoryLayout integrity check failed (magic/version mismatch).");
        }

        // 3) Logger memory
        loggerMem_ = reinterpret_cast<merai::multi_ring_logger_memory *>(
            loggerShm_.getPtr());
        if (!loggerMem_)
        {
            throw std::runtime_error("Failed to map multi_ring_logger_memory.");
        }

        // 4) Initialize HAL (simulate or real)
        // TODO: Wire simulate/real selection from GUI/runtime input instead of config.
        const bool simulateMode = false;
        if (simulateMode)
        {
            hal_ = std::make_unique<SimHAL>(rtLayout_, paramServerPtr_, loggerMem_);
        }
        else
        {
            hal_ = std::make_unique<RealHAL>(rtLayout_, paramServerPtr_, loggerMem_);
        }
    }

    Control::~Control()
    {
        // Cleanup if needed
    }

    bool Control::init()
    {
        // 1) Load robot model from ParameterServer
        if (!robotModel_.loadFromParameterServer(*paramServerPtr_))
        {
            merai::log_error(loggerMem_, "Control", 205, "[Control] RobotModel load failed");
            return false;
        }

        // 2) Initialize HAL
        if (!hal_->init())
            return false;

        // Determine drive count after HAL init (SimHAL sets it here)
        driveCount_ = static_cast<int>(hal_->getDriveCount());
        if (driveCount_ > merai::MAX_SERVO_DRIVES)
        {
            driveCount_ = merai::MAX_SERVO_DRIVES;
        }

        // 3) Initialize the drive manager & controller manager
        driveStateManager_ = std::make_unique<DriveStateManager>(
            driveCount_,
            loggerMem_);
        if (!driveStateManager_->init())
            return false;

        controllerManager_ = std::make_unique<ControllerManager>(
            paramServerPtr_,
            driveCount_,
            loggerMem_,
            static_cast<double>(loopPeriodNs_) / 1'000'000'000.0);

        // 4) Register controllers (GravityComp, Homing, etc.)
        auto gravityComp = std::make_shared<GravityCompController>(robotModel_, driveCount_);
        if (!controllerManager_->registerController(merai::ControllerID::GRAVITY_COMP, gravityComp, -3))
            return false;

        // Example for Homing
        double homePositions[7] = {-0.82, 1.336, 0.0, 0.4724, -0.504, 0.0, 0.0};
        auto homingCtrl = std::make_shared<HomingController>(
            homePositions,
            hal_->getDriveCount(),
            loggerMem_);
        if (!controllerManager_->registerController(merai::ControllerID::HOMING, homingCtrl, 8))
            return false;

        ControllerManager::FallbackPolicy fallbackPolicy{};
        fallbackPolicy.modeOfOperation = 8;
        fallbackPolicy.behavior = ControllerManager::FallbackPolicy::Behavior::HoldPosition;
        controllerManager_->setFallbackPolicy(fallbackPolicy);

        ControllerManager::ModeCompatibilityPolicy modePolicy{};
        modePolicy.allowedModes = {8, 10, -3};
        controllerManager_->setModeCompatibilityPolicy(modePolicy);

        if (!controllerManager_->init())
            return false;

        merai::log_info(loggerMem_, "Control", 200, "[Control] init complete");
        return true;
    }

    void Control::run()
    {
        merai::log_info(loggerMem_, "Control", 201, "[Control] Starting cyclic task");
        cyclicTask();
        merai::log_info(loggerMem_, "Control", 202, "[Control] Exiting cyclic task");
    }

    void Control::requestStop()
    {
        stopRequested_.store(true, std::memory_order_relaxed);
    }

    //----------------------------------------------------------------------------
    // The main real-time loophal_->getDriveCount()
    //----------------------------------------------------------------------------
    void Control::cyclicTask()
    {
        period_info pinfo;
        periodic_task_init(&pinfo, loopPeriodNs_); // e.g., 1 ms

        while (!stopRequested_.load(std::memory_order_relaxed))
        {
            timespec loopStart{};
            clock_gettime(CLOCK_MONOTONIC, &loopStart);
            const uint64_t loopStartNs = timespecToNs(loopStart);
            if (lastLoopStartNs_ != 0)
            {
                const uint64_t delta = loopStartNs - lastLoopStartNs_;
                if (delta > static_cast<uint64_t>(loopPeriodNs_) + overrunSlackNs_)
                {
                    loopOverrunCount_.fetch_add(1, std::memory_order_relaxed);
                    loopOverrunFlag_.store(true, std::memory_order_relaxed);
                }
                else
                {
                    loopOverrunFlag_.store(false, std::memory_order_relaxed);
                }
            }
            lastLoopStartNs_ = loopStartNs;

            // PHASE 0: bookkeeping
            auto cycleId = cycleId_.fetch_add(1, std::memory_order_relaxed) + 1;
            auto startTime = getMonotonicTime();

            // PHASE 1: gather inputs
            if (!hal_->read())
            {
                merai::log_error(loggerMem_, "Control", 220, "[Control] HAL read failed; stopping cyclic task");
                halErrorCount_.fetch_add(1, std::memory_order_relaxed);
                break;
            }

            ControlCycleInputs in{
                .jointControlFbk = hal_->jointControlFeedback(),
                .jointMotionFbk = hal_->jointMotionFeedback(),
                .jointIoFbk = hal_->jointIOFeedback(),
                .driveCmd = readDriveCommandSnapshot(),
                .ctrlCmd = readControllerCommandSnapshot(),
                .cycleId = cycleId,
                .timestamp = startTime,
            };

            ControlCycleOutputs out{
                .jointControlCmd = hal_->jointControlCommand(),
                .jointMotionCmd = hal_->jointMotionCommand(),
            };

            zeroJointCommands(out.jointControlCmd, out.jointMotionCmd);

            // PHASE 2: drive state machine
            driveStateManager_->update(in, out);

            // PHASE 3: controllers
            controllerManager_->update(in, out);

            // PHASE 5: publish feedback to shared memory
            out.ctrlFbk.loopOverrunCount = loopOverrunCount_.load(std::memory_order_relaxed);
            out.ctrlFbk.halErrorCount = halErrorCount_.load(std::memory_order_relaxed);
            out.ctrlFbk.loopOverrun = loopOverrunFlag_.load(std::memory_order_relaxed);
            out.ctrlFbk.controllerCommandFresh = true;
            out.ctrlFbk.driveCommandFresh = true;

            writeDriveFeedback(out.driveFbk);
            writeControllerFeedback(out.ctrlFbk);
            hal_->publishJointFeedbackToShm();

            // PHASE 6: send commands
            if (!hal_->write())
            {
                merai::log_error(loggerMem_, "Control", 221, "[Control] HAL write failed; stopping cyclic task");
                halErrorCount_.fetch_add(1, std::memory_order_relaxed);
                break;
            }

            // Sleep until next period
            wait_rest_of_period(&pinfo);
        }
    }

    //----------------------------------------------------------------------------
    // Periodic helpers
    //----------------------------------------------------------------------------
    void Control::periodic_task_init(period_info *pinfo, long periodNs)
    {
        clock_gettime(CLOCK_MONOTONIC, &pinfo->next_period);
        pinfo->period_ns = periodNs;
    }

    void Control::inc_period(period_info *pinfo)
    {
        pinfo->next_period.tv_nsec += pinfo->period_ns;
        if (pinfo->next_period.tv_nsec >= 1'000'000'000L)
        {
            pinfo->next_period.tv_nsec -= 1'000'000'000L;
            pinfo->next_period.tv_sec++;
        }
    }

    void Control::wait_rest_of_period(period_info *pinfo)
    {
        inc_period(pinfo);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, nullptr);
    }

    //----------------------------------------------------------------------------
    // Read ControllerCommand from shared memory
    //----------------------------------------------------------------------------
    void Control::readControllerCommand(merai::ControllerCommand *outCmd)
    {
        merai::read_snapshot(rtLayout_->controllerCommandBuffer, *outCmd);
    }

    // Read DriveCommandData from shared memory
    void Control::readDriveCommand(merai::DriveCommandData *outDriveCmd)
    {
        merai::read_snapshot(rtLayout_->driveCommandBuffer, *outDriveCmd);
    }

    //----------------------------------------------------------------------------
    // Write DriveFeedbackData to shared memory
    //----------------------------------------------------------------------------
    void Control::writeDriveFeedback(const merai::DriveFeedbackData &feedback)
    {
        int backIdx = merai::back_index(rtLayout_->driveFeedbackBuffer);
        rtLayout_->driveFeedbackBuffer.buffer[backIdx] = feedback;
        merai::publish(rtLayout_->driveFeedbackBuffer, backIdx);
    }

    // Write ControllerFeedback to shared memory
    void Control::writeControllerFeedback(const merai::ControllerFeedback &feedback)
    {
        int backIdx = merai::back_index(rtLayout_->controllerFeedbackBuffer);

        rtLayout_->controllerFeedbackBuffer.buffer[backIdx] = feedback;
        merai::publish(rtLayout_->controllerFeedbackBuffer, backIdx);
    }

    merai::DriveCommandData Control::readDriveCommandSnapshot()
    {
        merai::DriveCommandData buf{};
        readDriveCommand(&buf);
        forceDisableUnusedSlots(buf);
        return buf;
    }

    merai::ControllerCommand Control::readControllerCommandSnapshot()
    {
        merai::ControllerCommand cmd{};
        readControllerCommand(&cmd);
        return cmd;
    }

    void Control::forceDisableUnusedSlots(merai::DriveCommandData &cmds) const
    {
        for (int i = driveCount_; i < merai::MAX_SERVO_DRIVES; ++i)
        {
            cmds.commands[static_cast<std::size_t>(i)] = merai::DriveCommand::FORCE_DISABLE;
        }
    }

    void Control::zeroJointCommands(std::span<merai::JointControlCommand> jointCtrlCmd,
                                    std::span<merai::JointMotionCommand> jointMotionCmd) const
    {
        for (std::size_t i = 0; i < jointCtrlCmd.size(); ++i)
        {
            jointCtrlCmd[i] = {};
        }
        for (std::size_t i = 0; i < jointMotionCmd.size(); ++i)
        {
            jointMotionCmd[i] = {};
        }
    }

    TimePoint Control::getMonotonicTime() const
    {
        return std::chrono::steady_clock::now();
    }

} // namespace control
