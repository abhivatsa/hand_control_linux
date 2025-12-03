#include <stdexcept>

#include "control/Control.h"
#include "control/hardware_abstraction/SimHAL.h"
#include "control/hardware_abstraction/RealHAL.h"

// Example controllers
#include "control/controllers/GravityCompController.h"
#include "control/controllers/HomingController.h"

#include "merai/RTIpc.h"

namespace control
{
    //----------------------------------------------------------------------------
    // Constructor / Destructor
    //----------------------------------------------------------------------------
    Control::Control(const std::string &paramServerShmName, std::size_t paramServerShmSize,
                     const std::string &rtDataShmName,      std::size_t rtDataShmSize,
                     const std::string &loggerShmName,      std::size_t loggerShmSize)
        : paramServerShm_(paramServerShmName, paramServerShmSize, true),
          rtDataShm_     (rtDataShmName,      rtDataShmSize,      false),
          loggerShm_     (loggerShmName,      loggerShmSize,      false)
    {
        // 1) ParameterServer mapping
        paramServerPtr_ = reinterpret_cast<const merai::ParameterServer *>(
            paramServerShm_.getPtr());
        if (!paramServerPtr_)
        {
            throw std::runtime_error("[Control] Failed to map ParameterServer memory.");
        }

        // 2) RTMemoryLayout mapping
        rtLayout_ = reinterpret_cast<merai::RTMemoryLayout *>(
            rtDataShm_.getPtr());
        if (!rtLayout_)
        {
            throw std::runtime_error("[Control] Failed to map RTMemoryLayout memory.");
        }
        if (rtLayout_->magic != merai::RT_MEMORY_MAGIC ||
            rtLayout_->version != merai::RT_MEMORY_VERSION)
        {
            throw std::runtime_error("[Control] RTMemoryLayout magic/version mismatch.");
        }

        // 3) Logger memory
        loggerMem_ = reinterpret_cast<merai::multi_ring_logger_memory *>(
            loggerShm_.getPtr());
        if (!loggerMem_)
        {
            throw std::runtime_error("[Control] Failed to map logger shared memory.");
        }

        // 4) Initialize HAL (simulate or real)
        const bool simulateMode = false; // set true if you want a pure sim path
        if (simulateMode)
        {
            hal_ = std::make_unique<SimHAL>(rtLayout_, paramServerPtr_, loggerMem_);
        }
        else
        {
            hal_ = std::make_unique<RealHAL>(rtLayout_, paramServerPtr_, loggerMem_);
        }
    }

    Control::~Control() = default;

    //----------------------------------------------------------------------------
    // Initialization
    //----------------------------------------------------------------------------
    bool Control::init()
    {
        // 1) Load robot model from ParameterServer
        if (!robotModel_.loadFromParameterServer(*paramServerPtr_))
        {
            merai::log_error(loggerMem_, "Control", 205,
                             "[Control] RobotModel load failed");
            return false;
        }

        // 2) Initialize HAL
        if (!hal_ || !hal_->init())
        {
            merai::log_error(loggerMem_, "Control", 206,
                             "[Control] HAL init failed");
            return false;
        }

        // 3) Determine drive count after HAL init
        driveCount_ = static_cast<int>(hal_->getDriveCount());
        if (driveCount_ > merai::MAX_SERVO_DRIVES)
        {
            driveCount_ = merai::MAX_SERVO_DRIVES;
            merai::log_warn(loggerMem_, "Control", 207,
                            "[Control] driveCount > MAX_SERVO_DRIVES; clamped");
        }

        // 4) Initialize the drive manager
        driveStateManager_ = std::make_unique<DriveStateManager>(
            static_cast<std::size_t>(driveCount_),
            loggerMem_);
        if (!driveStateManager_ || !driveStateManager_->init())
        {
            merai::log_error(loggerMem_, "Control", 208,
                             "[Control] DriveStateManager init failed");
            return false;
        }

        // 5) Initialize the controller manager
        controllerManager_ = std::make_unique<ControllerManager>(
            static_cast<std::size_t>(driveCount_),
            loggerMem_);
        if (!controllerManager_)
        {
            merai::log_error(loggerMem_, "Control", 209,
                             "[Control] ControllerManager allocation failed");
            return false;
        }

        // Gravity comp
auto gravityComp = std::make_shared<GravityCompController>(robotModel_, driveCount_);
if (!controllerManager_->registerController(merai::ControllerID::GRAVITY_COMP,
                                            gravityComp,
                                            /*modeHint*/ 10)) // torque mode
    return false;

// Joint trajectory
auto jtCtrl = std::make_shared<JointTrajectoryController>(driveCount_, loggerMem_, rtLayout_);
if (!controllerManager_->registerController(merai::ControllerID::JOINT_TRAJECTORY,
                                            jtCtrl,
                                            /*modeHint*/ 8)) // CSP
    return false;

// Joint jog
auto jogCtrl = std::make_shared<JointJogController>(driveCount_, loggerMem_, rtLayout_);
if (!controllerManager_->registerController(merai::ControllerID::JOINT_JOG,
                                            jogCtrl,
                                            /*modeHint*/ 8)) // CSP
    return false;

        // 6) Register controllers (GravityComp, Homing, etc.)
        auto gravityComp = std::make_shared<GravityCompController>(
            robotModel_, driveCount_);
        if (!controllerManager_->registerController(
                merai::ControllerID::GRAVITY_COMP,
                gravityComp,
                -3))
        {
            merai::log_error(loggerMem_, "Control", 210,
                             "[Control] Failed to register GravityCompController");
            return false;
        }

        double homePositions[7] = {-0.82, 1.336, 0.0, 0.4724, -0.504, 0.0, 0.0};
        auto homingCtrl = std::make_shared<HomingController>(
            homePositions,
            hal_->getDriveCount(),
            loggerMem_);
        if (!controllerManager_->registerController(
                merai::ControllerID::HOMING,
                homingCtrl,
                8))
        {
            merai::log_error(loggerMem_, "Control", 211,
                             "[Control] Failed to register HomingController");
            return false;
        }

        // Fallback: hold position in CSP
ControllerManager::FallbackPolicy fp{};
fp.modeOfOperation = 8;
fp.behavior        = ControllerManager::FallbackPolicy::Behavior::HoldPosition;
fp.torqueLimit     = 0.0;
controllerManager_->setFallbackPolicy(fp);

        ControllerManager::FallbackPolicy fallbackPolicy{};
        fallbackPolicy.modeOfOperation = 8;
        fallbackPolicy.behavior        = ControllerManager::FallbackPolicy::Behavior::HoldPosition;
        controllerManager_->setFallbackPolicy(fallbackPolicy);

        if (!controllerManager_->init())
        {
            merai::log_error(loggerMem_, "Control", 212,
                             "[Control] ControllerManager init failed");
            return false;
        }

        merai::log_info(loggerMem_, "Control", 200,
                        "[Control] init complete");
        return true;
    }

    //----------------------------------------------------------------------------
    // Run / Stop
    //----------------------------------------------------------------------------
    void Control::run()
    {
        merai::log_info(loggerMem_, "Control", 201,
                        "[Control] Starting cyclic task");
        cyclicTask();
        merai::log_info(loggerMem_, "Control", 202,
                        "[Control] Exiting cyclic task");
    }

    void Control::requestStop()
    {
        stopRequested_.store(true, std::memory_order_relaxed);
    }

    //----------------------------------------------------------------------------
    // Main 1 kHz loop
    //----------------------------------------------------------------------------
    void Control::cyclicTask()
    {
        period_info pinfo{};
        periodic_task_init(&pinfo, loopPeriodNs_); // 1 ms

        while (!stopRequested_.load(std::memory_order_relaxed))
        {
            // 1) HAL read → local feedback arrays
            if (!hal_->read())
            {
                merai::log_error(loggerMem_, "Control", 220,
                                 "[Control] HAL read failed; stopping cyclic task");
                halErrorCount_.fetch_add(1, std::memory_order_relaxed);
                break;
            }

            // 2) Gather inputs for this cycle
            ControlCycleInputs in{
                .jointControlFbk = hal_->jointControlFeedback(),
                .jointMotionFbk  = hal_->jointMotionFeedback(),
                .jointIoFbk      = hal_->jointIOFeedback(),
                .driveCmd        = readDriveCommandSnapshot(),
                .ctrlCmd         = readControllerCommandSnapshot(),
            };

            // 3) Prepare outputs: views into HAL command arrays
            ControlCycleOutputs out{
                .jointControlCmd = hal_->jointControlCommand(),
                .jointMotionCmd  = hal_->jointMotionCommand(),
            };

            // Always start with zero commands
            zeroJointCommands(out.jointControlCmd, out.jointMotionCmd);

            // 4) Drive state machine
            driveStateManager_->update(in, out);

            // 5) Controllers
            controllerManager_->update(in, out);

            // 6) Fill controller feedback metadata
            out.ctrlFbk.loopOverrunCount       = 0;
            out.ctrlFbk.halErrorCount          = halErrorCount_.load(std::memory_order_relaxed);
            out.ctrlFbk.loopOverrun            = false;
            out.ctrlFbk.controllerCommandFresh = true;
            out.ctrlFbk.driveCommandFresh      = true;

            // 7) Publish drive & controller feedback to SHM
            writeDriveFeedback(out.driveFbk);
            writeControllerFeedback(out.ctrlFbk);
            hal_->publishJointFeedbackToShm();

            // 8) HAL write → SHM servoRxBuffer → fieldbus
            if (!hal_->write())
            {
                merai::log_error(loggerMem_, "Control", 221,
                                 "[Control] HAL write failed; stopping cyclic task");
                halErrorCount_.fetch_add(1, std::memory_order_relaxed);
                break;
            }

            // 9) Sleep until next period
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
            pinfo->next_period.tv_sec  += 1;
        }
    }

    void Control::wait_rest_of_period(period_info *pinfo)
    {
        inc_period(pinfo);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                        &pinfo->next_period, nullptr);
    }

    //----------------------------------------------------------------------------
    // SHM helpers: commands / feedback
    //----------------------------------------------------------------------------
    void Control::readControllerCommand(merai::ControllerCommand *outCmd)
    {
        merai::read_snapshot(rtLayout_->controllerCommandBuffer, *outCmd);
    }

    void Control::readDriveCommand(merai::DriveCommandData *outDriveCmd)
    {
        merai::read_snapshot(rtLayout_->driveCommandBuffer, *outDriveCmd);
    }

    void Control::writeDriveFeedback(const merai::DriveFeedbackData &feedback)
    {
        int backIdx = merai::back_index(rtLayout_->driveFeedbackBuffer);
        rtLayout_->driveFeedbackBuffer.buffer[backIdx] = feedback;
        merai::publish(rtLayout_->driveFeedbackBuffer, backIdx);
    }

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
        // We only use [0..driveCount_-1]; no need to force-disable others.
        return buf;
    }

    merai::ControllerCommand Control::readControllerCommandSnapshot()
    {
        merai::ControllerCommand cmd{};
        readControllerCommand(&cmd);
        return cmd;
    }

    void Control::zeroJointCommands(std::span<merai::JointControlCommand> jointCtrlCmd,
                                    std::span<merai::JointMotionCommand>  jointMotionCmd) const
    {
        for (auto &c : jointCtrlCmd)
        {
            c = {};
        }
        for (auto &m : jointMotionCmd)
        {
            m = {};
        }
    }

} // namespace control
