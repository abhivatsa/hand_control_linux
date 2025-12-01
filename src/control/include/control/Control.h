#pragma once

#include <memory>
#include <atomic>
#include <ctime>   // timespec, clock_gettime
#include <span>

#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/SharedLogger.h"
#include "merai/RAII_SharedMemory.h"
#include "merai/Enums.h"

#include "control/hardware_abstraction/BaseHAL.h"
#include "control/ControlData.h"
#include "control/DriveStateManager.h"
#include "control/ControllerManager.h"
#include "robotics_lib/RobotModel.h"

namespace control
{
    class Control
    {
    public:
        Control(const std::string &paramServerShmName, std::size_t paramServerShmSize,
                const std::string &rtDataShmName,      std::size_t rtDataShmSize,
                const std::string &loggerShmName,      std::size_t loggerShmSize);

        ~Control();

        bool init();
        void run();
        void requestStop();

    private:
        // Main periodic loop
        void cyclicTask();

        // Periodic task helper struct and methods
        struct period_info
        {
            struct timespec next_period;
            long period_ns;
        };

        void periodic_task_init(period_info *pinfo, long period_ns);
        void inc_period(period_info *pinfo);
        void wait_rest_of_period(period_info *pinfo);

        // Reading/writing additional commands/feedback
        void readControllerCommand(merai::ControllerCommand *outCmd);
        void readDriveCommand(merai::DriveCommandData *outDriveCmd);
        void writeControllerFeedback(const merai::ControllerFeedback &feedback);
        void writeDriveFeedback(const merai::DriveFeedbackData &feedback);

        merai::DriveCommandData  readDriveCommandSnapshot();
        merai::ControllerCommand readControllerCommandSnapshot();

        void zeroJointCommands(std::span<merai::JointControlCommand> jointCtrlCmd,
                               std::span<merai::JointMotionCommand>  jointMotionCmd) const;

    private:
        // HAL and managers
        std::unique_ptr<BaseHAL>          hal_;
        std::unique_ptr<ControllerManager> controllerManager_;
        std::unique_ptr<DriveStateManager> driveStateManager_;
        int                               driveCount_ = 0;

        std::atomic_bool    stopRequested_{false};
        std::atomic<uint64_t> halErrorCount_{0};

        static constexpr long loopPeriodNs_ = 1'000'000L; // 1 ms control loop

        // Shared memory & param server references
        merai::RAII_SharedMemory        paramServerShm_;
        const merai::ParameterServer   *paramServerPtr_ = nullptr;

        merai::RAII_SharedMemory        rtDataShm_;
        merai::RTMemoryLayout          *rtLayout_       = nullptr;

        merai::RAII_SharedMemory        loggerShm_;
        merai::multi_ring_logger_memory *loggerMem_     = nullptr;

        // Robot model (used by controllers such as GravityCompController)
        robot_lib::RobotModel           robotModel_;
    };

} // namespace control
