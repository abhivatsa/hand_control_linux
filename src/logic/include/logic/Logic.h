#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <time.h>

// merai / Common headers
#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/RAII_SharedMemory.h"
#include "merai/SharedLogger.h"
#include "merai/Enums.h"

// StateMachine
#include "logic/StateMachine.h"

// Safety Manager
#include "logic/SafetyManager.h"

#include "robotics_lib/RobotModel.h"

namespace logic
{
    class Logic
    {
    public:
        Logic(const std::string &paramServerShmName,
              std::size_t        paramServerShmSize,
              const std::string &rtDataShmName,
              std::size_t        rtDataShmSize,
              const std::string &loggerShmName,
              std::size_t        loggerShmSize);

        ~Logic();

        bool init();
        void run();
        void requestStop();

    private:
        void cyclicTask();

        // ---------------------------------------------------
        // SHM bridge I/O
        // ---------------------------------------------------
        void readUserCommands(merai::UserCommands &out);
        void readDriveFeedback(merai::DriveFeedbackData &out);
        void readControllerFeedback(merai::ControllerFeedback &out);
        void readPlannerResult(merai::PlannerResult &out);
        void readTrajectoryStatus(merai::JointTrajectoryStatus &out);

        void writeDriveCommands(const merai::DriveCommandData &in);
        void writeControllerCommand(const merai::ControllerCommand &in);
        void writeUserFeedback(merai::AppState currentState);
        void writePlannerRequest(const merai::PlannerRequest &in);

        // ---------------------------------------------------
        // Periodic scheduling helpers
        // ---------------------------------------------------
        struct period_info
        {
            struct timespec next_period;
            long            period_ns;
        };
        void periodic_task_init(period_info *pinfo, long periodNs);
        void inc_period(period_info *pinfo);
        void wait_rest_of_period(period_info *pinfo);

    private:
        std::atomic<bool> stopRequested_{false};

        // Shared Memory for ParameterServer
        merai::RAII_SharedMemory      paramServerShm_;
        const merai::ParameterServer *paramServerPtr_ = nullptr;

        // Shared Memory for real-time layout
        merai::RAII_SharedMemory rtDataShm_;
        merai::RTMemoryLayout   *rtLayout_ = nullptr;

        // Shared Memory for Logger
        merai::RAII_SharedMemory         loggerShm_;
        merai::multi_ring_logger_memory *loggerMem_ = nullptr;

        // Robot model (for safety/logic if needed)
        robot_lib::RobotModel robotModel_;

        // Managers
        std::unique_ptr<SafetyManager> safetyManager_;
        std::unique_ptr<StateMachine>  stateMachine_;

        // Cached snapshots per loop (optional; not strictly needed now)
        merai::UserCommands          userCmds_{};
        merai::DriveFeedbackData     driveFdbk_{};
        merai::ControllerFeedback    ctrlFdbk_{};
        merai::JointTrajectoryStatus trajStatus_{};

        // Planner / trajectory coordination (placeholder for future job management)
        std::uint32_t nextPlannerJobId_      = 1;
        std::uint32_t lastPlannerJobId_      = 0;
        std::uint32_t activeTrajectoryJobId_ = 0;
    };

} // namespace logic
