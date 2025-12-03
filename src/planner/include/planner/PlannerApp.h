#pragma once

#include <string>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/SharedLogger.h"
#include "merai/RAII_SharedMemory.h"

#include "robotics_lib/RobotModel.h"
#include "planner/JointTrajectoryPlanner.h"

namespace planner
{
    /**
     * @brief PlannerApp
     *
     * Non-RT process that:
     *  - attaches to /ParameterServerShm, /RTDataShm, /LoggerShm
     *  - builds RobotModel from ParameterServer
     *  - polls plannerRequestBuffer for PENDING jobs
     *  - runs JointTrajectoryPlanner
     *  - writes JointTrajectoryPlan + PlannerResult back to SHM
     */
    class PlannerApp
    {
    public:
        PlannerApp(const std::string& paramServerShmName, std::size_t paramServerShmSize,
                   const std::string& rtDataShmName,      std::size_t rtDataShmSize,
                   const std::string& loggerShmName,      std::size_t loggerShmSize);

        ~PlannerApp() = default;

        bool init();
        void run();
        void requestStop();

    private:
        void loopOnce();

        // SHM handles
        merai::RAII_SharedMemory         paramServerShm_;
        const merai::ParameterServer*    paramServerPtr_ = nullptr;

        merai::RAII_SharedMemory         rtDataShm_;
        merai::RTMemoryLayout*           rtLayout_       = nullptr;

        merai::RAII_SharedMemory         loggerShm_;
        merai::multi_ring_logger_memory* loggerMem_      = nullptr;

        // Robot model used by planners
        robot_lib::RobotModel            robotModel_;

        // Planners
        JointTrajectoryPlanner           jointTrajPlanner_;

        std::atomic<bool>                stopRequested_{false};
        std::uint32_t                    lastProcessedJobId_{0};
    };

} // namespace planner
