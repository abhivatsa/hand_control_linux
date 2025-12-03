#include "planner/PlannerApp.h"

#include <thread>
#include <chrono>
#include <stdexcept>

#include "merai/RTIpc.h" // read_snapshot, back_index, publish helpers

namespace planner
{
    PlannerApp::PlannerApp(const std::string& paramServerShmName, std::size_t paramServerShmSize,
                           const std::string& rtDataShmName,      std::size_t rtDataShmSize,
                           const std::string& loggerShmName,      std::size_t loggerShmSize)
        : paramServerShm_(paramServerShmName, paramServerShmSize, true),
          rtDataShm_(rtDataShmName, rtDataShmSize, false),
          loggerShm_(loggerShmName, loggerShmSize, false)
    {
        // ParameterServer mapping
        paramServerPtr_ = reinterpret_cast<const merai::ParameterServer*>(
            paramServerShm_.getPtr());
        if (!paramServerPtr_)
        {
            throw std::runtime_error("[PlannerApp] Failed to map ParameterServer memory.");
        }

        // RTMemoryLayout mapping
        rtLayout_ = reinterpret_cast<merai::RTMemoryLayout*>(rtDataShm_.getPtr());
        if (!rtLayout_)
        {
            throw std::runtime_error("[PlannerApp] Failed to map RTMemoryLayout memory.");
        }
        if (rtLayout_->magic != merai::RT_MEMORY_MAGIC ||
            rtLayout_->version != merai::RT_MEMORY_VERSION)
        {
            throw std::runtime_error("[PlannerApp] RTMemoryLayout integrity check failed.");
        }

        // Logger mapping
        loggerMem_ = reinterpret_cast<merai::multi_ring_logger_memory*>(loggerShm_.getPtr());
        if (!loggerMem_)
        {
            throw std::runtime_error("[PlannerApp] Failed to map Logger shared memory.");
        }

        merai::log_info(loggerMem_, "Planner", 5100, "[PlannerApp] SHM mapped");
    }

    bool PlannerApp::init()
    {
        // Load robot model from ParameterServer
        if (!robotModel_.loadFromParameterServer(*paramServerPtr_))
        {
            merai::log_error(loggerMem_, "Planner", 5101, "[PlannerApp] RobotModel load failed");
            return false;
        }

        // Set planner model and dof (use jointCount from ParameterServer)
        const std::size_t jointCount =
            static_cast<std::size_t>(paramServerPtr_->jointCount);
        jointTrajPlanner_.setRobotModel(&robotModel_, jointCount);

        merai::log_info(loggerMem_, "Planner", 5102, "[PlannerApp] init() complete");
        return true;
    }

    void PlannerApp::run()
    {
        merai::log_info(loggerMem_, "Planner", 5103, "[PlannerApp] run() start");

        using namespace std::chrono_literals;

        stopRequested_.store(false, std::memory_order_relaxed);

        while (!stopRequested_.load(std::memory_order_relaxed))
        {
            loopOnce();
            std::this_thread::sleep_for(5ms); // non-RT, modest polling rate
        }

        merai::log_info(loggerMem_, "Planner", 5104, "[PlannerApp] run() exit");
    }

    void PlannerApp::requestStop()
    {
        stopRequested_.store(true, std::memory_order_relaxed);
    }

    void PlannerApp::loopOnce()
    {
        if (!rtLayout_)
        {
            return;
        }

        merai::PlannerRequest req{};
        merai::read_snapshot(rtLayout_->plannerRequestBuffer, req);

        // Only handle new PENDING joint-trajectory jobs
        if (req.status != merai::PlannerJobStatus::PENDING)
        {
            return;
        }

        if (req.job_id == 0 || req.job_id == lastProcessedJobId_)
        {
            // Ignore invalid or duplicate job
            return;
        }

        if (req.type != merai::PlannerJobType::JOINT_TRAJECTORY)
        {
            // Future: could log REJECTED for unsupported type
            merai::PlannerResult res{};
            res.job_id = req.job_id;
            res.status = merai::PlannerJobStatus::REJECTED;

            int resBackIdx = merai::back_index(rtLayout_->plannerResultBuffer);
            rtLayout_->plannerResultBuffer.buffer[resBackIdx] = res;
            merai::publish(rtLayout_->plannerResultBuffer, resBackIdx);

            lastProcessedJobId_ = req.job_id;
            return;
        }

        merai::log_info(loggerMem_, "Planner", 5105, "[PlannerApp] Planning joint trajectory");

        // Plan trajectory
        merai::JointTrajectoryPlan plan{};
        int rc = jointTrajPlanner_.plan(req.jointTraj, plan);

        merai::PlannerResult result{};
        result.job_id = req.job_id;

        if (rc == 0)
        {
            // Fill job_id and publish plan
            plan.job_id = req.job_id;

            int planBackIdx = merai::back_index(rtLayout_->jointTrajectoryPlanBuffer);
            rtLayout_->jointTrajectoryPlanBuffer.buffer[planBackIdx] = plan;
            merai::publish(rtLayout_->jointTrajectoryPlanBuffer, planBackIdx);

            result.status = merai::PlannerJobStatus::DONE;
            merai::log_info(loggerMem_, "Planner", 5106, "[PlannerApp] Planning succeeded");
        }
        else
        {
            result.status = merai::PlannerJobStatus::FAILED;
            merai::log_error(loggerMem_, "Planner", 5107, "[PlannerApp] Planning failed");
        }

        // Publish result
        int resBackIdx = merai::back_index(rtLayout_->plannerResultBuffer);
        rtLayout_->plannerResultBuffer.buffer[resBackIdx] = result;
        merai::publish(rtLayout_->plannerResultBuffer, resBackIdx);

        lastProcessedJobId_ = req.job_id;
    }

} // namespace planner
