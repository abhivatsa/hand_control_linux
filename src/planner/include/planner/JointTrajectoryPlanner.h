#pragma once

#include <cstddef>

#include "merai/RTMemoryLayout.h"     // JointTrajectoryRequestPayload, JointTrajectoryPlan, MAX_SERVO_DRIVES
#include "robotics_lib/RobotModel.h"  // RobotModel

namespace planner
{
    /**
     * @brief Simple joint trajectory planner.
     *
     * For now:
     *  - uses only joint positions and maxVel from the request
     *  - generates one trajectory point per waypoint
     *  - interpolates time per segment based on max joint velocity
     *  - sets velocities / accelerations in the plan to zero
     *
     * This is intentionally minimal and deterministic; you can
     * refine the time-parameterization later without changing the API.
     */
    class JointTrajectoryPlanner
    {
    public:
        JointTrajectoryPlanner() = default;

        JointTrajectoryPlanner(const robot_lib::RobotModel* model,
                               std::size_t jointCount)
            : model_(model),
              jointCount_(jointCount)
        {
        }

        void setRobotModel(const robot_lib::RobotModel* model,
                           std::size_t jointCount)
        {
            model_      = model;
            jointCount_ = jointCount;
        }

        /**
         * @brief Plan a joint trajectory.
         *
         * @param req      Input waypoints + limits from SHM.
         * @param outPlan  Filled on success. Caller sets job_id separately.
         * @return 0 on success, non-zero error code on failure.
         */
        int plan(const merai::JointTrajectoryRequestPayload& req,
                 merai::JointTrajectoryPlan& outPlan) const;

    private:
        const robot_lib::RobotModel* model_      = nullptr; // not used yet, kept for future limits
        std::size_t                  jointCount_ = 0;
    };

} // namespace planner
