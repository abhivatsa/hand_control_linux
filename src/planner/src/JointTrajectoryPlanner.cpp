#include "planner/JointTrajectoryPlanner.h"

#include <algorithm>
#include <cmath>

namespace planner
{
    int JointTrajectoryPlanner::plan(const merai::JointTrajectoryRequestPayload& req,
                                     merai::JointTrajectoryPlan& outPlan) const
    {
        // Determine DOF: clamp to what the system actually uses.
        const std::size_t dof =
            std::min<std::size_t>(jointCount_, merai::MAX_SERVO_DRIVES);

        if (dof == 0)
        {
            // No joints to plan for
            return -1;
        }

        if (req.num_waypoints < 2 ||
            req.num_waypoints > merai::MAX_WAYPOINTS)
        {
            return -2;
        }

        const std::size_t N = static_cast<std::size_t>(req.num_waypoints);

        if (N > merai::MAX_TRAJECTORY_POINTS)
        {
            // Too many points for plan buffer; clamp or fail
            return -3;
        }

        outPlan.point_count = static_cast<std::uint32_t>(N);

        // Simple scheme:
        // - each waypoint becomes one trajectory point
        // - time between waypoints is based on max joint velocity
        // - velocities and accelerations in the plan are set to 0
        double t = 0.0;

        for (std::size_t k = 0; k < N; ++k)
        {
            auto& pt = outPlan.points[k];
            pt.time  = t;

            // Copy positions; zero velocities/accelerations
            for (std::size_t j = 0; j < dof; ++j)
            {
                pt.q[j]   = req.q[k][j];
                pt.qd[j]  = 0.0;
                pt.qdd[j] = 0.0;
            }

            // Compute time to next waypoint
            if (k + 1 < N)
            {
                double dt = 0.0;

                for (std::size_t j = 0; j < dof; ++j)
                {
                    const double q0 = req.q[k][j];
                    const double q1 = req.q[k + 1][j];
                    const double dq = std::fabs(q1 - q0);

                    // Use either request maxVel or a small default to avoid division by zero
                    double vmax = req.maxVel[j];
                    if (vmax < 1e-3)
                    {
                        vmax = 1e-3;
                    }

                    // Time needed for this joint if we just move at vmax
                    const double joint_dt = dq / vmax;
                    if (joint_dt > dt)
                    {
                        dt = joint_dt;
                    }
                }

                // Enforce a minimum segment duration for numerical robustness
                if (dt < 0.01) // 10 ms
                {
                    dt = 0.01;
                }

                t += dt;
            }
        }

        return 0;
    }

} // namespace planner
