#pragma once

#include "control/controllers/BaseController.h"
#include "merai/RTMemoryLayout.h"      // JointMotionFeedback / JointMotionCommand
#include "robotics_lib/RobotModel.h"
#include "robotics_lib/RobotDynamics.h"

namespace control
{
    /**
     * @brief GravityCompController
     *        A controller that applies torques based on inverse dynamics
     *        to compensate for gravity.
     */
    class GravityCompController : public BaseController
    {
    public:
        /// @param model     Robot model describing kinematics/dynamics.
        /// @param numJoints Number of joints this controller operates on.
        GravityCompController(const robot_lib::RobotModel &model,
                              std::size_t numJoints);

        /// One‑time init (parameter sanity checks etc.)
        bool init() override;

        /// Called when switching to this controller.
        bool start(std::span<const merai::JointMotionFeedback> motionFbk,
                   std::span<merai::JointMotionCommand> motionCmd) override;

        /// Called every control cycle (1 kHz).
        void update(std::span<const merai::JointMotionFeedback> motionFbk,
                    std::span<merai::JointMotionCommand> motionCmd,
                    double dt) override;

        /// Request controller to stop; transitions RUNNING -> STOPPED.
        void stop() override;

        /// Final cleanup.
        void teardown() override;

    private:
        std::size_t numJoints_ = 0;

        const robot_lib::RobotModel &model_;
        robot_lib::RobotDynamics    dynamics_;

        // Scale factor applied to computed gravity torques (usually 1.0).
        double gravityScale_ = 1.0;
    };

} // namespace control
