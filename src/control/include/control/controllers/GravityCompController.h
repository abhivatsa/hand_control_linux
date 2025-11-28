#pragma once

#include "control/controllers/BaseController.h"
#include "merai/RTMemoryLayout.h" // for JointMotionFeedback / JointMotionCommand
#include "robotics_lib/RobotModel.h"
#include "robotics_lib/RobotDynamics.h"

namespace control
{
    /**
     * @brief GravityCompController
     *        A controller that applies torques based on inverse dynamics
     *        to compensate for gravity (and possibly friction).
     */
    class GravityCompController : public BaseController
    {
    public:
        /**
         * @brief Constructor
         *
         * @param model     Robot model describing kinematics/dynamics.
         * @param numJoints Number of joints this controller operates on.
         */
        GravityCompController(const robot_lib::RobotModel &model,
                              std::size_t numJoints);

        /**
         * @brief init
         *   - Checks pointers, sets internal state to INIT.
         */
        bool init() override;

        /**
         * @brief start
         *   - Transitions from INIT or STOPPED to RUNNING.
         */
        bool start(std::span<const merai::JointMotionFeedback> motionFbk,
                   std::span<merai::JointMotionCommand> motionCmd) override;

        /**
         * @brief update
         *   - Called each control cycle (e.g. 1kHz).
         *   - Reads joint positions/velocities from feedbackPtr_,
         *     writes torque commands to commandPtr_.
         *   - Uses inverse dynamics for gravity compensation.
         *
         * @param dt  Timestep in seconds, e.g. 0.001 for 1kHz.
         */
        void update(std::span<const merai::JointMotionFeedback> motionFbk,
                    std::span<merai::JointMotionCommand> motionCmd,
                    double dt) override;

        /**
         * @brief stop
         *   - Transitions from RUNNING to STOPPED.
         */
        void stop() override;

        /**
         * @brief teardown
         *   - Final cleanup; sets state to UNINIT.
         */
        void teardown() override;

    private:
        std::size_t numJoints_ = 0;

        const robot_lib::RobotModel &model_;
        robot_lib::RobotDynamics dynamics_;

        // Optional parameter
        double gravityScale_ = 1.0; // scale factor for torque compensation
    };

} // namespace control
