#include "control/controllers/GravityCompController.h"
#include <algorithm>
#include "math_lib/Vector.h"

namespace control
{
    GravityCompController::GravityCompController(const robot_lib::RobotModel &model,
                                                 std::size_t numJoints)
        : numJoints_(numJoints),
          model_(model),
          dynamics_(model)
    {
        // Constructor only stores references and parameters. No heavy logic here.
    }

    bool GravityCompController::init()
    {
        // Validate parameters
        if (numJoints_ == 0)
        {
            return false; // or handle error
        }

        state_ = ControllerState::INIT;
        return true;
    }

    bool GravityCompController::start(std::span<const merai::JointMotionFeedback> /*motionFbk*/,
                                      std::span<merai::JointMotionCommand> motionCmd)
    {
        // Transition to RUNNING if allowed
        if (state_ == ControllerState::INIT || state_ == ControllerState::STOPPED)
        {
            state_ = ControllerState::RUNNING;
        }
        else
        {
            return false;
        }

        for (int i = 0; i < static_cast<int>(numJoints_) && i < static_cast<int>(motionCmd.size()); ++i)
        {
            motionCmd[static_cast<std::size_t>(i)].modeOfOperation = -3;
        }
        return true;
    }

    void GravityCompController::update(std::span<const merai::JointMotionFeedback> motionFbk,
                                       std::span<merai::JointMotionCommand> motionCmd,
                                       double dt)
    {
        // Only compute torques if RUNNING
        if (state_ != ControllerState::RUNNING)
        {
            return;
        }

        // Use the robot dynamics model for gravity compensation.
        using robot::math::Vector;

        const int maxDof = static_cast<int>(robot_lib::RobotModel::NUM_JOINTS);
        int dof = static_cast<int>(std::min<std::size_t>(numJoints_, motionFbk.size()));
        dof = std::min(dof, maxDof);

        Vector<7> jointAngles;
        Vector<7> jointVel;
        Vector<7> jointAcc;
        jointAngles.setZero();
        jointVel.setZero();
        jointAcc.setZero(); // for pure gravity comp, assume zero acceleration

        // 1) Read joint positions & velocities into your math vectors
        for (int i = 0; i < dof; ++i)
        {
            jointAngles[i] = motionFbk[static_cast<std::size_t>(i)].positionActual;
            jointVel[i] = motionFbk[static_cast<std::size_t>(i)].velocityActual;
        }

        // 2) Compute inverse dynamics
        Vector<7> outTorques;
        outTorques.setZero();

        int ret = dynamics_.computeInverseDynamics(jointAngles, jointVel, jointAcc, outTorques);
        if (ret != 0)
        {
            // fallback: zero torques if there's an error code
            for (int i = 0; i < dof; ++i)
            {
                if (i < static_cast<int>(motionCmd.size()))
                {
                    motionCmd[static_cast<std::size_t>(i)].targetTorque = 0.0;
                }
            }
            return;
        }

        // 3) Write torques to the command array
        for (int i = 0; i < dof; ++i)
        {
            if (i < static_cast<int>(motionCmd.size()))
            {
                motionCmd[static_cast<std::size_t>(i)].targetTorque = gravityScale_ * outTorques[i];
            }
        }
    }

    void GravityCompController::stop()
    {
        // Transition to STOPPED if running
        if (state_ == ControllerState::RUNNING)
        {
            state_ = ControllerState::STOPPED;
        }
    }

    void GravityCompController::teardown()
    {
        // Cleanup if needed
        state_ = ControllerState::UNINIT;
    }

} // namespace control
