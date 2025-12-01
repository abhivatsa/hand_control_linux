#include "control/controllers/GravityCompController.h"
#include <algorithm>
#include "math_lib/Vector.h"

namespace control
{
    namespace
    {
        // CiA‑402 modes (as used by RealHAL)
        constexpr std::uint8_t MODE_CSP = 8;   // Cyclic Synchronous Position
        constexpr std::uint8_t MODE_CST = 10;  // Cyclic Synchronous Torque
    }

    GravityCompController::GravityCompController(const robot_lib::RobotModel &model,
                                                 std::size_t numJoints)
        : numJoints_(numJoints),
          model_(model),
          dynamics_(model)
    {
        // Constructor: just store references, no heavy work.
    }

    bool GravityCompController::init()
    {
        if (numJoints_ == 0)
        {
            // Nothing to control -> fail init so Manager won't use us.
            return false;
        }

        state_ = ControllerState::STOPPED;
        return true;
    }

    bool GravityCompController::start(std::span<const merai::JointMotionFeedback> /*motionFbk*/,
                                      std::span<merai::JointMotionCommand> motionCmd)
    {
        // Only allow start from INACTIVE/STOPPED for now.
        if (state_ != ControllerState::STOPPED &&
            state_ != ControllerState::INACTIVE)
        {
            return false;
        }

        const std::size_t n = std::min(numJoints_, motionCmd.size());
        for (std::size_t i = 0; i < n; ++i)
        {
            // Put drives into torque mode with zero torque at start.
            motionCmd[i].modeOfOperation = MODE_CST;
            motionCmd[i].targetTorque    = 0.0;
            // targetPosition left at whatever zeroJointCommands() set (typically 0).
        }

        state_ = ControllerState::RUNNING;
        return true;
    }

    void GravityCompController::update(std::span<const merai::JointMotionFeedback> motionFbk,
                                       std::span<merai::JointMotionCommand> motionCmd,
                                       double /*dt*/)
    {
        if (state_ != ControllerState::RUNNING)
        {
            return;
        }

        using robot::math::Vector;

        constexpr int MAX_DOF = static_cast<int>(robot_lib::RobotModel::NUM_JOINTS);

        // Effective DOF for this cycle: limited by controller config, spans, and model.
        int dof = static_cast<int>(std::min(numJoints_, motionFbk.size()));
        dof     = std::min(dof, static_cast<int>(motionCmd.size()));
        dof     = std::min(dof, MAX_DOF);

        if (dof <= 0)
        {
            return;
        }

        Vector<7> jointAngles;
        Vector<7> jointVel;
        Vector<7> jointAcc;
        jointAngles.setZero();
        jointVel.setZero();
        jointAcc.setZero(); // pure gravity comp: assume zero acceleration

        // 1) Copy feedback into math vectors
        for (int i = 0; i < dof; ++i)
        {
            const auto &fbk = motionFbk[static_cast<std::size_t>(i)];
            jointAngles[i] = fbk.positionActual;
            jointVel[i]    = fbk.velocityActual;
        }

        // 2) Inverse dynamics (gravity term)
        Vector<7> outTorques;
        outTorques.setZero();

        int ret = dynamics_.computeInverseDynamics(jointAngles, jointVel, jointAcc, outTorques);
        if (ret != 0)
        {
            // On failure: keep drives in torque mode but command zero torque.
            for (int i = 0; i < dof; ++i)
            {
                auto &cmd = motionCmd[static_cast<std::size_t>(i)];
                cmd.modeOfOperation = MODE_CST;
                cmd.targetTorque    = 0.0;
            }
            return;
        }

        // 3) Write torque commands back into motionCmd
        for (int i = 0; i < dof; ++i)
        {
            auto &cmd = motionCmd[static_cast<std::size_t>(i)];
            cmd.modeOfOperation = MODE_CST;
            cmd.targetTorque    = gravityScale_ * outTorques[i];
            // targetPosition remains unused in CST (RealHAL sets servo targetPosition=0 for mode 10).
        }
    }

    void GravityCompController::stop()
    {
        if (state_ == ControllerState::RUNNING)
        {
            state_ = ControllerState::STOPPED;
        }
    }

    void GravityCompController::teardown()
    {
        // Any final cleanup can go here if needed later.
        state_ = ControllerState::INACTIVE;
    }

} // namespace control
