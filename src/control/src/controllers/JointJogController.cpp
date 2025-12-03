#include "control/controllers/JointJogController.h"

#include <algorithm>
#include <cmath>

namespace control
{
    JointJogController::JointJogController(
        merai::RTMemoryLayout*           rtLayout,
        const robot_lib::RobotModel&     model,
        std::size_t                      jointCount,
        merai::multi_ring_logger_memory* loggerMem)
        : rtLayout_(rtLayout),
          model_(model),
          loggerMem_(loggerMem),
          jointCount_(jointCount)
    {
    }

    bool JointJogController::init()
    {
        if (!rtLayout_ || jointCount_ == 0)
        {
            if (loggerMem_)
            {
                merai::log_error(loggerMem_, "Control", 6100,
                                 "[JointJogController] Invalid rtLayout or jointCount");
            }
            state_ = ControllerState::ERROR;
            return false;
        }

        state_ = ControllerState::STOPPED;
        return true;
    }

    bool JointJogController::start(
        std::span<const merai::JointMotionFeedback> motionFbk,
        std::span<merai::JointMotionCommand>        motionCmd)
    {
        const std::size_t dof =
            std::min<std::size_t>({ jointCount_, motionCmd.size(),
                                    motionFbk.size(),
                                    static_cast<std::size_t>(merai::MAX_SERVO_DRIVES) });

        // Initialize target positions to current positions
        for (std::size_t i = 0; i < dof; ++i)
        {
            motionCmd[i].modeOfOperation = 8; // CSP
            motionCmd[i].targetPosition  = motionFbk[i].positionActual;
            motionCmd[i].targetTorque    = 0.0;
        }

        state_ = ControllerState::RUNNING;

        if (loggerMem_)
        {
            merai::log_info(loggerMem_, "Control", 6101,
                            "[JointJogController] Started");
        }

        return true;
    }

    void JointJogController::update(
        std::span<const merai::JointMotionFeedback> motionFbk,
        std::span<merai::JointMotionCommand>        motionCmd,
        double                                      dt)
    {
        if (state_ != ControllerState::RUNNING)
        {
            return;
        }

        if (!rtLayout_)
        {
            return;
        }

        merai::JointJogCommand cmd{};
        merai::read_snapshot(rtLayout_->jointJogCommandBuffer, cmd);
        lastCmd_ = cmd; // keep last for possible debug

        const std::size_t dof =
            std::min<std::size_t>({ jointCount_, motionCmd.size(),
                                    motionFbk.size(),
                                    static_cast<std::size_t>(merai::MAX_SERVO_DRIVES) });

        for (std::size_t i = 0; i < dof; ++i)
        {
            motionCmd[i].modeOfOperation = 8; // CSP

            // If jog not active on this joint: hold current position
            if (!cmd.active[i])
            {
                motionCmd[i].targetPosition = motionFbk[i].positionActual;
                motionCmd[i].targetTorque   = 0.0;
                continue;
            }

            double vCmd  = cmd.velCmd[i];
            double vMax  = std::max(cmd.maxVel[i], 1e-6);
            vCmd         = std::clamp(vCmd, -vMax, vMax);

            // Simple velocity integration
            double qNew = motionFbk[i].positionActual + vCmd * dt;

            // Clamp to joint limits
            qNew = clampToLimits(qNew, i);

            motionCmd[i].targetPosition = qNew;
            motionCmd[i].targetTorque   = 0.0;
        }
    }

    void JointJogController::stop()
    {
        if (state_ == ControllerState::RUNNING)
        {
            state_ = ControllerState::STOPPED;

            if (loggerMem_)
            {
                merai::log_info(loggerMem_, "Control", 6102,
                                "[JointJogController] Stopped");
            }
        }
    }

    void JointJogController::teardown()
    {
        state_ = ControllerState::INACTIVE;
    }

    double JointJogController::clampToLimits(double q, std::size_t jointIndex) const
    {
        if (jointIndex >= robot_lib::RobotModel::NUM_JOINTS)
        {
            return q;
        }

        const double qMin =
            model_.getJointMinPosition(static_cast<int>(jointIndex));
        const double qMax =
            model_.getJointMaxPosition(static_cast<int>(jointIndex));

        return std::clamp(q, qMin, qMax);
    }

} // namespace control
