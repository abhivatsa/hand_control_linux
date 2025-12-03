#include "control/controllers/JointTrajectoryController.h"

#include <algorithm>
#include <cmath>

namespace control
{
    JointTrajectoryController::JointTrajectoryController(
        merai::RTMemoryLayout*           rtLayout,
        std::size_t                      jointCount,
        merai::multi_ring_logger_memory* loggerMem)
        : rtLayout_(rtLayout),
          loggerMem_(loggerMem),
          jointCount_(jointCount)
    {
    }

    bool JointTrajectoryController::init()
    {
        if (!rtLayout_ || jointCount_ == 0)
        {
            if (loggerMem_)
            {
                merai::log_error(loggerMem_, "Control", 6000,
                                 "[JointTrajectoryController] Invalid rtLayout or jointCount");
            }
            state_ = ControllerState::ERROR;
            return false;
        }

        state_ = ControllerState::STOPPED;
        return true;
    }

    bool JointTrajectoryController::start(
        std::span<const merai::JointMotionFeedback> /*motionFbk*/,
        std::span<merai::JointMotionCommand>        motionCmd)
    {
        if (!rtLayout_)
        {
            return false;
        }

        // Snapshot latest plan from SHM
        merai::read_snapshot(rtLayout_->jointTrajectoryPlanBuffer, plan_);

        if (plan_.numPoints == 0)
        {
            if (loggerMem_)
            {
                merai::log_error(loggerMem_, "Control", 6001,
                                 "[JointTrajectoryController] Plan has zero points");
            }
            state_  = ControllerState::ERROR;
            hasPlan_ = false;
            return false;
        }

        activeJobId_   = plan_.jobId;
        currentTime_   = 0.0;
        currentIndex_  = 0;
        hasPlan_       = true;
        finishedOnce_  = false;

        const std::size_t dof =
            std::min<std::size_t>({ jointCount_, motionCmd.size(),
                                    static_cast<std::size_t>(merai::MAX_SERVO_DRIVES) });

        // Initialize commands to first point in plan
        for (std::size_t j = 0; j < dof; ++j)
        {
            motionCmd[j].modeOfOperation = 8; // CSP
            motionCmd[j].targetPosition  = plan_.points[0].q[j];
            motionCmd[j].targetTorque    = 0.0;
        }

        state_ = ControllerState::RUNNING;
        publishStatus(0.0, true, false, false);

        if (loggerMem_)
        {
            merai::log_info(loggerMem_, "Control", 6002,
                            "[JointTrajectoryController] Started");
        }

        return true;
    }

    void JointTrajectoryController::update(
        std::span<const merai::JointMotionFeedback> /*motionFbk*/,
        std::span<merai::JointMotionCommand>        motionCmd,
        double                                      dt)
    {
        if (state_ != ControllerState::RUNNING || !hasPlan_)
        {
            return;
        }

        const std::uint32_t N = plan_.numPoints;
        if (N == 0)
        {
            state_  = ControllerState::STOPPED;
            hasPlan_ = false;
            publishStatus(1.0, false, true, false);
            return;
        }

        const std::size_t dof =
            std::min<std::size_t>({ jointCount_, motionCmd.size(),
                                    static_cast<std::size_t>(merai::MAX_SERVO_DRIVES) });

        currentTime_ += dt;

        // Advance currentIndex_ while we've passed the next point's time
        while ((currentIndex_ + 1u) < N &&
               currentTime_ > plan_.points[currentIndex_ + 1u].t)
        {
            ++currentIndex_;
        }

        if (currentIndex_ + 1u >= N)
        {
            // End of trajectory: hold final position & mark finished once
            holdFinalPosition(motionCmd);

            if (!finishedOnce_)
            {
                finishedOnce_ = true;
                state_        = ControllerState::STOPPED;
                publishStatus(1.0, false, true, false);

                if (loggerMem_)
                {
                    merai::log_info(loggerMem_, "Control", 6003,
                                    "[JointTrajectoryController] Finished");
                }
            }
            return;
        }

        const auto& p0 = plan_.points[currentIndex_];
        const auto& p1 = plan_.points[currentIndex_ + 1u];

        const double t0    = p0.t;
        const double t1    = p1.t;
        const double dtSeg = std::max(t1 - t0, 1e-6);

        const double alphaRaw = (currentTime_ - t0) / dtSeg;
        const double alpha    = std::clamp(alphaRaw, 0.0, 1.0);

        for (std::size_t j = 0; j < dof; ++j)
        {
            motionCmd[j].modeOfOperation = 8; // CSP
            const double q0 = p0.q[j];
            const double q1 = p1.q[j];
            motionCmd[j].targetPosition = (1.0 - alpha) * q0 + alpha * q1;
            motionCmd[j].targetTorque   = 0.0; // no FF torque for now
        }

        const double totalTime = plan_.points[N - 1].t;
        const double progress =
            (totalTime > 1e-6) ? std::clamp(currentTime_ / totalTime, 0.0, 1.0) : 1.0;

        publishStatus(progress, true, false, false);
    }

    void JointTrajectoryController::stop()
    {
        if (state_ == ControllerState::RUNNING)
        {
            state_  = ControllerState::STOPPED;
            hasPlan_ = false;
            publishStatus(1.0, false, true, false);

            if (loggerMem_)
            {
                merai::log_info(loggerMem_, "Control", 6004,
                                "[JointTrajectoryController] Stopped by request");
            }
        }
    }

    void JointTrajectoryController::teardown()
    {
        state_  = ControllerState::INACTIVE;
        hasPlan_ = false;
    }

    void JointTrajectoryController::publishStatus(double progress,
                                                  bool   running,
                                                  bool   finished,
                                                  bool   aborted)
    {
        if (!rtLayout_)
        {
            return;
        }

        const int backIdx = merai::back_index(rtLayout_->jointTrajectoryStatusBuffer);
        auto& st          = rtLayout_->jointTrajectoryStatusBuffer.buffer[backIdx];

        st.jobId    = activeJobId_;
        st.progress = progress;
        st.running  = running;
        st.finished = finished;
        st.aborted  = aborted;

        merai::publish(rtLayout_->jointTrajectoryStatusBuffer, backIdx);
    }

    void JointTrajectoryController::holdFinalPosition(
        std::span<merai::JointMotionCommand> motionCmd)
    {
        const std::uint32_t N = plan_.numPoints;
        if (N == 0)
        {
            return;
        }

        const auto& plast = plan_.points[N - 1];
        const std::size_t dof =
            std::min<std::size_t>({ jointCount_, motionCmd.size(),
                                    static_cast<std::size_t>(merai::MAX_SERVO_DRIVES) });

        for (std::size_t j = 0; j < dof; ++j)
        {
            motionCmd[j].modeOfOperation = 8;
            motionCmd[j].targetPosition  = plast.q[j];
            motionCmd[j].targetTorque    = 0.0;
        }
    }

} // namespace control
