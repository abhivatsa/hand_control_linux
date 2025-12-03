#include "logic/SafetyManager.h"

#include <array>
#include <cmath> // std::fabs
#include "merai/RTIpc.h"

namespace logic
{
    SafetyManager::SafetyManager(const merai::ParameterServer *paramServerPtr,
                                 merai::RTMemoryLayout *rtLayout,
                                 const robot_lib::RobotModel &model)
        : paramServerPtr_(paramServerPtr), rtLayout_(rtLayout), model_(model)
    {
        // No dynamic allocations, no strings.
    }

    bool SafetyManager::init()
    {
        if (!paramServerPtr_)
        {
            return false;
        }

        // Clamp drive/joint count to MAX_JOINTS.
        driveCount_ = paramServerPtr_->driveCount;
        if (driveCount_ > MAX_JOINTS)
        {
            driveCount_ = MAX_JOINTS;
        }

        // Load joint limits from ParameterServer.
        for (std::size_t i = 0; i < driveCount_; ++i)
        {
            jointMin_[i] = paramServerPtr_->joints[i].limits.position.min;
            jointMax_[i] = paramServerPtr_->joints[i].limits.position.max;
        }
        // Fill any unused entries with a sane default range.
        for (std::size_t i = driveCount_; i < MAX_JOINTS; ++i)
        {
            jointMin_[i] = -3.14;
            jointMax_[i] = 3.14;
        }

        faulted_ = false;
        return true;
    }

    bool SafetyManager::update(const DriveFeedbackData &driveFdbk,
                               const UserCommands &userCmds,
                               const ControllerFeedback &ctrlFdbk)
    {
        if (faulted_)
        {
            if (userCmds.resetFault && !userCmds.eStop)
            {
                bool anyFault = false;
                for (std::size_t i = 0; i < driveCount_; ++i)
                {
                    if (driveFdbk.status[i] == DriveStatus::FAULT)
                    {
                        anyFault = true;
                        break;
                    }
                }

                if (!anyFault)
                {
                    clearFault();
                }
            }
        }

        if (!faulted_)
        {
            if (userCmds.eStop)
            {
                forceFault();
            }

            for (std::size_t i = 0; i < driveCount_; ++i)
            {
                if (driveFdbk.status[i] == DriveStatus::FAULT)
                {
                    forceFault();
                    break;
                }
            }

            // optional: checkJointLimits(); checkTorqueOrKinematicLimits();
        }

        return faulted_;
    }
    s

        void
        SafetyManager::checkJointLimits()
    {
        if (!rtLayout_)
        {
            return;
        }

        std::array<merai::JointFeedbackData, merai::MAX_SERVO_DRIVES> jointFeedback{};
        merai::read_snapshot(rtLayout_->jointFeedbackBuffer, jointFeedback);

        for (std::size_t i = 0; i < driveCount_; ++i)
        {
            double pos = jointFeedback[i].motion.positionActual;
            if (pos < jointMin_[i] || pos > jointMax_[i])
            {
                forceFault();
                break;
            }
        }
    }

    void SafetyManager::checkTorqueOrKinematicLimits()
    {
        if (!rtLayout_)
        {
            return;
        }

        // Placeholder for future advanced checks:
        //  - torque > limit
        //  - kinematic singularities
        //  - dynamics-based checks using `model_`
        //
        // Example skeleton:
        //
        // int activeIdx =
        //     rtLayout_->jointFeedbackBuffer.activeIndex.load(std::memory_order_acquire);
        // const auto& jointFeedback = rtLayout_->jointFeedbackBuffer.buffer[activeIdx];
        //
        // for (std::size_t i = 0; i < driveCount_; ++i)
        // {
        //     double tau = jointFeedback[i].motion.torqueActual;
        //     double maxTau = paramServerPtr_->joints[i].limits.torque.max; // if available
        //     if (std::fabs(tau) > maxTau)
        //     {
        //         forceFault();
        //         break;
        //     }
        // }
    }

} // namespace logic
