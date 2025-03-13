#include "logic/SafetyManager.h"
#include <cmath> // for range checks

namespace hand_control
{
    namespace logic
    {
        SafetyManager::SafetyManager(const hand_control::merai::ParameterServer* paramServerPtr,
                                     hand_control::merai::RTMemoryLayout*        rtLayout,
                                     const hand_control::robotics::haptic_device::HapticDeviceModel& model)
            : paramServerPtr_(paramServerPtr),
              rtLayout_(rtLayout),
              model_(model),
              kinematics_(model),
              dynamics_(model)
        {
            // No dynamic memory or std::string usage in RT
        }

        bool SafetyManager::init()
        {
            if (!paramServerPtr_)
            {
                // For real-time, you may skip i/o or do minimal error logging
                return false;
            }

            // 1) read driveCount (or jointCount)
            driveCount_ = paramServerPtr_->driveCount;
            if (driveCount_ > MAX_JOINTS)
            {
                driveCount_ = MAX_JOINTS;
            }

            // 2) read joint min/max from paramServer
            for (std::size_t i = 0; i < driveCount_; ++i)
            {
                jointMin_[i] = paramServerPtr_->joints[i].joint_limit_min; 
                jointMax_[i] = paramServerPtr_->joints[i].joint_limit_max;
            }
            for (std::size_t i = driveCount_; i < MAX_JOINTS; ++i)
            {
                jointMin_[i] = -3.14;
                jointMax_[i] = 3.14;
            }

            // If needed, read torque limits or other thresholds:
            // e.g. paramServerPtr_->joints[i].max_torque

            faulted_ = false;
            return true;
        }

        bool SafetyManager::update(const hand_control::merai::DriveFeedbackData& driveFdbk,
                                   const hand_control::merai::UserCommands&      userCmds,
                                   const hand_control::merai::ControllerFeedbackData& ctrlFdbk)
        {
            // If already faulted, see if user wants to reset
            if (faulted_)
            {
                if (userCmds.resetFault)
                {
                    clearFault(); 
                }
            }

            // If still not cleared, skip new checks or continue
            if (!faulted_)
            {
                // (1) eStop => immediate fault
                if (userCmds.eStop)
                {
                    forceFault();
                }

                // (2) Check drive feedback 
                for (std::size_t i = 0; i < driveCount_; ++i)
                {
                    if (driveFdbk.feedback[i].fault)
                    {
                        forceFault();
                        break;
                    }
                }

                // (3) Possibly check if ctrlFdbk indicates an error 
                // e.g., if ctrlFdbk.feedback[0].controllerFailed => forceFault();

                // (4) Check joint limits
                checkJointLimits();

                // (5) Check torque or advanced kinematics if needed
                checkTorqueOrKinematicLimits();
            }

            return faulted_;
        }

        bool SafetyManager::isFaulted() const
        {
            return faulted_;
        }

        void SafetyManager::forceFault()
        {
            faulted_ = true;
        }

        void SafetyManager::clearFault()
        {
            // Optionally ensure we are safe to re-enable
            faulted_ = false;
        }

        void SafetyManager::checkJointLimits()
        {
            if (!rtLayout_)
            {
                return;
            }
            int frontIdx = rtLayout_->jointBuffer.frontIndex.load(std::memory_order_acquire);
            auto &jointStates = rtLayout_->jointBuffer.buffer[frontIdx].states;

            // For each joint, see if position is outside min/max
            for (std::size_t i = 0; i < driveCount_; ++i)
            {
                double pos = jointStates[i].position;
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
            int frontIdx = rtLayout_->jointBuffer.frontIndex.load(std::memory_order_acquire);
            auto &jointStates = rtLayout_->jointBuffer.buffer[frontIdx].states;

            // Example: read positions/velocities into local vectors
            // Then maybe compute forward kinematics, check for collisions, etc.
            // If out of range, call forceFault();

            // If you have torque limits, check jointStates[i].torque vs. max torque
            /*
            for (std::size_t i = 0; i < driveCount_; ++i)
            {
                double jointTorque = jointStates[i].torque;
                double maxTorque   = paramServerPtr_->joints[i].max_torque;
                if (std::fabs(jointTorque) > maxTorque)
                {
                    forceFault();
                    break;
                }
            }
            */

            // Or use dynamics_ for inverse dynamics checks (like check for passivity, etc.)
        }

    } // namespace logic
} // namespace hand_control
