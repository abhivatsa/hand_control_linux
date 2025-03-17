#include "logic/SafetyManager.h"
#include <cmath> // for std::fabs

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
                // Possibly log an error or handle differently in RT
                return false;
            }

            // 1) read driveCount (or jointCount) from paramServer
            driveCount_ = paramServerPtr_->driveCount;
            if (driveCount_ > MAX_JOINTS)
            {
                driveCount_ = MAX_JOINTS;
            }

            // 2) read joint min/max from paramServer
            for (std::size_t i = 0; i < driveCount_; ++i)
            {
                // Assume paramServer->joints[i] has fields: joint_limit_min, joint_limit_max
                jointMin_[i] = paramServerPtr_->joints[i].joint_limit_min; 
                jointMax_[i] = paramServerPtr_->joints[i].joint_limit_max;
            }
            // For any unused joints
            for (std::size_t i = driveCount_; i < MAX_JOINTS; ++i)
            {
                jointMin_[i] = -3.14;  // or your default
                jointMax_[i] =  3.14;  // or your default
            }

            // If needed, read torque limits or other thresholds from paramServer
            // e.g., paramServerPtr_->joints[i].max_torque

            faulted_ = false;
            return true;
        }

        bool SafetyManager::update(const hand_control::merai::DriveFeedbackData& driveFdbk,
                                   const hand_control::merai::UserCommands&      userCmds,
                                   const hand_control::merai::ControllerFeedback& ctrlFdbk)
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
                // (1) Check if eStop is active => immediate fault
                if (userCmds.eStop)
                {
                    forceFault();
                }

                // (2) Check drive feedback statuses
                //     If driveFdbk.status[i] == DriveStatus::FAULT => forceFault()
                for (std::size_t i = 0; i < driveCount_; ++i)
                {
                    if (driveFdbk.status[i] == hand_control::merai::DriveStatus::FAULT)
                    {
                        forceFault();
                        break;
                    }
                }

                // (3) Possibly check if ctrlFdbk indicates an error
                // If you have an enum like ControllerFeedbackState::ERROR or FAILED
                if (ctrlFdbk.feedbackState == hand_control::merai::ControllerFeedbackState::ERROR)
                {
                    forceFault();
                }

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

        bool SafetyManager::HomingStatus()
        {
            // Example placeholder. If you track homing done or not
            // Return a real value if needed.
            return true; 
        }

        void SafetyManager::checkJointLimits()
        {
            if (!rtLayout_)
            {
                return;
            }
            // We read from the jointBuffer (states) to see if any joint is out of range
            int frontIdx = rtLayout_->jointBuffer.frontIndex.load(std::memory_order_acquire);
            auto &jointStates = rtLayout_->jointBuffer.buffer[frontIdx].states;

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

            // For example, read joint states to see if torque > threshold
            int frontIdx = rtLayout_->jointBuffer.frontIndex.load(std::memory_order_acquire);
            auto &jointStates = rtLayout_->jointBuffer.buffer[frontIdx].states;

            // Example torque check if paramServer has max torque
            /*
            for (std::size_t i = 0; i < driveCount_; ++i)
            {
                double actualTorque = jointStates[i].torque;
                double maxTorque    = paramServerPtr_->joints[i].max_torque;
                if (std::fabs(actualTorque) > maxTorque)
                {
                    forceFault();
                    break;
                }
            }
            */

            // Or do advanced kinematic checks with kinematics_ or dynamics_, etc.
        }

    } // namespace logic
} // namespace hand_control
