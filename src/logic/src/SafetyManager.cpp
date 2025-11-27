#include "logic/SafetyManager.h"
#include <cmath> // for std::fabs

namespace seven_axis_robot
{
    namespace logic
    {
        SafetyManager::SafetyManager(const seven_axis_robot::merai::ParameterServer* paramServerPtr,
                                     seven_axis_robot::merai::RTMemoryLayout*        rtLayout,
                                     const seven_axis_robot::robotics::haptic_device::HapticDeviceModel& model)
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
                jointMin_[i] = paramServerPtr_->joints[i].limits.position.min; 
                jointMax_[i] = paramServerPtr_->joints[i].limits.position.max;
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

        bool SafetyManager::update(const seven_axis_robot::merai::DriveFeedbackData& driveFdbk,
                                   const seven_axis_robot::merai::UserCommands&      userCmds,
                                   const seven_axis_robot::merai::ControllerFeedback& ctrlFdbk)
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
                    if (driveFdbk.status[i] == seven_axis_robot::merai::DriveStatus::FAULT)
                    {
                        forceFault();
                        break;
                    }
                }

                // // (3) Possibly check if ctrlFdbk indicates an error
                // // If you have an enum like ControllerFeedbackState::ERROR or FAILED
                // if (ctrlFdbk.feedbackState == seven_axis_robot::merai::ControllerFeedbackState::ERROR)
                // {
                //     forceFault();
                // }

                // // (4) Check joint limits
                // checkJointLimits();

                // // (5) Check torque or advanced kinematics if needed
                // checkTorqueOrKinematicLimits();
            }

            return faulted_;
        }

        bool SafetyManager::isFaulted() const
        {
            return faulted_;
        }

        bool SafetyManager::isHomingCompleted(){
            if (!rtLayout_)
            {
                return false;
            }
            double homing_pos[7] = {0, 0, 0, 0, 0, 0, 0};
            // We read from the jointBuffer (states) to see if any joint is out of range
            int frontIdx = rtLayout_->jointBuffer.frontIndex.load(std::memory_order_acquire);
            auto &jointFeedback = rtLayout_->jointBuffer.buffer[frontIdx].feedback;

            for (std::size_t i = 0; i < driveCount_; ++i)
            {
                double pos = jointFeedback[i].motion.positionActual;
                if (fabs(homing_pos[i] - pos) > 0.001){
                    return false;
                }
            }

            return true;
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
            // We read from the jointBuffer (states) to see if any joint is out of range
            int frontIdx = rtLayout_->jointBuffer.frontIndex.load(std::memory_order_acquire);
            auto &jointFeedback = rtLayout_->jointBuffer.buffer[frontIdx].feedback;

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

            // For example, read joint states to see if torque > threshold
            int frontIdx = rtLayout_->jointBuffer.frontIndex.load(std::memory_order_acquire);
            auto &jointFeedback = rtLayout_->jointBuffer.buffer[frontIdx].feedback;

            for (int i = 0; i < driveCount_; i++)
            {
                
            }

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
} // namespace seven_axis_robot
