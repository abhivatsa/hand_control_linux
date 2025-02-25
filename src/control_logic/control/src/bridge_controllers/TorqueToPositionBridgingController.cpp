#include <iostream>
#include <cmath>

#include "control/bridge_controllers/TorqueToPositionBridgingController.h"
// If JointState / JointCommand are in motion_control::merai, also include:
#include "merai/RTMemoryLayout.h"

namespace motion_control
{
    namespace control
    {
        bool TorqueToPositionBridgingController::init(const std::string& controllerName)
        {
            name_ = controllerName;
            state_ = ControllerState::INIT;
            return true;
        }

        void TorqueToPositionBridgingController::start()
        {
            state_ = ControllerState::RUNNING;
            done_ = false;
            elapsedTime_ = 0.0;
            std::cout << "[TorqueToPositionBridging] start()\n";
        }

        // If JointState / JointCommand are in motion_control::merai, we fully qualify them:
        //   void TorqueToPositionBridgingController::update(
        //       const motion_control::merai::JointState* states,
        //       motion_control::merai::JointCommand* commands,
        //       double dt)

        void TorqueToPositionBridgingController::update(
            const motion_control::merai::JointState* states,
            motion_control::merai::JointCommand*     commands,
            double                                   dt)
        {
            if (state_ != ControllerState::RUNNING)
            {
                return;
            }

            elapsedTime_ += dt;
            double alpha = (elapsedTime_ / transitionTime_);
            if (alpha > 1.0)
            {
                alpha = 1.0;
            }

            for (int i = 0; i < jointCount_; ++i)
            {
                double oldEffort = states[i].effort; // assume current torque
                double newTorque = (1.0 - alpha) * oldEffort;
                commands[i].effort_command   = newTorque;
                commands[i].position_command = states[i].position;
                commands[i].velocity_command = 0.0;
            }

            // Possibly reconfigure drive to position mode once alpha > 0.5, etc.

            // Mark bridging done once we've ramped torque out
            if (alpha >= 1.0)
            {
                done_ = true;
            }
        }

        void TorqueToPositionBridgingController::stop()
        {
            state_ = ControllerState::STOPPED;
            std::cout << "[TorqueToPositionBridging] stop()\n";
        }

        void TorqueToPositionBridgingController::teardown()
        {
            std::cout << "[TorqueToPositionBridging] teardown()\n";
        }
    } // namespace control
} // namespace motion_control
