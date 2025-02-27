#include <cmath>
#include <iostream>

#include "control/bridge_controllers/TrajectoryAlignBridgingController.h"
// If JointState / JointCommand come from hand_control::merai, also include:
// #include "merai/RTMemoryLayout.h"

namespace hand_control
{
    namespace control
    {
        bool TrajectoryAlignBridgingController::init(const std::string &controllerName)
        {
            name_ = controllerName;
            state_ = ControllerState::INIT;
            return true;
        }

        void TrajectoryAlignBridgingController::start()
        {
            state_ = ControllerState::RUNNING;
            done_ = false;
            elapsedTime_ = 0.0;
            std::cout << "[TrajectoryAlignBridging] start()\n";
        }

        // If JointState / JointCommand are in hand_control::merai, we fully qualify them:
        //   void TrajectoryAlignBridgingController::update(
        //       const hand_control::merai::JointState* states,
        //       hand_control::merai::JointCommand* commands,
        //       double dt)

        void TrajectoryAlignBridgingController::update(const hand_control::merai::JointState *states,
                                                       hand_control::merai::JointCommand *commands,
                                                       int numJoints,
                                                       double dt)
        {
            if (state_ != ControllerState::RUNNING)
            {
                return;
            }

            elapsedTime_ += dt;

            // Simple approach: linearly interpolate from current pos to target
            double alpha = elapsedTime_ / maxTime_;
            if (alpha > 1.0)
            {
                alpha = 1.0;
            }

            bool allClose = true;

            for (int i = 0; i < jointCount_; ++i)
            {
                double currentPos = states[i].position;
                double targetPos =
                    (i < static_cast<int>(targetPositions_.size())) ? targetPositions_[i]
                                                                    : currentPos;

                // Linearly interpolate
                double desiredPos = (1.0 - alpha) * currentPos + alpha * targetPos;
                commands[i].position = desiredPos;
                commands[i].velocity = 0.0;
                commands[i].torque = 0.0; // or small if needed

                // Check if we're close
                double error = std::fabs(desiredPos - currentPos);
                if (error > 0.01) // some threshold
                {
                    allClose = false;
                }
            }

            // If alpha >= 1 or if we're all close
            if (alpha >= 1.0 && allClose)
            {
                done_ = true;
            }
        }

        void TrajectoryAlignBridgingController::stop()
        {
            state_ = ControllerState::STOPPED;
            std::cout << "[TrajectoryAlignBridging] stop()\n";
        }

        void TrajectoryAlignBridgingController::teardown()
        {
            std::cout << "[TrajectoryAlignBridging] teardown()\n";
        }

        void TrajectoryAlignBridgingController::setTargetPositions(const std::vector<double> &positions)
        {
            targetPositions_ = positions;
        }
    } // namespace control
} // namespace hand_control
