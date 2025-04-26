#include "control/controllers/GravityCompController.h"

namespace hand_control
{
    namespace control
    {
        GravityCompController::GravityCompController(
            const hand_control::robotics::haptic_device::HapticDeviceModel &model,
            hand_control::merai::JointMotionFeedback* feedbackPtr,
            hand_control::merai::JointMotionCommand* commandPtr,
            std::size_t numJoints)
            : model_(model),
              dynamics_(model),
              feedbackPtr_(feedbackPtr),
              commandPtr_(commandPtr),
              numJoints_(numJoints)
        {
            // Constructor only stores pointers and parameters. No heavy logic here.
        }

        bool GravityCompController::init()
        {
            // Validate pointers
            if (!feedbackPtr_ || !commandPtr_ || numJoints_ == 0)
            {
                return false; // or handle error
            }

            state_ = ControllerState::INIT;
            return true;
        }

        void GravityCompController::start()
        {
            // Transition to RUNNING if allowed
            if (state_ == ControllerState::INIT || state_ == ControllerState::STOPPED)
            {
                state_ = ControllerState::RUNNING;
            }
            
            for (int i = 0; i < numJoints_; ++i)
            {
                commandPtr_[i].modeOfOperation = -3;
            }

        }

        void GravityCompController::update(double dt)
        {
            // Only compute torques if RUNNING
            if (state_ != ControllerState::RUNNING)
            {
                return;
            }

            // If numJoints_ can exceed 6, adapt your math library or clamp dof to 6
            int dof = static_cast<int>(numJoints_);
            // if (numJoints_ > 6)
            // {
            //     numJoints_ = 6;
            // }

            // Prepare vectors for inverse dynamics
            hand_control::math::Vector<6> jointAngles, jointVel, jointAcc;
            jointAngles.setZero();
            jointVel.setZero();
            jointAcc.setZero(); // for pure gravity comp, assume zero acceleration

            // 1) Read joint positions & velocities into your math vectors
            for (int i = 0; i < dof; ++i)
            {
                jointAngles[i] = feedbackPtr_[i].positionActual;
                jointVel[i]    = feedbackPtr_[i].velocityActual;
            }

            // 2) Compute inverse dynamics
            hand_control::math::Vector<6> outTorques;
            outTorques.setZero();

            int ret = dynamics_.computeInverseDynamics(jointAngles, jointVel, jointAcc, outTorques);
            if (ret != 0)
            {
                // fallback: zero torques if there's an error code
                for (int i = 0; i < dof; ++i)
                {
                    commandPtr_[i].targetTorque = 0.0;
                }
                return;
            }

            // 3) Write torques to the command array
            for (int i = 0; i < dof; ++i)
            {
                commandPtr_[i].targetTorque = gravityScale_ * outTorques[i];
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
} // namespace hand_control
