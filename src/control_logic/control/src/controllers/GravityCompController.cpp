#include "control/controllers/GravityCompController.h"

namespace hand_control
{
    namespace control
    {
        GravityCompController::GravityCompController(
            const hand_control::robotics::haptic_device::HapticDeviceModel &model,
            hand_control::merai::JointState *statesPtr,
            hand_control::merai::JointCommand *commandsPtr,
            std::size_t numJoints)
            : model_(model),
              dynamics_(model),
              statesPtr_(statesPtr),
              commandsPtr_(commandsPtr),
              numJoints_(numJoints)
        {
            // Constructor only stores pointers and parameters. No heavy logic here.
        }

        bool GravityCompController::init()
        {
            // Validate pointers and set initial state
            if (!statesPtr_ || !commandsPtr_ || numJoints_ == 0)
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
        }

        void GravityCompController::update(double dt)
        {
            // Only compute torques if RUNNING
            if (state_ != ControllerState::RUNNING)
            {
                return;
            }

            // We'll assume a 6-DOF device, but if numJoints_ can vary, adapt the logic
            // For safety, limit dof to 6 if your math library uses 6-element vectors
            const int dof = static_cast<int>(numJoints_);
            if (dof > 6)
            {
                // Or handle an error, or adapt your math vectors to match dof
            }

            // Prepare vectors for inverse dynamics
            hand_control::math::Vector<6> jointAngles, jointVel, jointAcc;
            jointAngles.setZero();
            jointVel.setZero();
            jointAcc.setZero();  // for pure gravity comp, assume no acceleration

            // 1) Read joint positions & velocities into your math vectors
            for (int i = 0; i < dof; ++i)
            {
                jointAngles[i] = statesPtr_[i].position;
                jointVel[i]    = statesPtr_[i].velocity;
            }

            // 2) Compute inverse dynamics
            hand_control::math::Vector<6> outTorques;
            outTorques.setZero();

            int ret = dynamics_.computeInverseDynamics(jointAngles, jointVel, jointAcc, outTorques);
            if (ret != 0)
            {
                // fallback: zero torques if an error code
                for (int i = 0; i < dof; ++i)
                {
                    commandsPtr_[i].torque = 0.0;
                }
                return;
            }

            // 3) Write torques to commands array
            for (int i = 0; i < dof; ++i)
            {
                commandsPtr_[i].torque = gravityScale_ * outTorques[i];
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
