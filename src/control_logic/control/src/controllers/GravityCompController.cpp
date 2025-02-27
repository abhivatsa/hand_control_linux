#include "control/controllers/GravityCompController.h"
#include <iostream>

namespace hand_control
{
    namespace control
    {

        GravityCompController::GravityCompController(
            const hand_control::robotics::haptic_device::HapticDeviceModel &model)
            : model_(model), dynamics_(model) // Construct the SixAxisDynamics with the same model
        {
            // No dynamic allocations here
        }

        bool GravityCompController::init(const std::string &controllerName)
        {
            name_ = controllerName;
            std::cout << "[GravityCompController] init with name: " << name_ << std::endl;
            state_ = ControllerState::INIT;
            return true;
        }

        void GravityCompController::start()
        {
            if (state_ == ControllerState::INIT || state_ == ControllerState::STOPPED)
            {
                std::cout << "[GravityCompController] start\n";
                state_ = ControllerState::RUNNING;
            }
        }

        void GravityCompController::update(const hand_control::merai::JointState *states,
                                           hand_control::merai::JointCommand *commands,
                                           int numJoints,
                                           double /*dt*/)
        {
            if (state_ != ControllerState::RUNNING)
            {
                return;
            }

            // 1) Convert from arrays to math::Vector<6>
            //    (Assuming numJoints == 6; if variable, handle accordingly.)
            hand_control::math::Vector<6> jointAngles;
            hand_control::math::Vector<6> jointVel;
            hand_control::math::Vector<6> jointAcc;
            jointAngles.setZero();
            jointVel.setZero();
            jointAcc.setZero(); // For pure gravity comp, we can set acceleration = 0

            for (int i = 0; i < numJoints; ++i)
            {
                jointAngles[i] = states[i].position; // e.g. in radians
                jointVel[i] = states[i].velocity;    // if needed for complete inverse dynamics
            }

            // 2) Call computeInverseDynamics
            hand_control::math::Vector<6> outTorques;
            outTorques.setZero();

            // For pure gravity comp, we might set velocities to 0 if we only want gravity torque
            // but let's assume we keep them so if there's friction or other velocity terms.
            int ret = dynamics_.computeInverseDynamics(jointAngles, jointVel, jointAcc, outTorques);

            if (ret != 0)
            {
                // handle error
                // fallback: zero torques
                for (int i = 0; i < numJoints; i++)
                {
                    commands[i].torque = 0.0;
                }
                return;
            }

            // 3) Copy torques back to commands
            for (int i = 0; i < numJoints; ++i)
            {
                // Optionally scale or adjust for friction
                commands[i].torque = gravityScale_ * outTorques[i];
            }
        }

        void GravityCompController::stop()
        {
            if (state_ == ControllerState::RUNNING)
            {
                std::cout << "[GravityCompController] stop\n";
                state_ = ControllerState::STOPPED;
            }
        }

        void GravityCompController::teardown()
        {
            std::cout << "[GravityCompController] teardown\n";
            state_ = ControllerState::UNINIT;
        }

    } // namespace control
} // namespace hand_control
