#include "control/controllers/GravityCompController.h"
// #include <iostream> // If you want to remove all console output from RT code, comment or remove this

namespace hand_control
{
    namespace control
    {

        GravityCompController::GravityCompController(
            const hand_control::robotics::haptic_device::HapticDeviceModel &model)
            : model_(model), dynamics_(model) // Construct the HapticDeviceDynamics with the same model
        {
            // No dynamic allocations here
        }

        bool GravityCompController::init()
        {
            // No string usage. Just set state to INIT.
            // If you need some other param loads, do them here as well.
            state_ = ControllerState::INIT;
            return true;
        }

        void GravityCompController::start()
        {
            if (state_ == ControllerState::INIT || state_ == ControllerState::STOPPED)
            {
                // If you want to log or debug, use a static message or integer code, not strings
                // e.g. "GC start" or do nothing in strict RT
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

            // 1) Convert from arrays to some math vector
            //    (assuming numJoints == 6 for a 6-axis device; adapt if needed)
            hand_control::math::Vector<6> jointAngles, jointVel, jointAcc;
            jointAngles.setZero();
            jointVel.setZero();
            jointAcc.setZero();  // for pure gravity comp, we can set acceleration=0

            for (int i = 0; i < numJoints; ++i)
            {
                jointAngles[i] = states[i].position;
                jointVel[i]    = states[i].velocity; // if velocity-based friction is considered
            }

            // 2) Compute inverse dynamics
            hand_control::math::Vector<6> outTorques;
            outTorques.setZero();

            int ret = dynamics_.computeInverseDynamics(
                jointAngles, jointVel, jointAcc, outTorques);

            if (ret != 0)
            {
                // fallback: zero torques if an error code was returned
                for (int i = 0; i < numJoints; i++)
                {
                    commands[i].torque = 0.0;
                }
                return;
            }

            // 3) Copy torques back with optional scaling
            for (int i = 0; i < numJoints; ++i)
            {
                commands[i].torque = gravityScale_ * outTorques[i];
            }
        }

        void GravityCompController::stop()
        {
            if (state_ == ControllerState::RUNNING)
            {
                // optional logging w/o strings or do nothing
                state_ = ControllerState::STOPPED;
            }
        }

        void GravityCompController::teardown()
        {
            // optional logging w/o strings or do nothing
            state_ = ControllerState::UNINIT;
        }

    } // namespace control
} // namespace hand_control
