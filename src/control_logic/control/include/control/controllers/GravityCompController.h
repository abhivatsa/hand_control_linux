#pragma once

#include "control/controllers/BaseController.h"
#include "robotics_lib/haptic_device/HapticDeviceModel.h"
#include "robotics_lib/haptic_device/HapticDeviceDynamics.h"
#include "math_lib/Vector.h"

namespace hand_control
{
    namespace control
    {
        /**
         * @brief GravityCompController
         *        A controller that applies torques based on inverse dynamics
         *        to compensate for gravity (and possibly friction).
         */
        class GravityCompController : public BaseController
        {
        public:
            /**
             * @brief Constructor that stores references to the device model,
             *        as well as pointers to the joint states/commands arrays.
             *
             * @param model        A device model describing kinematics/dynamics.
             * @param statesPtr    Pointer to array of JointState (size = numJoints).
             * @param commandsPtr  Pointer to array of JointCommand (size = numJoints).
             * @param numJoints    Number of joints this controller operates on.
             */
            GravityCompController(
                const hand_control::robotics::haptic_device::HapticDeviceModel &model,
                hand_control::merai::JointState *statesPtr,
                hand_control::merai::JointCommand *commandsPtr,
                std::size_t numJoints);

            /**
             * @brief init
             *   - Checks pointers, sets internal state to INIT.
             */
            bool init() override;

            /**
             * @brief start
             *   - Transitions from INIT or STOPPED to RUNNING.
             */
            void start() override;

            /**
             * @brief update
             *   - Called each control cycle (e.g. 1kHz).
             *   - Reads joint states from statesPtr_, writes torque commands to commandsPtr_.
             *   - Uses inverse dynamics for gravity compensation.
             * 
             * @param dt  Timestep in seconds, e.g. 0.001.
             */
            void update(double dt) override;

            /**
             * @brief stop
             *   - Transitions from RUNNING to STOPPED.
             */
            void stop() override;

            /**
             * @brief teardown
             *   - Final cleanup; sets state to UNINIT.
             */
            void teardown() override;

        private:
            // Pointers to joint data (in SI units)
            hand_control::merai::JointState  *statesPtr_   = nullptr;
            hand_control::merai::JointCommand* commandsPtr_ = nullptr;
            std::size_t numJoints_ = 0;

            // The device model and dynamics object
            const hand_control::robotics::haptic_device::HapticDeviceModel &model_;
            hand_control::robotics::haptic_device::HapticDeviceDynamics dynamics_;

            // Controller parameter(s)
            double gravityScale_ = 1.0; // scale factor for torque compensation
        };

    } // namespace control
} // namespace hand_control
