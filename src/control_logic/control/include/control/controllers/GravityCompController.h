#pragma once

#include "control/controllers/BaseController.h"
#include "robotics_lib/haptic_device/HapticDeviceModel.h"
#include "robotics_lib/haptic_device/HapticDeviceDynamics.h"
#include "math_lib/Vector.h"
#include "merai/RTMemoryLayout.h" // for DriveCommand, DriveStatus, etc.

namespace seven_axis_robot
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
             *        as well as pointers to the joint feedback/command arrays.
             *
             * @param model        A device model describing kinematics/dynamics.
             * @param feedbackPtr  Pointer to array of JointMotionFeedback (size = numJoints).
             * @param commandPtr   Pointer to array of JointMotionCommand (size = numJoints).
             * @param numJoints    Number of joints this controller operates on.
             */
            GravityCompController(
                const seven_axis_robot::robotics::haptic_device::HapticDeviceModel &model,
                seven_axis_robot::merai::JointMotionFeedback* feedbackPtr,
                seven_axis_robot::merai::JointMotionCommand*  commandPtr,
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
             *   - Reads joint positions/velocities from feedbackPtr_,
             *     writes torque commands to commandPtr_.
             *   - Uses inverse dynamics for gravity compensation.
             *
             * @param dt  Timestep in seconds, e.g. 0.001 for 1kHz.
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
            // Pointers to motion feedback & command data (SI units)
            seven_axis_robot::merai::JointMotionFeedback* feedbackPtr_ = nullptr;
            seven_axis_robot::merai::JointMotionCommand*  commandPtr_  = nullptr;
            std::size_t numJoints_ = 0;

            // Device model and dynamics
            const seven_axis_robot::robotics::haptic_device::HapticDeviceModel &model_;
            seven_axis_robot::robotics::haptic_device::HapticDeviceDynamics dynamics_;

            // Optional parameter
            double gravityScale_ = 1.0; // scale factor for torque compensation
        };

    } // namespace control
} // namespace seven_axis_robot
