#pragma once

#include "control/controllers/BaseController.h"

// If your six-axis model lives here, include it:
#include "robotics_lib/haptic_device/HapticDeviceModel.h"
#include "robotics_lib/haptic_device/HapticDeviceDynamics.h"
#include "robotics_lib/haptic_device/HapticDeviceKinematics.h"

namespace hand_control
{
    namespace control
    {

        /**
         * @brief A simple controller for gravity compensation, referencing the 6-axis robot model.
         */
        class GravityCompController : public BaseController
        {
        public:
            /**
             * @brief Construct with a reference to the six-axis model.
             *        No dynamic memory used. The model must outlive this controller.
             */
            explicit GravityCompController(const hand_control::robotics::haptic_device::HapticDeviceModel &model);

            ~GravityCompController() override = default;

            /**
             * @brief init() 
             *  A parameterless init that ensures no std::string usage.
             */
            bool init() override;

            /**
             * @brief start 
             *  Called when transitioning to this controller.
             */
            void start() override;

            /**
             * @brief update 
             *  The main control loop function. Called each real-time cycle (e.g. 1ms).
             *
             * @param states   Pointer to array of JointState
             * @param commands Pointer to array of JointCommand
             * @param numJoints Number of joints
             * @param dt       Timestep in seconds
             */
            void update(const hand_control::merai::JointState *states,
                        hand_control::merai::JointCommand *commands,
                        int numJoints,
                        double dt) override;

            /**
             * @brief stop 
             *  Called when transitioning away from this controller to a new one.
             */
            void stop() override;

            /**
             * @brief teardown 
             *  For cleanup if needed.
             */
            void teardown() override;

        private:
            // Reference to the 6-axis robot model (loaded at init time).
            const hand_control::robotics::haptic_device::HapticDeviceModel &model_;

            /**
             * @brief A local SixAxisDynamics object that uses the same model.
             *        We'll use it to compute torque via Newton-Euler or similar method.
             */
            hand_control::robotics::haptic_device::HapticDeviceDynamics dynamics_;

            // Example param for torque scaling, friction offset, etc. 
            double gravityScale_ = 1.0;
        };

    } // namespace control
} // namespace hand_control
