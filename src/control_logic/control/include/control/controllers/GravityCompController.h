#pragma once

#include <string>
#include "control/controllers/BaseController.h"

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

            bool init(const std::string &controllerName) override;
            void start() override;

            /**
             * @brief Update method matching BaseController's pointer-based signature.
             * @param states   pointer to an array of JointState
             * @param commands pointer to an array of JointCommand
             * @param numJoints number of joints
             * @param dt time step
             */
            void update(const hand_control::merai::JointState *states,
                        hand_control::merai::JointCommand *commands,
                        int numJoints,
                        double dt) override;

            void stop() override;
            void teardown() override;

        private:
            // Reference to the 6-axis robot model (loaded at init time).
            const hand_control::robotics::haptic_device::HapticDeviceModel &model_;

            /**
             * @brief A local SixAxisDynamics object that uses the same model.
             *        We'll use it to compute torque via Newton-Euler.
             */
            hand_control::robotics::haptic_device::HapticDeviceDynamics dynamics_;

            // Example param for torque scaling or other offsets
            // (optional, could be a gravity scaling factor, friction offset, etc.)
            double gravityScale_ = 1.0;
        };

    } // namespace control
} // namespace hand_control
