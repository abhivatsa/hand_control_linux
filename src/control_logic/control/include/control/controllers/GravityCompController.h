#pragma once

#include <string>
#include "control/controllers/BaseController.h"
#include "robotics_lib/six_axis/SixAxisModel.h"
#include "robotics_lib/six_axis/SixAxisKinematics.h"
#include "robotics_lib/six_axis/SixAxisDynamics.h"

namespace motion_control
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
            explicit GravityCompController(const motion_control::robotics::six_axis::SixAxisModel &model);

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
            void update(const motion_control::merai::JointState *states,
                        motion_control::merai::JointCommand *commands,
                        int numJoints,
                        double dt) override;

            void stop() override;
            void teardown() override;

        private:
            // Reference to the 6-axis robot model (loaded at init time).
            const motion_control::robotics::six_axis::SixAxisModel &model_;

            /**
             * @brief A local SixAxisDynamics object that uses the same model.
             *        We'll use it to compute torque via Newton-Euler.
             */
            motion_control::robotics::six_axis::SixAxisDynamics dynamics_;

            // Example param for torque scaling or other offsets
            // (optional, could be a gravity scaling factor, friction offset, etc.)
            double gravityScale_ = 1.0;
        };

    } // namespace control
} // namespace motion_control
