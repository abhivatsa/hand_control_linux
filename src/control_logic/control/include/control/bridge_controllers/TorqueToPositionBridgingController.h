#pragma once

#include <string>

#include "control/bridge_controllers/BaseBridgingController.h"
// If JointState, JointCommand come from motion_control::merai, include the header defining them, e.g.:
// #include "merai/RTMemoryLayout.h"

namespace motion_control
{
    namespace control
    {
        class TorqueToPositionBridgingController : public BaseBridgingController
        {
        public:
            TorqueToPositionBridgingController() = default;
            ~TorqueToPositionBridgingController() override = default;

            bool init(const std::string& controllerName) override;
            void start() override;

            // If JointState/JointCommand are from motion_control::merai, fully qualify, e.g.:
            //   void update(const motion_control::merai::JointState* states,
            //               motion_control::merai::JointCommand* commands,
            //               double dt) override;

            void update(const motion_control::merai::JointState* states,
                        motion_control::merai::JointCommand* commands,
                        double dt) override;

            void stop() override;
            void teardown() override;

            bool isDone() const override
            {
                return done_;
            }

            void setJointCount(int count)
            {
                jointCount_ = count;
            }

            void setTransitionTime(double t)
            {
                transitionTime_ = t;
            }

        private:
            bool   done_{false};
            double elapsedTime_{0.0};
            double transitionTime_{1.0};
            int    jointCount_{6};  // or get from config
        };
    } // namespace control
} // namespace motion_control
