#pragma once

#include <string>

#include "control/bridge_controllers/BaseBridgingController.h"
// If JointState, JointCommand come from hand_control::merai, include the header defining them, e.g.:
// #include "merai/RTMemoryLayout.h"

namespace hand_control
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

            // If JointState/JointCommand are from hand_control::merai, fully qualify, e.g.:
            //   void update(const hand_control::merai::JointState* states,
            //               hand_control::merai::JointCommand* commands,
            //               double dt) override;

            void update(const hand_control::merai::JointState* states,
                        hand_control::merai::JointCommand* commands,
                        int numJoints,
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
} // namespace hand_control
