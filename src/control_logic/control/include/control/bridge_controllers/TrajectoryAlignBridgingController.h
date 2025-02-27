#pragma once

#include <string>
#include <vector>  // if we handle a path or just a single setpoint

#include "control/bridge_controllers/BaseBridgingController.h"
// If JointState / JointCommand come from hand_control::merai, include that header:
// #include "merai/RTMemoryLayout.h"

namespace hand_control
{
    namespace control
    {
        class TrajectoryAlignBridgingController : public BaseBridgingController
        {
        public:
            TrajectoryAlignBridgingController() = default;
            ~TrajectoryAlignBridgingController() override = default;

            bool init(const std::string& controllerName) override;
            void start() override;

            // If JointState / JointCommand come from hand_control::merai, fully qualify them here:
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

            /**
             * @brief setTargetPositions: store a desired final alignment pose,
             *        e.g., to align transitions between controllers.
             */
            void setTargetPositions(const std::vector<double>& positions);

        private:
            bool   done_{false};
            double elapsedTime_{0.0};
            double maxTime_{5.0};    // or something
            int    jointCount_{6};

            std::vector<double> targetPositions_;  // the next trajectory's start
        };
    } // namespace control
} // namespace hand_control
