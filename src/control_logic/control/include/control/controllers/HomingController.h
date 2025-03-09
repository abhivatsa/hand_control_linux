#pragma once

#include "control/controllers/BaseController.h"
#include "merai/RTMemoryLayout.h" // for JointState, JointCommand

namespace hand_control
{
    namespace control
    {
        /**
         * @brief A simple controller that moves all joints from their current position
         *        to a fixed "home" position. Uses a trapezoidal position trajectory
         *        (accelerate, cruise, decelerate) to reach the home point.
         */
        class HomingController : public BaseController
        {
        public:
            static constexpr int MAX_JOINTS = 6;

            /**
             * @brief Constructor.
             * @param homePositions Array of joint positions to serve as the homing target.
             * @param numJoints     Number of active joints (<= MAX_JOINTS).
             */
            HomingController(const double* homePositions, int numJoints);

            ~HomingController() override = default;

            /**
             * @brief init
             *  Set internal state to INIT; can do any param loading if needed.
             */
            bool init() override;

            /**
             * @brief start
             *  Called when transitioning to this controller. Here we can plan the trajectory
             *  from the current position (wherever the joints are) to the homePositions_.
             */
            void start() override;

            /**
             * @brief update
             *  Called each real-time cycle. If homing is active, step the trapezoidal trajectory
             *  until it finishes. Once finished, hold final position.
             *
             * @param states    Pointer to array of JointState
             * @param commands  Pointer to array of JointCommand
             * @param numJoints Number of joints
             * @param dt        Timestep in seconds
             */
            void update(const hand_control::merai::JointState* states,
                        hand_control::merai::JointCommand* commands,
                        int numJoints,
                        double dt) override;

            /**
             * @brief stop
             *  Called when transitioning away to another controller; stop the motion.
             */
            void stop() override;

            /**
             * @brief teardown
             *  Cleanup if needed, or reset to UNINIT.
             */
            void teardown() override;

        private:
            /**
             * @brief Plan the trapezoidal trajectory to move from current positions -> homePositions_.
             * @param currentPos Current joint positions
             */
            void planTrajectory(const double* currentPos);

            /**
             * @brief Helper to hold position when the trajectory is inactive.
             * @param states   JointState array
             * @param commands JointCommand array
             */
            void holdPosition(const hand_control::merai::JointState* states,
                              hand_control::merai::JointCommand* commands);

        private:
            int numJoints_{0};
            double homePositions_[MAX_JOINTS];  ///< The target homing position for each joint
            double iniPositions_[MAX_JOINTS];   ///< Positions at the start of homing
            double jointAcc_[MAX_JOINTS];       ///< Computed trapezoid acceleration (sign included)
            double segmentTime_{0.0};           ///< Current time in the homing trajectory
            bool  isActive_{false};             ///< True if a homing trajectory is in progress

            // Times for the trapezoidal motion
            double globalAccTime_{0.0};
            double globalCruiseTime_{0.0};
            double maxTime_{0.0}; // total time for the entire homing motion
        };

    } // namespace control
} // namespace hand_control
