#pragma once

#include "control/controllers/BaseController.h"
#include "merai/RTMemoryLayout.h" // for JointMotionFeedback, JointMotionCommand

namespace hand_control
{
    namespace control
    {
        /**
         * @brief A simple controller that moves all joints from their current position
         *        to a fixed "home" position using a trapezoidal position trajectory.
         *
         * In this “motion-only” approach, we read from JointMotionFeedback (positions, velocities)
         * and write new position commands to JointMotionCommand. We ignore control word / status word
         * or any I/O signals.
         */
        class HomingController : public BaseController
        {
        public:
            static constexpr int MAX_JOINTS = 6;

            /**
             * @brief Constructor.
             *
             * @param homePositions  Array of joint positions to serve as the homing target.
             * @param numJoints      Number of active joints (<= MAX_JOINTS).
             * @param feedbackPtr    Pointer to array of JointMotionFeedback (one per joint).
             * @param commandPtr     Pointer to array of JointMotionCommand (one per joint).
             */
            HomingController(const double* homePositions,
                             int numJoints,
                             hand_control::merai::JointMotionFeedback* feedbackPtr,
                             hand_control::merai::JointMotionCommand*  commandPtr);

            ~HomingController() override = default;

            bool init() override;
            void start() override;
            void update(double dt) override;
            void stop() override;
            void teardown() override;

        private:
            /**
             * @brief Plan the trapezoidal trajectory to move from current positions -> homePositions_.
             * @param currentPos Array of current joint positions.
             */
            void planTrajectory(const double* currentPos);

            /**
             * @brief Helper to hold position when the trajectory is inactive or done.
             */
            void holdPosition();

        private:
            hand_control::merai::JointMotionFeedback* feedbackPtr_ = nullptr;
            hand_control::merai::JointMotionCommand*  commandPtr_  = nullptr;
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
