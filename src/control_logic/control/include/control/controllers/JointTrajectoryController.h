#pragma once

#include <cstddef>
#include <cmath>
#include "merai/RTMemoryLayout.h"

namespace hand_control
{
    namespace control
    {

        class JointTrajectoryController
        {
        public:
            static constexpr size_t MAX_JOINTS = 6;

            JointTrajectoryController();
            ~JointTrajectoryController();

            bool init(size_t numJoints);

            void start();

            void update(const merai::JointState *jointStates,
                        merai::JointCommand *jointCommands,
                        double dt);

            void stop();

            // This is where we do multi-joint trapezoid “pre-processing” in one shot
            bool planTrajectory(const double *currentPos,
                                    const double *targetPos,
                                    double *jointVel, // velocity limits (size = numJoints)
                                    double *jointAcc, // acceleration limits (size = numJoints)
                                    double dt);

        private:
            size_t numJoints_;

            // Store for each joint: initial pos, final pos, the signed acceleration, etc.
            double iniPos_[MAX_JOINTS];
            double finalPos_[MAX_JOINTS];
            double jointAcc_[MAX_JOINTS]; // computed/recomputed so all joints sync
            double maxTime_;              // total time for the entire motion
            double globalAccTime_;        // time spent accelerating
            double globalCruiseTime_;     // time spent cruising
            double segmentTime_;          // current time within the trajectory
            bool isActive_;               // is the trajectory active?

            // We define accelerate, cruise, decelerate “phases”:
            // total_dec_time = globalAccTime_ (assuming symmetrical)

            // Helper to hold or clamp positions if we’re done or not started
            void holdPosition(const merai::JointState *jointStates,
                              merai::JointCommand *jointCommands) const;
        };

    } // namespace control
} // namespace hand_control
