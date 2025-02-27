#pragma once

#include "ErrorManager.h"
// If JointState is defined in hand_control::merai (or somewhere else), include it:
// #include "merai/RTMemoryLayout.h"  // or your actual header

namespace hand_control
{
    namespace logic
    {
        class SafetyManager
        {
        public:
            explicit SafetyManager(hand_control::logic::ErrorManager& errMgr);

            void setPositionLimits(double minPos, double maxPos);
            void setVelocityLimit(double maxVel);
            void setTemperatureLimit(double maxTemp);

            /**
             * @brief checkAllLimits Called periodically to check position, velocity, and temperature limits.
             * @param jointStates Pointer to array of JointState objects
             * @param jointCount  Number of joints
             *
             * If JointState is in hand_control::merai, we fully qualify it here:
             *    void checkAllLimits(const hand_control::merai::JointState* jointStates, int jointCount);
             */
            void checkAllLimits(const /* hand_control::merai:: */ JointState* jointStates,
                                int jointCount);

        private:
            hand_control::logic::ErrorManager& errorManager_;

            double minPosition_;
            double maxPosition_;
            double maxVelocity_;
            double maxTemperature_;
        };
    } // namespace logic
} // namespace hand_control
