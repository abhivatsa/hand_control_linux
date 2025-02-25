#pragma once

#include "ErrorManager.h"
// If JointState is defined in motion_control::merai (or somewhere else), include it:
// #include "merai/RTMemoryLayout.h"  // or your actual header

namespace motion_control
{
    namespace logic
    {
        class SafetyManager
        {
        public:
            explicit SafetyManager(motion_control::logic::ErrorManager& errMgr);

            void setPositionLimits(double minPos, double maxPos);
            void setVelocityLimit(double maxVel);
            void setTemperatureLimit(double maxTemp);

            /**
             * @brief checkAllLimits Called periodically to check position, velocity, and temperature limits.
             * @param jointStates Pointer to array of JointState objects
             * @param jointCount  Number of joints
             *
             * If JointState is in motion_control::merai, we fully qualify it here:
             *    void checkAllLimits(const motion_control::merai::JointState* jointStates, int jointCount);
             */
            void checkAllLimits(const /* motion_control::merai:: */ JointState* jointStates,
                                int jointCount);

        private:
            motion_control::logic::ErrorManager& errorManager_;

            double minPosition_;
            double maxPosition_;
            double maxVelocity_;
            double maxTemperature_;
        };
    } // namespace logic
} // namespace motion_control
