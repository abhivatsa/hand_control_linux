#include <cstdio>
#include "logic/SafetyManager.h"

// If JointState is in hand_control::merai, include it fully:
// #include "merai/RTMemoryLayout.h"

namespace hand_control
{
    namespace logic
    {
        SafetyManager::SafetyManager(hand_control::logic::ErrorManager& errMgr)
            : errorManager_(errMgr),
              minPosition_(-1000.0),
              maxPosition_(1000.0),
              maxVelocity_(10.0),
              maxTemperature_(80.0)
        {
        }

        void SafetyManager::setPositionLimits(double minPos, double maxPos)
        {
            minPosition_ = minPos;
            maxPosition_ = maxPos;
        }

        void SafetyManager::setVelocityLimit(double maxVel)
        {
            maxVelocity_ = maxVel;
        }

        void SafetyManager::setTemperatureLimit(double maxTemp)
        {
            maxTemperature_ = maxTemp;
        }

        // If JointState is in hand_control::merai, fully qualify below:
        //   void SafetyManager::checkAllLimits(const hand_control::merai::JointState* jointStates,
        //                                      int jointCount)
        void SafetyManager::checkAllLimits(const JointState* jointStates, int jointCount)
        {
            for (int i = 0; i < jointCount; i++)
            {
                double pos  = jointStates[i].position;
                double vel  = jointStates[i].velocity;
                double temp = jointStates[i].temperature;

                if (pos < minPosition_ || pos > maxPosition_)
                {
                    char msg[64];
                    std::snprintf(msg, sizeof(msg),
                                  "Pos limit exceeded joint=%d, pos=%.2f", i, pos);
                    errorManager_.reportError(ERROR_LIMIT_EXCEEDED, msg);
                }

                if (vel > maxVelocity_ || vel < -maxVelocity_)
                {
                    char msg[64];
                    std::snprintf(msg, sizeof(msg),
                                  "Vel limit exceeded joint=%d, vel=%.2f", i, vel);
                    errorManager_.reportError(ERROR_LIMIT_EXCEEDED, msg);
                }

                if (temp > maxTemperature_)
                {
                    char msg[64];
                    std::snprintf(msg, sizeof(msg),
                                  "Over-temp joint=%d, temp=%.1fC", i, temp);
                    errorManager_.reportError(ERROR_OVER_TEMPERATURE, msg);
                }
            }
        }
    } // namespace logic
} // namespace hand_control
