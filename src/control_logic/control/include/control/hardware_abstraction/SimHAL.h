#pragma once

#include <array>
#include <iostream>
#include <cmath>

#include "merai/RTMemoryLayout.h"          // for hand_control::merai::RTMemoryLayout, etc.
#include "merai/ParameterServer.h"         // for hand_control::merai::ParameterServer
#include "merai/SharedLogger.h"            // for hand_control::merai::multi_ring_logger_memory
#include "control/hardware_abstraction/BaseHAL.h"  // for BaseHAL interface

namespace hand_control
{
    namespace control
    {
        /**
         * @brief A simulation-based Hardware Abstraction Layer (HAL)
         *        that mimics RealHAL but doesn't interact with real EtherCAT hardware.
         */
        class SimHAL : public BaseHAL
        {
        public:
            SimHAL(hand_control::merai::RTMemoryLayout* rtLayout,
                   const hand_control::merai::ParameterServer* paramServerPtr,
                   hand_control::merai::multi_ring_logger_memory* loggerMem);

            ~SimHAL() override = default;

            // --------------------------------------------------------------------
            // BaseHAL interface
            // --------------------------------------------------------------------
            bool init() override;
            bool read() override;
            bool write() override;

            // ---------------------------
            // Control-level data access
            // ---------------------------
            hand_control::merai::JointControlCommand* getJointControlCommandPtr() override
            {
                return localJointControlCommand_.data();
            }

            hand_control::merai::JointControlFeedback* getJointControlFeedbackPtr() override
            {
                return localJointControlFeedback_.data();
            }

            // ---------------------------
            // Motion-level data access
            // ---------------------------
            hand_control::merai::JointMotionCommand* getJointMotionCommandPtr() override
            {
                return localJointMotionCommand_.data();
            }

            hand_control::merai::JointMotionFeedback* getJointMotionFeedbackPtr() override
            {
                return localJointMotionFeedback_.data();
            }

            // ---------------------------
            // IO data access
            // ---------------------------
            hand_control::merai::JointFeedbackIO* getJointFeedbackIOPtr() override
            {
                return localJointFeedbackIO_.data();
            }

            hand_control::merai::JointCommandIO* getJointCommandIOPtr() override
            {
                return localJointCommandIO_.data();
            }

            // --------------------------------------------------------------------
            // Misc
            // --------------------------------------------------------------------
            size_t getDriveCount() const override
            {
                return driveCount_;
            }

        private:
            // --------------------------------------------------------------------
            // References to shared memory, config, logger (as needed)
            // --------------------------------------------------------------------
            hand_control::merai::RTMemoryLayout*            rtLayout_       = nullptr;
            const hand_control::merai::ParameterServer*     paramServerPtr_ = nullptr;
            hand_control::merai::multi_ring_logger_memory*  loggerMem_      = nullptr;

            // Number of simulated drives/joints
            int driveCount_ = 0;

            // --------------------------------------------------------------------
            // Joint-level arrays (simulation data)
            // --------------------------------------------------------------------

            // Control commands / feedback
            std::array<hand_control::merai::JointControlCommand,
                       hand_control::merai::MAX_SERVO_DRIVES>   localJointControlCommand_{};
            std::array<hand_control::merai::JointControlFeedback,
                       hand_control::merai::MAX_SERVO_DRIVES>   localJointControlFeedback_{};

            // Motion commands / feedback (in SI units)
            std::array<hand_control::merai::JointMotionCommand,
                       hand_control::merai::MAX_SERVO_DRIVES>   localJointMotionCommand_{};
            std::array<hand_control::merai::JointMotionFeedback,
                       hand_control::merai::MAX_SERVO_DRIVES>   localJointMotionFeedback_{};

            // Joint I/O (digital, analog, etc.)
            std::array<hand_control::merai::JointFeedbackIO,
                       hand_control::merai::MAX_SERVO_DRIVES>   localJointFeedbackIO_{};
            std::array<hand_control::merai::JointCommandIO,
                       hand_control::merai::MAX_SERVO_DRIVES>   localJointCommandIO_{};

        private:
            // --------------------------------------------------------------------
            // Helpers for simulating behavior
            // --------------------------------------------------------------------
            void simulateDriveStateTransitions();
            void simulateJointIOChanges();
        };
    } // namespace control
} // namespace hand_control
