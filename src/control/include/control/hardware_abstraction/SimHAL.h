#pragma once

#include <array>
#include <cmath>

#include "merai/RTMemoryLayout.h"          // for seven_axis_robot::merai::RTMemoryLayout, etc.
#include "merai/ParameterServer.h"         // for seven_axis_robot::merai::ParameterServer
#include "merai/SharedLogger.h"            // for seven_axis_robot::merai::multi_ring_logger_memory
#include "control/hardware_abstraction/BaseHAL.h"  // for BaseHAL interface

namespace seven_axis_robot
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
            SimHAL(seven_axis_robot::merai::RTMemoryLayout* rtLayout,
                   const seven_axis_robot::merai::ParameterServer* paramServerPtr,
                   seven_axis_robot::merai::multi_ring_logger_memory* loggerMem);

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
            seven_axis_robot::merai::JointControlCommand* getJointControlCommandPtr() override
            {
                return localJointControlCommand_.data();
            }

            seven_axis_robot::merai::JointControlFeedback* getJointControlFeedbackPtr() override
            {
                return localJointControlFeedback_.data();
            }

            // ---------------------------
            // Motion-level data access
            // ---------------------------
            seven_axis_robot::merai::JointMotionCommand* getJointMotionCommandPtr() override
            {
                return localJointMotionCommand_.data();
            }

            seven_axis_robot::merai::JointMotionFeedback* getJointMotionFeedbackPtr() override
            {
                return localJointMotionFeedback_.data();
            }

            // ---------------------------
            // IO data access
            // ---------------------------
            seven_axis_robot::merai::JointFeedbackIO* getJointFeedbackIOPtr() override
            {
                return localJointFeedbackIO_.data();
            }

            seven_axis_robot::merai::JointCommandIO* getJointCommandIOPtr() override
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
            seven_axis_robot::merai::RTMemoryLayout*            rtLayout_       = nullptr;
            const seven_axis_robot::merai::ParameterServer*     paramServerPtr_ = nullptr;
            seven_axis_robot::merai::multi_ring_logger_memory*  loggerMem_      = nullptr;

            // Number of simulated drives/joints
            int driveCount_ = 0;

            // --------------------------------------------------------------------
            // Joint-level arrays (simulation data)
            // --------------------------------------------------------------------

            // Control commands / feedback
            std::array<seven_axis_robot::merai::JointControlCommand,
                       seven_axis_robot::merai::MAX_SERVO_DRIVES>   localJointControlCommand_{};
            std::array<seven_axis_robot::merai::JointControlFeedback,
                       seven_axis_robot::merai::MAX_SERVO_DRIVES>   localJointControlFeedback_{};

            // Motion commands / feedback (in SI units)
            std::array<seven_axis_robot::merai::JointMotionCommand,
                       seven_axis_robot::merai::MAX_SERVO_DRIVES>   localJointMotionCommand_{};
            std::array<seven_axis_robot::merai::JointMotionFeedback,
                       seven_axis_robot::merai::MAX_SERVO_DRIVES>   localJointMotionFeedback_{};

            // Joint I/O (digital, analog, etc.)
            std::array<seven_axis_robot::merai::JointFeedbackIO,
                       seven_axis_robot::merai::MAX_SERVO_DRIVES>   localJointFeedbackIO_{};
            std::array<seven_axis_robot::merai::JointCommandIO,
                       seven_axis_robot::merai::MAX_SERVO_DRIVES>   localJointCommandIO_{};

        private:
            // --------------------------------------------------------------------
            // Helpers for simulating behavior
            // --------------------------------------------------------------------
            void simulateDriveStateTransitions();
            void simulateJointIOChanges();
        };
    } // namespace control
} // namespace seven_axis_robot
