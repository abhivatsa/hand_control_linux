#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <cmath>

// merai includes
#include "merai/RTMemoryLayout.h"
#include "merai/ParameterServer.h"
#include "merai/SharedLogger.h"

#include "control/hardware_abstraction/BaseHAL.h"

namespace seven_axis_robot
{
    namespace control
    {
        class RealHAL : public BaseHAL
        {
        public:
            RealHAL(
                seven_axis_robot::merai::RTMemoryLayout*           rtLayout,
                const seven_axis_robot::merai::ParameterServer*    paramServerPtr,
                seven_axis_robot::merai::multi_ring_logger_memory* loggerMem);

            ~RealHAL() override = default;

            // --------------------------------------------------
            // Implementing BaseHAL
            // --------------------------------------------------
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

            // ---------------------------
            // Drive count
            // ---------------------------
            size_t getDriveCount() const override
            {
                return driveCount_;
            }

        private:
            // --------------------------------------------------
            // References to shared memory, config, logger
            // --------------------------------------------------
            seven_axis_robot::merai::RTMemoryLayout*           rtLayout_       = nullptr;
            const seven_axis_robot::merai::ParameterServer*    paramServerPtr_ = nullptr;
            seven_axis_robot::merai::multi_ring_logger_memory* loggerMem_      = nullptr;

            // Number of drives/joints
            int driveCount_ = 0;

            // --------------------------------------------------
            // Local joint-level data structures
            // --------------------------------------------------

            // Control commands & feedback
            std::array<seven_axis_robot::merai::JointControlCommand,
                       seven_axis_robot::merai::MAX_SERVO_DRIVES>   localJointControlCommand_;
            std::array<seven_axis_robot::merai::JointControlFeedback,
                       seven_axis_robot::merai::MAX_SERVO_DRIVES>   localJointControlFeedback_;

            // Motion commands & feedback (in SI units)
            std::array<seven_axis_robot::merai::JointMotionCommand,
                       seven_axis_robot::merai::MAX_SERVO_DRIVES>   localJointMotionCommand_;
            std::array<seven_axis_robot::merai::JointMotionFeedback,
                       seven_axis_robot::merai::MAX_SERVO_DRIVES>   localJointMotionFeedback_;

            // IO feedback & command arrays (digital/analog)
            std::array<seven_axis_robot::merai::JointFeedbackIO,
                       seven_axis_robot::merai::MAX_SERVO_DRIVES>   localJointFeedbackIO_;
            std::array<seven_axis_robot::merai::JointCommandIO,
                       seven_axis_robot::merai::MAX_SERVO_DRIVES>   localJointCommandIO_;

            // Local copy of each joint's config (gear ratio, offset, etc.)
            std::array<seven_axis_robot::merai::JointConfig,
                       seven_axis_robot::merai::MAX_SERVO_DRIVES>   localJointConfigs_;

        private:
            // --------------------------------------------------
            // Private helpers for reading/writing EtherCAT data
            // --------------------------------------------------

            /**
             * @brief Reads servo Tx data from shared memory, converts to SI units,
             *        and populates local feedback arrays in one pass.
             *
             * @param servoTxArray Reference to the array of ServoTxPdo from shared memory.
             * @return true if successful, false otherwise
             */
            bool mapAndConvertServoTxData(
                const std::array<seven_axis_robot::merai::ServoTxPdo,
                                 seven_axis_robot::merai::MAX_SERVO_DRIVES>& servoTxArray);

            /**
             * @brief Converts local joint commands (in SI) to raw servo units
             *        and maps them into the servo Rx array in shared memory.
             *
             * @param servoRxArray Reference to the array of ServoRxPdo in shared memory.
             * @return true if successful, false otherwise
             */
            bool convertAndMapJointCommandsToServoRxData(
                std::array<seven_axis_robot::merai::ServoRxPdo,
                           seven_axis_robot::merai::MAX_SERVO_DRIVES>& servoRxArray);
        };

    } // namespace control
} // namespace seven_axis_robot
