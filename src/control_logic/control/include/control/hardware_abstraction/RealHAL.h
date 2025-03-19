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

namespace hand_control
{
    namespace control
    {
        class RealHAL : public BaseHAL
        {
        public:
            RealHAL(
                hand_control::merai::RTMemoryLayout*           rtLayout,
                const hand_control::merai::ParameterServer*    paramServerPtr,
                hand_control::merai::multi_ring_logger_memory* loggerMem);

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
            hand_control::merai::RTMemoryLayout*           rtLayout_       = nullptr;
            const hand_control::merai::ParameterServer*    paramServerPtr_ = nullptr;
            hand_control::merai::multi_ring_logger_memory* loggerMem_      = nullptr;

            // Number of drives/joints
            int driveCount_ = 0;

            // --------------------------------------------------
            // Local joint-level data structures
            // --------------------------------------------------

            // Control commands & feedback
            std::array<hand_control::merai::JointControlCommand,
                       hand_control::merai::MAX_SERVO_DRIVES>   localJointControlCommand_;
            std::array<hand_control::merai::JointControlFeedback,
                       hand_control::merai::MAX_SERVO_DRIVES>   localJointControlFeedback_;

            // Motion commands & feedback (in SI units)
            std::array<hand_control::merai::JointMotionCommand,
                       hand_control::merai::MAX_SERVO_DRIVES>   localJointMotionCommand_;
            std::array<hand_control::merai::JointMotionFeedback,
                       hand_control::merai::MAX_SERVO_DRIVES>   localJointMotionFeedback_;

            // IO feedback & command arrays (digital/analog)
            std::array<hand_control::merai::JointFeedbackIO,
                       hand_control::merai::MAX_SERVO_DRIVES>   localJointFeedbackIO_;
            std::array<hand_control::merai::JointCommandIO,
                       hand_control::merai::MAX_SERVO_DRIVES>   localJointCommandIO_;

            // Local copy of each joint's config (gear ratio, offset, etc.)
            std::array<hand_control::merai::JointConfig,
                       hand_control::merai::MAX_SERVO_DRIVES>   localJointConfigs_;

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
                const std::array<hand_control::merai::ServoTxPdo,
                                 hand_control::merai::MAX_SERVO_DRIVES>& servoTxArray);

            /**
             * @brief Converts local joint commands (in SI) to raw servo units
             *        and maps them into the servo Rx array in shared memory.
             *
             * @param servoRxArray Reference to the array of ServoRxPdo in shared memory.
             * @return true if successful, false otherwise
             */
            bool convertAndMapJointCommandsToServoRxData(
                std::array<hand_control::merai::ServoRxPdo,
                           hand_control::merai::MAX_SERVO_DRIVES>& servoRxArray);
        };

    } // namespace control
} // namespace hand_control
