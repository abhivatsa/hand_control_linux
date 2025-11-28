#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <cmath>
#include <span>

// merai includes
#include "merai/RTMemoryLayout.h"
#include "merai/ParameterServer.h"
#include "merai/SharedLogger.h"

#include "control/hardware_abstraction/BaseHAL.h"

    namespace control
    {
        class RealHAL : public BaseHAL
        {
        public:
            RealHAL(
                merai::RTMemoryLayout*           rtLayout,
                const merai::ParameterServer*    paramServerPtr,
                merai::multi_ring_logger_memory* loggerMem);

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
            merai::JointControlCommand* getJointControlCommandPtr() override
            {
                return localJointControlCommand_.data();
            }

            merai::JointControlFeedback* getJointControlFeedbackPtr() override
            {
                return localJointControlFeedback_.data();
            }

            // ---------------------------
            // Motion-level data access
            // ---------------------------
            merai::JointMotionCommand* getJointMotionCommandPtr() override
            {
                return localJointMotionCommand_.data();
            }

            merai::JointMotionFeedback* getJointMotionFeedbackPtr() override
            {
                return localJointMotionFeedback_.data();
            }

            // ---------------------------
            // IO data access
            // ---------------------------
            merai::JointFeedbackIO* getJointFeedbackIOPtr() override
            {
                return localJointFeedbackIO_.data();
            }

            merai::JointCommandIO* getJointCommandIOPtr() override
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

            std::span<const merai::JointControlFeedback> jointControlFeedback() const override
            {
                return std::span<const merai::JointControlFeedback>(localJointControlFeedback_.data(), static_cast<std::size_t>(driveCount_));
            }

            std::span<const merai::JointMotionFeedback> jointMotionFeedback() const override
            {
                return std::span<const merai::JointMotionFeedback>(localJointMotionFeedback_.data(), static_cast<std::size_t>(driveCount_));
            }

            std::span<const merai::JointFeedbackIO> jointIOFeedback() const override
            {
                return std::span<const merai::JointFeedbackIO>(localJointFeedbackIO_.data(), static_cast<std::size_t>(driveCount_));
            }

            std::span<merai::JointControlCommand> jointControlCommand() override
            {
                return std::span<merai::JointControlCommand>(localJointControlCommand_.data(), static_cast<std::size_t>(driveCount_));
            }

            std::span<merai::JointMotionCommand> jointMotionCommand() override
            {
                return std::span<merai::JointMotionCommand>(localJointMotionCommand_.data(), static_cast<std::size_t>(driveCount_));
            }

            bool publishJointFeedbackToShm() override;

        private:
            // --------------------------------------------------
            // References to shared memory, config, logger
            // --------------------------------------------------
            merai::RTMemoryLayout*           rtLayout_       = nullptr;
            const merai::ParameterServer*    paramServerPtr_ = nullptr;
            merai::multi_ring_logger_memory* loggerMem_      = nullptr;

            // Number of drives/joints
            int driveCount_ = 0;

            // --------------------------------------------------
            // Local joint-level data structures
            // --------------------------------------------------

            // Control commands & feedback
            std::array<merai::JointControlCommand,
                       merai::MAX_SERVO_DRIVES>   localJointControlCommand_;
            std::array<merai::JointControlFeedback,
                       merai::MAX_SERVO_DRIVES>   localJointControlFeedback_;

            // Motion commands & feedback (in SI units)
            std::array<merai::JointMotionCommand,
                       merai::MAX_SERVO_DRIVES>   localJointMotionCommand_;
            std::array<merai::JointMotionFeedback,
                       merai::MAX_SERVO_DRIVES>   localJointMotionFeedback_;

            // IO feedback & command arrays (digital/analog)
            std::array<merai::JointFeedbackIO,
                       merai::MAX_SERVO_DRIVES>   localJointFeedbackIO_;
            std::array<merai::JointCommandIO,
                       merai::MAX_SERVO_DRIVES>   localJointCommandIO_;

            // Local copy of each joint's config (gear ratio, offset, etc.)
            std::array<merai::JointConfig,
                       merai::MAX_SERVO_DRIVES>   localJointConfigs_;

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
                const std::array<merai::ServoTxPdo,
                                 merai::MAX_SERVO_DRIVES>& servoTxArray);

            /**
             * @brief Converts local joint commands (in SI) to raw servo units
             *        and maps them into the servo Rx array in shared memory.
             *
             * @param servoRxArray Reference to the array of ServoRxPdo in shared memory.
             * @return true if successful, false otherwise
             */
            bool convertAndMapJointCommandsToServoRxData(
                std::array<merai::ServoRxPdo,
                           merai::MAX_SERVO_DRIVES>& servoRxArray);

        };

    } // namespace control
