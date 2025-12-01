#pragma once

#include <array>
#include <cstdint>
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
        RealHAL(merai::RTMemoryLayout           *rtLayout,
                const merai::ParameterServer    *paramServerPtr,
                merai::multi_ring_logger_memory *loggerMem);

        ~RealHAL() override = default;

        // BaseHAL interface
        bool init()  override;
        bool read()  override;
        bool write() override;

        // Pointer-based access
        merai::JointControlCommand *getJointControlCommandPtr() override
        {
            return localJointControlCommand_.data();
        }

        merai::JointControlFeedback *getJointControlFeedbackPtr() override
        {
            return localJointControlFeedback_.data();
        }

        merai::JointMotionCommand *getJointMotionCommandPtr() override
        {
            return localJointMotionCommand_.data();
        }

        merai::JointMotionFeedback *getJointMotionFeedbackPtr() override
        {
            return localJointMotionFeedback_.data();
        }

        merai::JointFeedbackIO *getJointFeedbackIOPtr() override
        {
            return localJointFeedbackIO_.data();
        }

        merai::JointCommandIO *getJointCommandIOPtr() override
        {
            return localJointCommandIO_.data();
        }

        std::size_t getDriveCount() const override
        {
            return static_cast<std::size_t>(driveCount_);
        }

        // Span-based access
        std::span<const merai::JointControlFeedback> jointControlFeedback() const override
        {
            return {localJointControlFeedback_.data(),
                    static_cast<std::size_t>(driveCount_)};
        }

        std::span<const merai::JointMotionFeedback> jointMotionFeedback() const override
        {
            return {localJointMotionFeedback_.data(),
                    static_cast<std::size_t>(driveCount_)};
        }

        std::span<const merai::JointFeedbackIO> jointIOFeedback() const override
        {
            return {localJointFeedbackIO_.data(),
                    static_cast<std::size_t>(driveCount_)};
        }

        std::span<merai::JointControlCommand> jointControlCommand() override
        {
            return {localJointControlCommand_.data(),
                    static_cast<std::size_t>(driveCount_)};
        }

        std::span<merai::JointMotionCommand> jointMotionCommand() override
        {
            return {localJointMotionCommand_.data(),
                    static_cast<std::size_t>(driveCount_)};
        }

        bool publishJointFeedbackToShm() override;

    private:
        // References to shared memory, config, logger
        merai::RTMemoryLayout           *rtLayout_       = nullptr;
        const merai::ParameterServer    *paramServerPtr_ = nullptr;
        merai::multi_ring_logger_memory *loggerMem_      = nullptr;

        int driveCount_ = 0;

        // Local joint-level data
        std::array<merai::JointControlCommand,
                   merai::MAX_SERVO_DRIVES>   localJointControlCommand_{};
        std::array<merai::JointControlFeedback,
                   merai::MAX_SERVO_DRIVES>   localJointControlFeedback_{};

        std::array<merai::JointMotionCommand,
                   merai::MAX_SERVO_DRIVES>   localJointMotionCommand_{};
        std::array<merai::JointMotionFeedback,
                   merai::MAX_SERVO_DRIVES>   localJointMotionFeedback_{};

        std::array<merai::JointFeedbackIO,
                   merai::MAX_SERVO_DRIVES>   localJointFeedbackIO_{};
        std::array<merai::JointCommandIO,
                   merai::MAX_SERVO_DRIVES>   localJointCommandIO_{};

        // Static joint config (gear ratio, offsets, etc.)
        std::array<merai::JointConfig,
                   merai::MAX_SERVO_DRIVES>   localJointConfigs_{};

    private:
        // Helpers for mapping raw servo data <-> SI joint data
        bool mapAndConvertServoTxData(
            const std::array<merai::ServoTxPdo,
                             merai::MAX_SERVO_DRIVES> &servoTxArray);

        bool convertAndMapJointCommandsToServoRxData(
            std::array<merai::ServoRxPdo,
                       merai::MAX_SERVO_DRIVES> &servoRxArray);
    };

} // namespace control
