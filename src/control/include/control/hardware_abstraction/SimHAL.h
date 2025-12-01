#pragma once

#include <array>
#include <cstddef>
#include <span>

#include "merai/RTMemoryLayout.h"
#include "merai/ParameterServer.h"
#include "merai/SharedLogger.h"

#include "control/hardware_abstraction/BaseHAL.h"

namespace control
{
    /**
     * @brief Simple simulation HAL.
     *
     * - Keeps local joint command/feedback arrays in SI units.
     * - Does NOT use fieldbus / servo PDOs.
     * - Can optionally publish joint feedback into RT shared memory so other
     *   processes can observe simulated motion/IO.
     *
     * It implements the same interface as RealHAL so Control.cpp doesn’t care
     * whether we’re in sim or real mode.
     */
    class SimHAL : public BaseHAL
    {
    public:
        SimHAL(merai::RTMemoryLayout           *rtLayout,
               const merai::ParameterServer    *paramServerPtr,
               merai::multi_ring_logger_memory *loggerMem);

        ~SimHAL() override = default;

        // --------------------------------------------------
        // BaseHAL: lifecycle
        // --------------------------------------------------
        bool init()  override;
        bool read()  override;
        bool write() override;

        // --------------------------------------------------
        // BaseHAL: pointer access
        // --------------------------------------------------
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

        // --------------------------------------------------
        // BaseHAL: metadata
        // --------------------------------------------------
        std::size_t getDriveCount() const override
        {
            return static_cast<std::size_t>(driveCount_);
        }

        // --------------------------------------------------
        // BaseHAL: span-based access (used by Control.cpp)
        // --------------------------------------------------
        std::span<const merai::JointControlFeedback> jointControlFeedback() const override
        {
            return {
                localJointControlFeedback_.data(),
                static_cast<std::size_t>(driveCount_)
            };
        }

        std::span<const merai::JointMotionFeedback> jointMotionFeedback() const override
        {
            return {
                localJointMotionFeedback_.data(),
                static_cast<std::size_t>(driveCount_)
            };
        }

        std::span<const merai::JointFeedbackIO> jointIOFeedback() const override
        {
            return {
                localJointFeedbackIO_.data(),
                static_cast<std::size_t>(driveCount_)
            };
        }

        std::span<merai::JointControlCommand> jointControlCommand() override
        {
            return {
                localJointControlCommand_.data(),
                static_cast<std::size_t>(driveCount_)
            };
        }

        std::span<merai::JointMotionCommand> jointMotionCommand() override
        {
            return {
                localJointMotionCommand_.data(),
                static_cast<std::size_t>(driveCount_)
            };
        }

        // Publish feedback into RT SHM (optional but nice for tooling)
        bool publishJointFeedbackToShm() override;

    private:
        void simulateDriveStateTransitions();
        void simulateJointIOChanges();

    private:
        // References
        merai::RTMemoryLayout           *rtLayout_       = nullptr;
        const merai::ParameterServer    *paramServerPtr_ = nullptr;
        merai::multi_ring_logger_memory *loggerMem_      = nullptr;

        int driveCount_ = 0; // 0..7 for your arm

        // Local joint-level state
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
    };

} // namespace control
