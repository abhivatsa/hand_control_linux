#pragma once

#include <array>
#include <cmath>
#include <span>

#include "merai/RTMemoryLayout.h"          // for merai::RTMemoryLayout, etc.
#include "merai/ParameterServer.h"         // for merai::ParameterServer
#include "merai/SharedLogger.h"            // for merai::multi_ring_logger_memory
#include "control/hardware_abstraction/BaseHAL.h"  // for BaseHAL interface

    namespace control
    {
        /**
         * @brief A simulation-based Hardware Abstraction Layer (HAL)
         *        that mimics RealHAL but doesn't interact with real EtherCAT hardware.
         */
        class SimHAL : public BaseHAL
        {
        public:
            SimHAL(merai::RTMemoryLayout* rtLayout,
                   const merai::ParameterServer* paramServerPtr,
                   merai::multi_ring_logger_memory* loggerMem);

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

            // --------------------------------------------------------------------
            // Misc
            // --------------------------------------------------------------------
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
            // --------------------------------------------------------------------
            // References to shared memory, config, logger (as needed)
            // --------------------------------------------------------------------
            merai::RTMemoryLayout*            rtLayout_       = nullptr;
            const merai::ParameterServer*     paramServerPtr_ = nullptr;
            merai::multi_ring_logger_memory*  loggerMem_      = nullptr;

            // Number of simulated drives/joints
            int driveCount_ = 0;

            // --------------------------------------------------------------------
            // Joint-level arrays (simulation data)
            // --------------------------------------------------------------------

            // Control commands / feedback
            std::array<merai::JointControlCommand,
                       merai::MAX_SERVO_DRIVES>   localJointControlCommand_{};
            std::array<merai::JointControlFeedback,
                       merai::MAX_SERVO_DRIVES>   localJointControlFeedback_{};

            // Motion commands / feedback (in SI units)
            std::array<merai::JointMotionCommand,
                       merai::MAX_SERVO_DRIVES>   localJointMotionCommand_{};
            std::array<merai::JointMotionFeedback,
                       merai::MAX_SERVO_DRIVES>   localJointMotionFeedback_{};

            // Joint I/O (digital, analog, etc.)
            std::array<merai::JointFeedbackIO,
                       merai::MAX_SERVO_DRIVES>   localJointFeedbackIO_{};
            std::array<merai::JointCommandIO,
                       merai::MAX_SERVO_DRIVES>   localJointCommandIO_{};

        private:
            // --------------------------------------------------------------------
            // Helpers for simulating behavior
            // --------------------------------------------------------------------
            void simulateDriveStateTransitions();
            void simulateJointIOChanges();
        };
    } // namespace control
