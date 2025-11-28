#pragma once

#include <cstddef>
#include <cstdint>
#include "control/ControlData.h"

// merai includes
#include "merai/RTMemoryLayout.h" // for DriveCommand, DriveStatus, etc.
#include "merai/SharedLogger.h"

namespace control
{
    /**
     * @brief DriveStateManager manages the CiA-402 state machine transitions
     *        using the joint-level statusWord and controlWord.
     *
     * It reads the 'statusWord' from JointControlFeedback and writes the 'controlWord'
     * into JointControlCommand. The higher-level logic of which command to issue (e.g.,
     * FAULT_RESET, ALLOW_OPERATION) comes in via the update() function.
     *
     * The enumerations DriveCommand/DriveStatus remain the same, but we no longer
     * deal with servo-level structures.
     */
    class DriveStateManager
    {
    public:
        /**
         * @brief Constructor taking pointers to the joint control and feedback arrays.
         *
         * @param jointCommandPtr   Pointer to array of JointControlCommand (one per drive).
         * @param jointFeedbackPtr  Pointer to array of JointControlFeedback (one per drive).
         * @param driveCount        Number of drives.
         * @param rtLayout          Shared memory layout for publishing drive feedback.
         */
        DriveStateManager(std::size_t driveCount,
                          merai::multi_ring_logger_memory *loggerMem);

        /**
         * @brief Initialize the manager (e.g., check pointers, set defaults).
         * @return True if pointers are valid, false otherwise.
         */
        bool init();

        /**
         * @brief Update the CiA-402 state machine for each drive, given
         *        a desired DriveCommand.
         *
         * Publishes resulting drive statuses to driveFeedbackBuffer in shared memory.
         *
         * @param driveCommands  Array of enumerated commands (one per drive).
         */
        void update(const ControlCycleInputs &in,
                    ControlCycleOutputs &out);

    private:
        /// Helper function for decoding statusWord bits into a DriveStatus enum.
        merai::DriveStatus decodeStatusword(uint16_t statusWord);

    private:
        std::size_t driveCount_ = 0;
        merai::multi_ring_logger_memory *loggerMem_ = nullptr;
    };

} // namespace control
