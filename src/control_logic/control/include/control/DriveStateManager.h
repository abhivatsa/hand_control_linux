#pragma once

#include <cstddef>
#include <cstdint>

// merai includes
#include "merai/RTMemoryLayout.h" // for DriveCommand, DriveStatus, etc.

namespace hand_control
{
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
             */
            DriveStateManager(hand_control::merai::JointControlCommand* jointCommandPtr,
                              hand_control::merai::JointControlFeedback* jointFeedbackPtr,
                              std::size_t driveCount);

            /**
             * @brief Initialize the manager (e.g., check pointers, set defaults).
             * @return True if pointers are valid, false otherwise.
             */
            bool init();

            /**
             * @brief Update the CiA-402 state machine for each drive, given
             *        a desired DriveCommand.
             * 
             * @param driveCommands  Array of enumerated commands (one per drive).
             * @param driveStatus    Array to store the resulting drive status for each drive.
             */
            void update(const hand_control::merai::DriveCommand* driveCommands,
                        hand_control::merai::DriveStatus* driveStatus);

        private:
            /// Helper function for decoding statusWord bits into a DriveStatus enum.
            hand_control::merai::DriveStatus decodeStatusword(uint16_t statusWord);

        private:
            // Pointers to the arrays returned by HAL (but now joint-level data).
            hand_control::merai::JointControlCommand*  jointCommandPtr_  = nullptr;
            hand_control::merai::JointControlFeedback* jointFeedbackPtr_ = nullptr;

            std::size_t driveCount_ = 0;
        };

    } // namespace control
} // namespace hand_control
