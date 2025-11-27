#pragma once

#include <cstddef>
#include <cstdint>

// merai includes
#include "merai/RTMemoryLayout.h" // for DriveCommand, DriveStatus, etc.
#include "merai/SharedLogger.h"

namespace seven_axis_robot
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
            DriveStateManager(seven_axis_robot::merai::JointControlCommand* jointCommandPtr,
                              seven_axis_robot::merai::JointControlFeedback* jointFeedbackPtr,
                              std::size_t driveCount,
                              seven_axis_robot::merai::multi_ring_logger_memory* loggerMem);

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
            void update(const seven_axis_robot::merai::DriveCommand* driveCommands,
                        seven_axis_robot::merai::DriveStatus* driveStatus);

        private:
            /// Helper function for decoding statusWord bits into a DriveStatus enum.
            seven_axis_robot::merai::DriveStatus decodeStatusword(uint16_t statusWord);

        private:
            // Pointers to the arrays returned by HAL (but now joint-level data).
            seven_axis_robot::merai::JointControlCommand*  jointCommandPtr_  = nullptr;
            seven_axis_robot::merai::JointControlFeedback* jointFeedbackPtr_ = nullptr;

            std::size_t driveCount_ = 0;
            seven_axis_robot::merai::multi_ring_logger_memory* loggerMem_ = nullptr;
        };

    } // namespace control
} // namespace seven_axis_robot
