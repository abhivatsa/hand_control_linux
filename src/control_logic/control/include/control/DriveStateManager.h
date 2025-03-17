#pragma once

#include <cstddef>
#include <cstdint>
#include <array>

#include "merai/RTMemoryLayout.h"  // hand_control::merai::DriveStatus, DriveCommand, etc.

namespace hand_control
{
    namespace control
    {
        /**
         * @brief The DriveStateManager handles CiA 402 transitions
         *        by decoding the statusWord and writing controlWord.
         *        It also sets a minimal DriveStatus for logic to read.
         */
        class DriveStateManager
        {
        public:
            // Pass the drive data arrays and driveCount during construction
            DriveStateManager(
                std::array<hand_control::merai::ServoRxControl, hand_control::merai::MAX_SERVO_DRIVES>& driveOutputControl,
                std::array<hand_control::merai::ServoTxControl, hand_control::merai::MAX_SERVO_DRIVES>& driveInputControl,
                std::size_t driveCount);

            bool init();

            // Updated to take DriveCommand as input and update DriveStatus as output
            void update(const hand_control::merai::DriveCommand* driveCommands, hand_control::merai::DriveStatus* driveStatus);

        private:
            // Direct references to the drive input and output control data
            std::array<hand_control::merai::ServoRxControl, hand_control::merai::MAX_SERVO_DRIVES>& driveOutputControl_;
            std::array<hand_control::merai::ServoTxControl, hand_control::merai::MAX_SERVO_DRIVES>& driveInputControl_;

            std::size_t driveCount_; // Drive count, initialized during constructor

            // Helper method to decode statusWord to DriveStatus
            hand_control::merai::DriveStatus decodeStatusword(uint16_t statusWord);
        };
    } // namespace control
} // namespace hand_control
