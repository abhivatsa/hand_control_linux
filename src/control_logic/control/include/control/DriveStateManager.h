#pragma once

#include <cstddef>
#include <cstdint>

// merai includes
#include "merai/RTMemoryLayout.h" // for DriveCommand, DriveStatus, etc.

namespace hand_control
{
    namespace control
    {
        class DriveStateManager
        {
        public:
            /**
             * @brief Constructor taking pointers to the drive output and drive input arrays.
             */
            DriveStateManager(hand_control::merai::ServoRxControl* driveOutputControlPtr,
                              hand_control::merai::ServoTxControl* driveInputControlPtr,
                              std::size_t driveCount);

            bool init();

            void update(const hand_control::merai::DriveCommand* driveCommands,
                        hand_control::merai::DriveStatus* driveStatus);

        private:
            // Pointers to the arrays returned by HAL
            hand_control::merai::ServoRxControl* driveOutputControlPtr_ = nullptr;
            hand_control::merai::ServoTxControl* driveInputControlPtr_ = nullptr;

            std::size_t driveCount_ = 0;

            // Helper function if you want to decode statusWord
            hand_control::merai::DriveStatus decodeStatusword(uint16_t statusWord);
        };

    } // namespace control
} // namespace hand_control
