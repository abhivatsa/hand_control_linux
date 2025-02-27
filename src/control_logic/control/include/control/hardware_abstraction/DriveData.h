#pragma once

#include <cstdint>

namespace hand_control
{
    namespace control
    {
        /**
         * @brief DriveInput contains the raw or lightly processed data
         *        read from the servo drive (EtherCAT TxPDO) or simulated.
         */
        struct DriveInput
        {
            uint16_t statusWord       = 0;
            int32_t  positionRaw      = 0;
            int32_t  velocityRaw      = 0;
            int16_t  torqueRaw        = 0;
            uint8_t  modeOfOperation  = 0;  // optional
        };

        /**
         * @brief DriveOutput contains the raw data to write to the servo drive
         *        (EtherCAT RxPDO) or a simulated environment.
         */
        struct DriveOutput
        {
            uint16_t controlWord       = 0;
            int32_t  targetPositionRaw = 0;
            int32_t  targetVelocityRaw = 0;
            int16_t  targetTorqueRaw   = 0;
            uint8_t  modeOfOperation   = 0; // optional
        };
    } // namespace control
} // namespace hand_control
