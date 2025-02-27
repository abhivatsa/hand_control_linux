#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace hand_control
{
    namespace logic
    {
        /**
         * @brief Example top-level system states.
         */
        enum class SystemState
        {
            INIT,
            READY,
            RUNNING,
            ERROR,
            SHUTDOWN
        };

        /**
         * @brief Fixed list of possible error codes.
         */
        enum ErrorCode
        {
            ERROR_NONE = 0,
            ERROR_DRIVE_FAULT,
            ERROR_OVER_TEMPERATURE,
            ERROR_LIMIT_EXCEEDED,

            ERROR_MAX  // Not a real code, just a sentinel
        };

        /**
         * @brief A structure for system-wide logic configuration.
         */
        struct LogicConfig
        {
            double updateRateHz = 1000.0;
            // Add more fields as needed...
        };
    } // namespace logic
} // namespace hand_control
