#pragma once

#include "ErrorCodes.h"
#include <cstring>  // for std::strncpy if you need it in .cpp

namespace hand_control
{
    namespace logic
    {
        /**
         * @brief Holds information about an error: code, active status, and a message.
         */
        struct ErrorInfo
        {
            ErrorCode code;
            bool      active;
            char      message[64];  // fixed-size message buffer
        };

        /**
         * @brief The ErrorManager class handles reporting, storing, and clearing errors.
         */
        class ErrorManager
        {
        public:
            static constexpr int MAX_ACTIVE_ERRORS = 16;

            ErrorManager();

            /**
             * @brief Report (or update) an error with a code and message.
             * @param code    ErrorCode enum value
             * @param message Null-terminated string describing the error
             */
            void reportError(ErrorCode code, const char* message);

            /**
             * @brief Clear a specific error, identified by its ErrorCode.
             * @param code ErrorCode to clear
             */
            void clearError(ErrorCode code);

            /**
             * @brief Checks if any error is active.
             * @return True if at least one error is active, false otherwise.
             */
            bool hasActiveErrors() const;

            /**
             * @brief Check if a specific error is active.
             * @param code ErrorCode to check
             * @return True if active, false otherwise.
             */
            bool isErrorActive(ErrorCode code) const;

            /**
             * @brief Clear all errors from the error array.
             */
            void clearAll();

        private:
            ErrorInfo errors_[MAX_ACTIVE_ERRORS];

            /**
             * @brief findErrorSlot Finds the index of an existing error code,
             *        or an empty slot if the error is not present.
             * @param code ErrorCode to find
             * @return Index in errors_ array, or -1 if none found.
             */
            int findErrorSlot(ErrorCode code) const;
        };
    } // namespace logic
} // namespace hand_control
