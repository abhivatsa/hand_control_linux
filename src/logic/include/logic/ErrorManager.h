#pragma once

#include <cstddef>   // for size_t
#include <cstdint>   // for int32_t
#include "logic/Severity.h" // Include the severity enum

namespace seven_axis_robot
{
    namespace logic
    {
        /**
         * @brief A minimal ErrorManager storing a ring buffer of (code, severity) pairs.
         *        - Single-thread usage only (no locking).
         *        - If a CRITICAL severity is reported, sets a flag for immediate action.
         */
        class ErrorManager
        {
        public:
            static constexpr std::size_t MAX_ERRORS = 32;  // capacity of the ring buffer

            ErrorManager() = default;
            ~ErrorManager() = default;

            /**
             * @brief Initializes/resets the error manager (clear buffer, reset flags).
             * @return true if successful
             */
            bool init();

            /**
             * @brief Report a new error with code and severity.
             *        Overwrites the oldest if the buffer is full.
             *
             * @param code     integer code (e.g. 101)
             * @param severity enumerated severity: INFO, WARNING, ERROR, CRITICAL
             */
            void reportError(int32_t code, Severity severity);

            /**
             * @brief Flush stored errors out of the ring buffer, removing them from internal storage.
             *
             * @param outCodes       array to receive the error codes
             * @param outSeverities  array to receive the severities
             * @param maxCount       max number of errors you can retrieve
             * @return number of errors actually retrieved
             */
            std::size_t flushErrors(int32_t* outCodes,
                                    Severity* outSeverities,
                                    std::size_t maxCount);

            /**
             * @brief Check if a CRITICAL error has been reported but not cleared.
             */
            bool hasCriticalError() const;

            /**
             * @brief Clear the critical error flag (for system recovery).
             */
            void clearCriticalError();

        private:
            struct ErrorEntry
            {
                int32_t  code;
                Severity severity;
            };

            ErrorEntry  errors_[MAX_ERRORS];
            std::size_t head_  = 0; // index of oldest error
            std::size_t count_ = 0; // how many errors in the buffer

            bool criticalErrorActive_ = false;

            /**
             * @brief Check severity, set criticalErrorActive_ if it's CRITICAL.
             */
            void handleSeverity(Severity sev);
        };

    } // namespace logic
} // namespace seven_axis_robot
