#include "logic/ErrorManager.h"

namespace hand_control
{
    namespace logic
    {
        bool ErrorManager::init()
        {
            head_ = 0;
            count_ = 0;
            criticalErrorActive_ = false;
            return true;
        }

        void ErrorManager::reportError(int32_t code, Severity severity)
        {
            // Insert into ring buffer
            std::size_t insertIndex = (head_ + count_) % MAX_ERRORS;
            if (count_ < MAX_ERRORS)
            {
                count_++;
            }
            else
            {
                // buffer full => overwrite oldest
                head_ = (head_ + 1) % MAX_ERRORS;
            }

            errors_[insertIndex].code     = code;
            errors_[insertIndex].severity = severity;

            // check if severity is CRITICAL
            handleSeverity(severity);
        }

        std::size_t ErrorManager::flushErrors(int32_t* outCodes,
                                              Severity* outSeverities,
                                              std::size_t maxCount)
        {
            // flush up to maxCount or however many are stored
            std::size_t numFlushed = (count_ < maxCount) ? count_ : maxCount;

            for (std::size_t i = 0; i < numFlushed; i++)
            {
                std::size_t idx = (head_ + i) % MAX_ERRORS;

                if (outCodes)
                    outCodes[i] = errors_[idx].code;

                if (outSeverities)
                    outSeverities[i] = errors_[idx].severity;
            }

            // remove them from the ring
            head_ = (head_ + numFlushed) % MAX_ERRORS;
            count_ -= numFlushed;

            return numFlushed;
        }

        bool ErrorManager::hasCriticalError() const
        {
            return criticalErrorActive_;
        }

        void ErrorManager::clearCriticalError()
        {
            criticalErrorActive_ = false;
        }

        void ErrorManager::handleSeverity(Severity sev)
        {
            if (sev == Severity::CRITICAL)
            {
                criticalErrorActive_ = true;
            }
        }

    } // namespace logic
} // namespace hand_control
