#include <cstdio>    // printf
#include <cstring>   // strncpy

#include "logic/ErrorManager.h"

namespace motion_control
{
    namespace logic
    {
        ErrorManager::ErrorManager()
        {
            for (int i = 0; i < MAX_ACTIVE_ERRORS; i++)
            {
                errors_[i].code      = ERROR_NONE;
                errors_[i].active    = false;
                errors_[i].message[0] = '\0';
            }
        }

        void ErrorManager::reportError(ErrorCode code, const char* message)
        {
            if (code == ERROR_NONE)
            {
                return;
            }

            // Check if this error already exists
            int slot = findErrorSlot(code);
            if (slot >= 0)
            {
                // Update existing message if needed
                std::strncpy(errors_[slot].message, message, sizeof(errors_[slot].message) - 1);
                errors_[slot].message[sizeof(errors_[slot].message) - 1] = '\0';
                return;
            }

            // Otherwise, find an empty slot
            for (int i = 0; i < MAX_ACTIVE_ERRORS; i++)
            {
                if (!errors_[i].active)
                {
                    errors_[i].code   = code;
                    errors_[i].active = true;
                    std::strncpy(errors_[i].message, message, sizeof(errors_[i].message) - 1);
                    errors_[i].message[sizeof(errors_[i].message) - 1] = '\0';

                    std::printf("[ErrorManager] Error reported: code=%d, msg=%s\n",
                                static_cast<int>(code), errors_[i].message);
                    return;
                }
            }

            // If no free slots
            std::printf("[ErrorManager] No free error slots, error not recorded.\n");
        }

        void ErrorManager::clearError(ErrorCode code)
        {
            for (int i = 0; i < MAX_ACTIVE_ERRORS; i++)
            {
                if (errors_[i].active && errors_[i].code == code)
                {
                    errors_[i].active    = false;
                    errors_[i].code      = ERROR_NONE;
                    errors_[i].message[0] = '\0';
                    std::printf("[ErrorManager] Cleared error code=%d\n", static_cast<int>(code));
                }
            }
        }

        bool ErrorManager::hasActiveErrors() const
        {
            for (int i = 0; i < MAX_ACTIVE_ERRORS; i++)
            {
                if (errors_[i].active)
                {
                    return true;
                }
            }
            return false;
        }

        bool ErrorManager::isErrorActive(ErrorCode code) const
        {
            return (findErrorSlot(code) >= 0);
        }

        void ErrorManager::clearAll()
        {
            for (int i = 0; i < MAX_ACTIVE_ERRORS; i++)
            {
                errors_[i].active    = false;
                errors_[i].code      = ERROR_NONE;
                errors_[i].message[0] = '\0';
            }
        }

        int ErrorManager::findErrorSlot(ErrorCode code) const
        {
            for (int i = 0; i < MAX_ACTIVE_ERRORS; i++)
            {
                if (errors_[i].active && errors_[i].code == code)
                {
                    return i;
                }
            }
            return -1;
        }
    } // namespace logic
} // namespace motion_control
