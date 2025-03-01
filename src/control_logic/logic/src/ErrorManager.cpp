#include "logic/ErrorManager.h"
#include <iostream>

namespace hand_control
{
    namespace logic
    {
        bool ErrorManager::init()
        {
            // If you need to load config or set up a file, do it here
            return true;
        }

        void ErrorManager::reportError(int errorCode, const std::string& message)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            errors_.push_back({errorCode, message});

            // Example: treat codes >= 100 as critical
            if (errorCode >= 100)
            {
                criticalErrorActive_ = true;
            }
            // Also print or log
            std::cerr << "[ErrorManager] Error " << errorCode << ": " << message << "\n";
        }

        std::vector<std::string> ErrorManager::flushErrors()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            std::vector<std::string> msgs;
            msgs.reserve(errors_.size());
            for (auto &e : errors_)
            {
                msgs.push_back("Code " + std::to_string(e.code) + ": " + e.message);
            }
            errors_.clear();
            return msgs;
        }

        bool ErrorManager::hasCriticalError() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return criticalErrorActive_;
        }

        void ErrorManager::clearCriticalError()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            criticalErrorActive_ = false;
        }
    }
}
