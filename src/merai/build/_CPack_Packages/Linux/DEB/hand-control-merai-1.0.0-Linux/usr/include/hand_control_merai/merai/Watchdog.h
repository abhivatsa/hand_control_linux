#pragma once

#include <string>
#include <map>
#include <chrono>
#include <mutex>

namespace hand_control
{
    namespace merai
    {
        class Watchdog
        {
        public:
            static Watchdog& instance();

            bool init(const std::string& configFilePath);

            void registerHeartbeat(const std::string& processName, double timeoutSec);

            void beat(const std::string& processName);

            // The main monitoring loop (run in a separate thread or event loop)
            void monitorLoop();

        private:
            Watchdog() = default;
            ~Watchdog() = default;

            struct ProcessInfo
            {
                double timeoutSec;
                std::chrono::steady_clock::time_point lastBeat;
            };

            std::map<std::string, ProcessInfo> processes_;
            std::mutex                         mtx_;
            bool                               running_ = true;
        };
    } // namespace merai
} // namespace hand_control
