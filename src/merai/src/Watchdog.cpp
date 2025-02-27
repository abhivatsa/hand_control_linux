#include "merai/Watchdog.h"
#include "merai/Logger.h"    // Adjust to your logger path
#include "merai/ConfigParser.h"  // If you parse JSON config
#include <thread>
#include <chrono>

namespace hand_control
{
    namespace merai
    {
        Watchdog& Watchdog::instance()
        {
            static Watchdog instance;
            return instance;
        }

        bool Watchdog::init(const std::string& configFilePath)
        {
            // Parse config file for default timeouts or process definitions
            // If the file is not found, return false or use defaults
            // parseJsonConfig(configFilePath, ...);
            return true;
        }

        void Watchdog::registerHeartbeat(const std::string& processName, double timeoutSec)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            processes_[processName] = { timeoutSec, std::chrono::steady_clock::now() };
            Logger::info("Watchdog registered heartbeat for: " + processName);
        }

        void Watchdog::beat(const std::string& processName)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            auto it = processes_.find(processName);
            if (it != processes_.end())
            {
                it->second.lastBeat = std::chrono::steady_clock::now();
            }
            else
            {
                // Possibly log an error if an unregistered process tries to beat
                Logger::warning("Watchdog: Unregistered process " + processName);
            }
        }

        void Watchdog::monitorLoop()
        {
            using namespace std::chrono_literals;

            while (running_)
            {
                {
                    std::lock_guard<std::mutex> lock(mtx_);
                    auto now = std::chrono::steady_clock::now();

                    for (auto& kv : processes_)
                    {
                        double elapsedSec =
                            std::chrono::duration<double>(now - kv.second.lastBeat).count();

                        if (elapsedSec > kv.second.timeoutSec)
                        {
                            Logger::error("Watchdog timeout for process: " + kv.first);
                            // Trigger error handling or safe shutdown
                            // ...
                        }
                    }
                }
                std::this_thread::sleep_for(100ms); // Check interval
            }
        }
    } // namespace merai
} // namespace hand_control
